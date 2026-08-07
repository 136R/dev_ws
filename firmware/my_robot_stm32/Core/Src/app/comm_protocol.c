/*
 * comm_protocol.c — USART1 binary protocol between STM32 and ROS2
 *
 * Frame format (all frames):
 *   [0xAA] [0x55] [TYPE] [LEN] [DATA × LEN bytes] [XOR]
 *   XOR = TYPE ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]
 *   Byte order: little-endian
 *
 * TYPE 0x01  ROS2→STM32  Velocity command (9 bytes total)
 *   DATA: left_mrad_s  int16 LE,  right_mrad_s  int16 LE
 *
 * TYPE 0x02  STM32→ROS2  Raw feedback  (41 bytes total, 100 Hz)
 *   DATA: left_delta   int32 LE, right_delta  int32 LE,
 *         acc_xyz      int32 LE × 3  [mm/s²]
 *         gyro_xyz     int32 LE × 3  [urad/s]
 *         yaw_mdeg     int32 LE      [millidegrees, Fusion AHRS Euler yaw]
 *   Note: mag field removed — ICM-42688-P is 6-axis (no magnetometer)
 */

#include "app/comm_protocol.h"
#include "app/motor_controller.h"
#include "app/imu.h"
#include "usart.h"
#include <math.h>
#include <string.h>

/* ── Internal state ─────────────────────────────────────────────────── */

static uint8_t rx_buf[COMM_RX_BUF_SIZE];

/* TX buffer must be static: DMA reads it after HAL_UART_Transmit_DMA returns */
static uint8_t tx_buf[COMM_FEEDBACK_FRAME_SIZE];
static volatile uint8_t tx_busy = 0;   /* 1 while DMA TX is in progress */

static volatile float g_target_left_rad_s  = 0.0f;
static volatile float g_target_right_rad_s = 0.0f;
static volatile uint32_t g_watchdog_ticks  = 0u;

/* Last known IMU values (updated when imu_data_ready()) */
static volatile int32_t g_acc_mms2[3]   = {0, 0, 0};
static volatile int32_t g_gyro_urad_s[3] = {0, 0, 0};

static void comm_start_rx_dma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, COMM_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/* ── Helpers ────────────────────────────────────────────────────────── */

static uint8_t calc_xor(uint8_t type, uint8_t len, const uint8_t *data)
{
    uint8_t x = type ^ len;
    for (uint8_t i = 0; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

/* Pack int32 little-endian into buf */
static void pack_i32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)( val        & 0xFF);
    buf[1] = (uint8_t)((val >>  8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* Unpack int16 little-endian from buf */
static int16_t unpack_i16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static int32_t to_i32_scaled(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

/* ── RX parsing ─────────────────────────────────────────────────────── */

static void parse_rx(const uint8_t *buf, uint32_t len)
{
    /* Search for a valid 0x01 velocity command frame */
    for (uint32_t i = 0; i + COMM_VEL_FRAME_SIZE <= len; i++) {
        if (buf[i]   != COMM_HEADER_1) continue;
        if (buf[i+1] != COMM_HEADER_2) continue;
        if (buf[i+2] != COMM_TYPE_VEL_CMD) continue;
        if (buf[i+3] != COMM_VEL_CMD_LEN) continue;

        const uint8_t *data = &buf[i+4];
        uint8_t expected_xor = calc_xor(COMM_TYPE_VEL_CMD, COMM_VEL_CMD_LEN, data);
        if (buf[i + 4 + COMM_VEL_CMD_LEN] != expected_xor) continue;

        /* Valid frame — update targets (continue scanning; last frame wins) */
        int16_t left_mrad_s  = unpack_i16(&data[0]);
        int16_t right_mrad_s = unpack_i16(&data[2]);
        g_target_left_rad_s  = left_mrad_s  / 1000.0f;
        g_target_right_rad_s = right_mrad_s / 1000.0f;
        g_watchdog_ticks = 0u;  /* reset watchdog */
        i += COMM_VEL_FRAME_SIZE - 1u;  /* advance past this frame (loop does i++) */
    }
}

/* ── Public API ─────────────────────────────────────────────────────── */

void comm_protocol_init(void)
{
    g_target_left_rad_s  = 0.0f;
    g_target_right_rad_s = 0.0f;
    g_watchdog_ticks = 0u;
    tx_busy = 0;
    memset((void *)g_acc_mms2,    0, sizeof(g_acc_mms2));
    memset((void *)g_gyro_urad_s, 0, sizeof(g_gyro_urad_s));
    memset(rx_buf, 0, sizeof(rx_buf));

    comm_start_rx_dma();
}

void comm_protocol_tick(int32_t left_delta, int32_t right_delta)
{
    /* ── 1. Update IMU snapshot if new data is ready ── */
    if (imu_data_ready()) {
        imu_data_t imu;
        imu_get_data(&imu);
        g_acc_mms2[0] = to_i32_scaled(imu.accel[0], 1000.0f);
        g_acc_mms2[1] = to_i32_scaled(imu.accel[1], 1000.0f);
        g_acc_mms2[2] = to_i32_scaled(imu.accel[2], 1000.0f);

        g_gyro_urad_s[0] = to_i32_scaled(imu.gyro[0], 1000000.0f);
        g_gyro_urad_s[1] = to_i32_scaled(imu.gyro[1], 1000000.0f);
        g_gyro_urad_s[2] = to_i32_scaled(imu.gyro[2], 1000000.0f);
    }

    /* ── 2. Watchdog ── */
    if (g_watchdog_ticks < COMM_WATCHDOG_TICKS) {
        g_watchdog_ticks++;
        if (g_watchdog_ticks == COMM_WATCHDOG_TICKS) {
            /* Exactly 500 ms elapsed — stop motors */
            g_target_left_rad_s  = 0.0f;
            g_target_right_rad_s = 0.0f;
            motor_controller_stop();
        }
    }

    /* ── 3. Transmit raw feedback at 100 Hz ── */
    if (tx_busy) {
        /* Previous DMA TX still running — skip this cycle to avoid corruption */
        return;
    }

    /* Build payload (36 bytes): encoder(8) + accel(12) + gyro(12) + yaw(4) */
    uint8_t payload[COMM_FEEDBACK_LEN];
    pack_i32(&payload[0],  left_delta);
    pack_i32(&payload[4],  right_delta);
    pack_i32(&payload[8],  g_acc_mms2[0]);
    pack_i32(&payload[12], g_acc_mms2[1]);
    pack_i32(&payload[16], g_acc_mms2[2]);
    pack_i32(&payload[20], g_gyro_urad_s[0]);
    pack_i32(&payload[24], g_gyro_urad_s[1]);
    pack_i32(&payload[28], g_gyro_urad_s[2]);
    pack_i32(&payload[32], (int32_t)(g_yaw * 1000.0f));

    /* Assemble full frame into static TX buffer */
    tx_buf[0] = COMM_HEADER_1;
    tx_buf[1] = COMM_HEADER_2;
    tx_buf[2] = COMM_TYPE_FEEDBACK;
    tx_buf[3] = COMM_FEEDBACK_LEN;
    memcpy(&tx_buf[4], payload, COMM_FEEDBACK_LEN);
    tx_buf[4 + COMM_FEEDBACK_LEN] = calc_xor(COMM_TYPE_FEEDBACK, COMM_FEEDBACK_LEN, payload);

    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, tx_buf, COMM_FEEDBACK_FRAME_SIZE) != HAL_OK) {
        tx_busy = 0;  /* DMA busy or error — allow retry next cycle */
    }
}

float comm_protocol_get_left_rad_s(void)  { return g_target_left_rad_s; }
float comm_protocol_get_right_rad_s(void) { return g_target_right_rad_s; }

void comm_protocol_uart_rx_event_handler(uint16_t size)
{
    parse_rx(rx_buf, size);
    comm_start_rx_dma();
}

/* HAL DMA TX complete callback — clear busy flag */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_busy = 0;
    }
}

void comm_protocol_uart_error_handler(void)
{
    comm_start_rx_dma();
}
