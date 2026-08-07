// 稳定版
/*
 * imu.c — ICM-42688-P SPI 驱动
 *
 * 引脚：
 *   SPI1_SCK  PA5
 *   SPI1_MISO PA6  (SDO/SA0)
 *   SPI1_MOSI PA7  (SDA/MOSI)
 *   CS        PA4  (GPIO Output, active low)
 *   INT1      PB12 (EXTI rising edge, data-ready)
 *
 * 量程配置：
 *   陀螺仪：±500 dps  → GYRO_SCALE = 500/32768 * π/180  rad/s/LSB
 *   加速度：±4 g      → ACCEL_SCALE = 4/32768 * 9.80665 m/s²/LSB
 *   温度：  T(°C) = raw / 132.48 + 25.0
 *
 * 数据就绪机制（方案 A）：
 *   ICM INT1 上升沿 (PB12) → EXTI12 ISR → imu_int1_irq_handler() → g_ready = true
 *   TIM6 10ms → motor_controller_update → comm_protocol_tick
 *       → imu_data_ready() 为真时调用 imu_get_data()
 *       → imu_get_data() 内做 SPI 突发读（14 字节 ≈ 12 μs @ 9 MHz）
 */

#include "app/imu.h"
#include "app/fusion/FusionAhrs.h"
#include "spi.h"
#include "main.h"   /* IMU_CS_Pin, IMU_CS_GPIO_Port, IMU_INT1_Pin */
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ── CS 控制宏 ─────────────────────────────────────────────────────── */
#define IMU_CS_LOW()   HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define IMU_CS_HIGH()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

/* ── ICM-42688-P 寄存器地址（Bank 0）──────────────────────────────── */
#define ICM_REG_DEVICE_CONFIG   0x11u   /* bit[0] SOFT_RESET_CONFIG */
#define ICM_REG_INT_CONFIG      0x14u   /* INT1 极性 / 驱动 / 模式  */
#define ICM_REG_TEMP_DATA1      0x1Du   /* 突发读起始地址            */
#define ICM_REG_PWR_MGMT0       0x4Eu
#define ICM_REG_GYRO_CONFIG0    0x4Fu
#define ICM_REG_ACCEL_CONFIG0   0x50u
#define ICM_REG_INT_CONFIG1     0x64u
#define ICM_REG_INT_SOURCE0     0x65u   /* bit[3] UI_DRDY_INT1_EN   */
#define ICM_REG_WHO_AM_I        0x75u
#define ICM_REG_BANK_SEL        0x76u

#define ICM_WHO_AM_I_VAL        0x47u

/* ── SF 修正 ── */
#define GYRO_Z_SF_POS    1.0f   /* 正向修正系数*/
#define GYRO_Z_SF_NEG    1.0f   /* 负向修正系数*/

/* ── 量程 & 转换系数 ────────────────────────────────────────────────── */
/* 陀螺 ±500 dps → 灵敏度 65.5 LSB/(°/s) */
#define GYRO_SCALE   (500.0f / 32768.0f * (float)M_PI / 180.0f)
/* 加速 ±4 g → 灵敏度 8192 LSB/g */
#define ACCEL_SCALE  (4.0f / 32768.0f * 9.80665f)
/* 温度（ICM-42688-P 数据手册） */
#define TEMP_SENS    132.48f
#define TEMP_OFFSET  25.0f

/* ── 陀螺 Z 轴滞回死区（yaw 轴，偏置补偿后）───────────────────────── */
/* 进入死区阈值：静止时实测噪声峰值约 0.008 rad/s，取 1.25× 余量       */
/* 退出死区阈值：进入阈值的 2.5×，防止边界处反复抖动                   */
/* 调参方法：打印 g_gyro[2]，取静止时绝对值最大值 × 1.25 → DB_ENTER    */
#define GYRO_Z_DB_ENTER  0.01f   /* rad/s，进入零区 */
#define GYRO_Z_DB_EXIT   0.015f   /* rad/s，退出零区 */

/* ── Fusion AHRS（上层 100 Hz 调用周期）────────────────────────────── */
/* dt 使用固定常数 = 1/ODR，不依赖 HAL_GetTick() 的 1 ms 分辨率抖动 */
#define IMU_FUSION_DT_S             0.01f   /* 100 Hz → 10 ms */
#define IMU_FUSION_GAIN             0.01f
#define IMU_FUSION_ACCEL_REJECT_DEG 10.0f
#define IMU_FUSION_RECOVERY_TICKS   200u

/* ── 陀螺偏置标定 ───────────────────────────────────────────────────── */
#define CAL_NUM_SAMPLES  1000u   /* 5 s @ 100 Hz，机器人必须静止 */

/* ── 内部状态 ───────────────────────────────────────────────────────── */
static imu_data_t g_imu;
static volatile bool g_ready = false;
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static FusionAhrs g_ahrs;
static bool g_gyro_z_in_deadzone = true;   /* 滞回死区状态：开机默认在零区内 */

/* ── 调试全局变量（Live Expressions / 示波器用）────────────────────── */
volatile float    g_acc[3]              = {0.0f, 0.0f, 0.0f};
volatile float    g_gyro[3]             = {0.0f, 0.0f, 0.0f};
volatile float    g_mag[3]              = {0.0f, 0.0f, 0.0f};  /* 6 轴无磁力计，始终为 0 */
volatile float    g_gyro_bias_rad_s[3]  = {0.0f, 0.0f, 0.0f};
volatile float    g_yaw = 0.0f;
volatile float    yaw_gyro_accum_deg = 0.0f;     /* 原始imu累计 */
volatile uint32_t g_imu_packet_count    = 0u;
volatile uint32_t g_imu_parse_error_count = 0u;

/* ── SPI 底层：单寄存器读 ───────────────────────────────────────────── */
static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0x00u};
    uint8_t rx[2] = {0u, 0u};
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 10u);
    IMU_CS_HIGH();
    return rx[1];
}

/* ── SPI 底层：单寄存器写 ───────────────────────────────────────────── */
static void icm_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), data};
    IMU_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2u, 10u);
    IMU_CS_HIGH();
}

/* ── SPI 底层：突发读 14 字节（TEMP + ACCEL_XYZ + GYRO_XYZ）────────── */
/* 必须用 TransmitReceive 一次性完成，避免 STM32 SPI FIFO 在两次调用间残留脏数据 */
static void icm_burst_read_sensors(uint8_t *buf14)
{
    uint8_t tx_buf[15] = {0};
    uint8_t rx_buf[15] = {0};
    tx_buf[0] = (uint8_t)(ICM_REG_TEMP_DATA1 | 0x80u);  /* bit7=1 → 读操作 */
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 15u, 20u);
    IMU_CS_HIGH();
    /* rx_buf[0] 是地址字节期间收到的垃圾，有效数据从 rx_buf[1] 开始 */
    for (uint8_t i = 0u; i < 14u; i++) {
        buf14[i] = rx_buf[i + 1u];
    }
}

/* ── 内部：读取并换算所有传感器数据 ────────────────────────────────── */
static void icm_read_all(void)
{
    uint8_t raw[14];
    icm_burst_read_sensors(raw);

    /* 温度 (raw[0..1]) */
    int16_t t  = (int16_t)(((uint16_t)raw[0] << 8u) | raw[1]);
    g_imu.temp = (float)t / TEMP_SENS + TEMP_OFFSET;

    /* 加速度 (raw[2..7]：X1 X0 Y1 Y0 Z1 Z0) */
    int16_t ax = (int16_t)(((uint16_t)raw[2] << 8u) | raw[3]);
    int16_t ay = (int16_t)(((uint16_t)raw[4] << 8u) | raw[5]);
    int16_t az = (int16_t)(((uint16_t)raw[6] << 8u) | raw[7]);
    g_imu.accel[0] = (float)ax * ACCEL_SCALE;
    g_imu.accel[1] = (float)ay * ACCEL_SCALE;
    g_imu.accel[2] = (float)az * ACCEL_SCALE;

    /* 陀螺仪 (raw[8..13]：X1 X0 Y1 Y0 Z1 Z0) */
    int16_t gx = (int16_t)(((uint16_t)raw[8]  << 8u) | raw[9]);
    int16_t gy = (int16_t)(((uint16_t)raw[10] << 8u) | raw[11]);
    int16_t gz = (int16_t)(((uint16_t)raw[12] << 8u) | raw[13]);
    g_imu.gyro[0] = (float)gx * GYRO_SCALE - g_gyro_bias[0];
    g_imu.gyro[1] = (float)gy * GYRO_SCALE - g_gyro_bias[1];
//    g_imu.gyro[2] = (float)gz * GYRO_SCALE - g_gyro_bias[2];

    float gz_unbiased = (float)gz * GYRO_SCALE - g_gyro_bias[2];

    /* 正负分开做 SF 修正 */
    float sf_z = (gz_unbiased >= 0.0f) ? GYRO_Z_SF_POS : GYRO_Z_SF_NEG;
    g_imu.gyro[2] = gz_unbiased * sf_z;

    /* 单独累计yaw 无归一化 */
    yaw_gyro_accum_deg += g_imu.gyro[2] * IMU_FUSION_DT_S * 180.0f / M_PI;

    /* 陀螺 Z 轴滞回死区：抑制 yaw 噪声积分
     *   在死区内：|gz| > DB_EXIT → 退出死区，输出真实值
     *   在死区外：|gz| < DB_ENTER → 进入死区，输出 0
     * X/Y 轴保持原值（roll/pitch 由 AHRS 加速度计修正，无需死区） */
    if (g_gyro_z_in_deadzone) {
        if (fabsf(g_imu.gyro[2]) > GYRO_Z_DB_EXIT) {
            g_gyro_z_in_deadzone = false;
        } else {
        	g_imu.gyro[0] = 0.0f;
        	g_imu.gyro[1] = 0.0f;
            g_imu.gyro[2] = 0.0f;
        }
    } else {
        if (fabsf(g_imu.gyro[2]) < GYRO_Z_DB_ENTER) {
            g_gyro_z_in_deadzone = true;
            g_imu.gyro[0] = 0.0f;
            g_imu.gyro[1] = 0.0f;
            g_imu.gyro[2] = 0.0f;
        }
    }

    /* 6 轴无磁力计 */
    g_imu.mag[0]  = 0.0f;  g_imu.mag[1]  = 0.0f;  g_imu.mag[2]  = 0.0f;

    const FusionVector gyroscope = {.axis = {
        .x = FusionRadiansToDegrees(g_imu.gyro[0]),
        .y = FusionRadiansToDegrees(g_imu.gyro[1]),
        .z = FusionRadiansToDegrees(g_imu.gyro[2]),
    }};
    const FusionVector accelerometer = {.axis = {
        .x = g_imu.accel[0] / 9.80665f,
        .y = g_imu.accel[1] / 9.80665f,
        .z = g_imu.accel[2] / 9.80665f,
    }};


    FusionAhrsUpdateNoMagnetometer(&g_ahrs, gyroscope, accelerometer, IMU_FUSION_DT_S);

    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&g_ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);

    g_imu.quat[0] = quaternion.element.w;
    g_imu.quat[1] = quaternion.element.x;
    g_imu.quat[2] = quaternion.element.y;
    g_imu.quat[3] = quaternion.element.z;
    g_imu.roll = euler.angle.roll;
    g_imu.pitch = euler.angle.pitch;
    g_imu.yaw = euler.angle.yaw;

    /* 更新调试全局变量 */
    g_acc[0]  = g_imu.accel[0];
    g_acc[1]  = g_imu.accel[1];
    g_acc[2]  = g_imu.accel[2];
    g_gyro[0] = g_imu.gyro[0];
    g_gyro[1] = g_imu.gyro[1];
    g_gyro[2] = g_imu.gyro[2];

    g_yaw = g_imu.yaw;

    g_imu_packet_count++;
}

/* ── 公共 API ────────────────────────────────────────────────────────── */

void imu_init(void)
{
    memset(&g_imu, 0, sizeof(g_imu));
    g_imu.quat[0] = 1.0f;
    g_ready = false;
    g_gyro_bias[0] = 0.0f;  g_gyro_bias[1] = 0.0f;  g_gyro_bias[2] = 0.0f;
    g_imu_packet_count      = 0u;
    g_imu_parse_error_count = 0u;

    FusionAhrsInitialise(&g_ahrs);
    FusionAhrsSettings settings = {
        .convention = FusionConventionEnu,
        .gain = IMU_FUSION_GAIN,
        .gyroscopeRange = 500.0f,
        .accelerationRejection = IMU_FUSION_ACCEL_REJECT_DEG,
        .magneticRejection = 0.0f,
        .recoveryTriggerPeriod = IMU_FUSION_RECOVERY_TICKS,
    };
    FusionAhrsSetSettings(&g_ahrs, &settings);

    IMU_CS_HIGH();
    HAL_Delay(10u);

    /* 软复位 */
    icm_write_reg(ICM_REG_DEVICE_CONFIG, 0x01u);
    HAL_Delay(10u);   /* 等待复位完成（数据手册要求 ≥1 ms） */

    /* 验证 WHO_AM_I（期望 0x47） */
    if (icm_read_reg(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_VAL) {
        g_imu_parse_error_count++;
        /* 不 hang，继续初始化；调试时在此加断点检查 SPI 接线 */
    }

    /* 陀螺：±500 dps，ODR 100 Hz
     * GYRO_CONFIG0 [7:5]=010(±500dps)  [3:0]=1000(100Hz)
     * 注：0x47 的 bits[3:0]=0111 是 200 Hz，0x48 的 bits[3:0]=1000 才是 100 Hz */
    icm_write_reg(ICM_REG_GYRO_CONFIG0, 0x48u);

    /* 加速：±4 g，ODR 100 Hz
     * ACCEL_CONFIG0 [7:5]=010(±4g)  [3:0]=1000(100Hz) */
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, 0x48u);

    /* 上电：陀螺 + 加速均进入 Low-Noise 模式
     * PWR_MGMT0 [3:2]=11(GYRO LN)  [1:0]=11(ACCEL LN) */
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x0Fu);
    HAL_Delay(50u);    /* 等待传感器就绪（≥1 ms） */

    /* DLPF：陀螺 + 加速均 ~44Hz（ODR=100Hz，值 3 = max(400,100)/8 = 50Hz nominal，3dB≈44.4Hz）*/
    icm_write_reg(0x52u, 0x33u);

    /* INT1：Push-Pull，Active High，脉冲模式
     * INT_CONFIG [2]=0(pulse)  [1]=1(push-pull)  [0]=1(active-high) */
    icm_write_reg(ICM_REG_INT_CONFIG, 0x03u);

    /* 清除 INT_CONFIG1 异步复位位，防止中断丢失 */
    icm_write_reg(ICM_REG_INT_CONFIG1, 0x00u);

    /* 使能 UI Data-Ready 中断路由到 INT1
     * INT_SOURCE0 bit[3] = UI_DRDY_INT1_EN */
    icm_write_reg(ICM_REG_INT_SOURCE0, 0x08u);
}

/* 从 EXTI12 ISR 调用（PB12 上升沿，极轻量，仅置标志） */
void imu_int1_irq_handler(void)
{
    g_ready = true;
}

bool imu_data_ready(void)
{
    return g_ready;
}

void imu_get_data(imu_data_t *out)
{
    icm_read_all();    /* SPI 读取 + 换算（≈12 μs @ 9 MHz） */
    g_ready = false;
    if (out) { *out = g_imu; }
}

void imu_peek_data(imu_data_t *out)
{
    if (out) { *out = g_imu; }
}

void imu_calibrate_gyro_bias(void)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t count = 0u;
    uint8_t raw[14];
    int16_t gx, gy, gz;

    g_gyro_bias[0] = 0.0f;  g_gyro_bias[1] = 0.0f;  g_gyro_bias[2] = 0.0f;

    while (count < CAL_NUM_SAMPLES) {
        if (g_ready) {
            /* 直接读原始传感器数据，不经过 icm_read_all()，
             * 避免把带偏置的陀螺数据喂进 AHRS 滤波器 */
            icm_burst_read_sensors(raw);

            gx = (int16_t)(((uint16_t)raw[8]  << 8u) | raw[9]);
            gy = (int16_t)(((uint16_t)raw[10] << 8u) | raw[11]);
            gz = (int16_t)(((uint16_t)raw[12] << 8u) | raw[13]);
            sum[0] += (float)gx * GYRO_SCALE;
            sum[1] += (float)gy * GYRO_SCALE;
            sum[2] += (float)gz * GYRO_SCALE;

            g_ready = false;
            count++;
        }
    }

    g_gyro_bias[0] = sum[0] / (float)CAL_NUM_SAMPLES;
    g_gyro_bias[1] = sum[1] / (float)CAL_NUM_SAMPLES;
    g_gyro_bias[2] = sum[2] / (float)CAL_NUM_SAMPLES;

    g_gyro_bias_rad_s[0] = g_gyro_bias[0];
    g_gyro_bias_rad_s[1] = g_gyro_bias[1];
    g_gyro_bias_rad_s[2] = g_gyro_bias[2];

    g_gyro_z_in_deadzone = true;
    FusionAhrsRestart(&g_ahrs);
    g_yaw = 0.0f;
}
