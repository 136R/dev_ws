#ifndef APP_COMM_PROTOCOL_H
#define APP_COMM_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Frame format ───────────────────────────────────────────────────────
 *  [0xAA] [0x55] [TYPE] [LEN] [DATA × LEN] [XOR]
 *  XOR = TYPE ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]
 *  Byte order: little-endian
 * ──────────────────────────────────────────────────────────────────── */

#define COMM_HEADER_1           0xAAu
#define COMM_HEADER_2           0x55u

/* Message types */
#define COMM_TYPE_VEL_CMD       0x01u   /* ROS2 → STM32: velocity command    */
#define COMM_TYPE_FEEDBACK      0x02u   /* STM32 → ROS2: raw sensor feedback */
#define COMM_TYPE_BATTERY       0x03u   /* STM32 → ROS2: battery, 2 Hz       */

/* Payload version carried in the 0x03 frame.  Bump it whenever the meaning
 * of any STM32→ROS2 payload changes.  git tracks the source; this tracks the
 * binary actually flashed into the board, which is what the ROS side checks
 * so that "edited ROS, forgot to reflash" fails loudly instead of silently. */
#define COMM_PROTO_VER          1u

/* Payload sizes
 * TYPE 0x02 layout: left_delta(4) + right_delta(4) + acc_xyz(12) + gyro_xyz(12) + yaw_mdeg(4) = 36 bytes
 * mag_xyz removed — ICM-42688-P is 6-axis (no magnetometer)
 * TYPE 0x03 layout: voltage_mv(u16) + current_ma(i16) + flags(u8) + proto_ver(u8) + reserved(u16) = 8 bytes
 *   flags: bit0 voltage_valid, bit1 current_valid, bit2 vrefint_ok
 *   current_ma is always 0 in v1 — the field is reserved so that adding a
 *   coulomb counter later needs no protocol change */
#define COMM_VEL_CMD_LEN        4u      /* 2 × int16 (left, right mrad/s)    */
#define COMM_FEEDBACK_LEN       36u     /* 2×int32 delta + 6×int32 IMU + 1×int32 yaw */
#define COMM_BATTERY_LEN        8u

/* Total frame sizes (header 2 + type 1 + len 1 + payload + xor 1) */
#define COMM_VEL_FRAME_SIZE      (5u + COMM_VEL_CMD_LEN)      /*  9 bytes */
#define COMM_FEEDBACK_FRAME_SIZE (5u + COMM_FEEDBACK_LEN)     /* 41 bytes */
#define COMM_BATTERY_FRAME_SIZE  (5u + COMM_BATTERY_LEN)      /* 13 bytes */

/* Battery frame cadence: every 50 ticks of the 100 Hz loop = 2 Hz */
#define COMM_BATTERY_PERIOD_TICKS 50u

/* Safety: stop motors if no valid command received within 500 ms
 * (called at 100 Hz → 50 ticks = 500 ms)                          */
#define COMM_WATCHDOG_TICKS     50u

/* RX DMA buffer — large enough to hold multiple frames */
#define COMM_RX_BUF_SIZE        32u

/* ── API ────────────────────────────────────────────────────────────── */

/* Call once during system init (after MX_USART1_UART_Init) */
void comm_protocol_init(void);

/* Call from motor_controller_update() (TIM6 ISR, every 10 ms).
 * Runs the watchdog every tick and sends the 0x02 feedback frame at 100 Hz;
 * every 50th tick the 0x03 battery frame is appended to the same DMA burst. */
void comm_protocol_tick(int32_t left_delta, int32_t right_delta);

/* Retrieve the latest commanded wheel velocities [rad/s] */
float comm_protocol_get_left_rad_s(void);
float comm_protocol_get_right_rad_s(void);
void comm_protocol_uart_rx_event_handler(uint16_t size);
void comm_protocol_uart_error_handler(void);

#endif /* APP_COMM_PROTOCOL_H */
