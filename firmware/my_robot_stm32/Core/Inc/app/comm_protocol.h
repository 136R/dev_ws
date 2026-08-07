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

/* Payload sizes
 * TYPE 0x02 layout: left_delta(4) + right_delta(4) + acc_xyz(12) + gyro_xyz(12) + yaw_mdeg(4) = 36 bytes
 * mag_xyz removed — ICM-42688-P is 6-axis (no magnetometer) */
#define COMM_VEL_CMD_LEN        4u      /* 2 × int16 (left, right mrad/s)    */
#define COMM_FEEDBACK_LEN       36u     /* 2×int32 delta + 6×int32 IMU + 1×int32 yaw */

/* Total frame sizes (header 2 + type 1 + len 1 + payload + xor 1) */
#define COMM_VEL_FRAME_SIZE      (5u + COMM_VEL_CMD_LEN)      /*  9 bytes */
#define COMM_FEEDBACK_FRAME_SIZE (5u + COMM_FEEDBACK_LEN)     /* 37 bytes */

/* Safety: stop motors if no valid command received within 500 ms
 * (called at 100 Hz → 50 ticks = 500 ms)                          */
#define COMM_WATCHDOG_TICKS     50u

/* RX DMA buffer — large enough to hold multiple frames */
#define COMM_RX_BUF_SIZE        32u

/* ── API ────────────────────────────────────────────────────────────── */

/* Call once during system init (after MX_USART1_UART_Init) */
void comm_protocol_init(void);

/* Call from motor_controller_update() (TIM6 ISR, every 10 ms).
 * Runs the watchdog every tick and sends the 0x02 feedback frame at 50 Hz. */
void comm_protocol_tick(int32_t left_delta, int32_t right_delta);

/* Retrieve the latest commanded wheel velocities [rad/s] */
float comm_protocol_get_left_rad_s(void);
float comm_protocol_get_right_rad_s(void);
void comm_protocol_uart_rx_event_handler(uint16_t size);
void comm_protocol_uart_error_handler(void);

#endif /* APP_COMM_PROTOCOL_H */
