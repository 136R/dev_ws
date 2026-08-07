#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* ── Physical IMU data (SI units) ───────────────────────────────────── */
typedef struct {
    float accel[3];   /* m/s²   — linear_acceleration for sensor_msgs/Imu */
    float gyro[3];    /* rad/s  — angular_velocity    for sensor_msgs/Imu */
    float mag[3];     /* uT     — magnetic field      for sensor_msgs/MagneticField */
    float roll;       /* degrees */
    float pitch;      /* degrees */
    float yaw;        /* degrees */
    float temp;       /* °C */
    float quat[4];    /* w, x, y, z — orientation for sensor_msgs/Imu */
} imu_data_t;

/* Live Expressions / host-fusion debug variables */
extern volatile float g_acc[3];           /* m/s² */
extern volatile float g_gyro[3];          /* rad/s, bias-corrected */
extern volatile float g_mag[3];           /* uT */
extern volatile float g_gyro_bias_rad_s[3]; /* rad/s, estimated bias */
extern volatile float g_yaw;              /* degrees, Fusion AHRS Euler yaw */

/* ── API ────────────────────────────────────────────────────────────── */
void imu_init(void);
void imu_calibrate_gyro_bias(void);       /* call once after imu_init(), robot must be stationary (~3 s) */
bool imu_data_ready(void);                /* true when INT1 data-ready has fired */
void imu_get_data(imu_data_t *out);       /* SPI read + scale + clear ready flag */
void imu_peek_data(imu_data_t *out);      /* copy last data without clearing the ready flag */
void imu_int1_irq_handler(void);          /* call from EXTI12 ISR (PB12 rising edge) */

#endif /* APP_IMU_H */
