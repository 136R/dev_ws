#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "app/zupt.h"

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

/* g_imu_health 的位。置位后不再自清，供上电后一次性诊断 */
#define IMU_HEALTH_WHOAMI_BAD    (1u << 0)  /* SPI 通了但 WHO_AM_I 不对 */
#define IMU_HEALTH_CAL_TIMEOUT   (1u << 1)  /* 等不到 DRDY：零偏置 0 后照常返回，别让电机/串口陪葬 */
#define IMU_HEALTH_CAL_UNSTABLE  (1u << 2)  /* 标定期间被碰，重试用尽 */
#define IMU_HEALTH_AAF_BAD       (1u << 3)  /* AAF 寄存器回读与写入值不符 */

/* Live Expressions / host-fusion debug variables */
extern volatile float g_acc[3];           /* m/s² */
extern volatile float g_gyro[3];          /* rad/s, bias-corrected */
extern volatile float g_mag[3];           /* uT */
extern volatile float g_gyro_bias_rad_s[3]; /* rad/s, estimated bias */
extern volatile float g_yaw;              /* degrees, Fusion AHRS Euler yaw */
extern volatile uint32_t g_imu_health;    /* IMU_HEALTH_* 位或 */
extern volatile float    g_imu_dt_s;      /* s, DWT 实测的帧间隔 */
extern volatile bool     g_dwt_ok;        /* false 时 dt 退回标称值 */
extern volatile uint32_t g_dt_clamp_count;/* dt 越界被兜底的次数，开机后应只有首帧那 1 次 */
extern volatile uint32_t g_isr_max_us;    /* icm_read_all() 单次最长耗时 */

/* 非 static：验收要用 CubeIDE Live Expressions 直接看 state / gyro_bias / gyro_var */
extern zupt_t g_zupt;

/* ── API ────────────────────────────────────────────────────────────── */
void imu_init(void);
void imu_calibrate_gyro_bias(void);       /* call once after imu_init(), robot must be stationary (~2 s) */
bool imu_data_ready(void);                /* true when INT1 data-ready has fired */
void imu_get_data(imu_data_t *out);       /* SPI read + scale + clear ready flag */
void imu_peek_data(imu_data_t *out);      /* copy last data without clearing the ready flag */
void imu_int1_irq_handler(void);          /* call from EXTI12 ISR (PB12 rising edge) */

#endif /* APP_IMU_H */
