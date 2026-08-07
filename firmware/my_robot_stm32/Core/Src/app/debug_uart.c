#include "app/debug_uart.h"
#include "app/motor_controller.h"
#include "app/robot_config.h"
#include "app/imu.h"
#include "usart.h"
#include "stm32f3xx_hal.h"
#include <math.h>
#include <stdio.h>

extern UART_HandleTypeDef huart4;
extern volatile float ff_kv_left;
extern volatile float ff_kv_right;

static uint32_t last_tick;

static float debug_ff_pwm(float target_rad_s, float kV)
{
    if (fabsf(target_rad_s) < 0.01f) return 0.0f;
    return kV * (target_rad_s * RAD_S_TO_COUNTS);
}

void debug_uart_init(void)
{
    last_tick = HAL_GetTick();
}

void debug_output(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 10) return;   /* ~50 Hz */
    last_tick = now;

    float tgt_L, tgt_R, act_L, act_R, pwm_L, pwm_R;
    motor_controller_get_target(&tgt_L, &tgt_R);
    motor_controller_get_actual(&act_L, &act_R);
    motor_controller_get_pwm(&pwm_L, &pwm_R);

    float tgt_pwm_L = debug_ff_pwm(tgt_L, ff_kv_left);
    float tgt_pwm_R = debug_ff_pwm(tgt_R, ff_kv_right);

    imu_data_t imu;
    imu_peek_data(&imu);

    /* VOFA+ FireWater format: ch0,ch1,...,chN\n
     * ch0-3: left motor  (tgt, act, ff_pwm, pid_pwm)
     * ch4-7: right motor (tgt, act, ff_pwm, pid_pwm)
     * ch8:   imu yaw (deg)
     * ch9:   imu gyro_z (rad/s)                       */
    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "%.2f,%.2f,%.0f,%.0f,%.2f,%.2f,%.0f,%.0f,%.2f,%.4f\n",
        tgt_L, act_L, tgt_pwm_L, pwm_L,
        tgt_R, act_R, tgt_pwm_R, pwm_R,
        imu.yaw, imu.gyro[2]);

    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 5);
}
