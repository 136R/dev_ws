#include "app/motor_driver.h"
#include "app/robot_config.h"
#include "main.h"
#include "tim.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;

/* ── Pin mapping ─────────────────────────────────────────────── */
/* LEFT  = TIM2_CH3 (PA2),  direction = PC2 (MOTOR1_AN1)        */
/* RIGHT = TIM2_CH4 (PA3),  direction = PC3 (MOTOR2_AN1)        */

static const uint32_t pwm_channel[] = {TIM_CHANNEL_4, TIM_CHANNEL_3};

static GPIO_TypeDef *const dir_port[] = {MOTOR2_AN1_GPIO_Port, MOTOR1_AN1_GPIO_Port};
static const uint16_t      dir_pin[]  = {MOTOR2_AN1_Pin,       MOTOR1_AN1_Pin};

static const uint8_t dir_invert[] = {MOTOR_LEFT_DIR_INVERT, MOTOR_RIGHT_DIR_INVERT};

void motor_driver_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    motor_brake_all();
}

void motor_set_pwm(uint8_t id, int16_t pwm)
{
    if (id > MOTOR_RIGHT) return;

    /* clamp */
    if (pwm > PWM_PERIOD)  pwm = PWM_PERIOD;
    if (pwm < -PWM_PERIOD) pwm = -PWM_PERIOD;

    /* direction logic (apply inversion) */
    uint8_t forward = (pwm >= 0) ? 1 : 0;
    if (dir_invert[id]) forward = !forward;

    HAL_GPIO_WritePin(dir_port[id], dir_pin[id],
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* OCPolarity = LOW  =>  compare = PWM_PERIOD - duty
       duty = abs(pwm), so compare = PWM_PERIOD - abs(pwm)       */
    uint16_t compare = (uint16_t)(PWM_PERIOD - abs(pwm));
    __HAL_TIM_SET_COMPARE(&htim2, pwm_channel[id], compare);
}

void motor_brake_all(void)
{
    /* compare = PWM_PERIOD => 0% duty */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_PERIOD);
}
