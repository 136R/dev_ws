#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

void  motor_controller_init(void);
void  motor_controller_update(void);           /* call from TIM6 ISR */
void  motor_controller_set_target(float left_rad_s, float right_rad_s);
void  motor_controller_get_actual(float *left_rad_s, float *right_rad_s);
void  motor_controller_get_target(float *left_rad_s, float *right_rad_s);
void  motor_controller_get_pwm(float *left_pwm, float *right_pwm);
void  motor_controller_set_pi_gains(float kp_left, float ki_left,
                                    float kp_right, float ki_right);
void  motor_controller_get_pi_gains(float *kp_left, float *ki_left,
                                    float *kp_right, float *ki_right);
void  motor_controller_stop(void);

/* open-loop calibration mode: bypass PID, set raw PWM directly */
void  motor_controller_set_openloop(uint8_t enable);
void  motor_controller_set_raw_pwm(int16_t left_pwm, int16_t right_pwm);

/* debug-only: linear_x (m/s) + angular_z (rad/s) → wheel targets */
void  motor_controller_set_velocity_debug(float linear_x, float angular_z);

#endif /* MOTOR_CONTROLLER_H */
