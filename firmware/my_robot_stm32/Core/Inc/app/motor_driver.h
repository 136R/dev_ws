#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

void motor_driver_init(void);
void motor_set_pwm(uint8_t id, int16_t pwm);   /* -PWM_PERIOD ~ +PWM_PERIOD */
void motor_brake_all(void);

#endif /* MOTOR_DRIVER_H */
