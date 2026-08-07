#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#define CAL_KV_NUM_STEPS  5   /* number of PWM test points */

typedef struct {
    float    kV;                               /* fitted slope (PWM / counts_per_period) */
    uint8_t  valid_points;                     /* number of points used in average */
    float    kV_per_step[CAL_KV_NUM_STEPS];    /* per-step kV */
    float    speed_per_step[CAL_KV_NUM_STEPS]; /* measured steady-state (counts/period) */
} cal_result_t;

/* set to 1 via Live Expressions to trigger calibration */
extern volatile uint8_t cal_trigger;

/* results: [MOTOR_LEFT] / [MOTOR_RIGHT] — forward direction only */
extern volatile cal_result_t cal_results[2];

/* progress: 0..1 = current group, 2 = done */
extern volatile uint8_t cal_progress;

void    calibration_tick(void);           /* call from main loop */
uint8_t calibration_is_running(void);
void    calibration_print_results(void);  /* print via USART3 */

#endif /* CALIBRATION_H */
