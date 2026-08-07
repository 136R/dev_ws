#ifndef PID_H
#define PID_H

typedef struct {
    /* feedforward */
    float kV;           /* velocity feedforward (PWM / counts_per_period) */

    /* incremental PI gains */
    float kp;           /* proportional gain (PWM / count) */
    float ki;           /* integral gain, incremental form (PWM / count) */

    /* incremental PI state */
    float last_error;   /* e[k-1] */
    float last_output;  /* u_pi[k-1], PI contribution only (no FF) */

    /* limits */
    float output_limit; /* clamp on total output (PWM) */
} pid_controller_t;

void  pid_init(pid_controller_t *pid,
               float kV, float kp, float ki,
               float output_limit);

void  pid_set_gains(pid_controller_t *pid, float kp, float ki);
void  pid_set_feedforward(pid_controller_t *pid, float kV);

/* target and actual in counts/period */
float pid_compute(pid_controller_t *pid,
                  float target_counts, float actual_counts);

void  pid_reset(pid_controller_t *pid);

#endif /* PID_H */
