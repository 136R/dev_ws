#include "app/pid.h"
#include <math.h>

void pid_init(pid_controller_t *pid,
              float kV, float kp, float ki,
              float output_limit)
{
    pid->kV           = kV;
    pid->kp           = kp;
    pid->ki           = ki;
    pid->last_error   = 0.0f;
    pid->last_output  = 0.0f;
    pid->output_limit = output_limit;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki)
{
    pid->kp = kp;
    pid->ki = ki;
}

void pid_set_feedforward(pid_controller_t *pid, float kV)
{
    pid->kV = kV;
}

float pid_compute(pid_controller_t *pid, float target, float actual)
{
    /* target near zero: reset state and coast */
    if (fabsf(target) < 0.5f) {
        // pid->last_error  = 0.0f;
        // pid->last_error  = target - actual;
        pid->last_output = 0.0f;
        return 0.0f;
    }

    /* ── feedforward ── */
    float ff = pid->kV * target;

    /* ── incremental PI ──
     * Δu = kp*(e[k] - e[k-1]) + ki*e[k]
     * u_pi[k] = u_pi[k-1] + Δu
     */
    float error   = target - actual;
    float delta_u = pid->kp * (error - pid->last_error) + pid->ki * error;
    float u_pi    = pid->last_output + delta_u;

    /* ── total output with back-calculation anti-windup ──
     * When ff + u_pi saturates, back-calculate u_pi to the value that
     * exactly reaches the limit.  Storing this prevents the integrator
     * from accumulating beyond the saturation boundary even when FF
     * alone is large enough to push the output into the rail.
     */
    float output = ff + u_pi;
    if (output >  pid->output_limit) {
        output = pid->output_limit;
        u_pi   = output - ff;          /* back-calculate */
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        u_pi   = output - ff;          /* back-calculate */
    }

    pid->last_error  = error;
    pid->last_output = u_pi;
    /* 独立 clamp u_pi，防止减速时无界负向累积 */
    // float u_pi_max = pid->output_limit - ff;
    // float u_pi_min = -pid->output_limit - ff;
    // pid->last_output = fmaxf(u_pi_min, fminf(u_pi_max, u_pi));

    return output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->last_error  = 0.0f;
    pid->last_output = 0.0f;
}
