#include "app/motor_controller.h"
#include "app/robot_config.h"
#include "app/motor_driver.h"
#include "app/encoder.h"
#include "app/pid.h"
#include "app/comm_protocol.h"

/* ── per-wheel PID instances ── */
static pid_controller_t pid_left;
static pid_controller_t pid_right;

/* ── shared state (volatile: written in ISR, read in main) ── */
static volatile float target_left_rad_s;
static volatile float target_right_rad_s;

/* ── open-loop calibration mode ── */
static volatile uint8_t openloop_mode;
static volatile int16_t raw_pwm_left;
static volatile int16_t raw_pwm_right;

/* observable via Live Expressions */
volatile float actual_left_rad_s;
volatile float actual_right_rad_s;
volatile float pwm_left;
volatile float pwm_right;
volatile float ff_kv_left;
volatile float ff_kv_right;
volatile float pid_kp_left;
volatile float pid_ki_left;
volatile float pid_kp_right;
volatile float pid_ki_right;

int32_t delta_L;
int32_t delta_R;

void motor_controller_init(void)
{
    encoder_init();
    motor_driver_init();

    pid_init(&pid_left,  FF_KV_LEFT,  PI_KP_LEFT,  PI_KI_LEFT,  PI_OUTPUT_LIMIT);
    pid_init(&pid_right, FF_KV_RIGHT, PI_KP_RIGHT, PI_KI_RIGHT, PI_OUTPUT_LIMIT);

    target_left_rad_s  = 0.0f;
    target_right_rad_s = 0.0f;
    ff_kv_left   = FF_KV_LEFT;
    ff_kv_right  = FF_KV_RIGHT;
    pid_kp_left  = PI_KP_LEFT;
    pid_ki_left  = PI_KI_LEFT;
    pid_kp_right = PI_KP_RIGHT;
    pid_ki_right = PI_KI_RIGHT;
}

void motor_controller_update(void)
{
    /* 1. read encoder deltas (counts/period) */
    delta_L = encoder_get_delta(MOTOR_LEFT);
    delta_R = encoder_get_delta(MOTOR_RIGHT);

    /* 2. compute rad/s for observability and get_actual() */
    actual_left_rad_s  = (float)delta_L * COUNTS_TO_RAD_PER_SEC;
    actual_right_rad_s = (float)delta_R * COUNTS_TO_RAD_PER_SEC;

    if (openloop_mode) {
        pwm_left  = raw_pwm_left;
        pwm_right = raw_pwm_right;
    } else {
        pid_set_feedforward(&pid_left,  ff_kv_left);
        pid_set_feedforward(&pid_right, ff_kv_right);
        pid_set_gains(&pid_left,  pid_kp_left,  pid_ki_left);
        pid_set_gains(&pid_right, pid_kp_right, pid_ki_right);

        /* 3. convert targets to counts/period, run PID */
        float tgt_L = target_left_rad_s  * RAD_S_TO_COUNTS;
        float tgt_R = target_right_rad_s * RAD_S_TO_COUNTS;
        pwm_left  = pid_compute(&pid_left,  tgt_L, (float)delta_L);
        pwm_right = pid_compute(&pid_right, tgt_R, (float)delta_R);
    }

    /* 4. apply PWM */
    motor_set_pwm(MOTOR_LEFT,  (int16_t)pwm_left);
    motor_set_pwm(MOTOR_RIGHT, (int16_t)pwm_right);

    /* 5. send feedback frame and run watchdog (delta_L/R already computed above) */
    comm_protocol_tick(delta_L, delta_R);
}

void motor_controller_set_target(float left_rad_s, float right_rad_s)
{
    target_left_rad_s  = left_rad_s;
    target_right_rad_s = right_rad_s;
}

void motor_controller_get_actual(float *left_rad_s, float *right_rad_s)
{
    if (left_rad_s)  *left_rad_s  = actual_left_rad_s;
    if (right_rad_s) *right_rad_s = actual_right_rad_s;
}

void motor_controller_get_target(float *left_rad_s, float *right_rad_s)
{
    if (left_rad_s)  *left_rad_s  = target_left_rad_s;
    if (right_rad_s) *right_rad_s = target_right_rad_s;
}

void motor_controller_get_pwm(float *left_pwm, float *right_pwm)
{
    if (left_pwm)  *left_pwm  = pwm_left;
    if (right_pwm) *right_pwm = pwm_right;
}

void motor_controller_set_pi_gains(float kp_left, float ki_left,
                                   float kp_right, float ki_right)
{
    pid_kp_left  = kp_left;
    pid_ki_left  = ki_left;
    pid_kp_right = kp_right;
    pid_ki_right = ki_right;
}

void motor_controller_get_pi_gains(float *kp_left, float *ki_left,
                                   float *kp_right, float *ki_right)
{
    if (kp_left)  *kp_left  = pid_kp_left;
    if (ki_left)  *ki_left  = pid_ki_left;
    if (kp_right) *kp_right = pid_kp_right;
    if (ki_right) *ki_right = pid_ki_right;
}

void motor_controller_stop(void)
{
    target_left_rad_s  = 0.0f;
    target_right_rad_s = 0.0f;
    pid_reset(&pid_left);
    pid_reset(&pid_right);
    motor_brake_all();
}

void motor_controller_set_openloop(uint8_t enable)
{
    openloop_mode = enable;
    if (enable) {
        raw_pwm_left  = 0;
        raw_pwm_right = 0;
        pid_reset(&pid_left);
        pid_reset(&pid_right);
    }
}

void motor_controller_set_raw_pwm(int16_t left_pwm, int16_t right_pwm)
{
    raw_pwm_left  = left_pwm;
    raw_pwm_right = right_pwm;
}

void motor_controller_set_velocity_debug(float linear_x, float angular_z)
{
    float left  = (linear_x - angular_z * WHEEL_TREAD / 2.0f) / WHEEL_RADIUS;
    float right = (linear_x + angular_z * WHEEL_TREAD / 2.0f) / WHEEL_RADIUS;
    motor_controller_set_target(left, right);
}
