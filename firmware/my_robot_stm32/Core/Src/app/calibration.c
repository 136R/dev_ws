#include "app/calibration.h"
#include "app/motor_controller.h"
#include "app/robot_config.h"
#include "usart.h"
#include "stm32f3xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ── tuning knobs ── */
#define KV_SETTLE_TICKS     50      /* 500 ms settle before measuring */
#define KV_MEASURE_TICKS    100     /* 1000 ms measurement window */
#define KV_COAST_TICKS      50      /* 500 ms coast between steps */
#define GROUP_COAST_TICKS   100     /* 1000 ms coast between groups */
#define KV_SPEED_THRESH     0.5f    /* counts/period — skip step if below */

/* PWM test points centred on the 0.2 m/s operating speed (≈ 620 PWM).
 * Spread: two below, one at, two above the operating point. */
static const int16_t kv_pwm_table[CAL_KV_NUM_STEPS] =
    {350, 480, 620, 760, 900};

/* ── state machine ── */
typedef enum {
    CS_IDLE,
    CS_KV_SETTLE,
    CS_KV_MEASURE,
    CS_KV_COAST,
    CS_GROUP_COAST,
    CS_DONE
} cal_state_t;

/* ── public variables (Live Expressions) ── */
volatile uint8_t      cal_trigger;
volatile cal_result_t cal_results[2];   /* [MOTOR_LEFT], [MOTOR_RIGHT] */
volatile uint8_t      cal_progress;

/* ── private state ── */
static cal_state_t state;
static uint8_t     running;
static uint8_t     cur_motor;           /* MOTOR_LEFT / MOTOR_RIGHT */
static uint8_t     group_idx;           /* 0 = left, 1 = right */
static uint32_t    tick_cnt;
static uint32_t    last_ms;

/* kV step state */
static uint8_t  kv_step_idx;
static float    speed_accum;            /* accumulates rad/s samples */
static uint32_t sample_cnt;
static float    kv_sum_ps;             /* Σ(PWM · speed)  for least-squares */
static float    kv_sum_ss;             /* Σ(speed²)       for least-squares */
static uint8_t  kv_valid_cnt;

extern UART_HandleTypeDef huart4;

/* ── helpers ── */

static float read_speed_rad_s(void)
{
    float L, R;
    motor_controller_get_actual(&L, &R);
    return (cur_motor == MOTOR_LEFT) ? fabsf(L) : fabsf(R);
}

static void apply_pwm(int16_t pwm)
{
    if (cur_motor == MOTOR_LEFT)
        motor_controller_set_raw_pwm(pwm, 0);
    else
        motor_controller_set_raw_pwm(0, pwm);
}

/* ── group lifecycle ── */

static void start_group(void)
{
    cur_motor = group_idx;          /* MOTOR_LEFT=0, MOTOR_RIGHT=1 */
    cal_progress = group_idx;

    kv_step_idx  = 0;
    kv_sum_ps    = 0.0f;
    kv_sum_ss    = 0.0f;
    kv_valid_cnt = 0;
    tick_cnt     = 0;

    apply_pwm(kv_pwm_table[0]);
    state = CS_KV_SETTLE;
}

static void finish_group(void)
{
    /* least-squares through origin: kV = Σ(PWM·speed) / Σ(speed²) */
    if (kv_sum_ss > 0.0f)
        cal_results[cur_motor].kV = kv_sum_ps / kv_sum_ss;

    cal_results[cur_motor].valid_points = kv_valid_cnt;

    apply_pwm(0);
    tick_cnt = 0;
    state = CS_GROUP_COAST;
}

/* ── start / tick / query ── */

static void calibration_start(void)
{
    motor_controller_set_openloop(1);
    motor_controller_set_raw_pwm(0, 0);

    for (int m = 0; m < 2; m++) {
        cal_results[m].kV           = 0.0f;
        cal_results[m].valid_points = 0;
        for (int s = 0; s < CAL_KV_NUM_STEPS; s++) {
            cal_results[m].kV_per_step[s]    = 0.0f;
            cal_results[m].speed_per_step[s] = 0.0f;
        }
    }

    group_idx = 0;
    running   = 1;
    last_ms   = HAL_GetTick();
    start_group();
}

void calibration_tick(void)
{
    if (!running && cal_trigger) {
        cal_trigger = 0;
        calibration_start();
    }
    if (!running) return;

    uint32_t now = HAL_GetTick();
    if (now - last_ms < 10) return;
    last_ms = now;

    switch (state) {

    /* ── wait for steady state at current PWM step ── */
    case CS_KV_SETTLE:
        tick_cnt++;
        if (tick_cnt >= KV_SETTLE_TICKS) {
            tick_cnt    = 0;
            speed_accum = 0.0f;
            sample_cnt  = 0;
            state = CS_KV_MEASURE;
        }
        break;

    /* ── accumulate speed samples, compute kV ── */
    case CS_KV_MEASURE:
        tick_cnt++;
        speed_accum += read_speed_rad_s();
        sample_cnt++;

        if (tick_cnt >= KV_MEASURE_TICKS) {
            /* convert average rad/s → counts/period */
            float avg_counts = (speed_accum / (float)sample_cnt) * RAD_S_TO_COUNTS;
            int   idx        = kv_step_idx;

            cal_results[cur_motor].speed_per_step[idx] = avg_counts;

            if (avg_counts > KV_SPEED_THRESH) {
                float pwm_f = (float)kv_pwm_table[idx];
                cal_results[cur_motor].kV_per_step[idx] = pwm_f / avg_counts;
                kv_sum_ps += pwm_f * avg_counts;
                kv_sum_ss += avg_counts * avg_counts;
                kv_valid_cnt++;
            }

            kv_step_idx++;
            if (kv_step_idx >= CAL_KV_NUM_STEPS) {
                finish_group();
            } else {
                apply_pwm(0);
                tick_cnt = 0;
                state = CS_KV_COAST;
            }
        }
        break;

    /* ── coast between kV steps ── */
    case CS_KV_COAST:
        tick_cnt++;
        if (tick_cnt >= KV_COAST_TICKS) {
            tick_cnt = 0;
            apply_pwm(kv_pwm_table[kv_step_idx]);
            state = CS_KV_SETTLE;
        }
        break;

    /* ── coast between left/right groups ── */
    case CS_GROUP_COAST:
        tick_cnt++;
        if (tick_cnt >= GROUP_COAST_TICKS) {
            group_idx++;
            if (group_idx >= 2) {
                state = CS_DONE;
                running = 0;
                cal_progress = 2;
                motor_controller_set_openloop(0);
                calibration_print_results();
            } else {
                start_group();
            }
        }
        break;

    default:
        break;
    }
}

uint8_t calibration_is_running(void)
{
    return running;
}

void calibration_print_results(void)
{
    char buf[128];
    int  len;

    len = snprintf(buf, sizeof(buf), "\n=== Calibration Results ===\n");
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 20);

    static const char *names[] = {"LEFT", "RIGHT"};
    for (int m = 0; m < 2; m++) {
        volatile cal_result_t *r = &cal_results[m];
        len = snprintf(buf, sizeof(buf),
                       "%-5s  kV=%.4f  valid=%d\n",
                       names[m], r->kV, r->valid_points);
        HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 20);
    }

    /* copy-paste ready config */
    len = snprintf(buf, sizeof(buf), "\n--- robot_config.h ---\n");
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 20);

    len = snprintf(buf, sizeof(buf),
        "#define FF_KV_LEFT   %.4ff\n"
        "#define FF_KV_RIGHT  %.4ff\n",
        cal_results[MOTOR_LEFT].kV,
        cal_results[MOTOR_RIGHT].kV);
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, 20);
}
