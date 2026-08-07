#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Encoder & Gear ────────────────────────────────────────────── */
#define ENCODER_PPR             500
#define ENCODER_CPR             2000       /* quad decode (PPR x 4) */
#define GEAR_RATIO              34.0f
#define COUNTS_PER_WHEEL_REV    68000.0f   /* ENCODER_CPR x GEAR_RATIO */

/* ── Chassis ───────────────────────────────────────────────────── */
#define WHEEL_DIAMETER          0.065f     /* m */
#define WHEEL_RADIUS            0.0325f    /* m */
#define WHEEL_TREAD             0.171f     /* m  (center-to-center) */

/* ── Control Loop ──────────────────────────────────────────────── */
#define CONTROL_PERIOD_MS       10
#define CONTROL_FREQ_HZ         100
#define CONTROL_DT              0.01f      /* seconds */

/* ── Unit Conversion ───────────────────────────────────────────── */
/* delta_count/period -> rad/s */
#define COUNTS_TO_RAD_PER_SEC   (2.0f * (float)M_PI / COUNTS_PER_WHEEL_REV / CONTROL_DT)
/* rad/s -> counts/period  (use for converting target before PID) */
#define RAD_S_TO_COUNTS         (1.0f / COUNTS_TO_RAD_PER_SEC)

/* ── Output Shaft (post-gearbox) Speed ─────────────────────────── */
#define WHEEL_NO_LOAD_RAD_S     (300.0f * 2.0f * (float)M_PI / 60.0f)  /* ~31.4 rad/s */
#define WHEEL_MAX_RAD_S         (260.0f * 2.0f * (float)M_PI / 60.0f)  /* ~27.2 rad/s */

/* ── PWM ───────────────────────────────────────────────────────── */
#define PWM_PERIOD              3600       /* TIM2 ARR + 1 */

/* ── Feedforward ───────────────────────────────────────────────── */
/* kV unit: PWM / (counts/period)                                  */
/* Derived from old kV [PWM·s/rad] * COUNTS_TO_RAD_PER_SEC        */
/* Left:  99.5 * (2π/68000/0.01) ≈ 0.919                          */
/* Right: 101.0 * (2π/68000/0.01) ≈ 0.933                         */
#define FF_KV_LEFT              0.0f
#define FF_KV_RIGHT             0.0f

/* ── PI Gains ──────────────────────────────────────────────────── */
/* Incremental PI: Δu = kp*(e[k]-e[k-1]) + ki*e[k]                */
/* Units: PWM / (counts/period)                                    */
/* kp converted from old [PWM·s/rad]: 6*COUNTS_TO_RAD≈0.055 left  */
/* ki: start small, tune up to eliminate steady-state error        */
// 纯增量左：p：0.7，i：0.4，缺点为0可能会超调
// 加了前馈的纯增量问题：从0到12.3稳定，但是从12.3到6.15不稳定，加了ks感觉效果一般
#define PI_KP_LEFT              0.54f
#define PI_KI_LEFT              0.44f
#define PI_KP_RIGHT             0.56f
#define PI_KI_RIGHT             0.44f
#define PI_OUTPUT_LIMIT         3600.0f    /* total output clamp (PWM) */

/* ── Direction Inversion (set 1 to invert, confirm on real car) ── */
#define MOTOR_LEFT_DIR_INVERT   0
#define MOTOR_RIGHT_DIR_INVERT  1
#define ENC_LEFT_DIR_INVERT     1
#define ENC_RIGHT_DIR_INVERT    1

/* ── Odometry / PID Low-Pass Filter ────────────────────────────── */
/* Applied to encoder delta before PID feedback only.               */
/* α=0 → no filter, α→1 → frozen.  τ = α/(1-α) × CONTROL_DT       */
/* α=0.2 → τ≈2.5 ms at 100 Hz; increase α if you need more smoothing */
#define ODOM_LPF_ALPHA_LEFT     0.0f
#define ODOM_LPF_ALPHA_RIGHT    0.0f

/* ── Motor ID ──────────────────────────────────────────────────── */
#define MOTOR_LEFT              0
#define MOTOR_RIGHT             1

#endif /* ROBOT_CONFIG_H */
