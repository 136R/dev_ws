/*
 * zupt.h — Zero-velocity Update / 静止检测 + 陀螺零偏在线更新
 *
 * 用途：
 *   六轴 IMU 没有 yaw 的绝对观测，长时间稳定性完全依赖陀螺零偏的精度。
 *   本模块通过滑动窗口的均值/方差检测载体是否处于静止状态，仅在静止时
 *   用一阶低通融合更新陀螺零偏。配合 zupt_apply() 在静止期间强制陀螺
 *   输出为零，从根本上消除 yaw 噪声积分。
 *
 * 三态机：
 *   MOVING   ─静止判据成立─►  SETTLING
 *   SETTLING ─判据持续成立 settle_samples 帧─►  STATIC
 *            ─判据失效─►  MOVING
 *   STATIC   ─检测到瞬时运动─►  MOVING
 *
 * 调用约定：
 *   - zupt_update()  传入"未减零偏"的原始陀螺（rad/s）和加速度（m/s²）
 *   - zupt_apply()   把原始陀螺转成"已减零偏 + 静止时强制为 0"的最终输出
 *   - 两者通常都在同一采样周期内、同一线程上下文中调用
 *
 * 使用示例（在 icm_read_all() 内）：
 *     float gyro_raw[3] = { gx*GYRO_SCALE, gy*GYRO_SCALE, gz*GYRO_SCALE };
 *     float acc_raw[3]  = { ax*ACCEL_SCALE, ay*ACCEL_SCALE, az*ACCEL_SCALE };
 *     zupt_update(&g_zupt, gyro_raw, acc_raw);
 *     zupt_apply(&g_zupt, gyro_raw, g_imu.gyro);   // 写入最终陀螺
 *     // ...accel 直接复制，AHRS 用 g_imu.gyro / g_imu.accel
 */

#ifndef APP_ZUPT_H_
#define APP_ZUPT_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 滑动窗口长度（编译期常量）─────────────────────────────────────────
 * 默认 50 帧。@ 100 Hz 即 0.5 s 的统计窗口。
 * 若 ODR 提到 1 kHz，建议在编译参数里 -DZUPT_WINDOW_SIZE=500
 */
#ifndef ZUPT_WINDOW_SIZE
#define ZUPT_WINDOW_SIZE  50u
#endif

typedef enum {
    ZUPT_MOVING   = 0,   /* 运动中：陀螺减零偏后输出 */
    ZUPT_SETTLING = 1,   /* 疑似静止：判据已成立但未确认 */
    ZUPT_STATIC   = 2,   /* 已确认静止：陀螺强制为 0，零偏在线更新 */
} zupt_state_t;

typedef struct {
    /* ── 静止判据（严格阈值，进入 STATIC 必须全部满足）────────────── */
    float gyro_mean_thresh;   /* rad/s，|窗口均值 − 当前零偏| 上限      */
    float gyro_var_thresh;    /* (rad/s)²，每轴方差上限                 */
    float accel_var_thresh;   /* (m/s²)²，三轴方差之和上限              */

    /* ── 退出 STATIC 的瞬时阈值（宽松，任一超出即退出，构成滞回）── */
    float gyro_exit_thresh;   /* rad/s，|gyro_raw − bias| 任意轴超过即退出 */
    float accel_exit_thresh;  /* (m/s²)²，accel_var_sum 超过即退出       */

    /* ── 状态机参数 ──────────────────────────────────────────────── */
    uint32_t settle_samples;  /* SETTLING 中持续静止多少帧后进入 STATIC  */

    /* ── 零偏在线更新 ─────────────────────────────────────────────── */
    float bias_alpha;         /* 0..1，每次 STATIC 帧的低通系数；越大越快 */
} zupt_config_t;

typedef struct {
    /* 对外可读状态 */
    zupt_state_t state;
    float        gyro_bias[3];           /* rad/s，当前零偏估计           */

    /* 调试遥测（外部可直接读，挂 Live Expressions 用）*/
    float        gyro_mean[3];           /* 最近一次窗口均值              */
    float        gyro_var[3];            /* 最近一次窗口方差              */
    float        accel_var_sum;          /* 加速度三轴方差之和            */
    uint32_t     settle_counter;         /* SETTLING 倒计时进度           */

    /* 内部状态（不应被外部修改）*/
    float        gyro_buf[3][ZUPT_WINDOW_SIZE];
    float        accel_buf[3][ZUPT_WINDOW_SIZE];
    uint32_t     idx;
    uint32_t     fill_count;             /* 0 .. ZUPT_WINDOW_SIZE          */
    bool         bootstrapped;           /* 首次窗口填满后的零偏自吸附标记 */

    zupt_config_t cfg;
} zupt_t;

/* ── 默认预设（在 zupt.c 中定义）──────────────────────────────────────
 * 若使用 1 kHz ODR，请同时设置 -DZUPT_WINDOW_SIZE=500
 */
extern const zupt_config_t ZUPT_CFG_100HZ_DEFAULT;
/* ── API ──────────────────────────────────────────────────────────────── */

/**
 * 初始化 ZUPT 实例。可重复调用以复位状态。
 * cfg 内容会被复制进 z，调用后无需保持 cfg 存活。
 */
void zupt_init(zupt_t *z, const zupt_config_t *cfg);

/**
 * 推入一帧原始 IMU 数据并更新内部状态。
 *   gyro_raw[3]：原始陀螺（rad/s），未减零偏
 *   accel[3]   ：加速度（m/s²，缩放后即可，单位一致即可）
 * 调用频率应与 IMU ODR 严格一致。
 */
void zupt_update(zupt_t *z, const float gyro_raw[3], const float accel[3]);

/**
 * 输出最终陀螺：
 *   - STATIC 状态下三轴强制为 0（消除噪声积分）
 *   - 否则减去当前零偏估计
 * 通常在 zupt_update() 之后立即调用。
 */
void zupt_apply(const zupt_t *z, const float gyro_raw[3], float gyro_out[3]);

/* 状态查询 */
bool          zupt_is_static(const zupt_t *z);
zupt_state_t  zupt_get_state(const zupt_t *z);
const float * zupt_get_bias (const zupt_t *z);

/**
 * 强制设置初始零偏（例如来自 imu_calibrate_gyro_bias() 的 10 s 静态标定）。
 * 调用后 ZUPT 会以此为基准做后续的小幅在线修正。
 */
void zupt_set_bias(zupt_t *z, const float bias[3]);

#ifdef __cplusplus
}
#endif

#endif /* APP_ZUPT_H_ */
