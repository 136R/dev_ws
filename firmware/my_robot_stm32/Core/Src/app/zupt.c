/*
 * zupt.c — Zero-velocity Update / 静止检测 + 陀螺零偏在线更新
 *
 * 设计要点：
 *   1. 滑动窗口：环形缓冲区，每帧推入 1 个新样本、丢弃最旧的 1 个。
 *      只有窗口内全部样本都"安静"才会触发静止判据。
 *      因此窗口长度本身就是一道时间门槛。
 *   2. 滞回（hysteresis）：
 *      - 进入 STATIC：严格的窗口均值 + 方差检查
 *      - 退出 STATIC：宽松的瞬时阈值检查（响应快，不让动作被吃）
 *   3. 零偏更新只发生在 STATIC 状态。
 *   4. 自启动（bootstrap）：首次窗口填满时若已经静止，把窗口均值
 *      作为初始零偏直接吸附；这样即使外部没做静态标定也能起步。
 *   5. 一切阈值都是可调的；具体取值需要根据机器人实际振动谱做整定。
 */

#include "app/zupt.h"
#include <math.h>
#include <string.h>

/* ════════════════════════════════════════════════════════════════════════
 * 默认预设
 * ════════════════════════════════════════════════════════════════════════
 * 阈值思路：
 *   gyro_mean_thresh = 0.01 rad/s ≈ 0.57°/s
 *     —— 若机器人有意慢速旋转（< 0.5°/s），需调小此值，否则会被误判为静止
 *   gyro_var_thresh  = (0.005 rad/s)² = 2.5e-5
 *     —— ICM-42688-P @ 100 Hz 噪声底约 0.0003 rad/s，留足 ~16× 余量
 *        实测时把静止/运动的方差打 log，按实际噪声底再调
 *   accel_var_thresh = 0.5 (m/s²)²
 *     —— 重力 9.8 m/s² 的 ~3% 振幅，地面机器人电机怠速通常远低于此
 *
 *   退出阈值取入场阈值的 ~5×，防止在临界点反复跳变
 *
 *   bias_alpha：在 STATIC 状态下每帧融合一次。alpha=0.02 时
 *     时间常数 ≈ 50 帧 = 0.5 s @ 100 Hz，与窗口同量级，足够追温漂
 */
const zupt_config_t ZUPT_CFG_100HZ_DEFAULT = {
    .gyro_mean_thresh  = 0.01f,
    .gyro_var_thresh   = 2.5e-5f,
    .accel_var_thresh  = 0.5f,
    .gyro_exit_thresh  = 0.05f,
    .accel_exit_thresh = 2.5f,
    .settle_samples    = 25u,    /* 0.25 s @ 100 Hz */
    .bias_alpha        = 0.02f,
};

/* ════════════════════════════════════════════════════════════════════════
 * 内部辅助
 * ════════════════════════════════════════════════════════════════════════ */

/* 计算当前窗口均值 / 方差。结果写入 z->gyro_mean / gyro_var / accel_var_sum */
static void compute_stats(zupt_t *z)
{
    const uint32_t n = z->fill_count;
    if (n < 2u) {
        return;     /* 数据不足，保留上次结果 */
    }

    float sg[3] = {0.0f, 0.0f, 0.0f};
    float sa[3] = {0.0f, 0.0f, 0.0f};
    for (uint32_t k = 0u; k < n; k++) {
        sg[0] += z->gyro_buf[0][k];
        sg[1] += z->gyro_buf[1][k];
        sg[2] += z->gyro_buf[2][k];
        sa[0] += z->accel_buf[0][k];
        sa[1] += z->accel_buf[1][k];
        sa[2] += z->accel_buf[2][k];
    }
    const float inv_n = 1.0f / (float)n;
    z->gyro_mean[0] = sg[0] * inv_n;
    z->gyro_mean[1] = sg[1] * inv_n;
    z->gyro_mean[2] = sg[2] * inv_n;
    const float am0 = sa[0] * inv_n;
    const float am1 = sa[1] * inv_n;
    const float am2 = sa[2] * inv_n;

    float vg[3] = {0.0f, 0.0f, 0.0f};
    float va    = 0.0f;
    for (uint32_t k = 0u; k < n; k++) {
        const float dgx = z->gyro_buf[0][k] - z->gyro_mean[0];
        const float dgy = z->gyro_buf[1][k] - z->gyro_mean[1];
        const float dgz = z->gyro_buf[2][k] - z->gyro_mean[2];
        const float dax = z->accel_buf[0][k] - am0;
        const float day = z->accel_buf[1][k] - am1;
        const float daz = z->accel_buf[2][k] - am2;
        vg[0] += dgx * dgx;
        vg[1] += dgy * dgy;
        vg[2] += dgz * dgz;
        va    += dax * dax + day * day + daz * daz;
    }
    z->gyro_var[0]    = vg[0] * inv_n;
    z->gyro_var[1]    = vg[1] * inv_n;
    z->gyro_var[2]    = vg[2] * inv_n;
    z->accel_var_sum  = va    * inv_n;
}

/* 严格判据：当前窗口是否完全"安静"。
 * 必须满足：窗口已填满 + 均值贴近当前零偏 + 方差全低 */
static bool is_window_static(const zupt_t *z)
{
    if (z->fill_count < ZUPT_WINDOW_SIZE) {
        return false;
    }
    /* 均值偏离当前零偏不大（防止恒速转动这种"低方差但有真值"的情形）*/
    for (uint32_t i = 0u; i < 3u; i++) {
        const float dev = fabsf(z->gyro_mean[i] - z->gyro_bias[i]);
        if (dev > z->cfg.gyro_mean_thresh) {
            return false;
        }
    }
    /* 三轴陀螺方差均小（无颤抖、无小幅运动）*/
    for (uint32_t i = 0u; i < 3u; i++) {
        if (z->gyro_var[i] > z->cfg.gyro_var_thresh) {
            return false;
        }
    }
    /* 加速度方差小（无振动、无撞击）*/
    if (z->accel_var_sum > z->cfg.accel_var_thresh) {
        return false;
    }
    return true;
}

/* 宽松判据：STATIC 期间，一旦瞬时陀螺或加速度方差超过退出阈值就翻 MOVING */
static bool should_exit_static(const zupt_t *z, const float gyro_raw[3])
{
    for (uint32_t i = 0u; i < 3u; i++) {
        const float g = gyro_raw[i] - z->gyro_bias[i];
        if (fabsf(g) > z->cfg.gyro_exit_thresh) {
            return true;
        }
    }
    if (z->accel_var_sum > z->cfg.accel_exit_thresh) {
        return true;
    }
    return false;
}

/* ════════════════════════════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════════════════════════════ */

void zupt_init(zupt_t *z, const zupt_config_t *cfg)
{
    memset(z, 0, sizeof(*z));
    z->state = ZUPT_MOVING;
    z->cfg   = *cfg;
}

void zupt_set_bias(zupt_t *z, const float bias[3])
{
    z->gyro_bias[0] = bias[0];
    z->gyro_bias[1] = bias[1];
    z->gyro_bias[2] = bias[2];
}

const float *zupt_get_bias (const zupt_t *z) { return z->gyro_bias; }
bool          zupt_is_static(const zupt_t *z) { return z->state == ZUPT_STATIC; }
zupt_state_t  zupt_get_state(const zupt_t *z) { return z->state; }

void zupt_update(zupt_t *z, const float gyro_raw[3], const float accel[3])
{
    /* 1. 推入环形缓冲区 */
    const uint32_t i = z->idx;
    z->gyro_buf[0][i]  = gyro_raw[0];
    z->gyro_buf[1][i]  = gyro_raw[1];
    z->gyro_buf[2][i]  = gyro_raw[2];
    z->accel_buf[0][i] = accel[0];
    z->accel_buf[1][i] = accel[1];
    z->accel_buf[2][i] = accel[2];
    z->idx = (i + 1u) % ZUPT_WINDOW_SIZE;
    if (z->fill_count < ZUPT_WINDOW_SIZE) {
        z->fill_count++;
    }

    /* 2. 更新窗口统计 */
    compute_stats(z);

    /* 3. 自启动（bootstrap）：首次窗口填满时若三方差都小，
     *    把当前均值吸附为零偏；解决"开机时未做静态标定"的冷启动问题 */
    if (!z->bootstrapped && z->fill_count >= ZUPT_WINDOW_SIZE) {
        const bool var_ok = (z->gyro_var[0]   < z->cfg.gyro_var_thresh) &&
                            (z->gyro_var[1]   < z->cfg.gyro_var_thresh) &&
                            (z->gyro_var[2]   < z->cfg.gyro_var_thresh) &&
                            (z->accel_var_sum < z->cfg.accel_var_thresh);
        if (var_ok) {
            z->gyro_bias[0] = z->gyro_mean[0];
            z->gyro_bias[1] = z->gyro_mean[1];
            z->gyro_bias[2] = z->gyro_mean[2];
        }
        z->bootstrapped = true;     /* 不论是否吸附，仅尝试一次 */
    }

    /* 4. 状态机推进 */
    const bool quiet = is_window_static(z);

    switch (z->state) {
        case ZUPT_MOVING:
            if (quiet) {
                z->state           = ZUPT_SETTLING;
                z->settle_counter  = 0u;
            }
            break;

        case ZUPT_SETTLING:
            if (quiet) {
                z->settle_counter++;
                if (z->settle_counter >= z->cfg.settle_samples) {
                    z->state = ZUPT_STATIC;
                }
            } else {
                z->state          = ZUPT_MOVING;
                z->settle_counter = 0u;
            }
            break;

        case ZUPT_STATIC:
            if (should_exit_static(z, gyro_raw)) {
                z->state          = ZUPT_MOVING;
                z->settle_counter = 0u;
            } else {
                /* 一阶低通：bias ← (1−α)·bias + α·窗口均值
                 * 窗口均值在静止时即等于真实零偏（含温漂） */
                const float a = z->cfg.bias_alpha;
                z->gyro_bias[0] = (1.0f - a) * z->gyro_bias[0] + a * z->gyro_mean[0];
                z->gyro_bias[1] = (1.0f - a) * z->gyro_bias[1] + a * z->gyro_mean[1];
                z->gyro_bias[2] = (1.0f - a) * z->gyro_bias[2] + a * z->gyro_mean[2];
            }
            break;

        default:
            z->state = ZUPT_MOVING;
            break;
    }
}

void zupt_apply(const zupt_t *z, const float gyro_raw[3], float gyro_out[3])
{
    if (z->state == ZUPT_STATIC) {
        /* 静止：彻底切断噪声积分链路 */
        gyro_out[0] = 0.0f;
        gyro_out[1] = 0.0f;
        gyro_out[2] = 0.0f;
    } else {
        /* 运动 / 过渡：减零偏后正常输出 */
        gyro_out[0] = gyro_raw[0] - z->gyro_bias[0];
        gyro_out[1] = gyro_raw[1] - z->gyro_bias[1];
        gyro_out[2] = gyro_raw[2] - z->gyro_bias[2];
    }
}
