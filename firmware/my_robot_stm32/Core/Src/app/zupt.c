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
 *   gyro_mean_thresh —— 判的是窗口均值减掉当前零偏后的偏离量。
 *     机器人有意慢速旋转时这个偏离量就是转速本身，所以它必须小于最慢的有意转向
 *   gyro_var_thresh —— 固件内实测静止时三轴方差都在 1e-7 量级，本阈值余量约 200×。
 *     ⚠️ 别拿 ROS 侧 /imu_broad/imu 算出来的方差定这个值：串口传帧的抖动会把它
 *        抬高约 100 倍，照那个数调会误以为没余量。要看就挂 CubeIDE Live Expressions
 *        直接读 g_zupt.gyro_var。
 *   accel_var_thresh —— 见下方结构体上的实测说明，它在本机上没有鉴别力
 *
 *   退出阈值取入场阈值的 ~5×，防止在临界点反复跳变
 *
 *   bias_alpha：在 STATIC 状态下每帧融合一次。它作用的对象是窗口均值，
 *     所以实际链路是 EWMA(τ≈1/alpha 帧) 串联 MA(ZUPT_WINDOW_SIZE)，
 *     等效时间常数约为两者之和，而不是只有 EWMA 那一项
 */
const zupt_config_t ZUPT_CFG_100HZ_DEFAULT = {
    /* 0.115°/s。下界由标定后的残余漂移定（实测约 0.012°/s，见 docs/spec/ 那张
     * IMU yaw 稳定性卡），留了约 10× 余量；上界由"必须能否决匀速慢转"定 ——
     * 验收 A8 拿 0.3°/s 试，必须显著小于它。
     * ⚠️ 这里比的是减完零偏的偏离量，不是原始均值 —— 器件原始零偏实测已有 0.3°/s。 */
    .gyro_mean_thresh  = 2.0e-3f,
    .gyro_var_thresh   = 2.5e-5f,
    /* 实测（固件内读 accel_var_sum）：电机通电静止 0.00055 / 直线行驶 0.00053 ——
     * 两者一模一样。这条判据在本机上区分不了"行驶 vs 静止"，只能用来挡撞击、
     * 被搬动这类大扰动。别再拿它当"行驶时不会误判静止"的依据。 */
    .accel_var_thresh  = 0.1f,
    .gyro_exit_thresh  = 0.05f,
    .accel_exit_thresh = 2.5f,
    /* 500 帧 = 5 s @ 100 Hz。连续 10 个滑窗都安静才认，用来排除运动中的短暂平静期
     * （转向前的停顿、避障刹停）。⚠️ 它对匀速慢转无效 —— 那种情形方差本来就低，
     * 拦它的是 gyro_mean_thresh，不是这个。 */
    .settle_samples    = 500u,
    .bias_alpha        = 0.02f,
    /* 2.9°/s。零偏再怎么温漂也到不了这个量级，超了就说明估计已经跑飞。 */
    .bias_clip         = 0.05f,
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

/* 宽松判据：STATIC 期间，一旦瞬时陀螺或加速度方差超过退出阈值就翻 MOVING。
 * accel 那条用的是窗口方差，滞后一整个窗口，实际退出速度由陀螺那条瞬时判据决定；
 * 别指望调 accel_exit_thresh 能让退出变快。 */
static bool should_exit_static(const zupt_t *z, const float gyro_raw[3])
{
    for (uint32_t i = 0u; i < 3u; i++) {
        const float g = gyro_raw[i] - z->gyro_bias[i];
        if (fabsf(g) > z->cfg.gyro_exit_thresh) {
            return true;
        }
        /* 窗口均值的偏离量也要在这里查一遍。状态机进了 ZUPT_STATIC 之后就不再调
         * is_window_static()，只走本函数；少了这条，"先停住、再开始匀速慢转"
         * 的序列里 0.3°/s 远低于 gyro_exit_thresh，不触发退出，会被 EWMA
         * 一路吸收成零偏 —— 那是 F8 自锁的另一半入口。 */
        if (fabsf(z->gyro_mean[i] - z->gyro_bias[i]) > z->cfg.gyro_mean_thresh) {
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
                const float a    = z->cfg.bias_alpha;
                const float clip = z->cfg.bias_clip;
                for (uint32_t i = 0u; i < 3u; i++) {
                    float b = (1.0f - a) * z->gyro_bias[i] + a * z->gyro_mean[i];
                    if (b >  clip) { b =  clip; }
                    if (b < -clip) { b = -clip; }
                    z->gyro_bias[i] = b;
                }
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
