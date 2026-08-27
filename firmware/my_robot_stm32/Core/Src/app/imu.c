// 稳定版
/*
 * imu.c — ICM-42688-P SPI 驱动
 *
 * 引脚：
 *   SPI1_SCK  PA5
 *   SPI1_MISO PA6  (SDO/SA0)
 *   SPI1_MOSI PA7  (SDA/MOSI)
 *   CS        PA4  (GPIO Output, active low)
 *   INT1      PB12 (EXTI rising edge, data-ready)
 *
 * 量程配置：
 *   陀螺仪：±500 dps  → GYRO_SCALE = 500/32768 * π/180  rad/s/LSB
 *   加速度：±4 g      → ACCEL_SCALE = 4/32768 * 9.80665 m/s²/LSB
 *   温度：  T(°C) = raw / 132.48 + 25.0
 *
 * 数据就绪机制（方案 A）：
 *   ICM INT1 上升沿 (PB12) → EXTI12 ISR → imu_int1_irq_handler() → g_ready = true
 *   TIM6 10ms → motor_controller_update → comm_protocol_tick
 *       → imu_data_ready() 为真时调用 imu_get_data()
 *       → imu_get_data() 内做 SPI 突发读（14 字节 ≈ 12 μs @ 9 MHz）
 */

#include "app/imu.h"
#include "app/fusion/FusionAhrs.h"
#include "app/zupt.h"
#include "spi.h"
#include "main.h"   /* IMU_CS_Pin, IMU_CS_GPIO_Port, IMU_INT1_Pin */
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ── CS 控制宏 ─────────────────────────────────────────────────────── */
#define IMU_CS_LOW()   HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define IMU_CS_HIGH()  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

/* ── ICM-42688-P 寄存器地址（Bank 0）──────────────────────────────── */
#define ICM_REG_DEVICE_CONFIG   0x11u   /* bit[0] SOFT_RESET_CONFIG */
#define ICM_REG_INT_CONFIG      0x14u   /* INT1 极性 / 驱动 / 模式  */
#define ICM_REG_TEMP_DATA1      0x1Du   /* 突发读起始地址            */
#define ICM_REG_PWR_MGMT0       0x4Eu
#define ICM_REG_GYRO_CONFIG0    0x4Fu
#define ICM_REG_ACCEL_CONFIG0   0x50u
#define ICM_REG_INT_CONFIG1     0x64u
#define ICM_REG_INT_SOURCE0     0x65u   /* bit[3] UI_DRDY_INT1_EN   */
#define ICM_REG_WHO_AM_I        0x75u
#define ICM_REG_BANK_SEL        0x76u

#define ICM_WHO_AM_I_VAL        0x47u

/* ── 陀螺 Z 轴标度修正 ──
 * 保持 1.0。台架实测正反各 5 圈的正负标度不对称可被零偏完全解释，无剩余不对称，
 * 正负分离没有依据；且它原本要补的 ODR 标度误差现在由 DWT 实测 dt 直接消掉了。 */
#define GYRO_Z_SF_POS    1.0f
#define GYRO_Z_SF_NEG    1.0f

/* ── 陀螺 Z 轴滞回死区（作用于 ZUPT 输出，即已减零偏的值）─────────────
 * 取值空隙：实测零偏地板与器件噪声底都在 0.01~0.02 °/s 量级，而机器人有意转向
 * 最慢也有几 °/s（下界见 nav2_params 的 min rotation vel）—— 中间隔着两个数量级。
 * 这里取 0.05 °/s，高于零偏、远低于任何真实转向。
 * ⚠️ 它只是第三道防线：一旦行驶时零偏被振动混叠抬到阈值以上，这个死区根本不触发，
 *    真正兜底的是 ZUPT 的在线零偏跟踪和 AAF 带宽配置。 */
#define GZ_DB_ENTER  8.7e-4f   /* rad/s，= 0.05 °/s */
#define GZ_DB_EXIT   1.7e-3f   /* rad/s，= 0.10 °/s，滞回 */

/* ── 量程 & 转换系数 ────────────────────────────────────────────────── */
/* 陀螺 ±500 dps → 灵敏度 65.5 LSB/(°/s) */
#define GYRO_SCALE   (500.0f / 32768.0f * (float)M_PI / 180.0f)
/* 加速 ±4 g → 灵敏度 8192 LSB/g */
#define ACCEL_SCALE  (4.0f / 32768.0f * 9.80665f)
/* 温度（ICM-42688-P 数据手册） */
#define TEMP_SENS    132.48f
#define TEMP_OFFSET  25.0f

/* ── 帧间隔实测（DWT CYCCNT）────────────────────────────────────────
 * ICM 的 ODR 由它内部的 RC 振荡器决定，和 TIM6 的晶振基准不是同一个时钟源。
 * 硬编码 dt = 1/ODR 会把两者的频差原样变成 yaw 的标度误差。
 * dt 取连续两次读取时刻之差：累加时望远镜式抵消，N 帧总时长精确等于墙钟时长，
 * 单帧相位抖动是零均值误差，不累积 —— 所以不需要 FIFO，也不必在 EXTI 里读 SPI。
 * 选 DWT 不选新开定时器：TIM1/2/4/6 已占满，新开要动 .ioc 重新生成代码。 */
#define IMU_DT_NOMINAL_S            0.01f   /* g_dwt_ok 为假 / dt 越界时的兜底值 */
#define IMU_DT_MIN_S                0.002f
#define IMU_DT_MAX_S                0.050f

/* ── Fusion AHRS ─────────────────────────────────────────────────────── */
#define IMU_FUSION_GAIN             0.3f
#define IMU_FUSION_ACCEL_REJECT_DEG 10.0f
#define IMU_FUSION_RECOVERY_TICKS   200u

/* ── 陀螺偏置标定 ───────────────────────────────────────────────────── */
/* 200 点即 2 s @ ODR 100 Hz（ODR 见 imu_init 写 GYRO_CONFIG0 的值）。
 * 缩短不损精度：ZUPT 的 STATIC 段会以 ZUPT_CFG_100HZ_DEFAULT.bias_alpha
 * 持续跟踪温漂，静态标定只负责给它一个起点。 */
#define CAL_NUM_SAMPLES     200u
#define CAL_MAX_ATTEMPTS    3u
/* 总超时，覆盖全部重试。超了就放弃标定并正常返回 —— 死等 g_ready 时 TIM6 还没启动，
 * IMU 挂掉会连电机和串口一起卡死。 */
#define CAL_TIMEOUT_MS      10000u
#define CAL_ACCEL_NORM_TOL  0.5f   /* m/s²，|‖a‖ − g| 上限，用来发现标定期间被搬动 */

/* ── 内部状态 ───────────────────────────────────────────────────────── */
static imu_data_t g_imu;
static volatile bool g_ready = false;
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static FusionAhrs g_ahrs;
static bool g_gz_in_deadzone = true;   /* 开机默认在零区内 */
static uint32_t g_cyc_prev     = 0u;
static bool     g_dt_have_prev = false;
static float    g_cyc_to_s     = 0.0f;
zupt_t g_zupt;

/* ── 调试全局变量（Live Expressions / 示波器用）────────────────────── */
volatile float    g_acc[3]              = {0.0f, 0.0f, 0.0f};
volatile float    g_gyro[3]             = {0.0f, 0.0f, 0.0f};
volatile float    g_mag[3]              = {0.0f, 0.0f, 0.0f};  /* 6 轴无磁力计，始终为 0 */
volatile float    g_gyro_bias_rad_s[3]  = {0.0f, 0.0f, 0.0f};
volatile float    g_yaw = 0.0f;
volatile float    yaw_gyro_accum_deg = 0.0f;     /* 原始imu累计 */
volatile uint32_t g_imu_packet_count    = 0u;
volatile uint32_t g_imu_parse_error_count = 0u;
volatile uint32_t g_imu_health          = 0u;
volatile float    g_imu_dt_s            = IMU_DT_NOMINAL_S;
volatile bool     g_dwt_ok              = false;
volatile uint32_t g_dt_clamp_count      = 0u;
volatile uint32_t g_isr_max_us          = 0u;

/* DWT 是 Cortex-M4 内核自带的周期计数器，不占外设、不需要 CubeMX 改动。
 * 无调试器连接时它是否走字属于实测项，故这里自检一次并留 g_dwt_ok 兜底。 */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t t0 = DWT->CYCCNT;
    for (volatile uint32_t i = 0u; i < 100u; i++) { }
    g_dwt_ok   = (DWT->CYCCNT != t0);
    g_cyc_to_s = 1.0f / (float)SystemCoreClock;
}

/* ── SPI 底层：单寄存器读 ───────────────────────────────────────────── */
static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0x00u};
    uint8_t rx[2] = {0u, 0u};
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 10u);
    IMU_CS_HIGH();
    return rx[1];
}

/* ── SPI 底层：单寄存器写 ───────────────────────────────────────────── */
static void icm_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), data};
    IMU_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2u, 10u);
    IMU_CS_HIGH();
}

/* ── SPI 底层：突发读 14 字节（TEMP + ACCEL_XYZ + GYRO_XYZ）────────── */
/* 必须用 TransmitReceive 一次性完成，避免 STM32 SPI FIFO 在两次调用间残留脏数据 */
static void icm_burst_read_sensors(uint8_t *buf14)
{
    uint8_t tx_buf[15] = {0};
    uint8_t rx_buf[15] = {0};
    tx_buf[0] = (uint8_t)(ICM_REG_TEMP_DATA1 | 0x80u);  /* bit7=1 → 读操作 */
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 15u, 20u);
    IMU_CS_HIGH();
    /* rx_buf[0] 是地址字节期间收到的垃圾，有效数据从 rx_buf[1] 开始 */
    for (uint8_t i = 0u; i < 14u; i++) {
        buf14[i] = rx_buf[i + 1u];
    }
}

/* ── 内部：读取并换算所有传感器数据 ────────────────────────────────── */
static void icm_read_all(void)
{
    /* CYCCNT 是 32 位、@SYSCLK 约 60 s 回绕；无符号相减在回绕处依然给出正确差值 */
    const uint32_t cyc_entry = DWT->CYCCNT;
    float dt = IMU_DT_NOMINAL_S;
    if (g_dwt_ok) {
        if (g_dt_have_prev) {
            const float dt_meas = (float)(cyc_entry - g_cyc_prev) * g_cyc_to_s;
            if ((dt_meas >= IMU_DT_MIN_S) && (dt_meas <= IMU_DT_MAX_S)) {
                dt = dt_meas;
            } else {
                g_dt_clamp_count++;
            }
        } else {
            g_dt_have_prev = true;
            g_dt_clamp_count++;   /* 首帧没有前值，这一次计数是预期内的 */
        }
        g_cyc_prev = cyc_entry;
    }
    g_imu_dt_s = dt;

    uint8_t raw[14];
    icm_burst_read_sensors(raw);

    /* 温度 (raw[0..1]) */
    int16_t t  = (int16_t)(((uint16_t)raw[0] << 8u) | raw[1]);
    g_imu.temp = (float)t / TEMP_SENS + TEMP_OFFSET;

    /* 加速度 (raw[2..7]：X1 X0 Y1 Y0 Z1 Z0) */
    int16_t ax = (int16_t)(((uint16_t)raw[2] << 8u) | raw[3]);
    int16_t ay = (int16_t)(((uint16_t)raw[4] << 8u) | raw[5]);
    int16_t az = (int16_t)(((uint16_t)raw[6] << 8u) | raw[7]);
    g_imu.accel[0] = (float)ax * ACCEL_SCALE;
    g_imu.accel[1] = (float)ay * ACCEL_SCALE;
    g_imu.accel[2] = (float)az * ACCEL_SCALE;

    /* 陀螺仪 (raw[8..13]：X1 X0 Y1 Y0 Z1 Z0) */
    int16_t gx = (int16_t)(((uint16_t)raw[8]  << 8u) | raw[9]);
    int16_t gy = (int16_t)(((uint16_t)raw[10] << 8u) | raw[11]);
    int16_t gz = (int16_t)(((uint16_t)raw[12] << 8u) | raw[13]);
    /* zupt_update() 要的是「未减零偏」的原始值 —— 零偏由它自己在 STATIC 段在线跟踪，
     * 这里先减一遍会让它的窗口均值判据失去意义。见 zupt.h 的调用约定。 */
    const float gyro_raw[3] = {
        (float)gx * GYRO_SCALE,
        (float)gy * GYRO_SCALE,
        (float)gz * GYRO_SCALE,
    };
    zupt_update(&g_zupt, gyro_raw, g_imu.accel);
    zupt_apply(&g_zupt, gyro_raw, g_imu.gyro);

    const float sf_z = (g_imu.gyro[2] >= 0.0f) ? GYRO_Z_SF_POS : GYRO_Z_SF_NEG;
    g_imu.gyro[2] *= sf_z;

    /* 只动 z：roll/pitch 由 AHRS 用加速度计持续修正，不会像 yaw 那样纯积分发散 */
    if (g_gz_in_deadzone) {
        if (fabsf(g_imu.gyro[2]) > GZ_DB_EXIT) {
            g_gz_in_deadzone = false;
        } else {
            g_imu.gyro[2] = 0.0f;
        }
    } else if (fabsf(g_imu.gyro[2]) < GZ_DB_ENTER) {
        g_gz_in_deadzone = true;
        g_imu.gyro[2] = 0.0f;
    }

    /* 裸积分探针：取 ZUPT 门控之前的值。台架基线与验收 A2 都以它为准，
     * 一旦改成门控之后的值，历史基线就不可比了。 */
    yaw_gyro_accum_deg += (gyro_raw[2] - g_zupt.gyro_bias[2])
                          * dt * 180.0f / (float)M_PI;

    /* 6 轴无磁力计 */
    g_imu.mag[0]  = 0.0f;  g_imu.mag[1]  = 0.0f;  g_imu.mag[2]  = 0.0f;

    const FusionVector gyroscope = {.axis = {
        .x = FusionRadiansToDegrees(g_imu.gyro[0]),
        .y = FusionRadiansToDegrees(g_imu.gyro[1]),
        .z = FusionRadiansToDegrees(g_imu.gyro[2]),
    }};
    const FusionVector accelerometer = {.axis = {
        .x = g_imu.accel[0] / 9.80665f,
        .y = g_imu.accel[1] / 9.80665f,
        .z = g_imu.accel[2] / 9.80665f,
    }};


    FusionAhrsUpdateNoMagnetometer(&g_ahrs, gyroscope, accelerometer, dt);

    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&g_ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);

    g_imu.quat[0] = quaternion.element.w;
    g_imu.quat[1] = quaternion.element.x;
    g_imu.quat[2] = quaternion.element.y;
    g_imu.quat[3] = quaternion.element.z;
    g_imu.roll = euler.angle.roll;
    g_imu.pitch = euler.angle.pitch;
    g_imu.yaw = euler.angle.yaw;

    /* 更新调试全局变量 */
    g_acc[0]  = g_imu.accel[0];
    g_acc[1]  = g_imu.accel[1];
    g_acc[2]  = g_imu.accel[2];
    g_gyro[0] = g_imu.gyro[0];
    g_gyro[1] = g_imu.gyro[1];
    g_gyro[2] = g_imu.gyro[2];
    g_gyro_bias_rad_s[0] = g_zupt.gyro_bias[0];
    g_gyro_bias_rad_s[1] = g_zupt.gyro_bias[1];
    g_gyro_bias_rad_s[2] = g_zupt.gyro_bias[2];

    g_yaw = g_imu.yaw;

    g_imu_packet_count++;

    /* 白拿的 ISR 耗时探针（同一个计数器）：ZUPT 的滑窗统计在 -O0 下是否吃爆 10 ms 预算 */
    if (g_dwt_ok) {
        const uint32_t us = (uint32_t)((float)(DWT->CYCCNT - cyc_entry) * g_cyc_to_s * 1.0e6f);
        if (us > g_isr_max_us) {
            g_isr_max_us = us;
        }
    }
}

/* ── 公共 API ────────────────────────────────────────────────────────── */

void imu_init(void)
{
    dwt_init();
    memset(&g_imu, 0, sizeof(g_imu));
    g_imu.quat[0] = 1.0f;
    g_ready = false;
    g_gyro_bias[0] = 0.0f;  g_gyro_bias[1] = 0.0f;  g_gyro_bias[2] = 0.0f;
    g_imu_packet_count      = 0u;
    g_imu_parse_error_count = 0u;
    g_imu_health            = 0u;
    g_dt_clamp_count        = 0u;
    g_isr_max_us            = 0u;
    g_dt_have_prev          = false;

    zupt_init(&g_zupt, &ZUPT_CFG_100HZ_DEFAULT);

    FusionAhrsInitialise(&g_ahrs);
    FusionAhrsSettings settings = {
        .convention = FusionConventionEnu,
        .gain = IMU_FUSION_GAIN,
        .gyroscopeRange = 500.0f,
        .accelerationRejection = IMU_FUSION_ACCEL_REJECT_DEG,
        .magneticRejection = 0.0f,
        .recoveryTriggerPeriod = IMU_FUSION_RECOVERY_TICKS,
    };
    FusionAhrsSetSettings(&g_ahrs, &settings);

    IMU_CS_HIGH();
    HAL_Delay(10u);

    /* 软复位 */
    icm_write_reg(ICM_REG_DEVICE_CONFIG, 0x01u);
    HAL_Delay(10u);   /* 等待复位完成（数据手册要求 ≥1 ms） */

    /* 验证 WHO_AM_I（期望 0x47） */
    if (icm_read_reg(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_VAL) {
        g_imu_parse_error_count++;
        g_imu_health |= IMU_HEALTH_WHOAMI_BAD;
        /* 不 hang，继续初始化；调试时在此加断点检查 SPI 接线 */
    }

    /* 陀螺：±500 dps，ODR 100 Hz
     * GYRO_CONFIG0 [7:5]=010(±500dps)  [3:0]=1000(100Hz)
     * 注：0x47 的 bits[3:0]=0111 是 200 Hz，0x48 的 bits[3:0]=1000 才是 100 Hz */
    icm_write_reg(ICM_REG_GYRO_CONFIG0, 0x48u);

    /* 加速：±4 g，ODR 100 Hz
     * ACCEL_CONFIG0 [7:5]=010(±4g)  [3:0]=1000(100Hz) */
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, 0x48u);

    /* AAF（模拟抗混叠滤波）的复位带宽是陀螺 585 Hz / 加速 1220 Hz，远高于 ODR 100 Hz 的
     * Nyquist 50 Hz —— 电机振动里落在 100 Hz 整数倍附近的能量会折叠到 DC，表现为随
     * 转速变化的陀螺零偏。DELT=1 / DELTSQR=1 / BITSHIFT=15（手册 §5.2 表）是唯一压在
     * 50 Hz 以下的档位，42 Hz。
     * 这 6 个寄存器的 Reserved 位复位值全为 0，所以整字节写安全，不必 read-modify-write。 */
    static const uint8_t aaf_regs[6][3] = {   /* {bank, reg, value} */
        {1u, 0x0Cu, 0x01u}, {1u, 0x0Du, 0x01u}, {1u, 0x0Eu, 0xF0u},
        {2u, 0x03u, 0x02u}, {2u, 0x04u, 0x01u}, {2u, 0x05u, 0xF0u},
    };
    for (uint32_t i = 0u; i < 6u; i++) {
        icm_write_reg(ICM_REG_BANK_SEL, aaf_regs[i][0]);
        icm_write_reg(aaf_regs[i][1], aaf_regs[i][2]);
    }
    for (uint32_t i = 0u; i < 6u; i++) {
        icm_write_reg(ICM_REG_BANK_SEL, aaf_regs[i][0]);
        if (icm_read_reg(aaf_regs[i][1]) != aaf_regs[i][2]) {
            g_imu_parse_error_count++;
            g_imu_health |= IMU_HEALTH_AAF_BAD;
        }
    }
    /* 必须切回 Bank0：后面所有寄存器操作和 icm_burst_read_sensors() 都按 Bank0 地址走 */
    icm_write_reg(ICM_REG_BANK_SEL, 0x00u);

    /* 上电：陀螺 + 加速均进入 Low-Noise 模式
     * PWR_MGMT0 [3:2]=11(GYRO LN)  [1:0]=11(ACCEL LN) */
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x0Fu);
    HAL_Delay(50u);    /* 等待传感器就绪（≥1 ms） */

    /* DLPF：陀螺 + 加速均 ~44Hz（ODR=100Hz，值 3 = max(400,100)/8 = 50Hz nominal，3dB≈44.4Hz）*/
    icm_write_reg(0x52u, 0x33u);

    /* INT1：Push-Pull，Active High，脉冲模式
     * INT_CONFIG [2]=0(pulse)  [1]=1(push-pull)  [0]=1(active-high) */
    icm_write_reg(ICM_REG_INT_CONFIG, 0x03u);

    /* 清除 INT_CONFIG1 异步复位位，防止中断丢失 */
    icm_write_reg(ICM_REG_INT_CONFIG1, 0x00u);

    /* 使能 UI Data-Ready 中断路由到 INT1
     * INT_SOURCE0 bit[3] = UI_DRDY_INT1_EN */
    icm_write_reg(ICM_REG_INT_SOURCE0, 0x08u);
}

/* 从 EXTI12 ISR 调用（PB12 上升沿，极轻量，仅置标志） */
void imu_int1_irq_handler(void)
{
    g_ready = true;
}

bool imu_data_ready(void)
{
    return g_ready;
}

void imu_get_data(imu_data_t *out)
{
    icm_read_all();    /* SPI 读取 + 换算（≈12 μs @ 9 MHz） */
    g_ready = false;
    if (out) { *out = g_imu; }
}

void imu_peek_data(imu_data_t *out)
{
    if (out) { *out = g_imu; }
}

typedef enum {
    CAL_OK = 0,
    CAL_UNSTABLE,   /* 方差或重力模长超标：多半是被碰了，值得重试 */
    CAL_TIMEOUT,    /* 等不到 DRDY：重试也没用，直接放弃 */
} cal_result_t;

/* 采一窗静止数据并校验。deadline_ms 是 HAL_GetTick() 口径的绝对截止时刻。 */
static cal_result_t cal_collect(float bias_out[3], uint32_t deadline_ms)
{
    float sum[3]     = {0.0f, 0.0f, 0.0f};
    float sumsq[3]   = {0.0f, 0.0f, 0.0f};
    float acc_sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t count = 0u;
    uint8_t raw[14];

    while (count < CAL_NUM_SAMPLES) {
        /* 无符号差再转有符号，HAL_GetTick() 回绕时判定依然正确 */
        if ((int32_t)(HAL_GetTick() - deadline_ms) >= 0) {
            return CAL_TIMEOUT;
        }
        if (!g_ready) {
            continue;
        }

        /* 直接读原始传感器数据，不经过 icm_read_all()，
         * 避免把带偏置的陀螺数据喂进 AHRS 滤波器 */
        icm_burst_read_sensors(raw);
        g_ready = false;

        const float g[3] = {
            (float)(int16_t)(((uint16_t)raw[8]  << 8u) | raw[9])  * GYRO_SCALE,
            (float)(int16_t)(((uint16_t)raw[10] << 8u) | raw[11]) * GYRO_SCALE,
            (float)(int16_t)(((uint16_t)raw[12] << 8u) | raw[13]) * GYRO_SCALE,
        };
        const float a[3] = {
            (float)(int16_t)(((uint16_t)raw[2] << 8u) | raw[3]) * ACCEL_SCALE,
            (float)(int16_t)(((uint16_t)raw[4] << 8u) | raw[5]) * ACCEL_SCALE,
            (float)(int16_t)(((uint16_t)raw[6] << 8u) | raw[7]) * ACCEL_SCALE,
        };
        for (uint32_t i = 0u; i < 3u; i++) {
            sum[i]     += g[i];
            sumsq[i]   += g[i] * g[i];
            acc_sum[i] += a[i];
        }
        count++;
    }

    const float inv_n = 1.0f / (float)CAL_NUM_SAMPLES;
    for (uint32_t i = 0u; i < 3u; i++) {
        bias_out[i] = sum[i] * inv_n;
        /* 阈值沿用 ZUPT 的静止判据，避免"标定认为静止、ZUPT 认为在动"这种自相矛盾 */
        const float var = sumsq[i] * inv_n - bias_out[i] * bias_out[i];
        if (var > ZUPT_CFG_100HZ_DEFAULT.gyro_var_thresh) {
            return CAL_UNSTABLE;
        }
    }

    const float an = sqrtf(acc_sum[0] * acc_sum[0] +
                           acc_sum[1] * acc_sum[1] +
                           acc_sum[2] * acc_sum[2]) * inv_n;
    if (fabsf(an - 9.80665f) > CAL_ACCEL_NORM_TOL) {
        return CAL_UNSTABLE;
    }
    return CAL_OK;
}

void imu_calibrate_gyro_bias(void)
{
    const uint32_t deadline = HAL_GetTick() + CAL_TIMEOUT_MS;
    float bias[3] = {0.0f, 0.0f, 0.0f};
    cal_result_t res = CAL_UNSTABLE;

    for (uint32_t attempt = 0u; attempt < CAL_MAX_ATTEMPTS; attempt++) {
        res = cal_collect(bias, deadline);
        if (res != CAL_UNSTABLE) {
            break;
        }
    }

    if (res != CAL_OK) {
        bias[0] = 0.0f;  bias[1] = 0.0f;  bias[2] = 0.0f;
        g_imu_health |= (res == CAL_TIMEOUT) ? IMU_HEALTH_CAL_TIMEOUT
                                             : IMU_HEALTH_CAL_UNSTABLE;
    }

    g_gyro_bias[0] = bias[0];
    g_gyro_bias[1] = bias[1];
    g_gyro_bias[2] = bias[2];

    g_gz_in_deadzone = true;
    zupt_init(&g_zupt, &ZUPT_CFG_100HZ_DEFAULT);
    zupt_set_bias(&g_zupt, g_gyro_bias);
    /* 标定成功就跳过 bootstrap，别让窗口均值自吸附覆盖刚测好的零偏；
     * 标定失败时反过来，让 bootstrap 兜底把零偏捡回来。 */
    g_zupt.bootstrapped = (res == CAL_OK);

    g_gyro_bias_rad_s[0] = g_gyro_bias[0];
    g_gyro_bias_rad_s[1] = g_gyro_bias[1];
    g_gyro_bias_rad_s[2] = g_gyro_bias[2];

    FusionAhrsRestart(&g_ahrs);
    g_yaw = 0.0f;
}
