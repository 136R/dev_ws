/*
 * battery.c — battery voltage sensing on ADC3_IN1 (PB1)
 *
 * Ported from the vendor's Battery.c (docs/vendor/Src/APP/Battery.c) with
 * four deliberate deviations, each of them a fix rather than a preference:
 *
 *   1. Sampling time 2.5 -> 601.5 cycles.  The 47k/10k divider presents a
 *      ~8.2 kOhm source impedance; 2.5 cycles (~35 ns) cannot charge the ADC
 *      sample capacitor, so the vendor reading is systematically low and
 *      drifts with temperature.  601.5 cycles = 8.35 us at 72 MHz.
 *
 *   2. VREFINT is ADC_CHANNEL_18 on this part, not 17.  STM32F303xC selects
 *      the block at stm32f3xx_hal_adc_ex.h:516, where ADC_CHANNEL_VBAT = 17
 *      (:826) and ADC_CHANNEL_VREFINT = 18 (:835).  The vendor hard-codes
 *      ADC_CHANNEL_17, i.e. it calibrates against VBAT/2 instead of VREFINT.
 *
 *   3. VDDA is derived from the factory VREFINT_CAL value (0x1FFFF7BA)
 *      rather than a nominal 1.2 V constant, which removes the part-to-part
 *      spread of VREFINT itself (typ 1.23 V, range 1.16..1.30 V).
 *
 *   4. Stateless median filter (sort 8, average the middle 4) instead of the
 *      vendor's stateful sliding window with its "== 0 means uninitialised"
 *      hack, which biases the first readings after boot.
 *
 * Current sensing (PB0 / ADC3_IN12) is intentionally not implemented.
 */

#include "app/battery.h"
#include "adc.h"
#include "stm32f3xx_ll_adc.h"   /* VREFINT_CAL_ADDR / VREFINT_CAL_VREF */

/* ── Tunables ───────────────────────────────────────────────────────── */

#define ADC_MAX_COUNT           4095.0f   /* 12-bit full scale */
/* One conversion takes ~10 us, but the poll runs in the main loop and gets
 * preempted by the 100 Hz TIM6 ISR, so the timeout has to clear a couple of
 * ISR entries.  A timeout only costs one skipped 500 ms sample. */
#define ADC_POLL_TIMEOUT_MS     5u
#define SAMPLE_BURST            8u        /* samples per burst */
#define SAMPLE_TRIM             2u        /* discard N lowest and N highest */

/* Plausibility window for the raw VREFINT reading.  VREFINT_CAL is taken at
 * VDDA = 3.3 V and sits around 1500 counts; VDDA in 2.9..3.6 V keeps the
 * live reading inside this range.  Anything outside means the measurement
 * is not trustworthy (channel misconfigured, ADC not calibrated, ...). */
#define VREFINT_RAW_MIN         1200u
#define VREFINT_RAW_MAX         1900u

#define ADC_READ_FAILED         0xFFFFu

/* ── State ──────────────────────────────────────────────────────────── */

/* [15:0] voltage in mV, [23:16] BATTERY_FLAG_* — written as one word so a
 * reader in ISR context always sees a matching value/flags pair. */
static volatile uint32_t s_snapshot = 0u;

static uint32_t s_next_sample_tick = 0u;

/* ── ADC access ─────────────────────────────────────────────────────── */

/* Read one channel once.  Returns ADC_READ_FAILED on any HAL error.
 *
 * The ADC is left disabled on exit (HAL_ADC_Stop).  That matters: switching
 * to ADC_CHANNEL_VREFINT is only allowed to set CCR.VREFEN while the ADC is
 * disabled (stm32f3xx_hal_adc_ex.c:5791) — otherwise ConfigChannel returns
 * HAL_ERROR.  battery_init() latches VREFEN up front so the steady-state
 * path takes the bypass branch at :5777, but keeping Start/Stop balanced
 * means this stays correct even if the ADC is reset later. */
static uint16_t adc_read_once(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;

    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK) {
        return ADC_READ_FAILED;
    }
    if (HAL_ADC_Start(&hadc3) != HAL_OK) {
        return ADC_READ_FAILED;
    }
    if (HAL_ADC_PollForConversion(&hadc3, ADC_POLL_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&hadc3);
        return ADC_READ_FAILED;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return value;
}

/* Burst-read a channel and return the trimmed mean.
 * Insertion sort over 8 elements, then average the middle 4. */
static uint16_t adc_read_filtered(uint32_t channel)
{
    uint16_t s[SAMPLE_BURST];

    for (uint8_t i = 0; i < SAMPLE_BURST; i++) {
        uint16_t v = adc_read_once(channel);
        if (v == ADC_READ_FAILED) {
            return ADC_READ_FAILED;
        }
        s[i] = v;
    }

    for (uint8_t i = 1; i < SAMPLE_BURST; i++) {
        uint16_t key = s[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && s[j] > key) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = key;
    }

    uint32_t sum = 0u;
    for (uint8_t i = SAMPLE_TRIM; i < SAMPLE_BURST - SAMPLE_TRIM; i++) {
        sum += s[i];
    }
    return (uint16_t)(sum / (SAMPLE_BURST - 2u * SAMPLE_TRIM));
}

/* ── Public API ─────────────────────────────────────────────────────── */

void battery_init(void)
{
    s_snapshot = 0u;
    s_next_sample_tick = 0u;

    /* Both of the following require the ADC to be disabled, which is the
     * state MX_ADC3_Init() leaves it in (HAL_ADC_Init does not set ADEN). */

    /* STM32F3 ADCs do not reach their datasheet accuracy without this. */
    (void)HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

    /* Latch CCR.VREFEN once, now, while the ADC is guaranteed disabled.
     * Every later ConfigChannel(VREFINT) then hits the "already enabled,
     * bypass" branch and can never fail. */
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = ADC_CHANNEL_VREFINT;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    (void)HAL_ADC_ConfigChannel(&hadc3, &cfg);

    HAL_Delay(1);   /* VREFINT startup time is ~10 us; 1 ms is free here */
}

void battery_tick(void)
{
    uint32_t now = HAL_GetTick();

    /* Wrap-safe comparison (int32 difference), though a 32-bit ms tick
     * only wraps after 49 days. */
    if ((int32_t)(now - s_next_sample_tick) < 0) {
        return;
    }
    s_next_sample_tick = now + BATTERY_SAMPLE_PERIOD_MS;

    uint16_t vref_raw = adc_read_filtered(ADC_CHANNEL_VREFINT);
    uint16_t bat_raw  = adc_read_filtered(ADC_CHANNEL_1);

    if (bat_raw == ADC_READ_FAILED) {
        s_snapshot = 0u;   /* voltage_valid = 0 */
        return;
    }

    /* Derive VDDA from VREFINT when the reading looks sane; fall back to the
     * nominal 3.3 V rail otherwise, flagging that the result is uncalibrated. */
    uint16_t vref_cal = *VREFINT_CAL_ADDR;
    bool vrefint_ok = (vref_raw != ADC_READ_FAILED) &&
                      (vref_raw >= VREFINT_RAW_MIN) && (vref_raw <= VREFINT_RAW_MAX) &&
                      (vref_cal >= VREFINT_RAW_MIN) && (vref_cal <= VREFINT_RAW_MAX);

    float vdda_mv = (float)VREFINT_CAL_VREF;
    if (vrefint_ok) {
        vdda_mv = (float)VREFINT_CAL_VREF * (float)vref_cal / (float)vref_raw;
    }

    const float divider = (BATTERY_DIV_R1_KOHM + BATTERY_DIV_R2_KOHM) / BATTERY_DIV_R2_KOHM;
    float mv = vdda_mv * ((float)bat_raw / ADC_MAX_COUNT) * divider * BATTERY_GAIN_CORR;

    if (mv < 0.0f)      { mv = 0.0f; }
    if (mv > 65535.0f)  { mv = 65535.0f; }

    uint8_t flags = BATTERY_FLAG_VOLTAGE_VALID;
    if (vrefint_ok) {
        flags |= BATTERY_FLAG_VREFINT_OK;
    }

    s_snapshot = (uint32_t)(uint16_t)(mv + 0.5f) | ((uint32_t)flags << 16);
}

uint32_t battery_get_snapshot(void)
{
    return s_snapshot;
}
