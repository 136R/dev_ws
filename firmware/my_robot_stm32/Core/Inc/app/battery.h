#ifndef APP_BATTERY_H
#define APP_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

/* ── Battery voltage sensing via ADC3_IN1 (PB1) ──────────────────────────
 *
 *  Hardware (already present on the PCB, see docs/vendor/Inc/APP/Config.h):
 *      VBAT ──[ R1 47k ]──┬──[ R2 10k ]── GND
 *                         └── PB1 (ADC3_IN1)
 *  Divider ratio (R1+R2)/R2 = 5.7  →  full scale 3.3 V x 5.7 = 18.8 V
 *
 *  Sampling is BLOCKING (~0.3 ms) and must never run inside an ISR:
 *  HAL_ADC_PollForConversion() can stall for up to its timeout, while the
 *  TIM6 control ISR fires every 10 ms.  Call battery_tick() from the main
 *  loop; the ISR side only reads the cached snapshot.
 * ──────────────────────────────────────────────────────────────────── */

/* Resistor divider — must match the PCB */
#define BATTERY_DIV_R1_KOHM     47.0f
#define BATTERY_DIV_R2_KOHM     10.0f

/* Compile-time gain trim from the multimeter calibration.
 *
 * Calibrated 2026-08-07 against a 0.01 V resolution multimeter probing the
 * battery input connector, with the full ROS stack running (controllers +
 * lidar + EKF).  Two independent points:
 *      12.44 / 12.301  = 1.01179   (4 samples)
 *      12.30 / 12.1732 = 1.01042   (122 samples over 62 s)
 * Mean 1.0111; back-substituted residuals are -0.003 V and +0.008 V.
 *
 * ⚠️ Load-dependent.  An earlier point taken with only the MCU powered
 * (no ROS stack) gave 1.0070 — noticeably lower.  The multimeter probes the
 * battery terminal while the ADC sees a divider downstream of the cabling
 * and connectors, so a heavier load drops more voltage in between and the
 * firmware reads lower.  This constant therefore encodes the *operating*
 * load, which is the right choice: the low-battery return-to-base decision
 * is made with the stack running.  Re-calibrate if the power path changes.
 *
 * ⚠️ Single operating point — linearity is NOT verified.  A bench supply
 * was available but not used, so 10~11 V (exactly where the low-battery
 * threshold will sit) rests on the assumption that the error is pure gain
 * with no offset term.  If return-to-base later triggers early or late,
 * suspect this first: sweep a bench supply across three points and check
 * whether the ratio stays constant. */
#define BATTERY_GAIN_CORR       1.0111f

/* One sample burst every 500 ms — matches the 2 Hz 0x03 protocol frame */
#define BATTERY_SAMPLE_PERIOD_MS   500u

/* Snapshot flag bits — laid out to be copied straight into the 0x03 frame */
#define BATTERY_FLAG_VOLTAGE_VALID  0x01u
#define BATTERY_FLAG_CURRENT_VALID  0x02u   /* always 0 in v1 (no current sense) */
#define BATTERY_FLAG_VREFINT_OK     0x04u

/* Snapshot packing: a single 32-bit word so an ISR gets a consistent
 * reading with one atomic load (no disable-IRQ / no torn struct). */
#define BATTERY_SNAP_MV(s)      ((uint16_t)((s) & 0xFFFFu))
#define BATTERY_SNAP_FLAGS(s)   ((uint8_t)(((s) >> 16) & 0xFFu))

/* ── API ────────────────────────────────────────────────────────────── */

/* Call once after MX_ADC3_Init(), before HAL_TIM_Base_Start_IT(&htim6).
 * Runs the ADC self-calibration and latches CCR.VREFEN while the ADC is
 * still disabled — see the note in battery.c, this ordering is mandatory. */
void battery_init(void);

/* Call unconditionally from the main loop; internally rate-limited to
 * BATTERY_SAMPLE_PERIOD_MS.  Blocks ~0.3 ms when it actually samples. */
void battery_tick(void);

/* ISR-safe: one atomic 32-bit read of the latest snapshot */
uint32_t battery_get_snapshot(void);

#endif /* APP_BATTERY_H */
