/**
 * @file b3_line_cal.h
 * @brief Per-sensor auto-calibration of the analogue IR line sensors.
 *
 * Follows the calibration scheme of Pololu's QTRSensors library
 * (github.com/pololu/qtr-sensors-arduino, calibrate() / readCalibrated()):
 *
 *  - Each sensor learns its own minimum (white) and maximum (black) raw
 *    reading.  The range only ever WIDENS: it never forgets or drifts, so a
 *    sensor that sits on white for a long time keeps its full range and
 *    floor noise cannot start to look like line.
 *  - A new extreme is accepted only when a whole block of
 *    B3_CAL_BLOCK_SAMPLES consecutive readings is beyond it, so a single
 *    noisy spike cannot widen it.  Pololu uses 10 reads taken back to back
 *    within one calibrate() call; B3 samples every 20 ms, so a block of 3
 *    (60 ms) is used: long enough to reject a spike, short enough that a
 *    moving car crossing the line still teaches each sensor its black.
 *    Windows overlap: any three consecutive samples count, independent
 *    of where the sweep falls relative to task startup.
 *  - Readings are normalised (raw - min) * 1000 / (max - min), clamped
 *    to 0 = white .. 1000 = black.
 *
 * Pololu requires an explicit calibration sweep before use.  Here the
 * learned range is used once it spans at least B3_CAL_MIN_SPAN; until then
 * the team's tested fixed range (B3_CAL_DEFAULT_WHITE..BLACK, on-line at raw
 * 300) is used, so the line is seen from the very first sample.
 * Calibration restarts at every boot: slide the car across the line once
 * after power-up, and do not lift the car afterwards (lifted sensors read
 * very dark and would widen the black end).
 */

#ifndef B3_LINE_CAL_H
#define B3_LINE_CAL_H

#include <stdint.h>
#include <stdbool.h>

/* Fallback range until the sensor has learned a real one.  Chosen so the
 * team's tested raw threshold still holds: raw 300 normalises to 200, the
 * B3 "on the line" boundary (above 200, as in Pololu readLine()). */
#define B3_CAL_DEFAULT_WHITE (220U)
#define B3_CAL_DEFAULT_BLACK (620U)
/* Smallest learned span accepted as a real calibration. */
#define B3_CAL_MIN_SPAN (64U)
/* Consecutive readings that must all lie beyond an extreme to move it. */
#define B3_CAL_BLOCK_SAMPLES (3U)
#define B3_CAL_FULL_SCALE (1000U)

typedef struct {
	uint16_t white;      /* learned minimum (only lowered) */
	uint16_t black;      /* learned maximum (only raised)  */
	uint16_t block_min;  /* min / max of the current block */
	uint16_t block_max;
	uint8_t block_count;
	uint8_t window_index;
	uint16_t window[B3_CAL_BLOCK_SAMPLES];
} B3_LINE_CAL;

void b3_line_cal_init(B3_LINE_CAL *cal);
/* Scale measured high-is-black endpoints; invalid ranges return off-line. */
uint16_t b3_line_cal_scale(uint16_t raw, uint16_t white, uint16_t black);
/* True only after confirmed observations span the minimum contrast. */
bool b3_line_cal_ready(const B3_LINE_CAL *cal);
/* Learn from one raw 12-bit reading and return it normalised 0..1000. */
uint16_t b3_line_cal_update(B3_LINE_CAL *cal, uint16_t raw);

#endif
