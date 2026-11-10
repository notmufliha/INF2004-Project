/**
 * @file b3_line_cal.c
 * @brief Per-sensor auto-calibration of the analogue IR line sensors
 *        (Pololu QTRSensors scheme, see b3_line_cal.h).
 */

#include "b3_line_cal.h"

#include <stddef.h>

static void block_reset(B3_LINE_CAL *cal)
{
	cal->block_min = 0x0FFFU;
	cal->block_max = 0U;
	cal->block_count = 0U;
	cal->window_index = 0U;
	for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES; ++i) cal->window[i] = 0U;
}

void b3_line_cal_init(B3_LINE_CAL *cal)
{
	if (cal != NULL) {
		/* Nothing learned yet: min starts at full scale, max at 0. */
		cal->white = 0x0FFFU;
		cal->black = 0U;
		block_reset(cal);
	}
}

bool b3_line_cal_ready(const B3_LINE_CAL *cal)
{
	return cal != NULL && cal->black > cal->white &&
		(uint32_t)cal->black - cal->white >= B3_CAL_MIN_SPAN;
}

uint16_t b3_line_cal_update(B3_LINE_CAL *cal, uint16_t raw)
{
	uint32_t white;
	uint32_t black;

	if (cal == NULL) {
		return 0U;
	}
	raw &= 0x0FFFU;

	/* Confirm extremes with a rolling window, rather than disjoint blocks
	 * that can split three real surface samples across two blocks. */
	cal->window[cal->window_index] = raw;
	cal->window_index = (uint8_t)((cal->window_index + 1U) % B3_CAL_BLOCK_SAMPLES);
	if (cal->block_count < B3_CAL_BLOCK_SAMPLES) ++cal->block_count;
	if (cal->block_count >= B3_CAL_BLOCK_SAMPLES) {
		cal->block_min = 0x0FFFU;
		cal->block_max = 0U;
		for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES; ++i) {
			if (cal->window[i] < cal->block_min) cal->block_min = cal->window[i];
			if (cal->window[i] > cal->block_max) cal->block_max = cal->window[i];
		}
		if (cal->block_min > cal->black) {
			cal->black = cal->block_min;
		}
		if (cal->block_max < cal->white) {
			cal->white = cal->block_max;
		}
	}

	if (b3_line_cal_ready(cal)) {
		white = cal->white;
		black = cal->black;
	} else {
		white = B3_CAL_DEFAULT_WHITE;
		black = B3_CAL_DEFAULT_BLACK;
	}

	/* Pololu readCalibrated(): scale to 0..1000 and clamp. */
	if (raw <= white) {
		return 0U;
	}
	if (raw >= black) {
		return (uint16_t)B3_CAL_FULL_SCALE;
	}
	return (uint16_t)(((uint32_t)raw - white) * B3_CAL_FULL_SCALE /
			  (black - white));
}
