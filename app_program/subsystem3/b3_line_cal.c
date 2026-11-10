/**
 * @file b3_line_cal.c
 * @brief Per-sensor auto-calibration of the analogue IR line sensors.
 */
#include "b3_line_cal.h"

#include <stddef.h>

void b3_line_cal_init(B3_LINE_CAL *cal)
{
	if (cal != NULL) {
		cal->white = (uint16_t)B3_CAL_DEFAULT_WHITE;
		cal->black = (uint16_t)B3_CAL_DEFAULT_BLACK;
		cal->observed_min = 0x0FFFU;
		cal->observed_max = 0U;
	}
}

uint16_t b3_line_cal_update(B3_LINE_CAL *cal, uint16_t raw)
{
	uint32_t span;
	uint32_t value;

	if (cal == NULL) {
		return 0U;
	}
	raw &= 0x0FFFU;

	if (raw < cal->observed_min) cal->observed_min = raw;
	if (raw > cal->observed_max) cal->observed_max = raw;

	/* Learn from real observations rather than slowly migrating the
	 * defaults. Never shrink a learned range: a long bend or a pause on
	 * grey must not turn the same physical surface into a different error. */
	if ((uint32_t)cal->observed_max - cal->observed_min >= B3_CAL_MIN_SPAN) {
		cal->white = cal->observed_min;
		cal->black = cal->observed_max;
	} else {
		/* Provisional interpretation until both surfaces have been seen. */
		if (raw < cal->white) cal->white = raw;
		if (raw > cal->black) cal->black = raw;
	}

	span = (uint32_t)cal->black - cal->white;
	if (span == 0U || raw <= cal->white) {
		return 0U;
	}
	if (raw >= cal->black) {
		return (uint16_t)B3_CAL_FULL_SCALE;
	}
	value = ((uint32_t)raw - cal->white) * B3_CAL_FULL_SCALE / span;
	return (uint16_t)value;
}
