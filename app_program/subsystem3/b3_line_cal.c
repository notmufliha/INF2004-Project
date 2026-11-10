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

	/* Widen at once to anything lighter or darker than seen so far. */
	if (raw < cal->white) {
		cal->white = raw;
	}
	if (raw > cal->black) {
		cal->black = raw;
	}

	/* Slowly forget old extremes, keeping at least B3_CAL_MIN_SPAN. */
	if (raw > cal->white &&
	    (uint32_t)cal->black - cal->white >=
		    B3_CAL_MIN_SPAN + B3_CAL_FORGET_PER_SAMPLE) {
		cal->white = (uint16_t)(cal->white + B3_CAL_FORGET_PER_SAMPLE);
	}
	if (raw < cal->black &&
	    (uint32_t)cal->black - cal->white >=
		    B3_CAL_MIN_SPAN + B3_CAL_FORGET_PER_SAMPLE) {
		cal->black = (uint16_t)(cal->black - B3_CAL_FORGET_PER_SAMPLE);
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
