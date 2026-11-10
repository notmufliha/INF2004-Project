/**
 * @file b3_line_cal.h
 * @brief Per-sensor auto-calibration of the analogue IR line sensors.
 *
 * Each sensor learns its own white (lowest) and black (highest) raw ADC
 * reading and reports a normalised value: 0 = its white, 1000 = its black.
 * Comparing normalised values removes the bias between two sensors with
 * different sensitivity, which with raw values looks like a permanent
 * "line on one side" and steers the car off the course.
 *
 * The range widens immediately to any lighter or darker reading and slowly
 * forgets old extremes (B3_CAL_FORGET_PER_SAMPLE per sample) so it settles
 * on the real course, but never narrows below B3_CAL_MIN_SPAN.
 */
#ifndef B3_LINE_CAL_H
#define B3_LINE_CAL_H

#include <stdint.h>

/* Starting range until the sensor has seen the course.  Its midpoint is
 * the team's tested raw threshold (300).  The range starts NARROW on
 * purpose: widening to a darker or lighter reading is instant, while
 * narrowing is slow, so a too-wide default (e.g. black 3000) made sensors
 * whose black reads ~1000 report LINE LOST for tens of seconds. */
#define B3_CAL_DEFAULT_WHITE (100U)
#define B3_CAL_DEFAULT_BLACK (500U)
/* Smallest raw span kept between learned white and black. */
#define B3_CAL_MIN_SPAN (300U)
#define B3_CAL_FORGET_PER_SAMPLE (1U)
#define B3_CAL_FULL_SCALE (1000U)

typedef struct {
	uint16_t white;
	uint16_t black;
} B3_LINE_CAL;

void b3_line_cal_init(B3_LINE_CAL *cal);
/* Learn from one raw 12-bit reading and return it normalised 0..1000. */
uint16_t b3_line_cal_update(B3_LINE_CAL *cal, uint16_t raw);

#endif
