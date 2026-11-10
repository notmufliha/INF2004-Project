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

/* Starting range until the sensor has seen the course. */
#define B3_CAL_DEFAULT_WHITE (300U)
#define B3_CAL_DEFAULT_BLACK (3000U)
/* Smallest raw span kept between learned white and black. */
#define B3_CAL_MIN_SPAN (800U)
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
