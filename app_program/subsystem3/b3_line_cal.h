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
 * Sweep each sensor over white and black before driving. Once the observed
 * contrast reaches B3_CAL_MIN_SPAN, use the measured endpoints. Endpoints
 * widen when new extremes are observed; they never drift while parked on
 * one surface. Reboot and repeat the sweep when lighting or the course changes.
 */
#ifndef B3_LINE_CAL_H
#define B3_LINE_CAL_H

#include <stdint.h>

/* Provisional range before a white/black sweep. One stationary surface
 * cannot establish sensor polarity or calibrated contrast. */
#define B3_CAL_DEFAULT_WHITE (100U)
#define B3_CAL_DEFAULT_BLACK (500U)
/* Requiring 300 raw counts prevented low-output sensors learning from a
 * sweep. Verify this minimum contrast exceeds actual course noise. */
#define B3_CAL_MIN_SPAN (64U)
#define B3_CAL_FULL_SCALE (1000U)

typedef struct {
	uint16_t white;
	uint16_t black;
	uint16_t observed_min;
	uint16_t observed_max;
} B3_LINE_CAL;

void b3_line_cal_init(B3_LINE_CAL *cal);
/* Learn from one raw 12-bit reading and return it normalised 0..1000. */
uint16_t b3_line_cal_update(B3_LINE_CAL *cal, uint16_t raw);

#endif
