/*
 * Host test for app_program/subsystem3/b3_line_cal.c (no Pico needed).
 *
 *   S3-C01  two sensors with different sensitivity read the same surface as
 *           the same normalised value (no false "line on one side" bias)
 *   S3-C02  a grey floor beside the course, which a fixed raw > 300 rule
 *           called "on the line", reads as off the line after learning
 *   S3-C03  sitting still on black never collapses the range below
 *           B3_CAL_MIN_SPAN, and white is recognised again at once
 *   S3-C04  output stays within 0..1000 for any 12-bit input
 *
 * Build: gcc -std=c11 -Wall -Wextra -Werror -Iapp_program/subsystem3
 *        testing/subsystem3/line_cal_test.c app_program/subsystem3/b3_line_cal.c
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "b3_line_cal.h"

/* Car weaving across the line: each sensor sees its white and black. */
static void sweep(B3_LINE_CAL *cal, uint16_t white, uint16_t black,
		  unsigned cycles)
{
	for (unsigned i = 0U; i < cycles; ++i) {
		for (unsigned step = 0U; step <= 20U; ++step) {
			unsigned phase = step <= 10U ? step : 20U - step;
			uint16_t raw = (uint16_t)(white +
				(black - white) * phase / 10U);
			(void)b3_line_cal_update(cal, raw);
		}
	}
}

int main(void)
{
	B3_LINE_CAL left;
	B3_LINE_CAL centre;
	uint16_t a;
	uint16_t b;

	/* Left sensor is "hotter": white 700, black 3500.
	 * Centre sensor: white 200, black 2400. */
	b3_line_cal_init(&left);
	b3_line_cal_init(&centre);
	sweep(&left, 700U, 3500U, 40U);
	sweep(&centre, 200U, 2400U, 40U);

	/* Both on white: raw differ by 500, normalised nearly equal. */
	a = b3_line_cal_update(&left, 700U);
	b = b3_line_cal_update(&centre, 200U);
	assert(a <= 60U && b <= 60U && abs((int)a - (int)b) <= 60);
	/* Both half over the line edge: equal normalised readings. */
	a = b3_line_cal_update(&left, 2100U);
	b = b3_line_cal_update(&centre, 1300U);
	assert(abs((int)a - (int)b) <= 60);
	/* Both on black. */
	a = b3_line_cal_update(&left, 3500U);
	b = b3_line_cal_update(&centre, 2400U);
	assert(a >= 940U && b >= 940U);
	puts("PASS S3-C01 sensitivity difference removed");

	/* Grey floor off the course reads 1200 raw on the left sensor. */
	a = b3_line_cal_update(&left, 1200U);
	assert(a < 500U);
	puts("PASS S3-C02 grey floor is off the line");

	/* Parked on black for 60 s (3000 samples at 20 ms). */
	for (unsigned i = 0U; i < 3000U; ++i) {
		(void)b3_line_cal_update(&left, 3500U);
	}
	assert((unsigned)left.black - left.white >= B3_CAL_MIN_SPAN);
	assert(b3_line_cal_update(&left, 3500U) == B3_CAL_FULL_SCALE);
	assert(b3_line_cal_update(&left, 700U) == 0U);
	puts("PASS S3-C03 range kept while parked on black");

	b3_line_cal_init(&left);
	for (unsigned raw = 0U; raw <= 0x0FFFU; raw += 7U) {
		assert(b3_line_cal_update(&left, (uint16_t)raw) <=
		       B3_CAL_FULL_SCALE);
	}
	puts("PASS S3-C04 output bounded");
	return 0;
}
