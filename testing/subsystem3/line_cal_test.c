/*
 * Host test for app_program/subsystem3/b3_line_cal.c (no Pico needed).
 *
 *   S3-PD-C01  two sensors with different sensitivity read the same surface as
 *           the same normalised value (no false "line on one side" bias)
 *   S3-PD-C02  a grey floor beside the course, which a fixed raw > 300 rule
 *           called "on the line", reads as off the line after learning
 *   S3-PD-C03  sitting still on black never collapses the range below
 *           B3_CAL_MIN_SPAN, and white is recognised again at once
 *   S3-PD-C04  output stays within 0..1000 for any 12-bit input
 *   S3-PD-C05  line seen from the first black sample, before any learning
 *   S3-PD-C06  long time on white does NOT shrink the range (no drift): floor
 *           noise stays below the "on line" level (Pololu: range only widens)
 *   S3-PD-C07  a single noisy spike does not widen the learned range
 *
 * Build: gcc -std=c11 -Wall -Wextra -Werror -Iapp_program/subsystem3
 *        testing/subsystem3/line_cal_test.c app_program/subsystem3/b3_line_cal.c
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "b3_line_cal.h"

/* B3 "on the line" level (B3_LINE_ACTIVE_MIN, Pololu readLine > 200). */
#define ON_LINE (201U)

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
	puts("PASS S3-PD-C01 sensitivity difference removed");

	/* Grey floor off the course reads 1200 raw on the left sensor. */
	a = b3_line_cal_update(&left, 1200U);
	assert(a < ON_LINE);
	puts("PASS S3-PD-C02 grey floor is off the line");

	/* Parked on black for 60 s (3000 samples at 20 ms). */
	for (unsigned i = 0U; i < 3000U; ++i) {
		(void)b3_line_cal_update(&left, 3500U);
	}
	assert((unsigned)left.black - left.white >= B3_CAL_MIN_SPAN);
	assert(b3_line_cal_update(&left, 3500U) == B3_CAL_FULL_SCALE);
	assert(b3_line_cal_update(&left, 700U) == 0U);
	puts("PASS S3-PD-C03 range kept while parked on black");

	b3_line_cal_init(&left);
	for (unsigned raw = 0U; raw <= 0x0FFFU; raw += 7U) {
		assert(b3_line_cal_update(&left, (uint16_t)raw) <=
		       B3_CAL_FULL_SCALE);
	}
	puts("PASS S3-PD-C04 output bounded");

	/* S3-PD-C05: low-contrast sensor (white 150, black 800) reads the line
	 * from the very first black sample, without any learning delay. */
	b3_line_cal_init(&left);
	assert(b3_line_cal_update(&left, 150U) < ON_LINE);
	assert(b3_line_cal_update(&left, 800U) >= ON_LINE);
	assert(b3_line_cal_update(&left, 150U) < ON_LINE);
	/* Typical high-contrast sensor (white 250, black 3000), first pass. */
	b3_line_cal_init(&centre);
	assert(b3_line_cal_update(&centre, 250U) < ON_LINE);
	assert(b3_line_cal_update(&centre, 3000U) >= ON_LINE);
	assert(b3_line_cal_update(&centre, 250U) < ON_LINE);
	puts("PASS S3-PD-C05 line seen immediately, low and high contrast");

	/* S3-PD-C06: learn white 400 / black 2000, then drive 2 minutes with
	 * this sensor beside the line on white (6000 samples).  The old
	 * forgetting calibration shrank black to white + 300, so +80 counts of
	 * floor noise or a shadow read as line and steered the car off. */
	b3_line_cal_init(&left);
	sweep(&left, 400U, 2000U, 20U);
	b = left.black;
	assert(b >= 1700U);
	for (unsigned i = 0U; i < 6000U; ++i) {
		(void)b3_line_cal_update(&left, (uint16_t)(400U + (i % 5U) * 20U));
	}
	assert(left.black == b);
	assert(b3_line_cal_update(&left, 480U) < ON_LINE);
	assert(b3_line_cal_update(&left, 2000U) == B3_CAL_FULL_SCALE);
	puts("PASS S3-PD-C06 no drift after a long time on white");

	/* S3-PD-C07: one 4000-count spike between white samples is ignored. */
	b3_line_cal_init(&left);
	sweep(&left, 400U, 2000U, 20U);
	(void)b3_line_cal_update(&left, 4000U);
	for (unsigned i = 0U; i < 10U; ++i) {
		(void)b3_line_cal_update(&left, 400U);
	}
	assert(left.black < 2100U);
	puts("PASS S3-PD-C07 single spike ignored");

	/* S3-PD-C08: every phase of the sampling window must learn a low-output
	 * line within three black samples, not wait for a 300-count span.
	 * These black readings are BELOW the provisional raw threshold. */
	for (unsigned phase = 0U; phase < B3_CAL_BLOCK_SAMPLES; ++phase) {
		b3_line_cal_init(&left);
		for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES + phase; ++i) {
			(void)b3_line_cal_update(&left, 140U);
		}
		for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES; ++i) {
			a = b3_line_cal_update(&left, 260U);
		}
		assert(left.white == 140U && left.black == 260U);
		assert(b3_line_cal_ready(&left));
		assert(a == 1000U && a >= ON_LINE);
		assert(b3_line_cal_update(&left, 200U) == 500U);
		for (unsigned i = 0U; i < 30000U; ++i) {
			assert(b3_line_cal_update(&left, 140U) == 0U);
		}
		assert(left.black == 260U); /* No delayed drift while parked. */
	}
	puts("PASS S3-PD-C08 low-output line learned within three samples at every window phase");

	/* No fixed warmup timer: raw 301 is on-line before any calibration
	 * history; raw 300 is the documented off/on boundary. */
	b3_line_cal_init(&left);
	assert(b3_line_cal_update(&left, 300U) < ON_LINE);
	assert(b3_line_cal_update(&left, 301U) >= ON_LINE);
	assert(b3_line_cal_update(&left, 220U) == 0U);
	puts("PASS S3-PD-C09 provisional detection threshold available on first sample");

	b3_line_cal_init(&left);
	assert(!b3_line_cal_ready(NULL) && !b3_line_cal_ready(&left));
	for (unsigned i = 0U; i < 300U; ++i) {
		(void)b3_line_cal_update(&left, (uint16_t)(140U + (i % 5U)));
	}
	assert(!b3_line_cal_ready(&left)); /* Floor noise is not a full sweep. */
	puts("PASS S3-PD-C10 stationary noise does not claim learned calibration");
	return 0;
}
