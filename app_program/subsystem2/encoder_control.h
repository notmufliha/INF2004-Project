/** @file encoder_control.h
 * @brief Subsystem 2 encoder control interface.
 */

#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "../app.h"

ER encoder_init(void);

void encoder_get_counts(INT *p_left, INT *p_right);
/* Cumulative impossible A/B transitions; read atomically as a pair. */
void encoder_get_invalid_counts(UW *left, UW *right);
/* Wheel geometry and encoder resolution. */
#define ENC_WHEEL_DIAMETER_CM     6.5f

/*
 * Distance between the centres of the left and right wheels.
 * Preliminary measurement: 9.7 cm.
 * Calibrate using actual 90-degree turns.
 */
#define ENC_WHEEL_TRACK_CM       9.7f

/* Provisional turn-only correction: initial scale = 90/70.
 * Left scale 1.3932155 produced a reported approximately 100 degrees.
 * Current left trial = 1.3932155 * 90/100.
 * Right retains the initial scale after reported improvement.
 * Re-measure completed left/right turns on the same surface and load.
 * New scale = old scale * requested angle / measured angle.
 * Never tune these values from a cancelled, timed-out or faulted turn.
 */
#define ENC_TURN_LEFT_SCALE      1.2538940f
#define ENC_TURN_RIGHT_SCALE     1.2857143f

/* Preliminary encoder calibration values. */
#define ENC_LEFT_COUNTS_PER_REV  2499
#define ENC_RIGHT_COUNTS_PER_REV 2341

/* Calculate signed wheel travel from encoder count differences. */
void encoder_get_distance_cm(
    INT left_start,
    INT right_start,
    float *p_left_cm,
    float *p_right_cm
);

void encoder_test_task(INT stacd, void *p_exinf);

void encoder_speed_task(INT stacd, void *p_exinf);
#endif
