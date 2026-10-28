/** @file encoder_control.h
 * @brief Subsystem 2 encoder control interface.
 */

#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "../app.h"

ER encoder_init(void);

void encoder_get_counts(INT *p_left, INT *p_right);
/* Wheel geometry and encoder resolution. */
#define ENC_WHEEL_DIAMETER_CM     6.5f

/*
 * Distance between the centres of the left and right wheels.
 * Preliminary measurement: 9.7 cm.
 * Calibrate using actual 90-degree turns.
 */
#define ENC_WHEEL_TRACK_CM       9.7f

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
