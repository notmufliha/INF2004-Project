/** @file encoder_control.h
 * @brief Subsystem 2 encoder control interface.
 */

#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "../app.h"

ER encoder_init(void);

void encoder_get_counts(INT *p_left, INT *p_right);

void encoder_test_task(INT stacd, void *p_exinf);

void encoder_speed_task(INT stacd, void *p_exinf);
#endif
