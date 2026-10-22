#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "../app.h"

ER encoder_init(void);
void encoder_get_counts(INT *left, INT *right);
void encoder_test_task(INT stacd, void *exinf);
void encoder_speed_task(INT stacd, void *exinf);

#endif
