#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "../app.h"

/* Initialise the PID controller. */
void pid_init(void);

/* Temporary closed-loop motor test task. */
void pid_test_task(INT stacd, void *exinf);

#endif
