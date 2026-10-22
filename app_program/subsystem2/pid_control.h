#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "../app.h"

extern ID tid_pid;
// extern ER subsystem2_stop_pid_cyclic(void);

void pid_cyclic_handler(void *exinf);

/* Initialise the PID controller. */
void pid_init(void);

/* Temporary closed-loop motor test task. */
void pid_control_task(INT stacd, void *exinf);

#endif
