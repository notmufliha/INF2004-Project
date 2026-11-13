/** @file pid_control.h
 * @brief Subsystem 2 pid control interface.
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "../app.h"

/* Temporary straight-drive diagnosis; set 0 and rebuild to disable. */
#ifndef B2_SPEED_DIAGNOSTICS
#define B2_SPEED_DIAGNOSTICS 1
#endif
#define B2_SPEED_LOG_MS 500u

extern ID gh_pid_task;

void pid_cyclic_handler(void *p_exinf);
/* Initialise the PID controller. */

void pid_init(void);
/* Production motion execution task. */

void pid_control_task(INT stacd, void *p_exinf);
#endif
