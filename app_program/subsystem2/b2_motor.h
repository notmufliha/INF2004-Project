/** @file b2_motor.h
 * @brief Subsystem 2 b2 motor interface.
 */

#ifndef B2_MOTOR_H
#define B2_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "subsystem2.h"

/*
 * Buddy 5 -> Buddy 2 safety requests.
 */

void b2_request_stop(void);

void b2_request_forward(void);

bool b2_motor_is_stopped(void);
/*
 * Buddy 2 safety interface.
 */

ER b2_safety_init(void);
/*
 * Called by the PI task to determine whether
 * movement is currently permitted.
 *
 * A clear path alone does not authorize DRIVE.
 */

bool b2_safety_motion_allowed(void);
/*
 * Called after the PI task has applied zero PWM.
 */

void b2_safety_acknowledge_stop(void);
/*
 * Apply zero PWM while holding the safety lock.
 * Does not acknowledge a Buddy 5 STOP request.
 */

void b2_safety_force_stop(void);
/*
 * Legacy motor task declaration.
 * The task must not be started in the integrated design.
 */

void b2_motor_task(INT stacd, void *p_exinf);
/*
 * Called only by the PI task.
 * Applies motor outputs while enforcing Buddy 5 safety permission.
 */

void b2_safety_apply_pwm(
    int left_percent,
    int right_percent,
    uint32_t command_safety_generation
);
/*
 * Called by the PI task to determine whether a safety STOP
 * has interrupted its active DRIVE command.
 */

bool b2_safety_stop_requested(void);

uint32_t b2_safety_stop_generation(void);
#endif /* B2_MOTOR_H */
