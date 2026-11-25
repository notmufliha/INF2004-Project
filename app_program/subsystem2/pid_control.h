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

/* Provisional loaded-robot trial, not a completed motor calibration.
 * Applies to forward DRIVE/MOVE; set enable to 0 to restore shared 15% base.
 * PWM still includes the existing speed term, PI and breakaway kick.
 */
#ifndef B2_FORWARD_FEEDFORWARD_TRIAL
#define B2_FORWARD_FEEDFORWARD_TRIAL 1
#endif
#ifndef B2_LEFT_BASE_PWM
#define B2_LEFT_BASE_PWM 6
#endif
#ifndef B2_RIGHT_BASE_PWM
#define B2_RIGHT_BASE_PWM 12
#endif
/* Reverse MOVE (barcode rewind): both wheels shared one base and one
 * 22 % floor, and ramp up together, so the stronger left motor started
 * first and ran ahead; the tail swung right (24 Nov 19:12 rewind: left
 * -1280 vs right -980 counts/s at the start; at steady speed left 7 %
 * matched right 12 %). Shift the drive by this many PWM points between
 * the wheels: half off the left, half onto the right. The speed loop and
 * slew are unchanged. 0 = off. Raise it if the tail still swings right
 * when reversing, lower it if it swings left. */
#ifndef B2_REVERSE_PWM_SPLIT
#define B2_REVERSE_PWM_SPLIT (B2_RIGHT_BASE_PWM - B2_LEFT_BASE_PWM)
#endif
#if (B2_FORWARD_FEEDFORWARD_TRIAL != 0) && \
    (B2_FORWARD_FEEDFORWARD_TRIAL != 1)
#error B2_FORWARD_FEEDFORWARD_TRIAL must be 0 or 1
#endif
#if (B2_LEFT_BASE_PWM < 0) || (B2_LEFT_BASE_PWM > 35) || \
    (B2_RIGHT_BASE_PWM < 0) || (B2_RIGHT_BASE_PWM > 35)
#error B2 base PWM must be between 0 and 35 percent
#endif

extern ID gh_pid_task;

void pid_cyclic_handler(void *p_exinf);
/* Initialise the PID controller. */

void pid_init(void);
/* Production motion execution task. */

void pid_control_task(INT stacd, void *p_exinf);
#endif
