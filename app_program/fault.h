/** @file fault.h
 * @brief The shared fault bitmask functions (bits: fault_flags.h).
 * Implemented by Subsystem 1 (fault.c).
 *
 * The bitmask is a micro T-Kernel EVENT FLAG (flg_faults), not a plain
 * variable.  tk_set_flg / tk_clr_flg change the pattern inside the
 * kernel, so two tasks (or a task and an ISR) setting different bits at
 * the same time never lose each other's update, as a plain
 * "faults |= bit" read-modify-write could.  All three functions may be
 * called from an interrupt handler and never block.
 *
 * Owner: the Vehicle Controller (Week 8 Tribunal decision).  Subsystem 1
 * creates flg_faults in subsystem1_init(), which app_main.c runs before
 * any other init.  Subsystem 1 sends the whole mask in every telemetry
 * frame.
 *
 * Needs only <stdint.h> and the kernel's ID type, so host tests can
 * compile it against a mock kernel.
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include "app.h"
#include "fault_flags.h"

/* Event-flag handle; name kept for the team, B2 waits on it (DEV-1). */
extern ID flg_faults;

void    fault_raise (uint32_t bits);
void    fault_clear (uint32_t bits);
uint8_t fault_get (void); /* current mask, 0 if not created yet */

#endif /* FAULT_H */

/*** end of file ***/

