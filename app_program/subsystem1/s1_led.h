/** @file s1_led.h
 * @brief Subsystem 1 link-status LED on GP19 (design element D1.4).
 */

#ifndef S1_LED_H
#define S1_LED_H

#include <stdint.h>
#include "comm.h"

/* Task entry point, started by subsystem1_start() at S1_PRI_LED. */
void s1_led_task (INT stacd, void * p_exinf);

/* Cyclic handler, every S1_TICK_MS (interrupt context). */
void s1_led_cyclic_isr (void * p_exinf);

/* LED level (0/1) for a link state at a given tick (pure). */
uint32_t s1_led_level (comm_link_state_t state, uint32_t tick);

#endif /* S1_LED_H */

/*** end of file ***/

