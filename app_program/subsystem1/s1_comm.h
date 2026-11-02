/** @file s1_comm.h
 * @brief Subsystem 1 WiFi & Telemetry task (design element D1.2).
 */

#ifndef S1_COMM_H
#define S1_COMM_H

#include <stdint.h>
#include "../app.h"

/* Task entry point, started by subsystem1_start() at S1_PRI_COMM. */
void s1_comm_task (INT stacd, void * p_exinf);

/* Milliseconds since boot from the kernel's operating time (never set
 * back; wraps after ~49 days).  Callable from any task. */
uint32_t s1_comm_now_ms (void);

#endif /* S1_COMM_H */

/*** end of file ***/

