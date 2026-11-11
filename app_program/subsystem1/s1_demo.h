/** @file s1_demo.h
 * @brief Subsystem 1 standalone demo: a fake robot for bench tests
 * (only when built with S1_STANDALONE_DEMO=1).
 */

#ifndef S1_DEMO_H
#define S1_DEMO_H

#include "../app.h"

/* Starts the demo task when S1_STANDALONE_DEMO == 1; otherwise does
 * nothing.  Returns E_OK or the task creation error. */
ER s1_demo_start (void);

/* Task entry point (S1_STANDALONE_DEMO == 1 only). */
void s1_demo_task (INT stacd, void * p_exinf);

#endif /* S1_DEMO_H */

/*** end of file ***/

