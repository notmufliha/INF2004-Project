/** @file task_monitor.h
 * @brief Buddy 4 task monitor interface.
 */
#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "../app.h"
#include <stdint.h>

uint32_t task_monitor_line_count (void);
void     task_monitor_task (INT stacd, void * p_exinf);
ID       task_monitor_create_alarm (void);

#endif

/*** end of file ***/

