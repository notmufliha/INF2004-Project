/** @file subsystem4.h
 * @brief Buddy 4 subsystem4 interface.
 */
/*
 *  subsystem4.h
 *  Subsystem 4 -- IMU mock RTOS demo + legacy monitor. Student ID: 2503027.
 *
 *  Public interface of subsystem 4.  Other subsystems include it as
 *  "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no terminating NUL
   needed) to gh_summary, then set FLG_BATCH in gh_events. */
#define S4_SUMMARY_MAX 48
#define FLG_BATCH 0x0002U /* Set by subsystem 3. */

extern volatile ID gh_summary;
extern volatile ID gh_events;

ER subsystem4_init (void);
ER subsystem4_start (void);

#endif /* SUBSYSTEM4_H. */

/*** end of file ***/

