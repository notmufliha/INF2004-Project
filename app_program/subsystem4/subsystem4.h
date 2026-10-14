/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU mock RTOS demo + legacy monitor. Student ID: 2503027.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu_types.h"

#define S4_PRI_IMU_SAMPLE 18
#define S4_PRI_IMU_ANALYSIS 19
#define S4_IMU_PERIOD_MS 10U
#define S4_IMU_QUEUE_DEPTH 8U
/* Kernel message buffer uses a four-byte length header and aligned payload. */
#define S4_MESSAGE_BYTES(type) (4U + ((sizeof(type) + 3U) & ~3U))

IMPORT ID mbf_imu_data;
IMPORT void imu_sampling_task(INT stacd, void *exinf);
IMPORT void imu_analysis_task(INT stacd, void *exinf);
#if APP_TEST
/* Test-only event tap; not a Vehicle Controller interface. */
IMPORT ID mbf_imu_test_events;
#endif

#define S4_PRI_MONITOR		19	/* legacy example, same band as analysis */
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* Private to subsystem 4 */
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
