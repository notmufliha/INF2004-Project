/** @file s4_runtime.h
 * @brief Private B4 task configuration and kernel object declarations.
 */
#ifndef S4_RUNTIME_H
#define S4_RUNTIME_H

#include "../app.h"
#include "../robot_config.h"

/* Default: real GY-511. Set to 1 explicitly for the scripted mock demo. */
#ifndef S4_IMU_USE_MOCK
#define S4_IMU_USE_MOCK 0
#endif

/* Enable only when diagnosing raw magnetic readings. */
#ifndef S4_IMU_LOG_RAW_MAG
#define S4_IMU_LOG_RAW_MAG 0
#endif

#define S4_PRI_IMU_SAMPLE 18
#define S4_PRI_IMU_ANALYSIS 19
#define S4_IMU_PERIOD_MS 10U
#define S4_IMU_QUEUE_DEPTH 8
/* Kernel message buffer uses a four-byte length header and aligned payload. */

extern volatile ID gh_imu_data_buffer;
#if APP_TEST
/* Test-only event tap; not a Vehicle Controller interface. */
extern volatile ID gh_imu_test_events;
#endif

#define S4_PRI_MONITOR 19
#define S4_ALARM_MS 2000
#define S4_LED_TOGGLE_REPORTS 6U
#define FLG_ALARM 0x0001U /* Set by the private alarm callback. */
#define S4_IMU_READY_FLAG 0x0004U /* Fresh, accepted analysis is available. */

/* Private to subsystem 4. */
extern volatile ID gh_monitor_alarm;

#endif

/*** end of file ***/

