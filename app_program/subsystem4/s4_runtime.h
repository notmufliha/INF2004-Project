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

/* Keep fresh safety snapshots ahead of telemetry and line diagnostics.
 * Motor control (5), VC (6) and obstacle sensing (7) retain precedence. */
#if ROBOT_MODE
#define S4_PRI_IMU_SAMPLE 8
#define S4_PRI_IMU_ANALYSIS 9
#else
#define S4_PRI_IMU_SAMPLE 18
#define S4_PRI_IMU_ANALYSIS 19
#endif
#define S4_IMU_PERIOD_MS 10U
#define S4_IMU_QUEUE_DEPTH 8

/* Measured centre-to-centre tyre spacing, not the gap between tyres. */
#ifndef S4_WHEEL_TRACK_MM
#define S4_WHEEL_TRACK_MM 110.0f
#endif

/* Provisional roll thresholds; validate against flat-ground noise. */
#ifndef S4_ROLL_HUMP_ENTER_DEG
#define S4_ROLL_HUMP_ENTER_DEG 4.0f
#endif
#ifndef S4_ROLL_HUMP_LEVEL_DEG
#define S4_ROLL_HUMP_LEVEL_DEG 2.0f
#endif
#ifndef S4_ROLL_HUMP_TREND_DEG
#define S4_ROLL_HUMP_TREND_DEG 0.5f
#endif
/* Flat-ground false humps: roll must hold ENTER_DEG this long (steering
 * jerks only last a few 10 ms samples).  Raise if level runs still report
 * humps; lower if a real hump is missed (keep below the crossing time). */
#ifndef S4_ROLL_HUMP_ENTER_MS
#define S4_ROLL_HUMP_ENTER_MS 80U
#endif
/* No new hump while |left - right| wheel speed > this share of the faster
 * wheel: the curve's sideways acceleration reads as roll.  0 disables. */
#ifndef S4_ROLL_HUMP_CURVE_RATIO
#define S4_ROLL_HUMP_CURVE_RATIO 0.3f
#endif
/* Hump height peak: trimmed mean of this many unfiltered rolls (10 ms each),
 * dropping jolts of up to a third of the window.  Lower it if narrow humps
 * read low (the wheel must sit on top for about half the window). 1..9. */
#ifndef S4_ROLL_HUMP_PEAK_SAMPLES
#define S4_ROLL_HUMP_PEAK_SAMPLES 7U
#endif

/* Lift-stop (imu_lift.h): a pick-up must reach this upward speed and rise.
 * Lower them if lifting the car is not caught ("[B4] LIFTED" not printed);
 * raise them if the car stops while driving on the floor. */
#ifndef S4_LIFT_SPEED_MPS
#define S4_LIFT_SPEED_MPS 0.25f
#endif
#ifndef S4_LIFT_RISE_M
#define S4_LIFT_RISE_M 0.03f
#endif

/* Prioritize hump sampling; 0 restores full magnetic reads and their delays. */
#ifndef S4_IMU_MAG_IDLE_ONLY
#define S4_IMU_MAG_IDLE_ONLY 1
#endif
#if (S4_IMU_MAG_IDLE_ONLY != 0) && (S4_IMU_MAG_IDLE_ONLY != 1)
#error S4_IMU_MAG_IDLE_ONLY must be 0 or 1
#endif

/* Optional physical integration test; disabled for controller operation.
 * Enable only for isolated real robot APP_TEST trials with a clear route.
 */
#ifndef S4_MOTION_TEST_ENABLE
#define S4_MOTION_TEST_ENABLE 0
#endif
#define S4_MOTION_TEST_DISTANCE_CM 50U
#define S4_MOTION_TEST_SPEED_CPS 500
#define S4_MOTION_TEST_BOOT_MS 5000U
#define S4_MOTION_TEST_ACTIVE                                                  \
    (S4_MOTION_TEST_ENABLE && ROBOT_MODE && APP_TEST && (!S4_IMU_USE_MOCK))
/* Kernel message buffer uses a four-byte length header and aligned payload. */

extern volatile ID gh_imu_data_buffer;
#if APP_TEST
/* Test-only event tap; not a Vehicle Controller interface. */
extern volatile ID gh_imu_test_events;
#endif

#define S4_PRI_MONITOR 19
#define S4_ALARM_MS 2000
#define S4_LED_TOGGLE_REPORTS 6U
#define FLG_ALARM 0x0001U         /* Set by the private alarm callback. */
#define S4_IMU_READY_FLAG 0x0004U /* Fresh, accepted analysis is available. */
#define S4_RESET_PEAK_FLAG 0x0008U

/* Private to subsystem 4. */
extern volatile ID gh_monitor_alarm;

#endif

/*** end of file ***/

