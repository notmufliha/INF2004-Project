#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* REAL ROBOT: Cytron Maker Pi Pico Base two-input motor drivers.
 * Verify board and motor terminal labels before energizing wheels.
 * Rebuild and reflash the UF2 after changing this file.
 */
#ifndef ROBOT_MODE
#define ROBOT_MODE 1
#endif
/* Keep the integrated B4 calibration gate. */
#define ROBOT_IMU_STARTUP_TIMEOUT_MS 30000
#ifndef ROBOT_AUTOSTART
#define ROBOT_AUTOSTART 1
#endif
/* VC follows the line; Buddy 5 owns encoder-controlled bypass/recovery. */
#define ROBOT_FORWARD_STOP_MODE 0 /* VC owns normal line-following DRIVE. */
/* Open-space policy requested for forward driving. NO_RISE means assume
 * no target, NOT verified clearance. A disconnected ECHO wire looks the
 * same and cannot be detected by this single-sensor policy. Set 0 for
 * strict stop-on-missing-echo behavior. See the separate scan policy below. */
#ifndef ROBOT_NO_TARGET_FORWARD
#define ROBOT_NO_TARGET_FORWARD 1
#endif
/* Apply the same explicit open-space assumption to scan boundary rays.
 * A missing/disconnected sensor cannot be distinguished from no target.
 * Set both policies to 0 to require real return echoes throughout. */
#ifndef ROBOT_NO_TARGET_SCAN
#define ROBOT_NO_TARGET_SCAN 1
#endif
#ifndef ROBOT_ENABLE_DRIVE
#define ROBOT_ENABLE_DRIVE 1
#endif
/* Start by following the line, avoid a profiled obstacle, then reacquire
 * the line and resume following. Can be overridden with -D for bench builds. */
#ifndef ROBOT_ENABLE_BYPASS
#define ROBOT_ENABLE_BYPASS 1
#endif
#ifndef ROBOT_BYPASS_CALIBRATED
/* Calibration evidence flag only; enabling the requested autonomous profile
 * does not certify the provisional physical measurements below. */
#define ROBOT_BYPASS_CALIBRATED 0
#endif
/* This is an explicit course constraint, NOT an ultrasonic depth estimate.
 * Autonomous mode requires measured obstacles no deeper than this bound. */
#define ROBOT_BYPASS_MAX_DEPTH_CM 40.0f
#define ROBOT_BYPASS_SPEED_CPS 600
#define ROBOT_BYPASS_REVERSE_CM 8.0f
#define ROBOT_BYPASS_SEARCH_STEP_CM 2.0f
#define ROBOT_BYPASS_SEARCH_EXTRA_CM 10.0f
#define ROBOT_BYPASS_MAX_LEG_CM 150.0f
#define ROBOT_BYPASS_LEG_TIMEOUT_MS 60000u
#define ROBOT_BYPASS_TOTAL_TIMEOUT_MS 180000u
#define ROBOT_BYPASS_LINE_MAX_AGE_MS 100u
#define ROBOT_BYPASS_LINE_CONFIRM 3u
#define ROBOT_BYPASS_LINE_WINDOW_CM 5.0f
#define ROBOT_BYPASS_MIN_TURN_CM 20.0f

/* Visible ultrasonic silhouette only; centimetres. Measure chassis before
 * enabling a navigation bridge. Width beyond MAX is STOP, never blind reverse. */
#define ROBOT_SCAN_SAFE_MIN_DEG 70u
#define ROBOT_SCAN_SAFE_MAX_DEG 110u
#define ROBOT_OBSTACLE_SMALL_CM 10.0f
#define ROBOT_OBSTACLE_MAX_BYPASS_CM 30.0f
#define ROBOT_CHASSIS_HALF_WIDTH_CM 8.0f
#define ROBOT_CHASSIS_HALF_LENGTH_CM 10.0f
#define ROBOT_SENSOR_FORWARD_OFFSET_CM 0.0f
#define ROBOT_OBSTACLE_MARGIN_CM 8.0f

/* Pin pairs for M1 and M2 on Maker Pi Pico Base.
 * M1 forward defaults to GP9, M2 forward defaults to GP10, following the
 * motor_control.c mapping in the team's earlier motor code. If either wheel
 * turns backwards, swap that motor's FWD/REV definitions below.
 */
#define ROBOT_MOTOR_L_FWD_PIN 9u
#define ROBOT_MOTOR_L_REV_PIN 8u
#define ROBOT_MOTOR_R_FWD_PIN 10u
#define ROBOT_MOTOR_R_REV_PIN 11u
#define ROBOT_PWM_TOP 999u
#define ROBOT_FORWARD_PWM 500u /* 50% initial floor drive; calibrate for load and supply */
#define ROBOT_MOTOR_WATCHDOG_MS 350u
#define ROBOT_MOTOR_PERIOD_MS 20u
/* Neutral by default. Nonzero trim disables B2's zero-steering straight
 * correction and made the reported drift worse. Verify sensor/motor
 * mapping before using any measured trim. */
#ifndef ROBOT_LINE_STEER_TRIM_PERMILLE
#define ROBOT_LINE_STEER_TRIM_PERMILLE 0
#endif
#if ROBOT_LINE_STEER_TRIM_PERMILLE < -100 || ROBOT_LINE_STEER_TRIM_PERMILLE > 100
#error "Line steering trim must stay within +/-100 permille"
#endif
#define ROBOT_BOOT_DELAY_MS 2000u

/* Optional wheels-UP BOOT motor verification: set 1, rebuild and flash ONLY when
 * both wheels are lifted. Runs L then R then both, and then stops forever.
 * This test intentionally bypasses the ultrasonic safety gate, so NEVER
 * use with wheels on the floor. Restore to 0 before driving on the floor.
 */
#define ROBOT_WHEELS_UP_MOTOR_TEST 0
#define ROBOT_ENCODER_DIAGNOSTICS 0
/* Guarded B2 command tests: 0=off, 1=forward 10cm, 2=reverse 10cm,
 * 3=straight DRIVE 1s then STOP, 4=left 90deg, 5=right 90deg, 6=all.
 * Requires ROBOT_ENABLE_DRIVE=1 and valid Buddy 5 clearance.
 * One run per boot; any fault/obstacle aborts the entire run.
 */
#ifndef ROBOT_MOTION_TEST
#define ROBOT_MOTION_TEST 0
#endif
#define ROBOT_MOTION_TEST_SPEED_CPS 1000
#define ROBOT_MOTION_TEST_BOOT_MS 5000u
#define ROBOT_MOTOR_TEST_PWM 500u
#define ROBOT_MOTOR_TEST_PULSE_MS 1500u

/* Robot is stopped whenever a reading is invalid, close, or ambiguous.
 * Close reading stops immediately; 3 close readings trigger scanning.
 * Three readings >= RELEASE_CM allow forward motion again.
 */
/* Keep valid near-field echoes: they mean CLOSE, not a missing sensor.
 * The driver separately rejects pulses below its measurable range. */
#define ROBOT_IGNORE_ECHO_AT_OR_BELOW_CM 0.0f
/* Known repeatable echoes in the user's clear-area log (centimetres).
 * Suppress ONLY these narrow bands, rather than treating them as a clear
 * path. WARNING: real objects at these ranges become unobservable to this
 * ultrasonic sensor. A separate validated forward sensor is required to
 * guarantee detection at these distances. The robot stays stopped if no
 * other trustworthy clear reading exists. Set 0 to remove these blind bands.
 */
#define ROBOT_SUPPRESS_REPEATING_ECHO_BANDS 0
#define ROBOT_FALSE_ECHO_NEAR_MIN_CM 5.40f
#define ROBOT_FALSE_ECHO_NEAR_MAX_CM 6.20f
#define ROBOT_FALSE_ECHO_FAR_MIN_CM 23.70f
#define ROBOT_FALSE_ECHO_FAR_MAX_CM 24.70f
/* Three echoes must agree within max(3 cm, 15% of their average). */
#define ROBOT_ECHO_AGREEMENT_MIN_CM 3.0f
#define ROBOT_ECHO_AGREEMENT_FRACTION 0.15f
/* After the servo self-test, allow power and the sensor to settle. */
#define ROBOT_SENSOR_STARTUP_SETTLE_MS 2000u
#define ROBOT_SENSOR_WARMUP_PINGS 3u
/* GPIO reset/retry loop has been REMOVED from Buddy 5 source.
 * Invalid readings keep motors stopped, but do not reinitialize hardware
 * or repeatedly print [S5-RECOVER] messages.
 */
/* B5 raw echo diagnostics on USB CDC, including rejected samples. */
#define ROBOT_VERBOSE_SENSOR_LOG 1
/* Five separated pings, requiring at least 3 agreeing valid distances.
 * This rejects one-off 5.8 cm / 24.2 cm cross-talk echoes. */
#define ROBOT_ECHO_SAMPLES 5u
#define ROBOT_ECHO_REQUIRED_AGREEING 3u
/* One bounded retry if the first 5-ping batch contains too few agreeing echoes.
 * Never interpret an invalid batch as a clear path. */
#define ROBOT_ECHO_BATCH_ATTEMPTS 2u
#define ROBOT_ECHO_BATCH_RETRY_PAUSE_MS 120u
#define ROBOT_ECHO_GAP_AFTER_FAILURE_MS 95u
/* A short second attempt helps transient echoes, never turns a timeout
 * into a fabricated clear distance. */
#define ROBOT_SCAN_RETRY_DELAY_MS 120u
#define ROBOT_STOP_CM 35.0f
#define ROBOT_RELEASE_CM 45.0f
#define ROBOT_CLEAR_CONFIRM 3u
#define ROBOT_STOP_SETTLE_MS 250u

#if ROBOT_ENABLE_BYPASS && (ROBOT_MOTION_TEST || ROBOT_WHEELS_UP_MOTOR_TEST || !ROBOT_MODE)
#error "B5 autonomous mode needs ROBOT_MODE=1 and exclusive motor ownership: disable motor tests."
#endif
#if ROBOT_SCAN_SAFE_MIN_DEG > 70u || ROBOT_SCAN_SAFE_MAX_DEG < 110u || ROBOT_SCAN_SAFE_MAX_DEG > 180u
#error "Servo limits must contain the five-ray 70..110 degree coarse scan."
#endif
#endif
