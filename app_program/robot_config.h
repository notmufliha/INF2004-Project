#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* REAL ROBOT: Cytron Maker Pi Pico Base two-input motor drivers.
 * Verify board and motor terminal labels before energizing wheels.
 * Rebuild and reflash the UF2 after changing this file.
 */
#define ROBOT_MODE 1
#define ROBOT_ENABLE_DRIVE 0
/* Real autonomous bypass requires working B2 encoder feedback and B3 line
 * sensing, measured robot geometry, and independent path safety checks.
 * The uploaded team project does not contain those working interfaces.
 */
#define ROBOT_ENABLE_BYPASS 0

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
#define ROBOT_MOTOR_WATCHDOG_MS 1200u
#define ROBOT_MOTOR_PERIOD_MS 20u
#define ROBOT_BOOT_DELAY_MS 2000u

/* Optional wheels-UP BOOT motor verification: set 1, rebuild and flash ONLY when
 * both wheels are lifted. Runs L then R then both, and then stops forever.
 * This test intentionally bypasses the ultrasonic safety gate, so NEVER
 * use with wheels on the floor. Restore to 0 before driving on the floor.
 */
#define ROBOT_WHEELS_UP_MOTOR_TEST 0
#define ROBOT_MOTOR_TEST_PWM 500u
#define ROBOT_MOTOR_TEST_PULSE_MS 1500u

/* Robot is stopped whenever a reading is invalid, close, or ambiguous.
 * Close reading stops immediately; 3 close readings trigger scanning.
 * Three readings >= RELEASE_CM allow forward motion again.
 */
/* User-requested near-field echo suppression.  Echoes <= 5 cm are
 * discarded from obstacle profiling AND forward ranging; they are NOT
 * evidence of a clear path. If no other valid measurements remain, stop.
 * WARNING: A real obstacle closer than 5 cm will also be ignored as a
 * distance sample. Never drive unless another independent valid reading
 * confirms a clear path. */
#define ROBOT_IGNORE_ECHO_AT_OR_BELOW_CM 5.0f
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
/* Quiet operational logging. 0: state changes and scan summary only.
 * No diagnostic mode or raw-echo diagnostic stream is compiled. */
#define ROBOT_VERBOSE_SENSOR_LOG 0
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

#if ROBOT_ENABLE_BYPASS
#error "Physical bypass needs encoder, IR and geometry calibration; not yet available in uploaded project."
#endif
#endif
