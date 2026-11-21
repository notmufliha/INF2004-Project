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
/* Loaded RTOS runs have logged 110-ms acquisition gaps. Allow a bounded
 * 250-ms interval consistently in acquisition, processing and analysis. */
#define ROBOT_IMU_MAX_GAP_MS 250U
#ifndef ROBOT_AUTOSTART
#define ROBOT_AUTOSTART 1
#endif
/* Optional bypass bench tuning; normal driving always follows the line. */
#ifndef ROBOT_OBSTACLE_TEST
#define ROBOT_OBSTACLE_TEST 0
#endif
/* Buddy 5 owns encoder-controlled bypass/recovery. */
#define ROBOT_FORWARD_STOP_MODE 0 /* VC owns normal line-following DRIVE. */
/* Open-space policy requested for forward driving. NO_RISE means assume
 * no target, NOT verified clearance. A disconnected ECHO wire looks the
 * same and cannot be detected by this single-sensor policy. Set 0 for
 * strict stop-on-missing-echo behavior. See the separate scan policy below. */
#ifndef ROBOT_NO_TARGET_FORWARD
#define ROBOT_NO_TARGET_FORWARD 1
#endif
/* Completed far/no-target pulses also nominate assumed open space.
 * Unlike NO_TARGET_FORWARD, a missing rise never authorizes movement. */
#ifndef ROBOT_COMPLETED_FAR_ECHO
#define ROBOT_COMPLETED_FAR_ECHO 1
#endif
/* Apply the same explicit open-space assumption to scan boundary rays.
 * A missing/disconnected sensor cannot be distinguished from no target.
 * Set both policies to 0 to require real return echoes throughout. */
#ifndef ROBOT_NO_TARGET_SCAN
#define ROBOT_NO_TARGET_SCAN 0
#endif
#ifndef ROBOT_ENABLE_DRIVE
#define ROBOT_ENABLE_DRIVE 1
#endif
/* Start by following the line, avoid a profiled obstacle, then reacquire
 * the line and resume following. Can be overridden with -D for bench builds. */
#ifndef ROBOT_ENABLE_BYPASS
#define ROBOT_ENABLE_BYPASS 1
#endif
/* Default to the width sweep and guarded bypass. Set to 1 to use the
 * box bypass with its ramp-aware approach and line-width check. */
#ifndef ROBOT_BOX_BYPASS
#define ROBOT_BOX_BYPASS 0
#endif
#ifndef ROBOT_BYPASS_CALIBRATED
/* Calibration evidence flag only; enabling the requested autonomous profile
 * does not certify the provisional physical measurements below. */
#define ROBOT_BYPASS_CALIBRATED 0
#endif
/* Stop bypass on fresh black tape after the first completed outward turn. */
#ifndef ROBOT_BYPASS_LINE_HANDOFF
#define ROBOT_BYPASS_LINE_HANDOFF (!ROBOT_OBSTACLE_TEST)
#endif
/* This is an explicit course constraint, NOT an ultrasonic depth estimate.
 * Autonomous mode requires measured obstacles no deeper than this bound. */
#if ROBOT_OBSTACLE_TEST
#define ROBOT_BYPASS_MAX_DEPTH_CM 20.0f
#else
#define ROBOT_BYPASS_MAX_DEPTH_CM 40.0f
#endif
#define ROBOT_OBSTACLE_APPROACH_SPEED_CPS (ROBOT_OBSTACLE_TEST ? 720 : 540)
#define ROBOT_OBSTACLE_APPROACH_CM (ROBOT_OBSTACLE_TEST ? 30.0f : 80.0f)
#define ROBOT_BYPASS_SPEED_CPS (ROBOT_OBSTACLE_TEST ? 720 : 680)
#define ROBOT_BYPASS_TURN_SPEED_CPS (ROBOT_OBSTACLE_TEST ? 450 : 560)
#define ROBOT_COURSE_RETURN_SPEED_CPS 620
#define ROBOT_BYPASS_FORWARD_MS 5000u
#define ROBOT_BYPASS_EXTRA_DIAGONAL_CM 10.0f
#define ROBOT_COURSE_BYPASS_TURN_DEG 55
#define ROBOT_COURSE_PASS_TURN_DEG 40
#define ROBOT_COURSE_RETURN_TURN_DEG 40
#define ROBOT_COURSE_PASS_CM 15.0f
#define ROBOT_COURSE_CLEAR_STEP_CM 5.0f
#define ROBOT_COURSE_BYPASS_LEG_CM 25.0f
#define ROBOT_COURSE_BOX_DEPTH_CM 20.0f
#define ROBOT_BYPASS_SIDE_TIE_CM 0.5f
/* Fixed test route: if the first diagonal is blocked after meaningful travel,
 * take the scheduled opposite turn once rotation clearance is confirmed. */
#define ROBOT_BYPASS_BLOCKED_FIRST_MIN_CM 20.0f
#define ROBOT_BYPASS_DISTANCE_MODE ROBOT_OBSTACLE_TEST
#ifndef ROBOT_BYPASS_EXACT_SEQUENCE
#define ROBOT_BYPASS_EXACT_SEQUENCE ROBOT_OBSTACLE_TEST
#endif
/* Current test: 20 x 20 cm box centred ahead. Disable for unknown/off-centre
 * placement to retain the conservative full-width fallback. */
#ifndef ROBOT_BYPASS_CENTERED_TEST_BOX
#define ROBOT_BYPASS_CENTERED_TEST_BOX ROBOT_BYPASS_EXACT_SEQUENCE
#endif
#define ROBOT_BYPASS_RECOVERY_TURN_DEG ROBOT_BYPASS_TURN_DEG
#define ROBOT_BYPASS_TURN_DEG 40
/* Bypass offset uses encoder distance: obstacle edge + chassis + margin. */
/* Straight forward DRIVE trim: positive counters left drift by requesting
 * slightly more left-wheel travel and less right-wheel travel. Adjust from
 * measured floor runs. Latest trial: 90 (+9% left / -9% right), reduced
 * from 100 after reported right drift; earlier 80 drifted left. */
#ifndef ROBOT_STRAIGHT_TRIM_PERMILLE
#define ROBOT_STRAIGHT_TRIM_PERMILLE -20
#endif
#if ROBOT_STRAIGHT_TRIM_PERMILLE < -150 || ROBOT_STRAIGHT_TRIM_PERMILLE > 150
#error "Straight-drive trim must be between -150 and 150 permille."
#endif
#define ROBOT_BYPASS_REVERSE_CM 8.0f
/* Reverse wheel balance starts from the forward calibration. Tune separately
 * from measured reverse runs if gearbox/load asymmetry differs by direction.
 * Positive requests more left-wheel travel; negative more right-wheel travel. */
#ifndef ROBOT_REVERSE_TRIM_PERMILLE
#define ROBOT_REVERSE_TRIM_PERMILLE ROBOT_STRAIGHT_TRIM_PERMILLE
#endif
#if ROBOT_REVERSE_TRIM_PERMILLE < -150 || ROBOT_REVERSE_TRIM_PERMILLE > 150
#error "Reverse trim must be between -150 and 150 permille."
#endif
#define ROBOT_BYPASS_MIDDLE_DISTANCE_SCALE 1.0f
#define ROBOT_BYPASS_MIDDLE_MAX_CM 22.0f
#define ROBOT_BYPASS_DIAGONAL_DISTANCE_SCALE 0.80f
/* User-requested bounded retreat for floor tests; no rear sensor fitted.
 * Operator must keep rear space clear. Normal operation requires rear sensing. */
#ifndef ROBOT_TEST_BOUNDED_REVERSE
#define ROBOT_TEST_BOUNDED_REVERSE ROBOT_OBSTACLE_TEST
#endif
#define ROBOT_BYPASS_SEARCH_STEP_CM 2.0f
/* Search across the predicted line and up to 40 cm beyond it, to allow
 * a curved/displaced continuation. This remains a bounded recovery. */
#define ROBOT_BYPASS_SEARCH_EXTRA_CM 40.0f
#define ROBOT_BYPASS_MAX_LEG_CM 150.0f
#define ROBOT_BYPASS_LEG_TIMEOUT_MS 60000u
#define ROBOT_BYPASS_TOTAL_TIMEOUT_MS 180000u
#define ROBOT_BYPASS_LINE_MAX_AGE_MS 100u
#define ROBOT_BYPASS_LINE_CONFIRM 3u
/* After restoring forward heading, observe the line then scan headings
 * +/-13, +/-26, +/-39 degrees if the sensors rotated off the tape. */
#define ROBOT_BYPASS_ALIGN_OBSERVE_MS 300u
#define ROBOT_BYPASS_ALIGN_STEP_DEG 13
#define ROBOT_BYPASS_ALIGN_SWEEPS 6u
/* At 30 degrees, the configured 10 x 8 cm half-chassis sweeps forward
 * to 10*cos(30)+8*sin(30) = 12.66 cm. Retain 0.44 cm margin for the
 * requested close-range test. Full-angle normal mode keeps its 14 cm guard.
 * Geometry is provisional; this does not certify actual corner clearance. */
#if ROBOT_OBSTACLE_TEST && ROBOT_BYPASS_TURN_DEG <= 30
#define ROBOT_BYPASS_MIN_TURN_CM 13.1f
#elif ROBOT_OBSTACLE_TEST && ROBOT_BYPASS_TURN_DEG <= 45
/* 45-degree sweep reaches sqrt(10^2+8^2)=12.806 cm. Retain 0.49 cm
 * configured margin, rather than reverting to incompatible 14/15 cm gates. */
#define ROBOT_BYPASS_MIN_TURN_CM 13.3f
#else
#define ROBOT_BYPASS_MIN_TURN_CM 14.0f
#endif
/* Candidate opening for an in-place turn, followed by live monitoring.
 * Separate from the 40 cm silhouette classification used during scans. */
#if ROBOT_OBSTACLE_TEST
/* Candidate rotation only: latest scan has 13.37..14.14 cm.
 * 13.2 cm exceeds the 30-degree turn guard. Re-measure the corridor after
 * turning; this threshold never authorizes forward translation by itself. */
#if ROBOT_BYPASS_TURN_DEG <= 30
#define ROBOT_BYPASS_SIDE_CLEAR_CM 13.2f
#elif ROBOT_BYPASS_TURN_DEG <= 45
#define ROBOT_BYPASS_SIDE_CLEAR_CM 13.4f
#else
#define ROBOT_BYPASS_SIDE_CLEAR_CM 15.0f
#endif
#else
#define ROBOT_BYPASS_SIDE_CLEAR_CM 25.0f
#endif

/* Visible ultrasonic silhouette only; centimetres. Measure chassis before
 * enabling a navigation bridge. Width beyond MAX is STOP, never blind reverse. */
#define ROBOT_SCAN_SAFE_MIN_DEG 70u
#define ROBOT_SCAN_SAFE_MAX_DEG 110u
#define ROBOT_WIDTH_SCAN_MIN_DEG 45u
#define ROBOT_WIDTH_ROUTE_LEFT_FALLBACK_DEG 30u
#define ROBOT_WIDTH_SWEEP_MIN_DEG 30u
#define ROBOT_WIDTH_ROUTE_RIGHT_DEG 120u
#define ROBOT_WIDTH_SWEEP_MAX_DEG 120u
#define ROBOT_SCAN_SERVO_SETTLE_MS 85u
#define ROBOT_SONAR_BEAM_HALF_ANGLE_DEG 7.5f
/* Current mount points right at the lower pulse. Planner bearings are
 * 70=left, 90=front, 110=right. */
#ifndef ROBOT_SCAN_SERVO_REVERSED
#define ROBOT_SCAN_SERVO_REVERSED 1
#endif
/* Center pulse for the mounted sensor. */
#ifndef ROBOT_SCAN_SERVO_CENTER_US
#define ROBOT_SCAN_SERVO_CENTER_US 1450
#endif
#define ROBOT_OBSTACLE_SMALL_CM 10.0f
#if ROBOT_OBSTACLE_TEST
#define ROBOT_OBSTACLE_MAX_BYPASS_CM 20.0f
#else
#define ROBOT_OBSTACLE_MAX_BYPASS_CM 30.0f
#endif
#define ROBOT_CHASSIS_HALF_WIDTH_CM 8.0f
#define ROBOT_CHASSIS_HALF_LENGTH_CM 10.0f
#ifndef ROBOT_SENSOR_FORWARD_OFFSET_CM
#define ROBOT_SENSOR_FORWARD_OFFSET_CM 5.0f
#endif
/* Live front echo is measured from the sensor, not the axle/pivot. */
#define ROBOT_SENSOR_OFFSET_UNCERTAINTY_CM 1.0f
#define ROBOT_SENSOR_TURN_OFFSET_CM ((ROBOT_SENSOR_FORWARD_OFFSET_CM > ROBOT_SENSOR_OFFSET_UNCERTAINTY_CM) ? \
    ROBOT_SENSOR_FORWARD_OFFSET_CM - ROBOT_SENSOR_OFFSET_UNCERTAINTY_CM : 0.0f)
#define ROBOT_BYPASS_FRONT_TURN_CM (ROBOT_BYPASS_MIN_TURN_CM - ROBOT_SENSOR_TURN_OFFSET_CM)
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
#define ROBOT_SENSOR_STARTUP_SETTLE_MS 500u
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
#if ROBOT_OBSTACLE_TEST
/* Keep the earlier stop for the dedicated obstacle test profile. */
#define ROBOT_STOP_CM 50.0f
#define ROBOT_RELEASE_CM 55.0f
#define ROBOT_SCAN_OBJECT_CM 60.0f
/* After scanning and turning into an opening, advance in bounded steps.
 * These are bypass-only thresholds; normal forward approach still stops at 50. */
#if ROBOT_BYPASS_EXACT_SEQUENCE
/* Exact route: half-length 10 + obstacle margin 8 + stopping reserve 2.
 * The prior 30/35 cm recovery gates parked the car at a valid 27 cm echo. */
#define ROBOT_BYPASS_MOVE_STOP_CM 20.0f
#define ROBOT_BYPASS_MOVE_RELEASE_CM 25.0f
#else
#define ROBOT_BYPASS_MOVE_STOP_CM 30.0f
#define ROBOT_BYPASS_MOVE_RELEASE_CM 35.0f
#endif
#else
/* Close-range line-follow stop with 3 cm of restart hysteresis. */
#define ROBOT_STOP_CM 15.0f
#define ROBOT_RELEASE_CM 18.0f
#define ROBOT_SCAN_OBJECT_CM 40.0f
/* Use the same stop distance once the bypass is moving. Require another
 * 5 cm to resume, plus one step of headroom when starting the outward leg. */
#define ROBOT_BYPASS_MOVE_STOP_CM ROBOT_STOP_CM
#define ROBOT_BYPASS_MOVE_RELEASE_CM (ROBOT_BYPASS_MOVE_STOP_CM + 5.0f)
#endif
#define ROBOT_CLEAR_CONFIRM 3u
/* While already following, tolerate isolated invalid echoes for at most
 * 150 ms after actual clearance. Invalid echoes never renew B2's lease. */
#define ROBOT_FRONT_GLITCH_GRACE_MS 150u
#define ROBOT_STOP_SETTLE_MS 250u

#if ROBOT_ENABLE_BYPASS && (ROBOT_MOTION_TEST || ROBOT_WHEELS_UP_MOTOR_TEST || !ROBOT_MODE)
#error "B5 autonomous mode needs ROBOT_MODE=1 and exclusive motor ownership: disable motor tests."
#endif
#if ROBOT_SCAN_SAFE_MIN_DEG > 70u || ROBOT_SCAN_SAFE_MAX_DEG < 110u || ROBOT_SCAN_SAFE_MAX_DEG > 180u
#error "Servo limits must contain the five-ray 70..110 degree coarse scan."
#endif
#endif
