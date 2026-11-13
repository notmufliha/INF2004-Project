#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"
#include "../robot_config.h"

/* Buddy 5: Week 6 design-review pins. Confirm Maker Pi vs Robo Pico
 * conflicts with the team before connecting motor PWM or servo PWM. */
/* Clearance must run before telemetry (12..14), below motor control (5)
 * and VC (6). Echo waits yield; they must not starve line sampling. */
#if ROBOT_MODE
#define S5_PRI_MAIN 7
#else
#define S5_PRI_MAIN 20
#endif
#define S5_SIMULATION 0
#define S5_HARDWARE_ENABLED 1
#define S5_SERVO_ENABLED 1
#define S5_TRIG_PIN 16u
#define S5_ECHO_PIN 17u
#define S5_SERVO_PIN 12u
/* Reduced mechanical travel for the sensor mount in the user's photo.
 * Keep within 70..110 degrees until the horn/bracket is secured and checked.
 * This is NOT a substitute for fastening the horn and strain-relieving wires.
 */
#define S5_SAFE_MIN_DEG ROBOT_SCAN_SAFE_MIN_DEG
#define S5_SAFE_MAX_DEG ROBOT_SCAN_SAFE_MAX_DEG
#define S5_SERVO_STEP_DEG 2u
#define S5_SERVO_STEP_DELAY_MS 35u

/* Autonomous B2/B3 adapter follows the robot configuration. With bypass
 * disabled, retain the existing guarded forward/STOP demonstration. */
#define S5_MOTION_BRIDGE_ENABLED ROBOT_ENABLE_BYPASS

/* 0 = guarded forward/STOP test on a moving robot.
 * 1 = sensor/servo-only demo; B2 is held STOP even when the path is clear.
 * MOVING TEST: first validate with the wheels raised, then use a controlled
 * floor test with a physical emergency stop. B5 never authorises bypass. */
#define S5_DEMO_SENSOR_ONLY 0
/* Guarded straight-line test only: B5 submits B2 DRIVE commands while
 * B5's fresh front echo is clear. This is NOT physical obstacle bypass.
 * Must disable B2's separate one-shot ROBOT_MOTION_TEST. */
#define S5_DEMO_B2_FORWARD_COMMANDS 0
#define S5_DEMO_SPEED_CPS 600
#define S5_DEMO_DRIVE_REFRESH_MS 35u
#define S5_DEMO_CLEAR_MAX_AGE_MS 350u
#define S5_DEMO_B2_STATUS_MAX_AGE_MS 200u

/* Three unsafe/invalid moving readings in a short interval enter a
 * temporary STOP hold. Re-arm only after BOTH a cooldown and consecutive
 * independent, agreeing stationary clearance batches. Never re-arm on a
 * missing echo or a single good ping; repeat failures trigger another hold.
 * This avoids V2's permanent until-reboot lock after transient noise. */
#define S5_GUARD_STOP_LIMIT 3u
#define S5_GUARD_STOP_WINDOW_MS 15000u
#define S5_GUARD_COOLDOWN_MS 8000u
#define S5_GUARD_REARM_CLEAR_BATCHES 5u
#define S5_GUARD_REARM_MAX_GAP_MS 2500u
#if S5_DEMO_SENSOR_ONLY && S5_MOTION_BRIDGE_ENABLED
#error "B5 sensor-only demonstration must not enable live motion bridge"
#endif

/* Bench checks: startup sweep and first scan. No obstacle is required for
 * the initial scan, but valid echoes ARE required for a valid profile. */
#define S5_STARTUP_SERVO_TEST 1
#define S5_FORCE_STARTUP_SCAN 0
/* 1 = continuous coarse/fine scan, 0 = scan on confirmed obstacle. */
#define S5_CONTINUOUS_SWEEP_TEST 0
#define S5_FRONT_POLL_MS 80u
#define S5_SCAN_COOLDOWN_MS 3000u
#define S5_CONFIRM_HITS 3u
#define S5_PICO_DEMO_PAUSE_MS 15000u
#define S5_SERVO_SETTLE_MS 200u
#define S5_ECHO_GAP_MS 65u

IMPORT void s5_task(INT stacd, void *exinf);
#endif
