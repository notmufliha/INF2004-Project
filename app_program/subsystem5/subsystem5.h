#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"

/* Buddy 5: Week 6 design-review pins. Confirm Maker Pi vs Robo Pico
 * conflicts with the team before connecting motor PWM or servo PWM. */
#define S5_PRI_MAIN 20
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
#define S5_SAFE_MIN_DEG 70u
#define S5_SAFE_MAX_DEG 110u
#define S5_SERVO_STEP_DEG 3u
#define S5_SERVO_STEP_DELAY_MS 35u

/* Safety gate: leave at 0 until B2/B3 bridge is actually implemented.
 * With 0, B5 measures and recommends a path but NEVER moves the car. */
#define S5_MOTION_BRIDGE_ENABLED 0

/* Bench checks: startup sweep and first scan. No obstacle is required for
 * the initial scan, but valid echoes ARE required for a valid profile. */
#define S5_STARTUP_SERVO_TEST 1
#define S5_FORCE_STARTUP_SCAN 0
/* 1 = continuous coarse/fine scan, 0 = scan on confirmed obstacle. */
#define S5_CONTINUOUS_SWEEP_TEST 0
#define S5_FRONT_POLL_MS 200u
#define S5_SCAN_COOLDOWN_MS 3000u
#define S5_CONFIRM_HITS 3u
#define S5_PICO_DEMO_PAUSE_MS 15000u
#define S5_SERVO_SETTLE_MS 220u
#define S5_ECHO_GAP_MS 65u

IMPORT void s5_task(INT stacd, void *exinf);
#endif
