#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* REAL ROBOT: Cytron Maker Pi Pico Base two-input motor drivers.
 * Verify board and motor terminal labels before energizing wheels.
 * Rebuild and reflash the UF2 after changing this file.
 */
#define ROBOT_MODE 1
#define ROBOT_ENABLE_DRIVE 1
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
#define ROBOT_FORWARD_PWM 350u /* 35% initial drive, adjust for motor stall */
#define ROBOT_MOTOR_WATCHDOG_MS 1200u
#define ROBOT_MOTOR_PERIOD_MS 20u
#define ROBOT_BOOT_DELAY_MS 2000u

/* Dedicated wheels-UP motor diagnostic: set 1, rebuild and flash ONLY when
 * both wheels are lifted. Runs L then R then both, and then stops forever.
 * This test intentionally bypasses the ultrasonic safety gate, so NEVER
 * use with wheels on the floor. Restore to 0 before driving on the floor.
 */
#define ROBOT_WHEELS_UP_MOTOR_TEST 0
#define ROBOT_MOTOR_TEST_PWM 450u
#define ROBOT_MOTOR_TEST_PULSE_MS 1200u

/* Robot is stopped whenever a reading is invalid, close, or ambiguous.
 * Close reading stops immediately; 3 close readings trigger scanning.
 * Three readings >= RELEASE_CM allow forward motion again.
 */
#define ROBOT_STOP_CM 35.0f
#define ROBOT_RELEASE_CM 45.0f
#define ROBOT_CLEAR_CONFIRM 3u
#define ROBOT_STOP_SETTLE_MS 250u

#if ROBOT_ENABLE_BYPASS
#error "Physical bypass needs encoder, IR and geometry calibration; not yet available in uploaded project."
#endif
#endif
