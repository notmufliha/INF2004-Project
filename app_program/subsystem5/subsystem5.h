#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"

#define S5_PRI_MAIN 20
/* Real HC-SR04P + servo obstacle scanning: TRIG GP16, ECHO GP17, SERVO GP12. */
#define S5_SIMULATION 0
#define S5_HARDWARE_ENABLED 1
#define S5_SERVO_ENABLED 1
/* Use actual coarse/fine obstacle profiling rather than a fixed servo sweep. */
#define S5_SERVO_SWEEP_TEST 0
#define S5_TRIG_PIN 16u
#define S5_ECHO_PIN 17u
#define S5_SERVO_PIN 12u

/* Leave at 0 until the team's motor/line bridge is implemented.
 * At 0 the sensor and servo can be tested while the car is stationary;
 * at 1 Buddy 5 requires an active motion bridge to start. */
#define S5_MOTION_BRIDGE_ENABLED 0

/* Re-run all software scenarios automatically every 15 seconds. */
#define S5_PICO_DEMO_PAUSE_MS 15000u

IMPORT void s5_task(INT stacd, void *exinf);
#endif
