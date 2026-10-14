#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"

#define S5_PRI_MAIN 20
/* Simulation is ON by default: no hardware connection is required. */
#define S5_SIMULATION 1
#define S5_HARDWARE_ENABLED 0
#define S5_SERVO_ENABLED 0
/* Example pins only. Reserve in TEAM.md before changing simulation mode. */
#define S5_TRIG_PIN 2u
#define S5_ECHO_PIN 3u
#define S5_SERVO_PIN 15u

IMPORT void s5_task(INT stacd, void *exinf);
#endif
