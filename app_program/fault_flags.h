/*
 *	fault_flags.h
 *	The shared fault bitmask: one bit per fault condition, for every
 *	subsystem (Change Requirement 1, "Minimum-Bit Data Representation").
 *
 *	The bitmask is a micro T-Kernel EVENT FLAG (flg_faults), not a plain
 *	variable.  tk_set_flg / tk_clr_flg change the pattern inside the
 *	kernel, so two tasks (or a task and an ISR) setting different bits
 *	at the same time can never lose each other's update -- a plain
 *	"faults |= bit" read-modify-write could.  Both calls may be used
 *	from an interrupt handler.
 *
 *	Owner: the Vehicle Controller (Week 8 Tribunal decision).  Until the
 *	Vehicle Controller exists, subsystem 1 creates flg_faults in
 *	subsystem1_init(), which app_main.c runs before any other init.
 *	Each buddy sets and clears ONLY their own bit(s).
 *
 *	Subsystem 1 sends the whole mask in every telemetry message.
 */

#ifndef FAULT_FLAGS_H
#define FAULT_FLAGS_H

#include "app.h"

#define FAULT_MOTOR_STALL	0x01U	/* B2: wheel commanded but not turning */
#define FAULT_ENCODER		0x02U	/* B2: encoder reading implausible     */
#define FAULT_IMU		0x04U	/* B4: IMU unavailable / invalid       */
#define FAULT_NO_ECHO		0x08U	/* B5: no reliable ultrasonic echo     */
#define FAULT_WIFI_LOSS		0x10U	/* B1: WiFi/MQTT link lost             */
#define FAULT_LINE_LOST		0x20U	/* B3: line not found                  */
					/* 0x40, 0x80 spare                    */
#define FAULT_BITS		8	/* width of the mask on the wire       */
#define FAULT_ALL		0xFFU

IMPORT ID	flg_faults;

/* Never block.  Safe from tasks and interrupt handlers. */
IMPORT void fault_raise(UINT bits);
IMPORT void fault_clear(UINT bits);
IMPORT UB   fault_get(void);		/* current mask, 0 if not created */

#endif /* FAULT_FLAGS_H */
