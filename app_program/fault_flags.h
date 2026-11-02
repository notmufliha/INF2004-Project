/** @file fault_flags.h
 * @brief The shared fault bitmask: which bit belongs to which fault
 * (Change Requirement 1, "Minimum-Bit Data Representation").
 *
 * This file is the team-agreed bit assignment.  The functions that set,
 * clear and read the mask are in fault.h / fault.c (BARR-C 6.1.i:
 * public functions carry their module's name).  Including either
 * header gives both, so existing code keeps working unchanged.
 *
 * Each buddy sets and clears ONLY their own bit(s).
 */

#ifndef FAULT_FLAGS_H
#define FAULT_FLAGS_H

#define FAULT_MOTOR_STALL   0x01U /* B2: wheel commanded but not turning */
#define FAULT_ENCODER       0x02U /* B2: encoder reading implausible     */
#define FAULT_IMU           0x04U /* B4: IMU unavailable / invalid       */
#define FAULT_NO_ECHO       0x08U /* B5: no reliable ultrasonic echo     */
#define FAULT_WIFI_LOSS     0x10U /* B1: WiFi/MQTT link lost             */
#define FAULT_LINE_LOST     0x20U /* B3: line not found                  */
                                  /* 0x40, 0x80 spare                    */
#define FAULT_BITS          8U    /* width of the mask on the wire       */
#define FAULT_ALL           0xFFU

#include "fault.h"

#endif /* FAULT_FLAGS_H */

/*** end of file ***/

