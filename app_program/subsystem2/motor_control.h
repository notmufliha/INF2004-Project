/** @file motor_control.h
 * @brief Subsystem 2 motor control interface.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "../app.h"

ER motor_init(void);

void motor_left_set(int speed_percent);

void motor_right_set(int speed_percent);

void motor_stop(void);
#endif /* MOTOR_CONTROL_H */
