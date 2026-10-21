#ifndef B2_MOTOR_H
#define B2_MOTOR_H
#include "subsystem2.h"
#include <stdbool.h>
#include <stdint.h>
void b2_motor_task(INT stacd, void *exinf);
void b2_request_stop(void);
void b2_request_forward(void);
bool b2_motor_is_stopped(void);
#endif
