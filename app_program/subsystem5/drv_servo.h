#ifndef S5_DRV_SERVO_H
#define S5_DRV_SERVO_H
#include <stdint.h>
uint16_t s5_servo_pulse_us(uint16_t angle_deg);
void s5_servo_init(void);
void s5_servo_angle(uint16_t angle_deg);
#endif
