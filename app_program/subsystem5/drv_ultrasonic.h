#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H
#include <stdint.h>
#include <stdbool.h>

/* Unit-testable pure conversion function. */
float s5_echo_us_to_cm(uint32_t echo_us);
void s5_ultrasonic_init(void);
/* Bounded polling; only call from the owning S5 task, never an ISR. */
bool s5_ultrasonic_read_cm(float *out_cm);
#endif
