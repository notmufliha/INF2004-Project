#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    S5_ECHO_OK = 0,
    S5_ECHO_STUCK_HIGH,
    S5_ECHO_NO_RISE,
    S5_ECHO_NO_FALL,
    S5_ECHO_TOO_SHORT,
    S5_ECHO_OUT_OF_RANGE
} S5EchoStatus;
typedef struct {
    S5EchoStatus status;
    uint32_t pulse_us;
    float cm;
} S5EchoReading;

float s5_echo_us_to_cm(uint32_t echo_us);
void s5_ultrasonic_init(void);
/* Diagnostic version exposes the actual pulse and failure mode.
 * Only call from the Buddy 5 task (never from an ISR). */
S5EchoReading s5_ultrasonic_read_detailed(void);
bool s5_ultrasonic_read_cm(float *out_cm);
const char *s5_echo_status_name(S5EchoStatus status);
#endif
