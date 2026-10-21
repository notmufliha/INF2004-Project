#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* The RP2040 free-running TIMERAWL counts microseconds (32-bit wrap). */
static uint32_t now_us(void) { return (uint32_t)in_w(TIMER_TIMERAWL); }
static bool expired(uint32_t start, uint32_t duration_us)
{
    return (uint32_t)(now_us() - start) >= duration_us;
}

float s5_echo_us_to_cm(uint32_t echo_us)
{
    return ((float)echo_us * 0.0343f) / 2.0f;
}

void s5_ultrasonic_init(void)
{
    gpio_set_pin(S5_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(S5_TRIG_PIN, 0);
    gpio_set_pin(S5_ECHO_PIN, GPIO_MODE_IN);
}

bool s5_ultrasonic_read_cm(float *out_cm)
{
    uint32_t start, pulse;
    if (out_cm == 0) return false;

    /* Clear a lingering echo with a bounded wait. */
    start = now_us();
    while (gpio_get_val(S5_ECHO_PIN)) {
        if (expired(start, 30000u)) return false;
    }

    gpio_set_val(S5_TRIG_PIN, 0);
    start = now_us();
    while (!expired(start, 2u)) { }
    gpio_set_val(S5_TRIG_PIN, 1);
    start = now_us();
    while (!expired(start, 10u)) { }
    gpio_set_val(S5_TRIG_PIN, 0);

    start = now_us();
    while (!gpio_get_val(S5_ECHO_PIN)) {
        if (expired(start, 30000u)) return false;
    }
    start = now_us();
    while (gpio_get_val(S5_ECHO_PIN)) {
        if (expired(start, 30000u)) return false;
    }
    pulse = (uint32_t)(now_us() - start);
    *out_cm = s5_echo_us_to_cm(pulse);
    return pulse >= 100u; /* Reject implausibly short glitches. */
}
