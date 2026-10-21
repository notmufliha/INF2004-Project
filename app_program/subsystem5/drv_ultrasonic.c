#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/*
 * RP2040 HARDWARE echo-width capture, GP17 -> PWM slice 0, channel B.
 *
 * The old driver measured a pulse by subtracting two CPU-polling timestamps.
 * A task switch or interrupt between those timestamps could add ~1ms to
 * the apparent pulse (about 17cm) or miss an entire short pulse.
 *
 * PWM slice 0 DIVMODE=B_HIGH counts only while the GP17 input is high,
 * even while the Buddy 5 task is preempted. GP8/9 (motor slice 4),
 * GP10/11 (motor slice 5) and GP12 (servo slice 6) are unaffected.
 *
 * Assumes the existing firmware's 125 MHz system clock and a free PWM
 * slice 0. This build has USE_PTMR=0; do not claim slice 0 for anything
 * else while using this driver. A pulse wider than 30ms is rejected.
 */
#if S5_ECHO_PIN != 17u
#error "Hardware echo capture currently requires GP17 (PWM slice 0 B)"
#endif
#if S5_TRIG_PIN != 16u
#error "Expected TRIG on GP16; check your wiring and this driver"
#endif

#define S5_ECHO_PWM_SLICE_BASE PWM_CH0_BASE
#define S5_ECHO_CSR     (S5_ECHO_PWM_SLICE_BASE + PWM_CHx_CSR)
#define S5_ECHO_DIV     (S5_ECHO_PWM_SLICE_BASE + PWM_CHx_DIV)
#define S5_ECHO_CTR     (S5_ECHO_PWM_SLICE_BASE + PWM_CHx_CTR)
#define S5_ECHO_TOP     (S5_ECHO_PWM_SLICE_BASE + PWM_CHx_TOP)
#define S5_ECHO_PWM_B_HIGH_MODE (1u << 4u)
#define S5_ECHO_TIMEOUT_US 30000u

/* RP2040 1MHz timer, used only to bound polling; not to measure pulse. */
static uint32_t now_us(void) { return (uint32_t)in_w(TIMER_TIMERAWL); }
static bool expired(uint32_t start, uint32_t duration_us)
{
    return (uint32_t)(now_us() - start) >= duration_us;
}

float s5_echo_us_to_cm(uint32_t echo_us)
{
    return ((float)echo_us * 0.0343f) / 2.0f;
}

const char *s5_echo_status_name(S5EchoStatus status)
{
    switch (status) {
    case S5_ECHO_OK: return "OK";
    case S5_ECHO_STUCK_HIGH: return "ECHO_STUCK_HIGH";
    case S5_ECHO_NO_RISE: return "ECHO_NO_RISE";
    case S5_ECHO_NO_FALL: return "ECHO_NO_FALL";
    case S5_ECHO_TOO_SHORT: return "ECHO_GLITCH";
    case S5_ECHO_OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "UNKNOWN";
    }
}

void s5_ultrasonic_init(void)
{
    gpio_set_pin(S5_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(S5_TRIG_PIN, 0u);

    /* Disable GPIO output, enable input and select PWM function on GP17.
     * The SIO GPIO_IN register still reflects the pad input, independently
     * of the peripheral selected for the pin. */
    out_w(GPIO_OE_CLR, 1u << S5_ECHO_PIN);
    set_w(GPIO(S5_ECHO_PIN), GPIO_OD | GPIO_IE);
    out_w(GPIO_CTRL(S5_ECHO_PIN), GPIO_CTRL_FUNCSEL_PWM);

    /* Stop counter while reconfiguring. 125MHz/125 = 1MHz = 1 tick/us.
     * The DIV register has four fractional bits. */
    out_w(S5_ECHO_CSR, 0u);
    out_w(S5_ECHO_DIV, 125u << 4u);
    out_w(S5_ECHO_TOP, 0xffffu);
    out_w(S5_ECHO_CTR, 0u);
    out_w(S5_ECHO_CSR, S5_ECHO_PWM_B_HIGH_MODE | PWM_CH_CSR_EN);
}

S5EchoReading s5_ultrasonic_read_detailed(void)
{
    S5EchoReading r = { S5_ECHO_NO_RISE, 0u, 0.0f };
    uint32_t start, count;
    bool saw_high = false;

    /* A stuck-high pin is not a valid echo. */
    start = now_us();
    while (gpio_get_val(S5_ECHO_PIN)) {
        if (expired(start, S5_ECHO_TIMEOUT_US)) {
            r.status = S5_ECHO_STUCK_HIGH;
            return r;
        }
    }

    /* Clear the hardware count immediately before sending a fresh trigger.
     * Only the B pin's high time increments this counter. */
    out_w(S5_ECHO_CTR, 0u);
    gpio_set_val(S5_TRIG_PIN, 0u);
    start = now_us();
    while (!expired(start, 3u)) { }
    gpio_set_val(S5_TRIG_PIN, 1u);
    start = now_us();
    while (!expired(start, 12u)) { }
    gpio_set_val(S5_TRIG_PIN, 0u);

    /* A preemption can happen before we see ECHO rise.  If the hardware
     * counter is already nonzero, the pulse has begun or completed even
     * if the software never observed the high edge. */
    start = now_us();
    while (!saw_high) {
        count = in_w(S5_ECHO_CTR) & 0xffffu;
        if (gpio_get_val(S5_ECHO_PIN) || count != 0u) {
            saw_high = true;
            break;
        }
        if (expired(start, S5_ECHO_TIMEOUT_US)) {
            r.status = S5_ECHO_NO_RISE;
            return r;
        }
    }

    /* Counter keeps running only while ECHO is high.  Interrupts and task
     * switches cannot stretch the captured width. */
    start = now_us();
    while (gpio_get_val(S5_ECHO_PIN)) {
        if (expired(start, S5_ECHO_TIMEOUT_US)) {
            r.status = S5_ECHO_NO_FALL;
            return r;
        }
    }

    r.pulse_us = in_w(S5_ECHO_CTR) & 0xffffu;
    r.cm = s5_echo_us_to_cm(r.pulse_us);
    if (r.pulse_us < 100u) r.status = S5_ECHO_TOO_SHORT;
    else if (r.cm > 400.0f) r.status = S5_ECHO_OUT_OF_RANGE;
    else r.status = S5_ECHO_OK;
    return r;
}

bool s5_ultrasonic_read_cm(float *out_cm)
{
    S5EchoReading r;
    if (!out_cm) return false;
    *out_cm = 0.0f;
    r = s5_ultrasonic_read_detailed();
    if (r.status != S5_ECHO_OK) return false;
    *out_cm = r.cm;
    return true;
}
