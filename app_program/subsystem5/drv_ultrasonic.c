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
 * else while using this driver. A late falling edge gets one bounded
 * recovery window; a signal that remains high is still a fault.
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
#define S5_ECHO_TIMEOUT_US 45000u
#define S5_ECHO_LATE_FALL_US 15000u

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
    case S5_ECHO_OPEN_SPACE: return "OPEN_SPACE_ASSUMED";
    default: return "UNKNOWN";
    }
}

void s5_ultrasonic_init(void)
{
    /* The sensor must work independently of motor initialization order. */
    clr_w(0x4000c000u, 1u << 14u); /* RESETS_RESET: PWM */
    uint32_t reset_started = now_us();
    while (!(in_w(0x4000c008u) & (1u << 14u))) {
        if (expired(reset_started, 1000u)) {
            tm_printf((UB*)"[S5-INIT] ERROR: PWM peripheral held in reset\n");
            return;
        }
    }
    /* Set the output latch LOW before enabling the trigger pad. */
    gpio_set_val(S5_TRIG_PIN, 0u);
    gpio_set_pin(S5_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(S5_TRIG_PIN, 0u);

    /* Disable GPIO output, enable input and select PWM function on GP17.
     * The SIO GPIO_IN register still reflects the pad input, independently
     * of the peripheral selected for the pin. */
    out_w(GPIO_OE_CLR, 1u << S5_ECHO_PIN);
    set_w(GPIO(S5_ECHO_PIN), GPIO_OD | GPIO_IE);
    /* ECHO is externally driven: disable both internal pulls. */
    clr_w(GPIO(S5_ECHO_PIN), (1u << 2u) | (1u << 3u));
    out_w(GPIO_CTRL(S5_ECHO_PIN), GPIO_CTRL_FUNCSEL_PWM);

    /* Stop counter while reconfiguring. 125MHz/125 = 1MHz = 1 tick/us.
     * The DIV register has four fractional bits. */
    out_w(S5_ECHO_CSR, 0u);
    out_w(S5_ECHO_DIV, 125u << 4u);
    out_w(S5_ECHO_TOP, 0xffffu);
    out_w(S5_ECHO_CTR, 0u);
    out_w(S5_ECHO_CSR, S5_ECHO_PWM_B_HIGH_MODE | PWM_CH_CSR_EN);
    tm_printf((UB*)"[S5-INIT] TRIG=GP%d ECHO=GP%d echo_level=%d PWM_CSR=%d DIV=%d TOP=%d (expected 17/2000/65535)\n",
              (int)S5_TRIG_PIN, (int)S5_ECHO_PIN,
              (int)gpio_get_val(S5_ECHO_PIN), (int)in_w(S5_ECHO_CSR),
              (int)in_w(S5_ECHO_DIV), (int)in_w(S5_ECHO_TOP));
}

S5EchoReading s5_ultrasonic_read_detailed(void)
{
    S5EchoReading r = { S5_ECHO_NO_RISE, 0u, 0.0f };
    uint32_t start, count;
    UINT trigger_irq;
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
    out_w(S5_ECHO_CSR, S5_ECHO_PWM_B_HIGH_MODE);
    out_w(S5_ECHO_CTR, 0u);
    out_w(S5_ECHO_CSR, S5_ECHO_PWM_B_HIGH_MODE | PWM_CH_CSR_EN);
    gpio_set_val(S5_TRIG_PIN, 0u);
    start = now_us();
    while (!expired(start, 3u)) { }
    /* Keep the 12-us trigger bounded: a scheduler/USB/WiFi interrupt
     * must not stretch it into a millisecond pulse. Echo waits below
     * remain preemptible; hardware PWM captures their duration. */
    DI(trigger_irq);
    gpio_set_val(S5_TRIG_PIN, 1u);
    start = now_us();
    while (!expired(start, 12u)) { }
    gpio_set_val(S5_TRIG_PIN, 0u);
    EI(trigger_irq);

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
            /* Do not immediately report NO_FALL at 45ms. Allow a late
             * no-target pulse to finish, without retriggering while HIGH.
             * Total high wait stays below the 65.536ms PWM counter wrap
             * and far below the independent 350ms motor watchdog. */
            uint32_t recovery_start = now_us();
            while (gpio_get_val(S5_ECHO_PIN)) {
                if (expired(recovery_start, S5_ECHO_LATE_FALL_US)) {
                    r.pulse_us = in_w(S5_ECHO_CTR) & 0xffffu;
                    r.status = S5_ECHO_NO_FALL;
                    return r;
                }
            }
            /* GPIO did fall: classify the completed hardware pulse below.
             * Forward open-space policy accepts OUT_OF_RANGE; scans do not. */
#if ROBOT_VERBOSE_SENSOR_LOG
            tm_printf((UB*)"[S5-ECHO-RECOVER] late falling edge captured; pulse_us=%d\n",
                      (int)(in_w(S5_ECHO_CTR) & 0xffffu));
#endif
            break;
        }
    }

    r.pulse_us = in_w(S5_ECHO_CTR) & 0xffffu;
    r.cm = s5_echo_us_to_cm(r.pulse_us);
    if (r.pulse_us < 100u)
    {
        r.status = S5_ECHO_TOO_SHORT;
    }
    else if (r.cm > 400.0f) r.status = S5_ECHO_OUT_OF_RANGE;
    else r.status = S5_ECHO_OK;
    return r;
}

S5EchoReading s5_ultrasonic_read_front(void)
{
    S5EchoReading r = s5_ultrasonic_read_detailed();
#if ROBOT_NO_TARGET_FORWARD
    if (r.status == S5_ECHO_NO_RISE || r.status == S5_ECHO_OUT_OF_RANGE) {
        r.status = S5_ECHO_OPEN_SPACE;
        /* Control-only sentinel, explicitly labelled as assumed in logs.
         * Never use this wrapper for obstacle width or side profiling. */
        r.cm = 400.0f;
    }
#endif
    return r;
}

bool s5_ultrasonic_read_cm(float *out_cm)
{
    S5EchoReading r;
    if (!out_cm)
    {
        return false;
    }
    *out_cm = 0.0f;
    r = s5_ultrasonic_read_detailed();
    if (r.status != S5_ECHO_OK)
    {
        return false;
    }
    *out_cm = r.cm;
    return true;
}
