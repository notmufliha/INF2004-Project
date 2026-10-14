#include "subsystem5.h"
#include "drv_servo.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* Calibration starting values; do not force a servo past its hard stops. */
#define S5_MIN_PULSE_US 1000u
#define S5_MAX_PULSE_US 2000u

uint16_t s5_servo_pulse_us(uint16_t angle_deg)
{
    if (angle_deg > 180u) angle_deg = 180u;
    return (uint16_t)(S5_MIN_PULSE_US +
           ((uint32_t)angle_deg * (S5_MAX_PULSE_US - S5_MIN_PULSE_US)) / 180u);
}

void s5_servo_init(void)
{
    /* Repository's RP2040 clock is 125 MHz. Divider 125 -> 1 MHz tick.
       Register DIV is 8:4 fixed point, so integer 125 is 125 << 4.
       This configures the entire PWM slice; ensure no other subsystem
       uses its paired PWM channel before enabling this driver. */
    uint32_t slice = (S5_SERVO_PIN >> 1u) & 7u;
    uint32_t div_addr = PWM_BASE + slice * 0x14u + PWM_CHx_DIV;
    pwm_set_enabled(S5_SERVO_PIN, FALSE);
    out_w(div_addr, 125u << 4u);
    pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 20 ms at 1 MHz = 50 Hz */
    pwm_set_pin(S5_SERVO_PIN);
    pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(90u));
    pwm_set_enabled(S5_SERVO_PIN, TRUE);
}

void s5_servo_angle(uint16_t angle_deg)
{
    pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(angle_deg));
}
