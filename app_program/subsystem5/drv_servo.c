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
    ER e0 = pwm_set_enabled(S5_SERVO_PIN, FALSE);
    out_w(div_addr, 125u << 4u);
    ER e1 = pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 20 ms at 1 MHz = 50 Hz */
    ER e2 = pwm_set_pin(S5_SERVO_PIN);
    ER e3 = pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(90u));
    ER e4 = pwm_set_enabled(S5_SERVO_PIN, TRUE);
    tm_printf((UB*)"[S5-SERVO] GP%d PWM slice=%d 50Hz center=1500us init errors=%d,%d,%d,%d,%d\n",
              (int)S5_SERVO_PIN, (int)slice,
              (int)e0, (int)e1, (int)e2, (int)e3, (int)e4);
}

void s5_servo_angle(uint16_t angle_deg)
{
    ER er = pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(angle_deg));
    tm_printf((UB*)"[S5-SERVO] GP%d commanded=%d deg pulse=%d us pwm_result=%d\n",
              (int)S5_SERVO_PIN, (int)angle_deg,
              (int)s5_servo_pulse_us(angle_deg), (int)er);
}
