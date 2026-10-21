#include "subsystem5.h"
#include "drv_servo.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* Calibration starting values; do not force a servo past its hard stops. */
#define S5_MIN_PULSE_US 1000u
#define S5_MAX_PULSE_US 2000u
static uint16_t last_angle_deg = 90u;

uint16_t s5_servo_pulse_us(uint16_t angle_deg)
{
    if (angle_deg > 180u) angle_deg = 180u;
    return (uint16_t)(S5_MIN_PULSE_US +
           ((uint32_t)angle_deg * (S5_MAX_PULSE_US - S5_MIN_PULSE_US)) / 180u);
}

void s5_servo_init(void)
{
    last_angle_deg = 90u;
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

/* Gentle, bounded movements: never jump directly across the sensor mount.
 * 3 degree increments every 35ms give the servo time to follow the command.
 * This is a blocking RTOS task operation; it must only run while stopped.
 */
void s5_servo_angle(uint16_t angle_deg)
{
    uint16_t target = angle_deg;
    if (target < S5_SAFE_MIN_DEG) target = S5_SAFE_MIN_DEG;
    if (target > S5_SAFE_MAX_DEG) target = S5_SAFE_MAX_DEG;
    if (target != angle_deg)
        tm_printf((UB*)"[S5-SERVO] angle limited requested=%d safe=%d\n",
                  (int)angle_deg, (int)target);
    while (last_angle_deg != target) {
        uint16_t next;
        if (target > last_angle_deg) {
            uint16_t delta = target - last_angle_deg;
            next = (uint16_t)(last_angle_deg +
                    (delta > S5_SERVO_STEP_DEG ? S5_SERVO_STEP_DEG : delta));
        } else {
            uint16_t delta = last_angle_deg - target;
            next = (uint16_t)(last_angle_deg -
                    (delta > S5_SERVO_STEP_DEG ? S5_SERVO_STEP_DEG : delta));
        }
        if (pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(next)) != E_OK) {
            tm_printf((UB*)"[S5-SERVO-FAULT] PWM update failed; scan aborted\n");
            return;
        }
        last_angle_deg = next;
        tk_dly_tsk(S5_SERVO_STEP_DELAY_MS);
    }
    tm_printf((UB*)"[S5-SERVO] GP%d settled target=%d deg pulse=%d us\n",
              (int)S5_SERVO_PIN, (int)target,
              (int)s5_servo_pulse_us(target));
}
