#include "subsystem5.h"
#include "drv_servo.h"
#include "servo_mapping.h"
#include "../robot_config.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

/* Nominal pulse span across 180 degrees of shaft travel. */
#define S5_PULSE_SPAN_US 1000
static uint16_t last_angle_deg = 90u;

uint16_t
s5_servo_pulse_us (uint16_t angle_deg)
{
    int shaft_offset = (int)s5_servo_mount_angle(angle_deg) - 90;
    return (uint16_t)(ROBOT_SCAN_SERVO_CENTER_US +
        shaft_offset * S5_PULSE_SPAN_US / 180);
}

bool
s5_servo_init (void)
{
    /* Repository's RP2040 clock is 125 MHz. Divider 125 -> 1 MHz tick.
       Register DIV is 8:4 fixed point, so integer 125 is 125 << 4.
       This configures the entire PWM slice; ensure no other subsystem
       uses its paired PWM channel before enabling this driver. */
    uint32_t slice        = (S5_SERVO_PIN >> 1u) & 7u;
    uint32_t slice_base   = PWM_BASE + slice * 0x14u;
    uint16_t center_pulse = s5_servo_pulse_us(90u);
    ER       e0           = pwm_set_enabled(S5_SERVO_PIN, FALSE);
    out_w(slice_base + PWM_CHx_DIV, 125u << 4u);
    out_w(slice_base + PWM_CHx_CTR, 0u);
    ER e1 = pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 20 ms at 1 MHz = 50 Hz */
    ER e2 = pwm_set_cc(S5_SERVO_PIN, center_pulse);
    ER e3 = pwm_set_pin(S5_SERVO_PIN);
    ER e4 = pwm_set_enabled(S5_SERVO_PIN, TRUE);
    last_angle_deg = 90u;
    tm_printf((UB *)"[S5-SERVO] GP%d PWM slice=%d 50Hz center=%dus init "
                    "errors=%d,%d,%d,%d,%d\n",
        (int)S5_SERVO_PIN,
        (int)slice,
        (int)center_pulse,
        (int)e0,
        (int)e1,
        (int)e2,
        (int)e3,
        (int)e4);
    tm_printf((UB *)"[S5-SCAN-MAP] reversed=%d logical70=LEFT logical110=RIGHT "
                    "pulse_left=%d pulse_right=%d\n",
        (int)ROBOT_SCAN_SERVO_REVERSED,
        (int)s5_servo_pulse_us(70u),
        (int)s5_servo_pulse_us(110u));
    if (e0 != E_OK || e1 != E_OK || e2 != E_OK || e3 != E_OK || e4 != E_OK)
    {
        return false;
    }

    uint32_t function = in_w(GPIO_CTRL(S5_SERVO_PIN)) & 0x1fu;
    uint32_t csr      = in_w(slice_base + PWM_CHx_CSR);
    uint32_t compare  = in_w(slice_base + PWM_CHx_CC);
    uint32_t pulse    = (compare >> ((S5_SERVO_PIN & 1u) * 16u)) & 0xffffu;
    bool configured = function == GPIO_CTRL_FUNCSEL_PWM && (csr & 0x1u) != 0u &&
        pulse == center_pulse;
    tm_printf((UB *)"[S5-SERVO] GP%d startup PWM readback function=%u "
                    "enabled=%u pulse_us=%u\n",
        (int)S5_SERVO_PIN,
        (UW)function,
        (UW)(csr & 0x1u),
        (UW)pulse);
    return configured;
}

/* Gentle, bounded movements: never jump directly across the sensor mount.
 * Small increments reduce acceleration and mount vibration.
 * This is a blocking RTOS task operation; it must only run while stopped.
 */
bool
s5_servo_angle (uint16_t angle_deg)
{
    uint16_t target = angle_deg;
    if (target < ROBOT_WIDTH_SWEEP_MIN_DEG)
    {
        target = ROBOT_WIDTH_SWEEP_MIN_DEG;
    }
    if (target > ROBOT_WIDTH_SWEEP_MAX_DEG)
    {
        target = ROBOT_WIDTH_SWEEP_MAX_DEG;
    }
    if (target != angle_deg)
    {
        tm_printf((UB *)"[S5-SERVO] angle limited requested=%d safe=%d\n",
            (int)angle_deg,
            (int)target);
    }
    /* A cached angle tracks the last command, not the physical position. */
    if (last_angle_deg == target &&
        pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(target)) != E_OK)
    {
        tm_printf((UB *)"[S5-SERVO-FAULT] PWM update failed\n");
        return false;
    }
    while (last_angle_deg != target)
    {
        uint16_t next;
        if (target > last_angle_deg)
        {
            uint16_t delta = target - last_angle_deg;
            next           = (uint16_t)(last_angle_deg +
                (delta > S5_SERVO_STEP_DEG ? S5_SERVO_STEP_DEG : delta));
        }
        else
        {
            uint16_t delta = last_angle_deg - target;
            next           = (uint16_t)(last_angle_deg -
                (delta > S5_SERVO_STEP_DEG ? S5_SERVO_STEP_DEG : delta));
        }
        if (pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(next)) != E_OK)
        {
            tm_printf(
                (UB *)"[S5-SERVO-FAULT] PWM update failed; scan aborted\n");
            return false;
        }
        last_angle_deg = next;
        tk_dly_tsk(S5_SERVO_STEP_DELAY_MS);
    }
#if ROBOT_VERBOSE_SENSOR_LOG
    tm_printf((UB *)"[S5-SERVO] GP%d settled target=%d deg pulse=%d us\n",
        (int)S5_SERVO_PIN,
        (int)target,
        (int)s5_servo_pulse_us(target));
#endif
    return target == angle_deg;
}

/*** end of file ***/

