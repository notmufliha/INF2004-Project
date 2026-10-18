/*
 * Buddy 2 - Initial P-Only Speed Controller
 * RP2040 + micro T-Kernel 3.0
 */

#include "../app.h"
#include "pid_control.h"
#include "motor_control.h"
#include "encoder_control.h"

#define PID_TARGET_CPS   3000
#define PID_BASE_PWM     20
#define PID_MAX_PWM      35
#define PID_MIN_PWM      0
#define PID_KP           0.002f

static INT clamp_pwm(INT pwm)
{
    if (pwm > PID_MAX_PWM) return PID_MAX_PWM;
    if (pwm < PID_MIN_PWM) return PID_MIN_PWM;
    return pwm;
}

void pid_init(void)
{
    tm_printf((UB*)"[B2] P-only controller initialized\n");
}

/*
 * Temporary P-only test.
 *
 * Reads encoder counts over approximately 100 ms,
 * updates motor PWM, and stops after 5 seconds.
 */
void pid_test_task(INT stacd, void *exinf)
{
    INT left_prev, right_prev;
    INT left_now, right_now;
    INT left_speed, right_speed;
    INT left_pwm, right_pwm;
    INT left_error, right_error;

    SYSTIM previous_time, current_time;
    long long elapsed_ms;

    INT iteration;

    (void)stacd;
    (void)exinf;

    /* Allow time to prepare. */
    tk_dly_tsk(3000);

    encoder_get_counts(&left_prev, &right_prev);
    tk_get_tim(&previous_time);

    tm_printf((UB*)"[B2] P-only test starting\n");

    motor_left_set(PID_BASE_PWM);
    motor_right_set(PID_BASE_PWM);

    for (iteration = 0; iteration < 50; iteration++) {

        tk_dly_tsk(100);

        encoder_get_counts(&left_now, &right_now);
        tk_get_tim(&current_time);

        elapsed_ms =
            (((long long)current_time.hi << 32) |
             (unsigned long long)current_time.lo)
            -
            (((long long)previous_time.hi << 32) |
             (unsigned long long)previous_time.lo);

        if (elapsed_ms <= 0) {
            motor_stop();
            tm_printf((UB*)"[B2] Invalid PID timing\n");
            break;
        }

        left_speed =
            (INT)(((long long)(left_now - left_prev) * 1000)
                  / elapsed_ms);

        right_speed =
            (INT)(((long long)(right_now - right_prev) * 1000)
                  / elapsed_ms);

        left_error = PID_TARGET_CPS - left_speed;
        right_error = PID_TARGET_CPS - right_speed;

        left_pwm = clamp_pwm(
            PID_BASE_PWM + (INT)(PID_KP * left_error)
        );

        right_pwm = clamp_pwm(
            PID_BASE_PWM + (INT)(PID_KP * right_error)
        );

        motor_left_set(left_pwm);
        motor_right_set(right_pwm);

        if ((iteration % 5) == 0) {
            tm_printf(
                (UB*)"[B2] PID L=%d R=%d PWM_L=%d PWM_R=%d\n",
                left_speed, right_speed, left_pwm, right_pwm
            );
        }

        left_prev = left_now;
        right_prev = right_now;
        previous_time = current_time;
    }

    motor_stop();

    tm_printf((UB*)"[B2] P-only test completed - motors stopped\n");

    /* Temporary workaround for task-return hard fault. */
    while (1) {
        tk_dly_tsk(1000);
    }
}
