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

/* Initial PI gains — not yet tuned */
#define PID_KP           0.002f
#define PID_KI           0.005f

/* Limit accumulated integral contribution */
#define PID_I_LIMIT      10.0f

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
 * Buddy 2 - PI Speed Control Test
 *
 * Target: 3000 encoder counts/s per wheel
 * Control interval: approximately 100 ms
 * Test duration: approximately 5 seconds
 *
 * This is an initial tuning experiment.
 */

void pid_test_task(INT stacd, void *exinf)
{
    INT left_prev, right_prev;
    INT left_now, right_now;

    INT left_speed, right_speed;
    INT left_pwm, right_pwm;

    float left_error, right_error;
    float left_integral = 0.0f;
    float right_integral = 0.0f;

    float left_output, right_output;
    float dt;

    SYSTIM previous_time, current_time;
    long long elapsed_ms;

    INT iteration;
    ER err;

    (void)stacd;
    (void)exinf;

    /* Preparation delay */
    tk_dly_tsk(3000);

    encoder_get_counts(&left_prev, &right_prev);

    err = tk_get_tim(&previous_time);
    if (err != E_OK) {
        motor_stop();
        tm_printf((UB*)"[B2] PI initial timing failed\n");
        goto test_end;
    }

    tm_printf((UB*)"[B2] PI test starting\n");

    motor_left_set(PID_BASE_PWM);
    motor_right_set(PID_BASE_PWM);

    for (iteration = 0; iteration < 50; iteration++) {

        tk_dly_tsk(100);

        encoder_get_counts(&left_now, &right_now);

        err = tk_get_tim(&current_time);
        if (err != E_OK) {
            tm_printf((UB*)"[B2] PI timing read failed\n");
            break;
        }

        elapsed_ms =
            (((long long)current_time.hi << 32) |
             (unsigned long long)current_time.lo)
            -
            (((long long)previous_time.hi << 32) |
             (unsigned long long)previous_time.lo);

        if (elapsed_ms <= 0) {
            tm_printf((UB*)"[B2] Invalid PI timing\n");
            break;
        }

        dt = (float)elapsed_ms / 1000.0f;

        /* Measured wheel speeds */
        left_speed =
            (INT)(((long long)(left_now - left_prev) * 1000)
                  / elapsed_ms);

        right_speed =
            (INT)(((long long)(right_now - right_prev) * 1000)
                  / elapsed_ms);

        /* Speed errors */
        left_error = PID_TARGET_CPS - left_speed;
        right_error = PID_TARGET_CPS - right_speed;

        /* Integrate error using actual elapsed time */
        left_integral += PID_KI * left_error * dt;
        right_integral += PID_KI * right_error * dt;

        /* Basic integral clamping */
        if (left_integral > PID_I_LIMIT)
            left_integral = PID_I_LIMIT;
        if (left_integral < -PID_I_LIMIT)
            left_integral = -PID_I_LIMIT;

        if (right_integral > PID_I_LIMIT)
            right_integral = PID_I_LIMIT;
        if (right_integral < -PID_I_LIMIT)
            right_integral = -PID_I_LIMIT;

        /* PI control outputs */
        left_output =
            PID_BASE_PWM +
            PID_KP * left_error +
            left_integral;

        right_output =
            PID_BASE_PWM +
            PID_KP * right_error +
            right_integral;

        /* Round to nearest integer PWM percentage */
        left_pwm = clamp_pwm(
            (INT)(left_output + 0.5f)
        );

        right_pwm = clamp_pwm(
            (INT)(right_output + 0.5f)
        );

        motor_left_set(left_pwm);
        motor_right_set(right_pwm);

        /* Print approximately every 500 ms */
        if ((iteration % 5) == 0) {
            tm_printf(
                (UB*)"[B2] PI L=%d R=%d PWM_L=%d PWM_R=%d\n",
                left_speed, right_speed,
                left_pwm, right_pwm
            );
        }

        left_prev = left_now;
        right_prev = right_now;
        previous_time = current_time;
    }

test_end:
    motor_stop();

    tm_printf((UB*)"[B2] PI test completed - motors stopped\n");

    /* Temporary workaround for task-return hard fault */
    while (1) {
        tk_dly_tsk(1000);
    }
}
