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

#define PID_PERIOD_MS    10
#define PID_WINDOW_SIZE  10
#define PID_TEST_MS      5000

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
 * Buddy 2 - 10 ms PI Motion Control Test
 *
 * Control update: nominally every 10 ms
 * Speed estimate: rolling 100 ms window
 * Target: 3000 counts/s per wheel
 *
 * Experimental controller, wheels-off-ground only.
 */

void pid_test_task(INT stacd, void *exinf)
{
    INT left_history[PID_WINDOW_SIZE + 1] = {0};
    INT right_history[PID_WINDOW_SIZE + 1] = {0};

    long long time_history[PID_WINDOW_SIZE + 1] = {0};

    INT left_now, right_now;
    INT left_speed = 0, right_speed = 0;
    INT left_pwm = PID_BASE_PWM;
    INT right_pwm = PID_BASE_PWM;

    float left_integral = 0.0f;
    float right_integral = 0.0f;

    float left_error, right_error;
    float left_output, right_output;

    SYSTIM now;
    long long current_ms;
    long long elapsed_ms;

    INT iteration;
    INT index;
    INT old_index;

    ER err;

    (void)stacd;
    (void)exinf;

    tk_dly_tsk(3000);

    tm_printf((UB*)"[B2] 10 ms PI test starting\n");

    encoder_get_counts(&left_now, &right_now);

    err = tk_get_tim(&now);
    if (err != E_OK) {
        tm_printf((UB*)"[B2] Initial time read failed\n");
        goto test_end;
    }

    current_ms =
        ((long long)now.hi * 4294967296LL) +
        (unsigned long long)now.lo;

    left_history[0] = left_now;
    right_history[0] = right_now;
    time_history[0] = current_ms;

    motor_left_set(PID_BASE_PWM);
    motor_right_set(PID_BASE_PWM);

    for (iteration = 1;
         iteration <= PID_TEST_MS / PID_PERIOD_MS;
         iteration++) {

        tk_dly_tsk(PID_PERIOD_MS);

        encoder_get_counts(&left_now, &right_now);

        err = tk_get_tim(&now);
        if (err != E_OK) {
            tm_printf((UB*)"[B2] Time read failed\n");
            break;
        }

        current_ms =
            ((long long)now.hi * 4294967296LL) +
            (unsigned long long)now.lo;

        index = iteration % (PID_WINDOW_SIZE + 1);

        left_history[index] = left_now;
        right_history[index] = right_now;
        time_history[index] = current_ms;

        /*
         * Wait until the first 10 samples have been collected.
         */
        if (iteration >= PID_WINDOW_SIZE) {

            old_index =
                (iteration - PID_WINDOW_SIZE)
                % (PID_WINDOW_SIZE + 1);

            elapsed_ms =
                current_ms - time_history[old_index];

            if (elapsed_ms <= 0) {
                tm_printf((UB*)"[B2] Invalid elapsed time\n");
                break;
            }

            left_speed =
                (INT)(((long long)
                    (left_now - left_history[old_index]) * 1000)
                    / elapsed_ms);

            right_speed =
                (INT)(((long long)
                    (right_now - right_history[old_index]) * 1000)
                    / elapsed_ms);

            left_error = PID_TARGET_CPS - left_speed;
            right_error = PID_TARGET_CPS - right_speed;

            /*
             * Integral accumulation uses the actual time
             * between controller updates, not the full
             * 100 ms speed-estimation window.
             */
            {
                INT prev_index =
                    (iteration - 1) % (PID_WINDOW_SIZE + 1);

                float dt =
                    (float)(current_ms - time_history[prev_index])
                    / 1000.0f;

                if (dt <= 0.0f || dt > 0.1f) {
                    tm_printf((UB*)"[B2] PI timing out of range\n");
                    break;
                }

                left_integral += PID_KI * left_error * dt;
                right_integral += PID_KI * right_error * dt;
            }

            if (left_integral > PID_I_LIMIT)
                left_integral = PID_I_LIMIT;
            if (left_integral < -PID_I_LIMIT)
                left_integral = -PID_I_LIMIT;

            if (right_integral > PID_I_LIMIT)
                right_integral = PID_I_LIMIT;
            if (right_integral < -PID_I_LIMIT)
                right_integral = -PID_I_LIMIT;

            left_output =
                PID_BASE_PWM +
                PID_KP * left_error +
                left_integral;

            right_output =
                PID_BASE_PWM +
                PID_KP * right_error +
                right_integral;

            /*
             * Round positive PWM values to nearest integer.
             */
            left_pwm = clamp_pwm((INT)(left_output + 0.5f));
            right_pwm = clamp_pwm((INT)(right_output + 0.5f));

            motor_left_set(left_pwm);
            motor_right_set(right_pwm);
        }

        /* Print approximately every 500 ms. */
        if ((iteration % 50) == 0) {
            tm_printf(
                (UB*)"[B2] 10ms PI L=%d R=%d PWM_L=%d PWM_R=%d\n",
                left_speed, right_speed,
                left_pwm, right_pwm
            );
        }
    }

test_end:
    motor_stop();

    tm_printf((UB*)"[B2] 10 ms PI test completed - motors stopped\n");

    /* Temporary workaround for task-return hard fault. */
    while (1) {
        tk_dly_tsk(1000);
    }
}
