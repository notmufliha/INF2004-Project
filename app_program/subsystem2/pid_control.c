
/*
 * Buddy 2 - PI Motor Speed Controller
 * RP2040 + micro T-Kernel 3.0
 *
 * Nominal control period: 10 ms (cyclic handler)
 * Speed estimation: 10-sample rolling window
 * Target speed: 3000 encoder counts/s
 *
 * Experimental test: wheels off the ground.
 */

#include "../app.h"
#include "pid_control.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "subsystem2.h"

#define PID_TARGET_CPS    3000
#define PID_BASE_PWM      20
#define PID_MAX_PWM       35
#define PID_MIN_PWM       0

#define PID_KP            0.002f
#define PID_KI            0.005f
#define PID_I_LIMIT       10.0f

#define PID_PERIOD_MS     10
#define PID_WINDOW_SIZE   10
#define PID_TEST_MS       5000
#define PID_TEST_STEPS    (PID_TEST_MS / PID_PERIOD_MS)

/* Shared task ID used by the cyclic handler. */
ID tid_pid = 0;

/*
 * Cyclic handler.
 * Runs in timer interrupt context.
 * Keep it short and non-blocking.
 */
EXPORT void pid_cyclic_handler(void *exinf)
{
    (void)exinf;

    if (tid_pid > 0) {
        (void)tk_wup_tsk(tid_pid);
    }
}

/* Limit motor PWM to the permitted test range. */
static INT clamp_pwm(INT pwm)
{
    if (pwm > PID_MAX_PWM) {
        return PID_MAX_PWM;
    }

    if (pwm < PID_MIN_PWM) {
        return PID_MIN_PWM;
    }

    return pwm;
}

void pid_init(void)
{
    tm_printf((UB*)"[B2] PI controller initialized\n");
}

/* Convert micro T-Kernel system time to milliseconds. */
static long long pid_time_ms(const SYSTIM *time)
{
    return ((long long)time->hi * 4294967296LL) +
           (unsigned long long)time->lo;
}

/*
 * Read current encoder counts and system time.
 * This is used after each cyclic activation.
 */
static ER pid_read_sample(INT *left,
                          INT *right,
                          long long *time_ms)
{
    SYSTIM now;
    ER err;

    encoder_get_counts(left, right);

    err = tk_get_tim(&now);

    if (err != E_OK) {
        return err;
    }

    *time_ms = pid_time_ms(&now);

    return E_OK;
}

/*
 * Wait for a cyclic activation.
 *
 * Clear stale wake-ups first so that an old
 * queued activation cannot immediately release
 * the task.
 */
static ER pid_wait_activation(void)
{
    (void)tk_can_wup(TSK_SELF);

    return tk_slp_tsk(TMO_FEVR);
}

/*
 * Temporary PI control test.
 */
void pid_test_task(INT stacd, void *exinf)
{
    INT left_history[PID_WINDOW_SIZE + 1] = {0};
    INT right_history[PID_WINDOW_SIZE + 1] = {0};

    long long time_history[PID_WINDOW_SIZE + 1] = {0};

    INT left_now = 0;
    INT right_now = 0;

    INT left_speed = 0;
    INT right_speed = 0;

    INT left_pwm = PID_BASE_PWM;
    INT right_pwm = PID_BASE_PWM;

    float left_integral = 0.0f;
    float right_integral = 0.0f;

    float left_error;
    float right_error;

    float left_output;
    float right_output;

    long long current_ms = 0;
    long long elapsed_ms;
    long long loop_dt_ms;

    long long loop_min_ms = 999999;
    long long loop_max_ms = 0;
    long long loop_total_ms = 0;

    INT loop_samples = 0;
    INT loop_overruns = 0;
    INT loop_invalid = 0;

    INT iteration;
    INT index;
    INT old_index;
    INT previous_index;

    ER err;

    (void)stacd;
    (void)exinf;

    /*
     * Allow time to prepare the robot.
     * The cyclic handler is already running,
     * but accumulated wake-ups will be cleared.
     */
    tk_dly_tsk(3000);

    tm_printf((UB*)"[B2] 10 ms cyclic PI test starting\n");

    /*
     * Synchronize the initial measurement with
     * a fresh cyclic activation.
     */
    err = pid_wait_activation();

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Initial activation failed\n");
        goto test_end;
    }

    err = pid_read_sample(
        &left_now,
        &right_now,
        &current_ms
    );

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Initial sample failed\n");
        goto test_end;
    }

    left_history[0] = left_now;
    right_history[0] = right_now;
    time_history[0] = current_ms;

    motor_left_set(PID_BASE_PWM);
    motor_right_set(PID_BASE_PWM);

    /*
     * Main cyclic PI control loop.
     */
    for (iteration = 1;
         iteration <= PID_TEST_STEPS;
         iteration++) {

        err = pid_wait_activation();

        if (err != E_OK) {
            tm_printf((UB*)"[B2] PI activation failed\n");
            break;
        }

        err = pid_read_sample(
            &left_now,
            &right_now,
            &current_ms
        );

        if (err != E_OK) {
            tm_printf((UB*)"[B2] Sample read failed\n");
            break;
        }

        previous_index =
            (iteration - 1) % (PID_WINDOW_SIZE + 1);

        loop_dt_ms =
            current_ms - time_history[previous_index];

        /*
         * Record and report invalid intervals.
         * Never divide by zero or run the PI
         * controller with invalid timing.
         */
        if (loop_dt_ms <= 0) {

            loop_invalid++;

            tm_printf(
                (UB*)"[B2] Invalid interval dt=%d current=%d previous=%d\n",
                (INT)loop_dt_ms,
                (INT)current_ms,
                (INT)time_history[previous_index]
            );

            break;
        }

        if (loop_dt_ms < loop_min_ms) {
            loop_min_ms = loop_dt_ms;
        }

        if (loop_dt_ms > loop_max_ms) {
            loop_max_ms = loop_dt_ms;
        }

        loop_total_ms += loop_dt_ms;
        loop_samples++;

        if (loop_dt_ms > PID_PERIOD_MS) {
            loop_overruns++;
        }

        /*
         * Store the current encoder sample.
         */
        index =
            iteration % (PID_WINDOW_SIZE + 1);

        left_history[index] = left_now;
        right_history[index] = right_now;
        time_history[index] = current_ms;

        /*
         * Calculate speed after collecting
         * the first 10 intervals.
         */
        if (iteration >= PID_WINDOW_SIZE) {

            old_index =
                (iteration - PID_WINDOW_SIZE)
                % (PID_WINDOW_SIZE + 1);

            elapsed_ms =
                current_ms - time_history[old_index];

            if (elapsed_ms <= 0) {
                tm_printf((UB*)"[B2] Invalid speed window\n");
                break;
            }

            left_speed =
                (INT)(
                    ((long long)
                    (left_now - left_history[old_index])
                    * 1000LL) / elapsed_ms
                );

            right_speed =
                (INT)(
                    ((long long)
                    (right_now - right_history[old_index])
                    * 1000LL) / elapsed_ms
                );

            /*
             * Calculate speed errors.
             */
            left_error =
                (float)(PID_TARGET_CPS - left_speed);

            right_error =
                (float)(PID_TARGET_CPS - right_speed);

            /*
             * PI integral update using actual
             * measured loop interval.
             */
            {
                float dt =
                    (float)loop_dt_ms / 1000.0f;

                if (dt <= 0.0f || dt > 0.1f) {
                    tm_printf((UB*)"[B2] PI timing out of range\n");
                    break;
                }

                left_integral +=
                    PID_KI * left_error * dt;

                right_integral +=
                    PID_KI * right_error * dt;
            }

            /*
             * Integral anti-windup limits.
             */
            if (left_integral > PID_I_LIMIT)
                left_integral = PID_I_LIMIT;

            if (left_integral < -PID_I_LIMIT)
                left_integral = -PID_I_LIMIT;

            if (right_integral > PID_I_LIMIT)
                right_integral = PID_I_LIMIT;

            if (right_integral < -PID_I_LIMIT)
                right_integral = -PID_I_LIMIT;

            /*
             * Calculate PI output.
             */
            left_output =
                PID_BASE_PWM +
                PID_KP * left_error +
                left_integral;

            right_output =
                PID_BASE_PWM +
                PID_KP * right_error +
                right_integral;

            /*
             * Clamp PWM outputs.
             */
            left_pwm =
                clamp_pwm((INT)(left_output + 0.5f));

            right_pwm =
                clamp_pwm((INT)(right_output + 0.5f));

            /*
             * Update both motors.
             */
            motor_left_set(left_pwm);
            motor_right_set(right_pwm);
        }

        /*
         * Print approximately every 500 ms,
         * assuming 10 ms activations.
         */
        if ((iteration % 50) == 0) {

            tm_printf(
                (UB*)"[B2] Cyclic PI L=%d R=%d PWM_L=%d PWM_R=%d\n",
                left_speed,
                right_speed,
                left_pwm,
                right_pwm
            );
        }
    }

test_end:

    /*
     * Safety: stop motors and cyclic wake-ups.
     */
    motor_stop();

    (void)subsystem2_stop_pid_cyclic();

    tm_printf(
        (UB*)"[B2] Cyclic PI test completed - motors stopped\n"
    );

    /*
     * Print control-loop timing statistics.
     */
    if (loop_samples > 0) {

        long long avg_x100 =
            (loop_total_ms * 100LL) / loop_samples;

        tm_printf(
            (UB*)"[B2] Timing min=%d max=%d avg=%d.%02d ms\n",
            (INT)loop_min_ms,
            (INT)loop_max_ms,
            (INT)(avg_x100 / 100),
            (INT)(avg_x100 % 100)
        );

        tm_printf(
            (UB*)"[B2] Timing samples=%d over10ms=%d invalid=%d\n",
            loop_samples,
            loop_overruns,
            loop_invalid
        );
    } else {

        tm_printf(
            (UB*)"[B2] No valid timing samples; invalid=%d\n",
            loop_invalid
        );
    }

    /*
     * Temporary task-return workaround.
     * Motors remain stopped.
     */
    while (1) {
        tk_dly_tsk(1000);
    }
}
