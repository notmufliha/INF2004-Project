/*
 * Buddy 2 - Command-Driven PI Motor Controller
 * RP2040 + micro T-Kernel 3.0
 *
 * Initial implementation:
 * - DRIVE forward/reverse
 * - STOP
 * - 100 ms DRIVE watchdog
 * - 10 ms cyclic PI control
 * - 10-sample speed estimation
 *
 * Test with wheels off the ground.
 */

#include "../app.h"
#include "pid_control.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "motion_command.h"
#include "b2_motor.h"

#define PID_BASE_PWM       20
#define PID_MAX_PWM        35
#define PID_MIN_PWM         0

#define PID_KP             0.002f
#define PID_KI             0.005f
#define PID_I_LIMIT       10.0f

#define PID_PERIOD_MS     10
#define PID_WINDOW_SIZE   10
#define PID_WATCHDOG_MS  100

/*
 * Maximum permitted interval before the
 * PI controller enters its safe error state.
 *
 * Preliminary threshold: verify through
 * timing measurements on hardware.
 */
#define PID_MAX_INTERVAL_MS 20

ID tid_pid = 0;

static INT clamp_pwm(INT pwm)
{
    if (pwm > PID_MAX_PWM)
        return PID_MAX_PWM;

    if (pwm < PID_MIN_PWM)
        return PID_MIN_PWM;

    return pwm;
}

void pid_init(void)
{
    b2_safety_force_stop();
    tm_printf((UB*)"[B2] Command-driven PI initialized\n");
}

/*
 * Cyclic handler: wake the PI task.
 * Keep this short and non-blocking.
 */
EXPORT void pid_cyclic_handler(void *exinf)
{
    (void)exinf;

    if (tid_pid > 0) {
        (void)tk_wup_tsk(tid_pid);
    }
}

static long long pid_time_ms(const SYSTIM *time)
{
    return ((long long)time->hi * 4294967296LL)
         + (unsigned long long)time->lo;
}

static ER pid_read_sample(
    INT *left,
    INT *right,
    long long *time_ms)
{
    SYSTIM now;
    ER err;

    encoder_get_counts(left, right);

    err = tk_get_tim(&now);

    if (err != E_OK)
        return err;

    *time_ms = pid_time_ms(&now);
    return E_OK;
}

/*
 * Synchronize with a fresh cyclic activation.
 */
static ER pid_wait_activation(void)
{
    (void)tk_can_wup(TSK_SELF);
    return tk_slp_tsk(TMO_FEVR);
}

void pid_control_task(INT stacd, void *exinf)
{
    INT left_history[PID_WINDOW_SIZE + 1] = {0};
    INT right_history[PID_WINDOW_SIZE + 1] = {0};
    long long time_history[PID_WINDOW_SIZE + 1] = {0};

    MOTION_COMMAND active_command = {0};
    MOTION_COMMAND received_command;

    INT left_now = 0;
    INT right_now = 0;
    INT left_speed = 0;
    INT right_speed = 0;

    INT left_pwm = 0;
    INT right_pwm = 0;

    float left_integral = 0.0f;
    float right_integral = 0.0f;

    long long current_ms = 0;
    long long previous_ms = 0;
    long long last_drive_ms = 0;
    long long elapsed_ms;
    long long dt_ms;

    INT sample_count = 0;
    INT index;
    INT old_index;
    INT direction_sign;
    INT target_speed;

    ER err;
    ER cmd_err;
    bool safety_stop_latched = false;
    uint32_t last_seen_stop_generation;

    (void)stacd;
    (void)exinf;

    active_command.type = MOTION_CMD_STOP;
    active_command.direction = MOTION_DIR_STOP;

    last_seen_stop_generation = b2_safety_stop_generation();

    b2_safety_force_stop();

    tm_printf((UB*)"[B2] Controller ready - STOPPED\n");

    err = pid_wait_activation();

    if (err != E_OK)
        goto controller_error;

    err = pid_read_sample(
        &left_now, &right_now, &current_ms
    );

    if (err != E_OK)
        goto controller_error;

    previous_ms = current_ms;

    while (1) {

        err = pid_wait_activation();

        if (err != E_OK)
            goto controller_error;

        err = pid_read_sample(
            &left_now, &right_now, &current_ms
        );

        if (err != E_OK)
            goto controller_error;

        dt_ms = current_ms - previous_ms;
        previous_ms = current_ms;

        if (dt_ms <= 0 || dt_ms > PID_MAX_INTERVAL_MS) {
            tm_printf(
                (UB*)"[B2] Invalid PI interval: %d ms\n",
                (INT)dt_ms
            );
            goto controller_error;
        }

        /*
        * Check Buddy 5 safety state before accepting
        * any motion command.
        */
        uint32_t current_stop_generation =
            b2_safety_stop_generation();

        if (current_stop_generation != last_seen_stop_generation ||
            b2_safety_stop_requested() ||
            !b2_safety_motion_allowed()) {

            last_seen_stop_generation = current_stop_generation;

            if (active_command.type == MOTION_CMD_DRIVE) {
                tm_printf(
                    (UB*)"[B2] DRIVE cancelled by safety interlock\n"
                );
            }

            b2_safety_acknowledge_stop();

            active_command.type = MOTION_CMD_STOP;
            active_command.direction = MOTION_DIR_STOP;

            left_integral = 0.0f;
            right_integral = 0.0f;
            left_pwm = 0;
            right_pwm = 0;
            sample_count = 0;

            safety_stop_latched = true;
        }

        /*
        * Check for a new command.
        */
        cmd_err = motion_receive_command(
            &received_command
        );

        if (cmd_err == E_OK) {

            if (received_command.type == MOTION_CMD_STOP) {

                active_command = received_command;

                b2_safety_force_stop();

                left_integral = 0.0f;
                right_integral = 0.0f;
                left_pwm = 0;
                right_pwm = 0;
                sample_count = 0;

                tm_printf((UB*)"[B2] STOP received\n");

            } else if (received_command.type == MOTION_CMD_DRIVE) {

                /*
                * Reject DRIVE if its authorization is outdated
                * or the safety interlock does not permit motion.
                */
                if (received_command.safety_generation !=
                        b2_safety_stop_generation() ||
                    !b2_safety_motion_allowed()) {

                    tm_printf(
                        (UB*)"[B2] DRIVE rejected: safety interlock active\n"
                    );

                    active_command.type = MOTION_CMD_STOP;
                    active_command.direction = MOTION_DIR_STOP;

                    /*
                    * Rejecting DRIVE must not acknowledge
                    * a separate safety STOP event.
                    */
                    b2_safety_force_stop();

                    continue;
                }

                safety_stop_latched = false;

                /*
                 * Reset PI history when starting or
                 * changing direction/target speed.
                 */
                if (active_command.type != MOTION_CMD_DRIVE ||
                    active_command.direction != received_command.direction ||
                    active_command.target_speed_cps !=
                        received_command.target_speed_cps) {

                    b2_safety_force_stop();

                    left_integral = 0.0f;
                    right_integral = 0.0f;
                    sample_count = 0;

                    tm_printf(
                        (UB*)"[B2] DRIVE target=%d direction=%d\n",
                        received_command.target_speed_cps,
                        received_command.direction
                    );
                }

                active_command = received_command;
                last_drive_ms = current_ms;
            }

        } else if (cmd_err != E_TMOUT) {
            goto controller_error;
        }

        /*
         * DRIVE watchdog.
         */
        if (active_command.type == MOTION_CMD_DRIVE &&
            current_ms - last_drive_ms >= PID_WATCHDOG_MS) {

            b2_safety_force_stop();

            active_command.type = MOTION_CMD_STOP;
            active_command.direction = MOTION_DIR_STOP;

            left_integral = 0.0f;
            right_integral = 0.0f;
            sample_count = 0;

            tm_printf((UB*)"[B2] DRIVE watchdog timeout\n");
        }

        /*
        * An active DRIVE must remain authorized under
        * the current safety STOP generation.
        *
        * This prevents an old DRIVE command from
        * continuing after a STOP and subsequent clearance.
        */
        if (active_command.type == MOTION_CMD_DRIVE &&
            active_command.safety_generation !=
                b2_safety_stop_generation()) {

            b2_safety_force_stop();

            active_command.type = MOTION_CMD_STOP;
            active_command.direction = MOTION_DIR_STOP;

            left_integral = 0.0f;
            right_integral = 0.0f;
            left_pwm = 0;
            right_pwm = 0;
            sample_count = 0;

            tm_printf(
                (UB*)"[B2] Active DRIVE cancelled: outdated safety generation\n"
            );
        }

        /*
         * Stay stopped until a DRIVE command arrives.
         */
        if (active_command.type != MOTION_CMD_DRIVE) {
            continue;
        }

        /*
         * Select direction.
         */
        direction_sign =
            (active_command.direction == MOTION_DIR_REVERSE)
            ? -1 : 1;

        target_speed = active_command.target_speed_cps;

        /*
         * Save speed-history indexing before building
         */
        index = sample_count % (PID_WINDOW_SIZE + 1);

        /* Reserve current slot; calculate using older sample first. */

        if (sample_count >= PID_WINDOW_SIZE) {

            old_index =
                (sample_count - PID_WINDOW_SIZE)
                % (PID_WINDOW_SIZE + 1);

            elapsed_ms =
                current_ms - time_history[old_index];

            if (elapsed_ms <= 0)
                goto controller_error;

            left_speed =
                (INT)(((long long)(
                    left_now - left_history[old_index])
                    * 1000LL) / elapsed_ms);

            right_speed =
                (INT)(((long long)(
                    right_now - right_history[old_index])
                    * 1000LL) / elapsed_ms);

            /*
             * Compare speed magnitude with target.
             * Encoder direction is checked separately.
             */
            {
                float left_error =
                    (float)(target_speed -
                        direction_sign * left_speed);

                float right_error =
                    (float)(target_speed -
                        direction_sign * right_speed);

                float dt = (float)dt_ms / 1000.0f;

                float left_output;
                float right_output;

                left_integral += PID_KI * left_error * dt;
                right_integral += PID_KI * right_error * dt;

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

                left_pwm = clamp_pwm(
                    (INT)(left_output + 0.5f)
                );

                right_pwm = clamp_pwm(
                    (INT)(right_output + 0.5f)
                );
            }
        } else {
            left_pwm = PID_BASE_PWM;
            right_pwm = PID_BASE_PWM;
        }

        /* Save current encoder sample after speed calculation. */
        left_history[index] = left_now;
        right_history[index] = right_now;
        time_history[index] = current_ms;

        /*
        * Apply signed PWM through the Buddy 5
        * safety interface.
        */
        b2_safety_apply_pwm(
            direction_sign * left_pwm,
            direction_sign * right_pwm,
            active_command.safety_generation
        );

        sample_count++;

        if ((sample_count % 50) == 0) {
            tm_printf(
                (UB*)"[B2] PI L=%d R=%d PWM_L=%d PWM_R=%d\n",
                left_speed,
                right_speed,
                left_pwm,
                right_pwm
            );
        }
    }

controller_error:

    b2_request_stop();
    b2_safety_acknowledge_stop();

    tm_printf((UB*)"[B2] PI controller stopped due to error\n");

    while (1) {
        tk_dly_tsk(1000);
    }
}
