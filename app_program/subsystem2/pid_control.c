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

#include <stdint.h>
#include "pid_control.h"
#include "encoder_control.h"
#include "motion_command.h"
#include "b2_motor.h"
#define PID_BASE_PWM       20
#define PID_MAX_PWM        35
#define PID_MIN_PWM         0
#define PID_KP             0.002f
#define PID_KI             0.005f
#define PID_I_LIMIT       10.0f
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

typedef struct
{
    INT left_history[PID_WINDOW_SIZE + 1];
    INT right_history[PID_WINDOW_SIZE + 1];
    int64_t time_history[PID_WINDOW_SIZE + 1];
    motion_command_t active_command;
    motion_command_t received_command;
    INT left_now;
    INT right_now;
    INT left_speed;
    INT right_speed;
    INT left_pwm;
    INT right_pwm;
    float left_integral;
    float right_integral;
    int64_t current_ms;
    int64_t previous_ms;
    int64_t last_drive_ms;
    int64_t elapsed_ms;
    int64_t dt_ms;
    INT sample_count;
    INT index;
    INT old_index;
    INT direction_sign;
    INT target_speed;
    uint32_t last_seen_stop_generation;
} pid_state_t;
ID gh_pid_task = 0;

static ER pid_next_sample(pid_state_t *p_state);
static void pid_check_safety(pid_state_t *p_state);
static ER pid_accept_command(pid_state_t *p_state);
static void pid_check_watchdog(pid_state_t *p_state);
static ER pid_update_output(pid_state_t *p_state);
static void pid_compute_pwm(pid_state_t *p_state);
static ER pid_accept_drive(pid_state_t *p_state);

static INT clamp_pwm(INT pwm);
static int64_t pid_time_ms(const SYSTIM *p_time);
static ER pid_read_sample(
    INT *p_left,
    INT *p_right,
    int64_t *p_time_ms);
static ER pid_wait_activation(void);

void pid_init(void)
{
    b2_safety_force_stop();
    tm_printf((UB *)"[B2] Command-driven PI initialized\n");
}

/** @brief Wake the PI task without blocking in the cyclic handler. */
EXPORT void pid_cyclic_handler(void *p_exinf)
{
    (void)p_exinf;
    if (gh_pid_task > 0)
    {
        (void)tk_wup_tsk(gh_pid_task);
    }
}

void pid_control_task(INT stacd, void *p_exinf)
{
    pid_state_t state = {0};
    pid_state_t *p_state = &state;
    ER err;
    ER cmd_err;

    (void)stacd;
    (void)p_exinf;
    p_state->active_command.type = MOTION_CMD_STOP;
    p_state->active_command.direction = MOTION_DIR_STOP;
    p_state->last_seen_stop_generation = b2_safety_stop_generation();
    b2_safety_force_stop();
    tm_printf((UB *)"[B2] Controller ready - STOPPED\n");
    err = pid_wait_activation();

    if (E_OK != err)
    {
        goto controller_error;
    }
    err = pid_read_sample(
        &p_state->left_now, &p_state->right_now, &p_state->current_ms
    );

    if (E_OK != err)
    {
        goto controller_error;
    }
    p_state->previous_ms = p_state->current_ms;
    for (;;)
    {
        if (E_OK != pid_next_sample(p_state))
        {
            goto controller_error;
        }
        pid_check_safety(p_state);
        cmd_err = pid_accept_command(p_state);

        if (E_OK != cmd_err && E_TMOUT != cmd_err)
        {
            goto controller_error;
        }

        /*
        * Always check the DRIVE watchdog, even when
        * no new motion command has arrived.
        */
        pid_check_watchdog(p_state);
        /*
         * Stay stopped until a DRIVE command arrives.
         */
        if (MOTION_CMD_DRIVE != p_state->active_command.type)
        {
            continue;
        }
        cmd_err = pid_update_output(p_state);
        if (E_TMOUT == cmd_err)
        {
            continue;
        }
        if (E_OK != cmd_err)
        {
            goto controller_error;
        }
    }
controller_error:
    b2_request_stop();
    b2_safety_acknowledge_stop();
    tm_printf((UB *)"[B2] PI controller stopped due to error\n");
    for (;;)
    {
        tk_dly_tsk(1000);
    }
}

static INT clamp_pwm(INT pwm)
{
    if (pwm > PID_MAX_PWM)
    {
        return (PID_MAX_PWM);
    }
    if (pwm < PID_MIN_PWM)
    {
        return (PID_MIN_PWM);
    }
    return (pwm);
}

static int64_t pid_time_ms(const SYSTIM *p_time)
{
    return ((int64_t)p_time->hi * 4294967296LL)
    + (int64_t)p_time->lo;
}

static ER pid_read_sample(
    INT *p_left,
    INT *p_right,
    int64_t *p_time_ms)
{
    SYSTIM now;
    ER err;

    encoder_get_counts(p_left, p_right);
    err = tk_get_tim(&now);

    if (E_OK != err)
    {
        return (err);
    }
    *p_time_ms = pid_time_ms(&now);
    return (E_OK);
}

/** @brief Discard old wakeups and wait for a fresh activation. */
static ER pid_wait_activation(void)
{
    (void)tk_can_wup(TSK_SELF);
    return (tk_slp_tsk(TMO_FEVR));
}

/** @brief Cancel motion and acknowledge a safety STOP when required. */
static void pid_check_safety(pid_state_t *p_state)
{
    /*
     * Check Buddy 5 safety state before accepting
     * any motion command.
     */
    uint32_t current_stop_generation =
        b2_safety_stop_generation();

    if (current_stop_generation != p_state->last_seen_stop_generation ||
        b2_safety_stop_requested() ||
        !b2_safety_motion_allowed())
    {
        p_state->last_seen_stop_generation = current_stop_generation;
        if (MOTION_CMD_DRIVE == p_state->active_command.type)
        {
            tm_printf(
                (UB *)"[B2] DRIVE cancelled by safety interlock\n"
            );
        }
        b2_safety_acknowledge_stop();
        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;
        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
    }
}

/** @brief Process one pending STOP or DRIVE command. */
static ER pid_accept_command(pid_state_t *p_state)
{
    ER cmd_err;
    /*
     * Check for a new command.
     */
    cmd_err = motion_receive_command(
        &p_state->received_command
    );

    if (E_OK == cmd_err)
    {
        if (MOTION_CMD_STOP == p_state->received_command.type)
        {
            p_state->active_command = p_state->received_command;
            b2_safety_force_stop();
            p_state->left_integral = 0.0f;
            p_state->right_integral = 0.0f;
            p_state->left_pwm = 0;
            p_state->right_pwm = 0;
            p_state->sample_count = 0;
            tm_printf((UB *)"[B2] STOP received\n");
        }
        else if (MOTION_CMD_DRIVE == p_state->received_command.type)
        {
            return (pid_accept_drive(p_state));
        }
        else
        {
            /* No additional action is required. */
        }
    }
    else if (E_TMOUT != cmd_err)
    {
        return (E_SYS);
    }
    else
    {
        /* No additional action is required. */
    }
    return (E_OK);
}

/** @brief Cancel expired DRIVE commands or outdated authorization. */
static void pid_check_watchdog(pid_state_t *p_state)
{
    /*
     * DRIVE watchdog.
     */
    if (MOTION_CMD_DRIVE == p_state->active_command.type &&
        p_state->current_ms - p_state->last_drive_ms >= PID_WATCHDOG_MS)
    {
        b2_safety_force_stop();
        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;
        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
        tm_printf((UB *)"[B2] DRIVE watchdog timeout\n");
    }
    /*
     * An active DRIVE must remain authorized under
     * the current safety STOP generation.
     *
     * This prevents an old DRIVE command from
     * continuing after a STOP and subsequent clearance.
     */
    if (MOTION_CMD_DRIVE == p_state->active_command.type &&
        p_state->active_command.safety_generation !=
            b2_safety_stop_generation())
        {
        b2_safety_force_stop();
        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;
        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
        tm_printf(
            (UB *)"[B2] Active DRIVE cancelled: outdated safety generation\n"
        );
    }
}

/** @brief Estimate speed, calculate PWM, and apply safety gating. */
static ER pid_update_output(pid_state_t *p_state)
{
    /*
     * Select direction.
     */
    p_state->direction_sign =
        (MOTION_DIR_REVERSE == p_state->active_command.direction)
        ? -1 : 1;
    p_state->target_speed = p_state->active_command.target_speed_cps;
    /*
     * Save speed-history indexing before building
     */
    p_state->index = p_state->sample_count % (PID_WINDOW_SIZE + 1);
    /* Reserve current slot; calculate using older sample first. */
    if (p_state->sample_count >= PID_WINDOW_SIZE)
    {
        p_state->old_index =
            (p_state->sample_count - PID_WINDOW_SIZE)
            % (PID_WINDOW_SIZE + 1);
        p_state->elapsed_ms =
            p_state->current_ms - p_state->time_history[p_state->old_index];
        if (p_state->elapsed_ms <= 0)
        {
            return (E_SYS);
        }
        p_state->left_speed =
            (INT)(((int64_t)(
                p_state->left_now - p_state->left_history[p_state->old_index])
                * 1000LL) / p_state->elapsed_ms);
        p_state->right_speed =
            (INT)(((int64_t)(
                p_state->right_now - p_state->right_history[p_state->old_index])
                * 1000LL) / p_state->elapsed_ms);
        pid_compute_pwm(p_state);
    }
    else
    {
        p_state->left_pwm = PID_BASE_PWM;
        p_state->right_pwm = PID_BASE_PWM;
    }
    /* Save current encoder sample after speed calculation. */
    p_state->left_history[p_state->index] = p_state->left_now;
    p_state->right_history[p_state->index] = p_state->right_now;
    p_state->time_history[p_state->index] = p_state->current_ms;
    /*
     * Apply signed PWM through the Buddy 5
     * safety interface.
     */
    b2_safety_apply_pwm(
        p_state->direction_sign * p_state->left_pwm,
        p_state->direction_sign * p_state->right_pwm,
        p_state->active_command.safety_generation
    );
    p_state->sample_count++;
    if ((p_state->sample_count % 50) == 0)
    {
        tm_printf(
            (UB *)"[B2] PI L=%d R=%d PWM_L=%d PWM_R=%d\n",
            p_state->left_speed,
            p_state->right_speed,
            p_state->left_pwm,
            p_state->right_pwm
        );
    }
    return (E_OK);
}

/** @brief Integrate speed error and clamp both wheel PWM outputs. */
static void pid_compute_pwm(pid_state_t *p_state)
{
    /*
     * Compare speed magnitude with target.
     * Encoder direction is checked separately.
     */
    {
        float left_error =
            (float)(p_state->target_speed -
                p_state->direction_sign * p_state->left_speed);
        float right_error =
            (float)(p_state->target_speed -
                p_state->direction_sign * p_state->right_speed);
        float interval_seconds = (float)p_state->dt_ms / 1000.0f;
        float left_output;
        float right_output;
        p_state->left_integral += PID_KI * left_error * interval_seconds;
        p_state->right_integral += PID_KI * right_error * interval_seconds;
        if (p_state->left_integral > PID_I_LIMIT)
        {
            p_state->left_integral = PID_I_LIMIT;
        }
        if (p_state->left_integral < -PID_I_LIMIT)
        {
            p_state->left_integral = -PID_I_LIMIT;
        }
        if (p_state->right_integral > PID_I_LIMIT)
        {
            p_state->right_integral = PID_I_LIMIT;
        }
        if (p_state->right_integral < -PID_I_LIMIT)
        {
            p_state->right_integral = -PID_I_LIMIT;
        }
        left_output =
            PID_BASE_PWM +
            PID_KP * left_error +
            p_state->left_integral;
        right_output =
            PID_BASE_PWM +
            PID_KP * right_error +
            p_state->right_integral;
        p_state->left_pwm = clamp_pwm(
            (INT)(left_output + 0.5f)
        );
        p_state->right_pwm = clamp_pwm(
            (INT)(right_output + 0.5f)
        );
    }
}

/** @brief Validate DRIVE authorization before replacing the command. */
static ER pid_accept_drive(pid_state_t *p_state)
{
    /*
     * Reject DRIVE if its authorization is outdated
     * or the safety interlock does not permit motion.
     */
    if (p_state->received_command.safety_generation !=
            b2_safety_stop_generation() ||
            !b2_safety_motion_allowed())
        {
        tm_printf(
            (UB *)"[B2] DRIVE rejected: safety interlock active\n"
        );
        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;
        /*
         * Rejecting DRIVE must not acknowledge
         * a separate safety STOP event.
         */
        b2_safety_force_stop();
        return (E_TMOUT);
    }
    /*
     * Reset PI history when starting or
     * changing direction/target speed.
     */
    if (MOTION_CMD_DRIVE != p_state->active_command.type ||
        p_state->active_command.direction
            != p_state->received_command.direction ||
        p_state->active_command.target_speed_cps !=
            p_state->received_command.target_speed_cps)
        {
        b2_safety_force_stop();
        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->sample_count = 0;
        tm_printf(
            (UB *)"[B2] DRIVE target=%d direction=%d\n",
            p_state->received_command.target_speed_cps,
            p_state->received_command.direction
        );
    }
    p_state->active_command = p_state->received_command;
    p_state->last_drive_ms = p_state->current_ms;
    return (E_OK);
}

/** @brief Read a fresh sample and validate the control interval. */
static ER pid_next_sample(pid_state_t *p_state)
{
    ER err;
    err = pid_wait_activation();

    if (E_OK != err)
    {
        return (E_SYS);
    }
    err = pid_read_sample(
        &p_state->left_now, &p_state->right_now, &p_state->current_ms
    );

    if (E_OK != err)
    {
        return (E_SYS);
    }
    p_state->dt_ms = p_state->current_ms - p_state->previous_ms;
    p_state->previous_ms = p_state->current_ms;
    if (p_state->dt_ms <= 0 || p_state->dt_ms > PID_MAX_INTERVAL_MS)
    {
        tm_printf(
            (UB *)"[B2] Invalid PI interval: %d ms\n",
            (INT)p_state->dt_ms
        );
        return (E_SYS);
    }
    return (E_OK);
}
