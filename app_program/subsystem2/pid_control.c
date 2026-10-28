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
#include "../subsystem1/subsystem1.h"
#include <stdbool.h>
#define PID_BASE_PWM       20
#define PID_MAX_PWM        35
#define PID_MIN_PWM         0
#define PID_KP             0.002f
#define PID_KI             0.005f
#define PID_I_LIMIT       10.0f
#define PID_WINDOW_SIZE   10
#define PID_WATCHDOG_MS  100
#define PID_MOVE_TIMEOUT_MS 10000
#define PID_TURN_TIMEOUT_MS 10000
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
    /* Latest execution result retained across control cycles. */
    motion_status_t motion_status;
    /* Preserve original command type after transitioning to STOP. */
    motion_cmd_type_t status_command_type;
    motion_command_t received_command;
    INT left_now;
    INT right_now;
    /* Previous encoder samples for Buddy 1 telemetry. */
    INT telemetry_prev_left;
    INT telemetry_prev_right;

    /* Previous timestamp for telemetry speed calculation. */
    int64_t telemetry_prev_ms;

    /* Cumulative absolute wheel travel in millimetres. */
    float telemetry_distance_mm;

    /* Whether the first telemetry sample has been captured. */
    bool telemetry_initialized;

    INT telemetry_left_speed_cps;
    INT telemetry_right_speed_cps;

    /* Encoder positions captured when a MOVE begins. */
    INT move_start_left;
    INT move_start_right;

    /* Encoder positions captured when a TURN begins. */
    INT turn_start_left;
    INT turn_start_right;

    /* Timestamp used to enforce the TURN timeout. */
    int64_t turn_start_ms;

    INT left_speed;
    INT right_speed;
    INT left_pwm;
    INT right_pwm;
    float left_integral;
    float right_integral;
    int64_t current_ms;
    int64_t previous_ms;
    int64_t last_drive_ms;
    int64_t move_start_ms;
    int64_t elapsed_ms;
    int64_t dt_ms;
    INT sample_count;
    INT index;
    INT old_index;
    INT left_direction_sign;
    INT right_direction_sign;
    INT target_speed;
    INT left_target_speed;
    INT right_target_speed;
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
static ER pid_accept_move(pid_state_t *p_state);
static ER pid_accept_turn(pid_state_t *p_state);
static bool pid_move_target_reached(pid_state_t *p_state);
static bool pid_turn_target_reached(pid_state_t *p_state);
static void pid_update_telemetry(pid_state_t *p_state);
static void pid_publish_status(pid_state_t *p_state);

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
    p_state->motion_status = MOTION_STATUS_IDLE;
    p_state->status_command_type = MOTION_CMD_STOP;
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
        /*
        * Update Buddy 1 telemetry every sampling cycle,
        * including when the motors are stopped.
        */
        pid_update_telemetry(p_state);
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
        * Publish the latest execution state after
        * processing safety, commands and timeouts.
        */
        pid_publish_status(p_state);

        /*
         * Stay stopped until a DRIVE command arrives.
         */
        /*
        * Execute DRIVE or MOVE commands only.
        * TURN will be implemented separately.
        */
        /*
        * Execute DRIVE, MOVE, or TURN.
        */
        if (MOTION_CMD_DRIVE != p_state->active_command.type &&
            MOTION_CMD_MOVE != p_state->active_command.type &&
            MOTION_CMD_TURN != p_state->active_command.type)
        {
            continue;
        }
        /*
        * Stop a distance-limited MOVE once the target
        * distance has been reached.
        */
        if (MOTION_CMD_MOVE == p_state->active_command.type &&
            pid_move_target_reached(p_state))
        {
            /*
            * Immediately disable motor output.
            * The safety generation may change, so this
            * command must not resume automatically.
            */
            b2_safety_force_stop();
            p_state->motion_status = MOTION_STATUS_COMPLETED;
            pid_publish_status(p_state);
            p_state->active_command.type = MOTION_CMD_STOP;
            p_state->active_command.direction = MOTION_DIR_STOP;

            p_state->left_integral = 0.0f;
            p_state->right_integral = 0.0f;
            p_state->left_pwm = 0;
            p_state->right_pwm = 0;
            p_state->sample_count = 0;

            tm_printf((UB *)"[B2] MOVE completed: target reached\n");
            continue;
        }
        /*
        * Stop an in-place TURN when both wheels
        * have reached their required travel distance.
        */
        if (MOTION_CMD_TURN == p_state->active_command.type &&
            pid_turn_target_reached(p_state))
        {
            b2_safety_force_stop();
            p_state->motion_status = MOTION_STATUS_COMPLETED;
            pid_publish_status(p_state);
            p_state->active_command.type = MOTION_CMD_STOP;
            p_state->active_command.direction = MOTION_DIR_STOP;

            p_state->left_integral = 0.0f;
            p_state->right_integral = 0.0f;
            p_state->left_pwm = 0;
            p_state->right_pwm = 0;
            p_state->sample_count = 0;

            tm_printf((UB *)"[B2] TURN completed: target reached\n");
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
    /*
     * Request a safety stop before reporting
     * the controller fault.
     */
    b2_request_stop();
    b2_safety_acknowledge_stop();

    /*
     * Record and publish the fault so that
     * the Vehicle Controller can detect it.
     */
    p_state->motion_status = MOTION_STATUS_FAULT;
    p_state->active_command.type = MOTION_CMD_STOP;
    p_state->active_command.direction = MOTION_DIR_STOP;

    pid_publish_status(p_state);

    tm_printf((UB *)"[B2] PI controller stopped due to error\n");
    for (;;)
    {
        tk_dly_tsk(1000);
    }
}

/**
 * @brief Check whether a distance-limited MOVE has reached its target.
 *
 * The requested distance is expressed in left-wheel reference counts.
 * Both wheel distances are converted to centimetres before averaging.
 */
static bool pid_move_target_reached(pid_state_t *p_state)
{
    float left_cm;
    float right_cm;
    float target_cm;
    float travelled_cm;

    if (MOTION_CMD_MOVE != p_state->active_command.type)
    {
        return false;
    }

    encoder_get_distance_cm(
        p_state->move_start_left,
        p_state->move_start_right,
        &left_cm,
        &right_cm
    );

    /*
     * Convert the reference-wheel target from encoder
     * counts into centimetres.
     */
    target_cm =
        ((float)p_state->active_command.target_distance_counts *
         3.14159265f * ENC_WHEEL_DIAMETER_CM) /
        ENC_LEFT_COUNTS_PER_REV;

    /*
     * Account for forward and reverse movement.
     */
    if (MOTION_DIR_REVERSE == p_state->active_command.direction)
    {
        left_cm = -left_cm;
        right_cm = -right_cm;
    }

    travelled_cm = (left_cm + right_cm) * 0.5f;

    return (travelled_cm >= target_cm);
}

/**
 * @brief Check whether an in-place TURN has reached its target angle.
 *
 * Both wheels travel the same physical distance in
 * opposite directions during an in-place turn.
 */
static bool pid_turn_target_reached(pid_state_t *p_state)
{
    float left_cm;
    float right_cm;
    float target_cm;
    float left_progress;
    float right_progress;

    if (MOTION_CMD_TURN != p_state->active_command.type)
    {
        return false;
    }

    encoder_get_distance_cm(
        p_state->turn_start_left,
        p_state->turn_start_right,
        &left_cm,
        &right_cm
    );

    /*
     * Calculate the required wheel travel from the
     * requested turn angle and wheel track width.
     */
    target_cm =
        (3.14159265f * ENC_WHEEL_TRACK_CM *
         (float)p_state->active_command.target_angle_deg) /
        360.0f;

    /*
     * Convert signed encoder travel into positive
     * progress in the expected turning direction.
     */
    if (MOTION_TURN_LEFT ==
        p_state->active_command.turn_direction)
    {
        left_progress = -left_cm;
        right_progress = right_cm;
    }
    else
    {
        left_progress = left_cm;
        right_progress = -right_cm;
    }

    /*
     * Require both wheels to reach their target.
     * This avoids declaring completion when only
     * one wheel has travelled far enough.
     */
    return (left_progress >= target_cm &&
            right_progress >= target_cm);
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
        if (MOTION_CMD_DRIVE == p_state->active_command.type ||
            MOTION_CMD_MOVE == p_state->active_command.type ||
            MOTION_CMD_TURN == p_state->active_command.type)
        {
            p_state->motion_status = MOTION_STATUS_CANCELLED;

            tm_printf(
                (UB *)"[B2] Movement cancelled by safety interlock\n"
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
            /*
            * Preserve the interrupted movement's command ID
            * so its cancellation can be identified.
            */
            if (MOTION_CMD_DRIVE == p_state->active_command.type ||
                MOTION_CMD_MOVE == p_state->active_command.type ||
                MOTION_CMD_TURN == p_state->active_command.type)
            {
                p_state->motion_status = MOTION_STATUS_CANCELLED;
            }

            /*
            * Change only the active type and direction.
            * Retain the previous command ID.
            */
            p_state->active_command.type = MOTION_CMD_STOP;
            p_state->active_command.direction = MOTION_DIR_STOP;

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
        else if (MOTION_CMD_MOVE == p_state->received_command.type)
        {
            return (pid_accept_move(p_state));
        }
        else if (MOTION_CMD_TURN == p_state->received_command.type)
        {
            return (pid_accept_turn(p_state));
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
        p_state->motion_status = MOTION_STATUS_TIMEOUT;
        tm_printf((UB *)"[B2] DRIVE watchdog timeout\n");
    }
    /*
     * All active movement commands must remain authorized
     * under the current safety STOP generation.
     *
     * This prevents an old DRIVE command from
     * continuing after a STOP and subsequent clearance.
     */
    if ((MOTION_CMD_DRIVE == p_state->active_command.type ||
        MOTION_CMD_MOVE == p_state->active_command.type ||
        MOTION_CMD_TURN == p_state->active_command.type) &&
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
        p_state->motion_status = MOTION_STATUS_CANCELLED;
        tm_printf(
            (UB *)"[B2] Active movement cancelled: outdated safety generation\n"
        );
    }
    /*
    * Stop MOVE if it exceeds the maximum permitted duration.
    * This protects against stalled motors or missing encoder pulses.
    */
    if (MOTION_CMD_MOVE == p_state->active_command.type &&
        p_state->current_ms - p_state->move_start_ms >=
            PID_MOVE_TIMEOUT_MS)
    {
        b2_safety_force_stop();

        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;

        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
        p_state->motion_status = MOTION_STATUS_TIMEOUT;
        tm_printf((UB *)"[B2] MOVE timeout: movement cancelled\n");
    }

    /*
    * Cancel TURN if it exceeds the permitted duration.
    * Prevents indefinite motor operation if a wheel
    * stalls or encoder feedback is missing.
    */
    if (MOTION_CMD_TURN == p_state->active_command.type &&
        p_state->current_ms - p_state->turn_start_ms >=
            PID_TURN_TIMEOUT_MS)
    {
        b2_safety_force_stop();

        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;

        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
        p_state->motion_status = MOTION_STATUS_TIMEOUT;
        tm_printf((UB *)"[B2] TURN timeout: movement cancelled\n");
    }
}

/** @brief Estimate speed, calculate PWM, and apply safety gating. */
static ER pid_update_output(pid_state_t *p_state)
{
    /*
     * Select direction.
     */
    /*
    * Select each wheel's direction independently.
    */
    if (MOTION_CMD_TURN == p_state->active_command.type)
    {
        if (MOTION_TURN_LEFT ==
            p_state->active_command.turn_direction)
        {
            p_state->left_direction_sign = -1;
            p_state->right_direction_sign = 1;
        }
        else
        {
            p_state->left_direction_sign = 1;
            p_state->right_direction_sign = -1;
        }
    }
    else
    {
        INT sign =
            (MOTION_DIR_REVERSE == p_state->active_command.direction)
            ? -1 : 1;

        p_state->left_direction_sign = sign;
        p_state->right_direction_sign = sign;
    }
    /*
    * Convert the reference speed into wheel-specific
    * encoder targets to maintain equal physical speeds.
    *
    * target_speed_cps is referenced to the left wheel.
    */
    p_state->target_speed = p_state->active_command.target_speed_cps;

    p_state->left_target_speed = p_state->target_speed;

    p_state->right_target_speed =
        (INT)(((int64_t)p_state->target_speed *
            ENC_RIGHT_COUNTS_PER_REV) /
            ENC_LEFT_COUNTS_PER_REV);
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
        p_state->left_direction_sign * p_state->left_pwm,
        p_state->right_direction_sign * p_state->right_pwm,
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
            (float)(p_state->left_target_speed -
                p_state->left_direction_sign * p_state->left_speed);

        float right_error =
            (float)(p_state->right_target_speed -
                p_state->right_direction_sign * p_state->right_speed);
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

        /*
        * A rejected DRIVE must not leave a previous
        * movement reported as RUNNING.
        */
        if (MOTION_CMD_DRIVE == p_state->active_command.type ||
            MOTION_CMD_MOVE == p_state->active_command.type ||
            MOTION_CMD_TURN == p_state->active_command.type)
        {
            p_state->motion_status = MOTION_STATUS_CANCELLED;
        }

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
    p_state->status_command_type = p_state->active_command.type;
    p_state->motion_status = MOTION_STATUS_RUNNING;
    p_state->last_drive_ms = p_state->current_ms;
    return (E_OK);
}

/** @brief Accept a distance-limited MOVE command. */
static ER pid_accept_move(pid_state_t *p_state)
{
    /*
     * Reject movement without valid safety authorization.
     */
    if (p_state->received_command.safety_generation !=
            b2_safety_stop_generation() ||
        !b2_safety_motion_allowed())
    {
        tm_printf((UB *)"[B2] MOVE rejected: safety interlock\n");

        /* Fail safe if authorization is no longer valid. */
        if (MOTION_CMD_DRIVE == p_state->active_command.type ||
            MOTION_CMD_MOVE == p_state->active_command.type ||
            MOTION_CMD_TURN == p_state->active_command.type)
        {
            p_state->motion_status = MOTION_STATUS_CANCELLED;
        }

        b2_safety_force_stop();
        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;
        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;
        return (E_TMOUT);
    }
    /*
     * Stop the previous movement before starting MOVE.
     * This does not change the safety generation.
     */
    b2_safety_force_stop();

    /*
     * Capture the encoder positions at the start
     * of this movement.
     */
    p_state->move_start_left = p_state->left_now;
    p_state->move_start_right = p_state->right_now;
    p_state->move_start_ms = p_state->current_ms;

    /*
     * Reset the PI controller for a new movement.
     */
    p_state->left_integral = 0.0f;
    p_state->right_integral = 0.0f;
    p_state->left_pwm = 0;
    p_state->right_pwm = 0;
    p_state->sample_count = 0;

    p_state->active_command = p_state->received_command;
    p_state->status_command_type = p_state->active_command.type;
    p_state->motion_status = MOTION_STATUS_RUNNING;

    tm_printf(
        (UB *)"[B2] MOVE accepted: distance=%d counts speed=%d cps\n",
        p_state->active_command.target_distance_counts,
        p_state->active_command.target_speed_cps
    );

    return (E_OK);
}

/** @brief Accept an encoder-based in-place TURN command. */
static ER pid_accept_turn(pid_state_t *p_state)
{
    /*
     * Reject TURN if safety authorization is outdated
     * or Buddy 5 has not permitted movement.
     */
    if (p_state->received_command.safety_generation !=
            b2_safety_stop_generation() ||
        !b2_safety_motion_allowed())
    {
        tm_printf((UB *)"[B2] TURN rejected: safety interlock\n");

        /*
        * Fail safe: stop any currently active movement
        * when safety authorization is invalid.
        */
        if (MOTION_CMD_DRIVE == p_state->active_command.type ||
            MOTION_CMD_MOVE == p_state->active_command.type ||
            MOTION_CMD_TURN == p_state->active_command.type)
        {
            p_state->motion_status = MOTION_STATUS_CANCELLED;
        }

        b2_safety_force_stop();

        p_state->active_command.type = MOTION_CMD_STOP;
        p_state->active_command.direction = MOTION_DIR_STOP;

        p_state->left_integral = 0.0f;
        p_state->right_integral = 0.0f;
        p_state->left_pwm = 0;
        p_state->right_pwm = 0;
        p_state->sample_count = 0;

        return (E_TMOUT);
    }
    /*
     * Stop the previous movement before starting TURN.
     * This does not change the safety generation.
     */
    b2_safety_force_stop();

    /*
     * Record the starting encoder positions and time.
     */
    p_state->turn_start_left = p_state->left_now;
    p_state->turn_start_right = p_state->right_now;
    p_state->turn_start_ms = p_state->current_ms;

    /*
     * Reset PI control for the new manoeuvre.
     */
    p_state->left_integral = 0.0f;
    p_state->right_integral = 0.0f;
    p_state->left_pwm = 0;
    p_state->right_pwm = 0;
    p_state->sample_count = 0;

    p_state->active_command = p_state->received_command;
    p_state->status_command_type = p_state->active_command.type;
    p_state->motion_status = MOTION_STATUS_RUNNING;

    tm_printf(
        (UB *)"[B2] TURN accepted: direction=%d angle=%d degrees\n",
        p_state->active_command.turn_direction,
        p_state->active_command.target_angle_deg
    );

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

/** @brief Update Buddy 1 motion telemetry from encoder samples. */
static void pid_update_telemetry(pid_state_t *p_state)
{
    int64_t dt_ms;
    int64_t delta_left;
    int64_t delta_right;
    float left_mm;
    float right_mm;
    float left_speed_mm_s;
    float right_speed_mm_s;

    /* First sample establishes the reference position and time. */
    if (!p_state->telemetry_initialized)
    {
        p_state->telemetry_prev_left = p_state->left_now;
        p_state->telemetry_prev_right = p_state->right_now;
        p_state->telemetry_prev_ms = p_state->current_ms;
        p_state->telemetry_distance_mm = 0.0f;
        p_state->telemetry_initialized = true;
        p_state->telemetry_left_speed_cps = 0;
        p_state->telemetry_right_speed_cps = 0;
        comm_set_motion(0, 0,
                        p_state->left_now,
                        p_state->right_now, 0);
        return;
    }

    dt_ms = p_state->current_ms - p_state->telemetry_prev_ms;

    if (dt_ms <= 0)
    {
        return;
    }

    delta_left = (int64_t)p_state->left_now -
                 p_state->telemetry_prev_left;
    delta_right = (int64_t)p_state->right_now -
                  p_state->telemetry_prev_right;
    /*
    * Signed measured encoder speed in counts/s.
    * Updated even when no movement command is active.
    */
    {
        int64_t left_cps = delta_left * 1000LL / dt_ms;
        int64_t right_cps = delta_right * 1000LL / dt_ms;

        if (left_cps > 2147483647LL) left_cps = 2147483647LL;
        if (left_cps < -2147483647LL - 1LL)
            left_cps = -2147483647LL - 1LL;

        if (right_cps > 2147483647LL) right_cps = 2147483647LL;
        if (right_cps < -2147483647LL - 1LL)
            right_cps = -2147483647LL - 1LL;

        p_state->telemetry_left_speed_cps = (INT)left_cps;
        p_state->telemetry_right_speed_cps = (INT)right_cps;
    }

    /* Convert encoder counts into physical wheel travel (mm). */
    left_mm = (float)delta_left *
              (3.14159265f * ENC_WHEEL_DIAMETER_CM * 10.0f) /
              ENC_LEFT_COUNTS_PER_REV;

    right_mm = (float)delta_right *
               (3.14159265f * ENC_WHEEL_DIAMETER_CM * 10.0f) /
               ENC_RIGHT_COUNTS_PER_REV;

    /* Signed wheel speeds in mm/s. */
    left_speed_mm_s = left_mm * 1000.0f / (float)dt_ms;
    right_speed_mm_s = right_mm * 1000.0f / (float)dt_ms;

    /* Accumulate average absolute wheel travel. */
    p_state->telemetry_distance_mm +=
        ((left_mm < 0.0f ? -left_mm : left_mm) +
         (right_mm < 0.0f ? -right_mm : right_mm)) * 0.5f;

    /* Clamp speeds to Buddy 1's signed 16-bit H range. */
    if (left_speed_mm_s > 32767.0f) left_speed_mm_s = 32767.0f;
    if (left_speed_mm_s < -32768.0f) left_speed_mm_s = -32768.0f;
    if (right_speed_mm_s > 32767.0f) right_speed_mm_s = 32767.0f;
    if (right_speed_mm_s < -32768.0f) right_speed_mm_s = -32768.0f;

    /*
    * Largest representable float below INT32_MAX.
    * Prevents out-of-range conversion to signed W.
    */
    if (p_state->telemetry_distance_mm > 2147483520.0f)
    {
        p_state->telemetry_distance_mm = 2147483520.0f;
    }

    comm_set_motion(
        (H)left_speed_mm_s,
        (H)right_speed_mm_s,
        (W)p_state->left_now,
        (W)p_state->right_now,
        (W)p_state->telemetry_distance_mm
    );

    p_state->telemetry_prev_left = p_state->left_now;
    p_state->telemetry_prev_right = p_state->right_now;
    p_state->telemetry_prev_ms = p_state->current_ms;
}

/** @brief Publish the current motion execution snapshot. */
static void pid_publish_status(pid_state_t *p_state)
{
    motion_status_snapshot_t snapshot = {0};

    snapshot.command_type = p_state->status_command_type;
    snapshot.status = p_state->motion_status;

    /*
    * Associate this status with the command
    * currently being executed or last completed.
    */
    snapshot.command_id = p_state->active_command.command_id;

    snapshot.left_encoder_counts = p_state->left_now;
    snapshot.right_encoder_counts = p_state->right_now;

    snapshot.left_speed_cps =
        p_state->telemetry_left_speed_cps;
    snapshot.right_speed_cps =
        p_state->telemetry_right_speed_cps;

    snapshot.safety_generation =
        b2_safety_stop_generation();

    /*
     * Non-blocking publication. A busy semaphore
     * may cause this snapshot to be skipped.
     */
    (void)motion_update_status(&snapshot);
}
