/** @file pid_control.c
 * @brief Encoder-based motion execution. Only this task applies motor PWM.
 */
#include <stdbool.h>
#include <stdint.h>
#include "pid_control.h"
#include "encoder_control.h"
#include "motion_command.h"
#include "motion_profile.h"
#include "b2_motor.h"

#define PID_PERIOD_MS 10
#define PID_MAX_INTERVAL_MS 20
#define PID_WATCHDOG_MS 100
#define PID_MAX_MOTION_MS 60000
#define PID_STALL_MS 500
#define PID_SLOW_STALL_MS 5000
#define PID_SETTLE_MS 100
#define PID_SETTLE_TIMEOUT_MS 1500
#define PID_WINDOW_SIZE 10
#define PID_BASE_PWM 20.0f
#define PID_MAX_PWM 35
#define PID_KP 0.002f
#define PID_KI 0.005f
#define PID_I_LIMIT 10.0f
#define PID_PI 3.14159265f

typedef struct
{
    motion_command_t command;
    motion_status_t status;
    motion_fault_t fault;
    INT left_now;
    INT right_now;
    INT left_previous;
    INT right_previous;
    INT left_start;
    INT right_start;
    INT left_history[PID_WINDOW_SIZE + 1];
    INT right_history[PID_WINDOW_SIZE + 1];
    int64_t time_history[PID_WINDOW_SIZE + 1];
    UINT sample_count;
    UINT history_index;
    int64_t now_ms;
    int64_t previous_ms;
    int64_t started_ms;
    int64_t refreshed_ms;
    int64_t left_progress_ms;
    int64_t right_progress_ms;
    int64_t quiet_since_ms;
    int64_t settling_since_ms;
    int64_t duration_ms;
    float left_mm;
    float right_mm;
    float total_mm;
    float left_speed_cps;
    float right_speed_cps;
    float left_integral;
    float right_integral;
    INT left_sign;
    INT right_sign;
    bool settling;
    bool stopped;
    uint32_t generation;
    uint32_t control_interval_ms;
    uint32_t maximum_interval_ms;
} pid_state_t;

ID gh_pid_task = 0;

static float pid_mm_per_count(bool left)
{
    return PID_PI * ENC_WHEEL_DIAMETER_CM * 10.0f /
        (left ? ENC_LEFT_COUNTS_PER_REV : ENC_RIGHT_COUNTS_PER_REV);
}

static float pid_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool pid_movement(const pid_state_t *state)
{
    return state->command.type != MOTION_CMD_STOP &&
        state->status == MOTION_STATUS_RUNNING;
}

static void pid_publish(const pid_state_t *state)
{
    motion_status_snapshot_t snapshot = {0};
    snapshot.command_type = state->command.type;
    snapshot.command_id = state->command.command_id;
    snapshot.status = state->status;
    snapshot.fault = state->fault;
    snapshot.direction = state->command.direction;
    snapshot.turn_direction = state->command.turn_direction;
    snapshot.left_encoder_counts = state->left_now;
    snapshot.right_encoder_counts = state->right_now;
    snapshot.left_speed_cps = (INT)state->left_speed_cps;
    snapshot.right_speed_cps = (INT)state->right_speed_cps;
    snapshot.left_distance_mm = state->left_mm;
    snapshot.right_distance_mm = state->right_mm;
    snapshot.total_distance_mm = state->total_mm;
    snapshot.physically_stopped = state->stopped;
    snapshot.sample_time_ms = (uint32_t)state->now_ms;
    snapshot.control_interval_ms = state->control_interval_ms;
    snapshot.maximum_interval_ms = state->maximum_interval_ms;
    snapshot.safety_generation = b2_safety_stop_generation();
    (void)motion_update_status(&snapshot);
}

static void pid_reset_pi(pid_state_t *state)
{
    state->left_integral = 0.0f;
    state->right_integral = 0.0f;
    state->sample_count = 0u;
    state->history_index = 0u;
}

static void pid_finish(pid_state_t *state, motion_status_t status,
    motion_fault_t fault)
{
    b2_safety_force_stop();
    state->status = status;
    state->fault = fault;
    state->settling = false;
    pid_reset_pi(state);
    pid_publish(state);
}

/* Snapshot counters once per cycle. Widen before subtraction. */
static ER pid_sample(pid_state_t *state)
{
    SYSTIM time;
    int64_t left_delta;
    int64_t right_delta;
    int64_t interval;
    if (tk_get_tim(&time) != E_OK)
    {
        return E_SYS;
    }
    state->now_ms = (int64_t)time.hi * INT64_C(4294967296) + time.lo;
    interval = state->now_ms - state->previous_ms;
    state->control_interval_ms = interval > 0 ? (uint32_t)interval : 0u;
    if (state->control_interval_ms > state->maximum_interval_ms)
    {
        state->maximum_interval_ms = state->control_interval_ms;
    }
    if (interval <= 0 || interval > PID_MAX_INTERVAL_MS)
    {
        return E_SYS;
    }
    encoder_get_counts(&state->left_now, &state->right_now);
    left_delta = (int64_t)state->left_now - state->left_previous;
    right_delta = (int64_t)state->right_now - state->right_previous;
    state->total_mm += (pid_abs((float)left_delta *
        pid_mm_per_count(true)) + pid_abs((float)right_delta *
        pid_mm_per_count(false))) * 0.5f;
    state->left_speed_cps = (float)left_delta * 1000.0f / interval;
    state->right_speed_cps = (float)right_delta * 1000.0f / interval;
    if (left_delta != 0 || right_delta != 0)
    {
        state->quiet_since_ms = state->now_ms;
    }
    state->stopped = b2_motor_is_stopped() &&
        state->now_ms - state->quiet_since_ms >= PID_SETTLE_MS;
    state->left_mm = (float)((int64_t)state->left_now -
        state->left_start) * pid_mm_per_count(true);
    state->right_mm = (float)((int64_t)state->right_now -
        state->right_start) * pid_mm_per_count(false);
    return E_OK;
}

static void pid_set_directions(pid_state_t *state)
{
    state->left_sign = state->command.direction == MOTION_DIR_REVERSE
        ? -1 : 1;
    state->right_sign = state->left_sign;
    if (state->command.type == MOTION_CMD_TURN)
    {
        state->left_sign = state->command.turn_direction ==
            MOTION_TURN_LEFT ? -1 : 1;
        state->right_sign = -state->left_sign;
    }
}

static float pid_turn_track(const motion_command_t *command)
{
    float scale = command->turn_direction == MOTION_TURN_LEFT ?
        ENC_TURN_LEFT_SCALE : ENC_TURN_RIGHT_SCALE;
    return ENC_WHEEL_TRACK_CM * scale;
}

static int64_t pid_deadline(const motion_command_t *command)
{
    float counts = (float)command->target_distance_counts;
    float milliseconds;
    if (command->timeout_ms != 0u)
    {
        return command->timeout_ms;
    }
    if (command->type == MOTION_CMD_TURN)
    {
        counts = pid_turn_track(command) * command->target_angle_deg *
            ENC_LEFT_COUNTS_PER_REV / (360.0f * ENC_WHEEL_DIAMETER_CM);
    }
    if (command->target_speed_cps <= 0)
    {
        return PID_MAX_MOTION_MS;
    }
    milliseconds = counts * 2000.0f / command->target_speed_cps + 2000.0f;
    if (milliseconds > PID_MAX_MOTION_MS)
    {
        return PID_MAX_MOTION_MS;
    }
    return (int64_t)milliseconds;
}

static void pid_begin(pid_state_t *state, const motion_command_t *command)
{
    if (state->status == MOTION_STATUS_RUNNING)
    {
        pid_finish(state, MOTION_STATUS_CANCELLED, MOTION_FAULT_NONE);
    }
    b2_safety_force_stop();
    state->command = *command;
    state->generation = command->safety_generation;
    state->status = MOTION_STATUS_RUNNING;
    state->fault = MOTION_FAULT_NONE;
    state->left_start = state->left_now;
    state->right_start = state->right_now;
    state->left_mm = 0.0f;
    state->right_mm = 0.0f;
    state->started_ms = state->now_ms;
    state->duration_ms = pid_deadline(command);
    state->refreshed_ms = state->now_ms;
    state->left_progress_ms = state->now_ms;
    state->right_progress_ms = state->now_ms;
    state->settling = command->type == MOTION_CMD_STOP;
    state->settling_since_ms = state->now_ms;
    pid_set_directions(state);
    pid_reset_pi(state);
}

static void pid_accept(pid_state_t *state)
{
    motion_command_t command;
    ER error = motion_receive_command(&command);
    bool refresh;
    if (error == E_TMOUT)
    {
        return;
    }
    if (error != E_OK)
    {
        pid_finish(state, MOTION_STATUS_FAULT, MOTION_FAULT_COMMAND);
        return;
    }
    if (command.type != MOTION_CMD_STOP &&
        (command.safety_generation != b2_safety_stop_generation() ||
         !b2_safety_motion_allowed()))
    {
        pid_begin(state, &command);
        pid_finish(state, MOTION_STATUS_CANCELLED, MOTION_FAULT_SAFETY);
        return;
    }
    refresh = command.type == MOTION_CMD_DRIVE &&
        state->command.type == MOTION_CMD_DRIVE &&
        state->status == MOTION_STATUS_RUNNING &&
        command.direction == state->command.direction;
    if (!refresh)
    {
        pid_begin(state, &command);
        return;
    }
    /* Refresh replaces an ID but retains feedback history for smooth control.
     * Reset straight-motion origin after an intentional steering change.
     */
    state->status = MOTION_STATUS_CANCELLED;
    pid_publish(state);
    if (command.steering_permille != state->command.steering_permille)
    {
        state->left_start = state->left_now;
        state->right_start = state->right_now;
        state->left_mm = 0.0f;
        state->right_mm = 0.0f;
    }
    state->command = command;
    state->status = MOTION_STATUS_RUNNING;
    state->refreshed_ms = state->now_ms;
}

static void pid_guard(pid_state_t *state)
{
    bool interrupted = b2_safety_stop_requested() ||
        (pid_movement(state) &&
         (state->generation != b2_safety_stop_generation() ||
          !b2_safety_motion_allowed()));
    if (interrupted)
    {
        b2_safety_acknowledge_stop();
        if (pid_movement(state))
        {
            pid_finish(state, MOTION_STATUS_CANCELLED,
                MOTION_FAULT_SAFETY);
        }
    }
    if (!pid_movement(state))
    {
        return;
    }
    if (state->command.type == MOTION_CMD_DRIVE &&
        state->now_ms - state->refreshed_ms >= PID_WATCHDOG_MS)
    {
        pid_finish(state, MOTION_STATUS_TIMEOUT, MOTION_FAULT_WATCHDOG);
    }
    else if (state->command.type != MOTION_CMD_DRIVE &&
        state->now_ms - state->started_ms >= state->duration_ms)
    {
        pid_finish(state, MOTION_STATUS_TIMEOUT, MOTION_FAULT_DURATION);
    }
}

static INT pid_wheel_pwm(float target_cps, float measured_cps,
    float *integral, float dt)
{
    float error;
    float output;
    float candidate;
    if (target_cps <= 0.0f)
    {
        *integral = 0.0f;
        return 0;
    }
    error = target_cps - measured_cps;
    candidate = *integral + PID_KI * error * dt;
    if (candidate > PID_I_LIMIT)
    {
        candidate = PID_I_LIMIT;
    }
    if (candidate < -PID_I_LIMIT)
    {
        candidate = -PID_I_LIMIT;
    }
    output = PID_BASE_PWM + PID_KP * error + candidate;
    /* Conditional integration prevents winding further into saturation. */
    if (!((output > PID_MAX_PWM && error > 0.0f) ||
          (output < 0.0f && error < 0.0f)))
    {
        *integral = candidate;
    }
    output = PID_BASE_PWM + PID_KP * error + *integral;
    if (output > PID_MAX_PWM)
    {
        return PID_MAX_PWM;
    }
    return output > 0.0f ? (INT)(output + 0.5f) : 0;
}

static motion_profile_t pid_profile(pid_state_t *state)
{
    float target = 0.0f;
    float speed = state->command.target_speed_cps * pid_mm_per_count(true);
    bool bounded = state->command.type != MOTION_CMD_DRIVE;
    if (state->command.type == MOTION_CMD_MOVE)
    {
        target = state->command.target_distance_counts *
            pid_mm_per_count(true);
    }
    else if (state->command.type == MOTION_CMD_TURN)
    {
        target = PID_PI * pid_turn_track(&state->command) * 10.0f *
            state->command.target_angle_deg / 360.0f;
    }
    return motion_profile_targets(speed,
        state->command.steering_permille / 1000.0f,
        state->left_mm * state->left_sign,
        state->right_mm * state->right_sign, target, bounded, true);
}

static bool pid_stalled(pid_state_t *state, motion_profile_t profile)
{
    int64_t left_limit = profile.left_speed >= 5.0f ?
        PID_STALL_MS : PID_SLOW_STALL_MS;
    int64_t right_limit = profile.right_speed >= 5.0f ?
        PID_STALL_MS : PID_SLOW_STALL_MS;
    if (((int64_t)state->left_now - state->left_previous) *
        state->left_sign > 0 || profile.left_speed <= 0.0f)
    {
        state->left_progress_ms = state->now_ms;
    }
    if (((int64_t)state->right_now - state->right_previous) *
        state->right_sign > 0 || profile.right_speed <= 0.0f)
    {
        state->right_progress_ms = state->now_ms;
    }
    return state->now_ms - state->left_progress_ms >= left_limit ||
        state->now_ms - state->right_progress_ms >= right_limit;
}

static void pid_apply(pid_state_t *state, motion_profile_t profile)
{
    UINT slot = state->history_index;
    float left_cps = state->left_speed_cps;
    float right_cps = state->right_speed_cps;
    float dt = (float)(state->now_ms - state->previous_ms) / 1000.0f;
    INT left_pwm;
    INT right_pwm;
    if (state->sample_count >= PID_WINDOW_SIZE)
    {
        UINT old = (slot + 1u) % (PID_WINDOW_SIZE + 1u);
        float elapsed = (float)(state->now_ms - state->time_history[old]);
        left_cps = (float)((int64_t)state->left_now -
            state->left_history[old]) * 1000.0f / elapsed;
        right_cps = (float)((int64_t)state->right_now -
            state->right_history[old]) * 1000.0f / elapsed;
    }
    left_pwm = pid_wheel_pwm(profile.left_speed / pid_mm_per_count(true),
        left_cps * state->left_sign, &state->left_integral, dt);
    right_pwm = pid_wheel_pwm(profile.right_speed / pid_mm_per_count(false),
        right_cps * state->right_sign, &state->right_integral, dt);
    state->left_history[slot] = state->left_now;
    state->right_history[slot] = state->right_now;
    state->time_history[slot] = state->now_ms;
    state->history_index = (slot + 1u) % (PID_WINDOW_SIZE + 1u);
    if (state->sample_count < PID_WINDOW_SIZE)
    {
        state->sample_count++;
    }
    b2_safety_apply_pwm(state->left_sign * left_pwm,
        state->right_sign * right_pwm, state->generation);
    if (left_pwm != 0 || right_pwm != 0)
    {
        state->stopped = false;
        state->quiet_since_ms = state->now_ms;
    }
}

static void pid_execute(pid_state_t *state)
{
    motion_profile_t profile;
    if (state->settling)
    {
        b2_safety_force_stop();
        if (state->stopped)
        {
            pid_finish(state, MOTION_STATUS_COMPLETED, MOTION_FAULT_NONE);
        }
        else if (state->now_ms - state->settling_since_ms >=
            PID_SETTLE_TIMEOUT_MS)
        {
            pid_finish(state, MOTION_STATUS_FAULT, MOTION_FAULT_STOPPING);
        }
        return;
    }
    if (!pid_movement(state))
    {
        return;
    }
    profile = pid_profile(state);
    if (profile.left_reached && profile.right_reached)
    {
        b2_safety_force_stop();
        state->settling = true;
        state->settling_since_ms = state->now_ms;
        return;
    }
    if (pid_stalled(state, profile))
    {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        pid_finish(state, MOTION_STATUS_FAULT, MOTION_FAULT_STALL);
        return;
    }
    pid_apply(state, profile);
}

void pid_init(void)
{
    b2_safety_force_stop();
}

void pid_cyclic_handler(void *p_exinf)
{
    (void)p_exinf;
    if (gh_pid_task > 0)
    {
        (void)tk_wup_tsk(gh_pid_task);
    }
}

static ER pid_initialize_state(pid_state_t *state)
{
    SYSTIM time;
    if (tk_get_tim(&time) != E_OK)
    {
        return E_SYS;
    }
    state->previous_ms = (int64_t)time.hi * INT64_C(4294967296) + time.lo;
    state->quiet_since_ms = state->previous_ms;
    encoder_get_counts(&state->left_now, &state->right_now);
    state->left_previous = state->left_now;
    state->right_previous = state->right_now;
    state->left_start = state->left_now;
    state->right_start = state->right_now;
    return E_OK;
}

void pid_control_task(INT stacd, void *p_exinf)
{
    pid_state_t state = {0};
    ER error;
    (void)stacd;
    (void)p_exinf;
    error = pid_initialize_state(&state);
    while (error == E_OK)
    {
        (void)tk_can_wup(TSK_SELF);
        error = tk_slp_tsk(TMO_FEVR);
        if (error != E_OK || pid_sample(&state) != E_OK)
        {
            break;
        }
        pid_guard(&state);
        pid_accept(&state);
        pid_guard(&state);
        pid_execute(&state);
        pid_publish(&state);
        state.left_previous = state.left_now;
        state.right_previous = state.right_now;
        state.previous_ms = state.now_ms;
    }
    b2_request_stop();
    b2_safety_acknowledge_stop();
    pid_finish(&state, MOTION_STATUS_FAULT, MOTION_FAULT_TIMING);
    tm_printf((UB *)"[B2] Controller fault; stopped until restart\n");
    for (;;)
    {
        tk_dly_tsk(1000);
    }
}
