/** @file motion_command.c
 * @brief One pending command, immediate STOP, retained execution results.
 */
#include "motion_command.h"
#include "b2_motor.h"
#include "encoder_control.h"
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

static motion_command_t g_pending_command;
static bool gb_command_pending;
static ID gh_command_sem;
static uint32_t g_next_command_id = 1u;
static motion_status_snapshot_t g_motion_status;
static motion_status_snapshot_t g_results[MOTION_RESULT_HISTORY];
static UINT g_result_next;
static bool gb_status_ready;
#if TK_SUPPORT_SMP
static T_SPLOCK g_status_lock;
#endif

static void motion_lock(UINT *status)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_status_lock, status);
#else
    DI(*status);
#endif
}

static void motion_unlock(UINT status)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_status_lock, status);
#else
    EI(status);
#endif
}

static bool motion_terminal(motion_status_t status)
{
    return status == MOTION_STATUS_COMPLETED ||
        status == MOTION_STATUS_CANCELLED ||
        status == MOTION_STATUS_TIMEOUT || status == MOTION_STATUS_FAULT;
}

/* Caller holds the status lock. The first terminal result is immutable. */
static void motion_retain(const motion_status_snapshot_t *snapshot)
{
    UINT index;
    if (!motion_terminal(snapshot->status) || snapshot->command_id == 0u)
    {
        return;
    }
    for (index = 0u; index < MOTION_RESULT_HISTORY; index++)
    {
        if (g_results[index].command_id == snapshot->command_id)
        {
            return;
        }
    }
    g_results[g_result_next] = *snapshot;
    g_result_next = (g_result_next + 1u) % MOTION_RESULT_HISTORY;
}

static bool motion_valid(const motion_command_t *command)
{
    INT steering;
    if (command->type == MOTION_CMD_STOP)
    {
        return true;
    }
    if (command->type != MOTION_CMD_DRIVE &&
        command->type != MOTION_CMD_MOVE && command->type != MOTION_CMD_TURN)
    {
        return false;
    }
    if (command->target_speed_cps <= 0 || command->target_speed_cps > 5000)
    {
        return false;
    }
    if (command->steering_permille < -1000 ||
        command->steering_permille > 1000)
    {
        return false;
    }
    steering = command->steering_permille < 0 ?
        -command->steering_permille : command->steering_permille;
    if ((int64_t)command->target_speed_cps * (1000 + steering) > 5000000)
    {
        return false;
    }
    if (command->type != MOTION_CMD_DRIVE && steering != 0)
    {
        return false;
    }
    if (command->timeout_ms != 0u &&
        (command->timeout_ms < 100u || command->timeout_ms > 60000u ||
         command->type == MOTION_CMD_DRIVE))
    {
        return false;
    }
    if (command->type == MOTION_CMD_TURN)
    {
        return (command->turn_direction == MOTION_TURN_LEFT ||
                command->turn_direction == MOTION_TURN_RIGHT) &&
            command->target_angle_deg > 0 && command->target_angle_deg <= 360;
    }
    return (command->direction == MOTION_DIR_FORWARD ||
            command->direction == MOTION_DIR_REVERSE) &&
        (command->type != MOTION_CMD_MOVE ||
         command->target_distance_counts > 0);
}

ER motion_command_init(void)
{
    T_CSEM sem = {0};
    UINT index;
    sem.sematr = TA_TFIFO;
    sem.isemcnt = 1;
    sem.maxsem = 1;
    gh_command_sem = tk_cre_sem(&sem);
    if (gh_command_sem < E_OK)
    {
        return gh_command_sem;
    }
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&g_status_lock);
#endif
    gb_command_pending = false;
    g_next_command_id = 1u;
    g_result_next = 0u;
    g_motion_status = (motion_status_snapshot_t){0};
    for (index = 0u; index < MOTION_RESULT_HISTORY; index++)
    {
        g_results[index] = (motion_status_snapshot_t){0};
    }
    gb_status_ready = true;
    return E_OK;
}

/* STOP also records cancellation for an unconsumed command. */
static void motion_cancel_pending(void)
{
    motion_status_snapshot_t snapshot;
    UINT status;
    motion_lock(&status);
    snapshot = g_motion_status;
    snapshot.command_id = g_pending_command.command_id;
    snapshot.command_type = g_pending_command.type;
    snapshot.direction = g_pending_command.direction;
    snapshot.turn_direction = g_pending_command.turn_direction;
    snapshot.left_distance_mm = 0.0f;
    snapshot.right_distance_mm = 0.0f;
    snapshot.status = MOTION_STATUS_CANCELLED;
    snapshot.fault = MOTION_FAULT_NONE;
    motion_retain(&snapshot);
    motion_unlock(status);
}

ER motion_send_command_with_id(const motion_command_t *command,
    uint32_t *command_id)
{
    ER error;
    if (command_id == NULL)
    {
        return E_PAR;
    }
    *command_id = 0u;
    if (command == NULL || !motion_valid(command))
    {
        return E_PAR;
    }
    if (gh_command_sem <= 0)
    {
        return E_OBJ;
    }
    /* STOP disables PWM even if the queue lock is briefly occupied.
     * It revokes earlier authorizations; Buddy 5 must release it again.
     */
    if (command->type == MOTION_CMD_STOP)
    {
        b2_request_stop();
    }
    error = tk_wai_sem(gh_command_sem, 1, TMO_POL);
    if (error != E_OK)
    {
        return error;
    }
    if (command->type != MOTION_CMD_STOP &&
        command->safety_generation != b2_safety_stop_generation())
    {
        error = E_OBJ;
    }
    else if (gb_command_pending &&
        (command->type != MOTION_CMD_STOP ||
         g_pending_command.type == MOTION_CMD_STOP))
    {
        error = E_OBJ;
    }
    else
    {
        if (gb_command_pending)
        {
            motion_cancel_pending();
        }
        g_pending_command = *command;
        g_pending_command.command_id = g_next_command_id++;
        if (g_next_command_id == 0u)
        {
            g_next_command_id = 1u;
        }
        *command_id = g_pending_command.command_id;
        gb_command_pending = true;
    }
    (void)tk_sig_sem(gh_command_sem, 1);
    return error;
}

ER motion_send_command(const motion_command_t *command)
{
    uint32_t id;
    return motion_send_command_with_id(command, &id);
}

ER motion_speed_from_percent(INT percent, INT *speed_cps)
{
    if (speed_cps == NULL)
    {
        return E_PAR;
    }
    *speed_cps = 0;
    if (percent < 0 || percent > 100)
    {
        return E_PAR;
    }
    *speed_cps = percent * 50;
    return E_OK;
}

ER motion_move_cm(float distance_cm, INT speed_cps,
    motion_direction_t direction, uint32_t *command_id)
{
    motion_command_t command = {0};
    float counts = distance_cm * ENC_LEFT_COUNTS_PER_REV /
        (3.14159265f * ENC_WHEEL_DIAMETER_CM);
    if (command_id == NULL)
    {
        return E_PAR;
    }
    *command_id = 0u;
    /* This also rejects NaN/infinity before converting float to integer. */
    if (!(counts >= 1.0f && counts <= 2147483520.0f))
    {
        return E_PAR;
    }
    command.type = MOTION_CMD_MOVE;
    command.direction = direction;
    command.target_speed_cps = speed_cps;
    command.target_distance_counts = (INT)(counts + 0.5f);
    command.safety_generation = b2_safety_stop_generation();
    return motion_send_command_with_id(&command, command_id);
}

ER motion_turn_degrees(INT angle, INT speed_cps,
    motion_turn_direction_t direction, uint32_t *command_id)
{
    motion_command_t command = {0};
    command.type = MOTION_CMD_TURN;
    command.turn_direction = direction;
    command.target_angle_deg = angle;
    command.target_speed_cps = speed_cps;
    command.safety_generation = b2_safety_stop_generation();
    return motion_send_command_with_id(&command, command_id);
}

ER motion_receive_command(motion_command_t *command)
{
    ER error;
    if (command == NULL)
    {
        return E_PAR;
    }
    if (gh_command_sem <= 0)
    {
        return E_OBJ;
    }
    error = tk_wai_sem(gh_command_sem, 1, TMO_POL);
    if (error != E_OK)
    {
        return error;
    }
    if (!gb_command_pending)
    {
        error = E_TMOUT;
    }
    else
    {
        /* Deliver stale commands too, so the controller reports their IDs. */
        *command = g_pending_command;
        gb_command_pending = false;
    }
    (void)tk_sig_sem(gh_command_sem, 1);
    return error;
}

ER motion_get_status(motion_status_snapshot_t *snapshot)
{
    UINT status;
    if (snapshot == NULL)
    {
        return E_PAR;
    }
    if (!gb_status_ready)
    {
        return E_OBJ;
    }
    motion_lock(&status);
    *snapshot = g_motion_status;
    motion_unlock(status);
    return E_OK;
}

ER motion_get_result(uint32_t id, motion_status_snapshot_t *snapshot)
{
    UINT status;
    UINT index;
    ER result = E_OBJ;
    if (snapshot == NULL || id == 0u)
    {
        return E_PAR;
    }
    if (!gb_status_ready)
    {
        return E_OBJ;
    }
    motion_lock(&status);
    for (index = 0u; index < MOTION_RESULT_HISTORY; index++)
    {
        if (g_results[index].command_id == id)
        {
            *snapshot = g_results[index];
            result = E_OK;
            break;
        }
    }
    motion_unlock(status);
    return result;
}

ER motion_update_status(const motion_status_snapshot_t *snapshot)
{
    UINT status;
    if (snapshot == NULL)
    {
        return E_PAR;
    }
    if (!gb_status_ready)
    {
        return E_OBJ;
    }
    motion_lock(&status);
    g_motion_status = *snapshot;
    motion_retain(snapshot);
    motion_unlock(status);
    return E_OK;
}
