/*
 * Buddy 2 - Motion Command Interface
 * RP2040 + micro T-Kernel 3.0
 *
 * Receives and validates motion commands
 * from Buddy 1 Vehicle Controller.
 *
 * Initial implementation:
 * - DRIVE and STOP supported
 * - TURN reserved for later
 * - One pending command slot
 */

#include <stdbool.h>
#include <stdint.h>
#include "motion_command.h"
#include "b2_motor.h"

/*
 * Latest pending motion command.
 *
 * Access is protected by a semaphore.
 */

static motion_command_t g_pending_command;
static bool gb_command_pending = false;
static ID gh_command_sem = 0;
/*
 * Latest motion status shared with the Vehicle Controller.
 *
 * Protected by a separate semaphore to prevent
 * inconsistent reads across tasks.
 */
static motion_status_snapshot_t g_motion_status;

static ID gh_status_sem = 0;

/*
 * Safety STOP generation associated with
 * the pending command.
 */

static uint32_t g_pending_safety_generation = 0u;

/*
 * Monotonically increasing command identifier.
 *
 * Protected by gh_command_sem.
 * Zero is reserved to mean no assigned command.
 */
static uint32_t g_next_command_id = 1u;

/*
 * All commands that can move the motors require
 * valid safety authorization.
 */
static bool motion_is_movement(motion_cmd_type_t type)
{
    return (MOTION_CMD_DRIVE == type ||
            MOTION_CMD_MOVE == type ||
            MOTION_CMD_TURN == type);
}

/** @brief Create the pending-command semaphore; call once at startup. */
ER motion_command_init(void)
{
    T_CSEM csem = {0};
    csem.sematr = TA_TFIFO;
    csem.isemcnt = 1;
    csem.maxsem = 1;
    gh_command_sem = tk_cre_sem(&csem);

    if (gh_command_sem < E_OK)
    {
        return (gh_command_sem);
    }
    g_pending_command.type = MOTION_CMD_STOP;
    g_pending_command.direction = MOTION_DIR_STOP;
    g_pending_command.target_speed_cps = 0;
    gb_command_pending = false;
    g_pending_safety_generation = b2_safety_stop_generation();
    g_next_command_id = 1u;

    /*
    * Create the semaphore protecting the motion-status snapshot.
    */
    {
        T_CSEM status_sem = {0};

        status_sem.sematr = TA_TFIFO;
        status_sem.isemcnt = 1;
        status_sem.maxsem = 1;

        gh_status_sem = tk_cre_sem(&status_sem);

        if (gh_status_sem < E_OK)
        {
            return (gh_status_sem);
        }
    }

    /*
    * Initialize motion status to IDLE.
    */
    g_motion_status = (motion_status_snapshot_t){0};

    g_motion_status.command_type = MOTION_CMD_STOP;
    g_motion_status.status = MOTION_STATUS_IDLE;
    g_motion_status.safety_generation =
        b2_safety_stop_generation();

    tm_printf((UB *)"[B2] Motion p_command interface initialized\n");
    return (E_OK);
}

/** @brief Validate and queue a command, preserving pending STOP priority. */
ER motion_send_command_with_id(
    const motion_command_t *p_command,
    uint32_t *p_command_id)
{
    ER err;
    uint32_t assigned_id;

    if (NULL == p_command_id)
    {
        return (E_PAR);
    }

    *p_command_id = 0u;

    if (NULL == p_command)
    {
        return (E_PAR);
    }
    if (MOTION_CMD_STOP != p_command->type &&
        !motion_is_movement(p_command->type))
    {
        return (E_PAR);
    }
    /*
    * Validate movement speed for DRIVE, MOVE and TURN.
    */
    if (motion_is_movement(p_command->type))
    {
        if (p_command->target_speed_cps <= 0 ||
            p_command->target_speed_cps > 5000)
        {
            return (E_PAR);
        }
    }

    /*
    * DRIVE and MOVE require a valid travel direction.
    */
    if (MOTION_CMD_DRIVE == p_command->type ||
        MOTION_CMD_MOVE == p_command->type)
    {
        if (MOTION_DIR_FORWARD != p_command->direction &&
            MOTION_DIR_REVERSE != p_command->direction)
        {
            return (E_PAR);
        }
    }

    /*
    * MOVE must have a positive target distance.
    * Distance is expressed in reference-wheel counts.
    */
    if (MOTION_CMD_MOVE == p_command->type)
    {
        if (p_command->target_distance_counts <= 0)
        {
            return (E_PAR);
        }
    }

    /*
    * TURN requires a valid direction and angle.
    */
    if (MOTION_CMD_TURN == p_command->type)
    {
        if (MOTION_TURN_LEFT != p_command->turn_direction &&
            MOTION_TURN_RIGHT != p_command->turn_direction)
        {
            return (E_PAR);
        }

        if (p_command->target_angle_deg <= 0 ||
            p_command->target_angle_deg > 360)
        {
            return (E_PAR);
        }
    }
    /*
     * Reject DRIVE commands authorized under
     * an outdated safety STOP generation.
     *
     * STOP commands do not require authorization.
     */
    if (motion_is_movement(p_command->type) &&
        p_command->safety_generation !=
            b2_safety_stop_generation())
    {
        return (E_OBJ);
    }
    if (gh_command_sem <= 0)
    {
        return (E_OBJ);
    }
    /*
     * Wait for exclusive access to the
     * pending command.
     */
    err = tk_wai_sem(gh_command_sem, 1, TMO_FEVR);

    if (E_OK != err)
    {
        return (err);
    }
    /*
     * Recheck DRIVE authorization after acquiring
     * the command semaphore.
     *
     * A safety STOP may have occurred while
     * waiting for the semaphore.
     */
    if (motion_is_movement(p_command->type) &&
        p_command->safety_generation !=
            b2_safety_stop_generation())
        {
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_OBJ);
    }
    /*
     * STOP commands have priority.
     *
     * If a STOP is already pending, a DRIVE
     * command must not overwrite it.
     */
    if (gb_command_pending &&
        MOTION_CMD_STOP == g_pending_command.type &&
        motion_is_movement(p_command->type))
    {
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_OBJ);
    }
    /*
    * Do not silently overwrite a pending movement command.
    *
    * STOP remains allowed to replace a pending movement
    * because stopping has priority.
    */
    if (gb_command_pending &&
        motion_is_movement(p_command->type))
    {
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_OBJ);
    }
    /*
    * Assign an ID only after the command has passed
    * validation and pending-command checks.
    *
    * Zero is reserved as an invalid command ID.
    */
    assigned_id = g_next_command_id++;

    if (0u == g_next_command_id)
    {
        g_next_command_id = 1u;
    }

    /*
    * Store the assigned ID in the queued copy.
    * Do not modify the caller's command structure.
    */
    g_pending_safety_generation =
        b2_safety_stop_generation();

    g_pending_command = *p_command;
    g_pending_command.command_id = assigned_id;

    gb_command_pending = true;

    *p_command_id = assigned_id;

    (void)tk_sig_sem(gh_command_sem, 1);
    return (E_OK);
}

/*
 * Backward-compatible command submission.
 *
 * Callers that do not need the assigned ID
 * can continue using motion_send_command().
 */
ER motion_send_command(const motion_command_t *p_command)
{
    uint32_t ignored_command_id = 0u;

    return motion_send_command_with_id(
        p_command,
        &ignored_command_id
    );
}

/** @brief Retrieve a pending command if its authorization is current. */
ER motion_receive_command(motion_command_t *p_command)
{
    ER err;

    if (NULL == p_command)
    {
        return (E_PAR);
    }
    if (gh_command_sem <= 0)
    {
        return (E_OBJ);
    }
    err = tk_wai_sem(gh_command_sem, 1, TMO_POL);

    if (E_OK != err)
    {
        return (err);
    }
    if (!gb_command_pending)
    {
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_TMOUT);
    }
    /*
     * Reject a pending DRIVE if either:
     * 1. Its original authorization is outdated, or
     * 2. A new safety STOP occurred after it was queued.
     */
    if (motion_is_movement(g_pending_command.type) &&
        (g_pending_command.safety_generation !=
            b2_safety_stop_generation() ||
            g_pending_safety_generation !=
            b2_safety_stop_generation()))
        {
        gb_command_pending = false;
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_TMOUT);
    }
    /*
     * Deliver the valid pending command.
     */
    *p_command = g_pending_command;
    gb_command_pending = false;

    (void)tk_sig_sem(gh_command_sem, 1);
    return (E_OK);
}

/** @brief Read the latest synchronized motion-status snapshot. */
ER motion_get_status(motion_status_snapshot_t *p_status)
{
    ER err;

    if (NULL == p_status)
    {
        return (E_PAR);
    }

    if (gh_status_sem <= 0)
    {
        return (E_OBJ);
    }

    /*
     * Non-blocking access: return E_TMOUT if
     * another task currently owns the semaphore.
     */
    err = tk_wai_sem(gh_status_sem, 1, TMO_POL);

    if (E_OK != err)
    {
        return (err);
    }

    *p_status = g_motion_status;

    (void)tk_sig_sem(gh_status_sem, 1);

    return (E_OK);
}

/** @brief Publish the latest motion-status snapshot. */
ER motion_update_status(const motion_status_snapshot_t *p_status)
{
    ER err;

    if (NULL == p_status)
    {
        return (E_PAR);
    }

    if (gh_status_sem <= 0)
    {
        return (E_OBJ);
    }

    /*
    * Allow a short, bounded wait for the status semaphore.
    * A busy reader should not immediately cause us to
    * discard an important status update.
    */
    err = tk_wai_sem(gh_status_sem, 1, 1);

    if (E_OK != err)
    {
        return (err);
    }

    g_motion_status = *p_status;

    (void)tk_sig_sem(gh_status_sem, 1);

    return (E_OK);
}
