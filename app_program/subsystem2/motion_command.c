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
 * Safety STOP generation associated with
 * the pending command.
 */

static uint32_t g_pending_safety_generation = 0u;

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

    tm_printf((UB *)"[B2] Motion p_command interface initialized\n");
    return (E_OK);
}

/** @brief Validate and queue a command, preserving pending STOP priority. */
ER motion_send_command(const motion_command_t *p_command)
{
    ER err;

    if (NULL == p_command)
    {
        return (E_PAR);
    }
    if (MOTION_CMD_STOP != p_command->type &&
        MOTION_CMD_DRIVE != p_command->type)
    {
        return (E_PAR);
    }
    if (MOTION_CMD_DRIVE == p_command->type)
    {
        if (MOTION_DIR_FORWARD != p_command->direction &&
            MOTION_DIR_REVERSE != p_command->direction)
        {
            return (E_PAR);
        }
        if (p_command->target_speed_cps <= 0 ||
            p_command->target_speed_cps > 5000)
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
    if (p_command->safety_generation !=
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
    if (MOTION_CMD_DRIVE == p_command->type &&
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
        MOTION_CMD_DRIVE == p_command->type)
    {
        (void)tk_sig_sem(gh_command_sem, 1);
        return (E_OBJ);
    }
    /*
     * Associate the command with the current
     * safety STOP generation.
     */
    g_pending_safety_generation =
        b2_safety_stop_generation();
    g_pending_command = *p_command;
    gb_command_pending = true;

    (void)tk_sig_sem(gh_command_sem, 1);
    return (E_OK);
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
    if (MOTION_CMD_DRIVE == g_pending_command.type &&
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
