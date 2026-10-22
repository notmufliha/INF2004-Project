
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

#include "../app.h"
#include "motion_command.h"

/*
 * Latest pending motion command.
 *
 * Access is protected by a semaphore.
 */
static MOTION_COMMAND pending_command;
static BOOL command_pending = FALSE;
static ID command_sem = 0;

/*
 * Initialize command interface.
 * Call once during subsystem2_init().
 */
ER motion_command_init(void)
{
    T_CSEM csem = {0};

    csem.sematr = TA_TFIFO;
    csem.isemcnt = 1;
    csem.maxsem = 1;

    command_sem = tk_cre_sem(&csem);

    if (command_sem < E_OK) {
        return command_sem;
    }

    pending_command.type = MOTION_CMD_STOP;
    pending_command.direction = MOTION_DIR_STOP;
    pending_command.target_speed_cps = 0;

    command_pending = FALSE;

    tm_printf((UB*)"[B2] Motion command interface initialized\n");

    return E_OK;
}

/*
 * Validate and submit a motion command.
 *
 * Latest-command-wins behaviour:
 * a new command replaces the pending command.
 */
ER motion_send_command(const MOTION_COMMAND *command)
{
    ER err;

    if (command == NULL) {
        return E_PAR;
    }

    if (command->type != MOTION_CMD_STOP &&
        command->type != MOTION_CMD_DRIVE) {
        return E_PAR;
    }

    if (command->type == MOTION_CMD_DRIVE) {

        if (command->direction != MOTION_DIR_FORWARD &&
            command->direction != MOTION_DIR_REVERSE) {
            return E_PAR;
        }

        if (command->target_speed_cps <= 0 ||
            command->target_speed_cps > 5000) {
            return E_PAR;
        }
    }

    if (command_sem <= 0) {
        return E_OBJ;
    }

    /*
     * Wait for exclusive access to the
     * pending command.
     */
    err = tk_wai_sem(command_sem, 1, TMO_FEVR);

    if (err != E_OK) {
        return err;
    }

    pending_command = *command;
    command_pending = TRUE;

    (void)tk_sig_sem(command_sem, 1);

    return E_OK;
}

/*
 * Retrieve a pending command.
 *
 * Returns:
 * E_OK   - command retrieved
 * E_TMOUT - no command pending
 */
ER motion_receive_command(MOTION_COMMAND *command)
{
    ER err;

    if (command == NULL) {
        return E_PAR;
    }

    if (command_sem <= 0) {
        return E_OBJ;
    }

    err = tk_wai_sem(command_sem, 1, TMO_POL);

    if (err != E_OK) {
        return err;
    }

    if (!command_pending) {
        (void)tk_sig_sem(command_sem, 1);
        return E_TMOUT;
    }

    *command = pending_command;
    command_pending = FALSE;

    (void)tk_sig_sem(command_sem, 1);

    return E_OK;
}
