
#include "../app.h"
#include "motion_command.h"


void motion_command_test_task(INT stacd, void *exinf)
{
    MOTION_COMMAND cmd = {0};
    ER err;
    INT i;

    (void)stacd;
    (void)exinf;

    /* Give time to prepare with wheels off the ground. */
    tk_dly_tsk(3000);

    cmd.type = MOTION_CMD_DRIVE;
    cmd.direction = MOTION_DIR_FORWARD;
    cmd.target_speed_cps = 3000;

    /* Refresh DRIVE every 50 ms for approximately 2 seconds. */
    for (i = 0; i < 40; i++) {
        err = motion_send_command(&cmd);

        if (err != E_OK) {
            tm_printf((UB*)"[B2 TEST] DRIVE failed: %d\n", err);
            break;
        }

        tk_dly_tsk(50);
    }

    /* Always request STOP after the test. */
    cmd.type = MOTION_CMD_STOP;
    cmd.direction = MOTION_DIR_STOP;
    cmd.target_speed_cps = 0;

    err = motion_send_command(&cmd);

    tm_printf(
        (UB*)"[B2 TEST] STOP result=%d\n",
        err
    );

    while (1) {
        tk_dly_tsk(1000);
    }
}
