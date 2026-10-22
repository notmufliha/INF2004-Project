
/*
 * subsystem2.c
 *
 * Buddy 2 - Motion Control Subsystem
 *
 * Preserves the original producer task,
 * semaphore and fixed-size memory pool.
 *
 * Adds:
 * - Motor initialization
 * - Encoder initialization
 * - PI controller initialization
 * - 10 ms cyclic handler
 */

#include "subsystem2.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "pid_control.h"
#include "motion_command.h"

/* Temporary test task simulating Buddy 1 commands. */
extern void motion_command_test_task(INT stacd, void *exinf);

/* Cyclic handler ID for the PI controller. */
static ID cyc_pid = 0;

/* Original subsystem resources. */
EXPORT ID sem_slots;
EXPORT ID mpf_samples;

/*
 * Initialize Subsystem 2.
 */
EXPORT ER subsystem2_init(void)
{
    T_CSEM csem = {0};
    T_CMPF cmpf = {0};

    ER err;

    /*
     * Original producer semaphore.
     */
    csem.sematr = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem = S2_POOL_COUNT;

    sem_slots = tk_cre_sem(&csem);

    if (sem_slots < E_OK) {
        return sem_slots;
    }

    /*
     * Original fixed-size memory pool.
     */
    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz = sizeof(SAMPLE_MSG);

    mpf_samples = tk_cre_mpf(&cmpf);

    if (mpf_samples < E_OK) {
        return mpf_samples;
    }

    /*
     * Initialize motor hardware.
     */
    motor_init();

    /*
     * Initialize quadrature encoder interrupts.
     */
    err = encoder_init();

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Encoder initialization failed\n");
        return err;
    }

    /*
     * Initialize PI controller.
     */
    pid_init();

    /* Initialize Buddy 2 motion command interface. */
    err = motion_command_init();

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Motion command initialization failed\n");
        return err;
    }

    /*
     * Create 10 ms cyclic handler.
     *
     * The handler wakes the PI task.
     * The PI calculations run in task context.
     */
    {
        T_CCYC ccyc = {0};

        ccyc.cycatr = TA_HLNG;
        ccyc.cychdr = (FP)pid_cyclic_handler;

        ccyc.cyctim = 10;
        ccyc.cycphs = 10;

        cyc_pid = tk_cre_cyc(&ccyc);

        if (cyc_pid < E_OK) {
            tm_printf((UB*)"[B2] Cyclic handler creation failed\n");
            return cyc_pid;
        }
    }

    return E_OK;
}

/*
 * Stop the PI cyclic handler.
 */
EXPORT ER subsystem2_stop_pid_cyclic(void)
{
    if (cyc_pid <= 0) {
        return E_OBJ;
    }

    return tk_stp_cyc(cyc_pid);
}

/*
 * Start Subsystem 2 tasks.
 */

EXPORT ER subsystem2_start(void)
{
    ID tid;
    ER err;

    /* Original producer task. */
    tid = app_start_task(
        producer_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "producer_task"
    );

    if (tid < E_OK) {
        return tid;
    }

    /* Encoder speed monitoring task. */
    tid = app_start_task(
        (FP)encoder_speed_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "encoder_speed_task"
    );

    if (tid < E_OK) {
        return tid;
    }

    /* Start command-driven PI controller. */
    tid_pid = app_start_task(
        (FP)pid_control_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "pid_control_task"
    );

    if (tid_pid < E_OK) {
        return tid_pid;
    }

    /* Start 10 ms cyclic handler. */
    err = tk_sta_cyc(cyc_pid);

    if (err != E_OK) {
        motor_stop();
        return err;
    }

    tm_printf((UB*)"[B2] 10 ms PI controller started\n");

    /* Temporary Buddy 1 command simulator. */
    tid = app_start_task(
        (FP)motion_command_test_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "motion_command_test_task"
    );

    if (tid < E_OK) {
        motor_stop();
        (void)tk_stp_cyc(cyc_pid);
        return tid;
    }

    return E_OK;
}
