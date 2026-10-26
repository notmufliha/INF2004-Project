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
#include "../robot_config.h"
#include "b2_motor.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "pid_control.h"
#include "motion_command.h"

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
    T_CCYC ccyc = {0};
    ER err;

    /* Original producer semaphore. */
    csem.sematr  = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem  = S2_POOL_COUNT;

    sem_slots = tk_cre_sem(&csem);
    if (sem_slots < E_OK) {
        return sem_slots;
    }

    /* Original fixed-size memory pool. */
    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz  = sizeof(SAMPLE_MSG);

    mpf_samples = tk_cre_mpf(&cmpf);
    if (mpf_samples < E_OK) {
        return mpf_samples;
    }

    /*
    * Initialize motor PWM.
    * Do not continue if initialization fails.
    */
    err = motor_init();

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Motor initialization failed: %d\n", err);
        return err;
    }

    motor_stop();

    err = b2_safety_init();
    if (err != E_OK) {
        return err;
    }

    err = encoder_init();
    if (err != E_OK) {
        return err;
    }

    pid_init();

    err = motion_command_init();
    if (err != E_OK) {
        return err;
    }

    /*
     * Create the 10 ms PI cyclic handler.
     */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)pid_cyclic_handler;
    ccyc.cyctim = 10;
    ccyc.cycphs = 10;

    cyc_pid = tk_cre_cyc(&ccyc);
    if (cyc_pid < E_OK) {
        return cyc_pid;
    }

    tm_printf((UB*)"[B2] Motion subsystem initialized\n");

    return E_OK;
}

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
        b2_request_stop();
        b2_safety_acknowledge_stop();
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
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return tid;
    }

    /* PI controller task. */
    tid_pid = app_start_task(
        (FP)pid_control_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "pid_control_task"
    );

    if (tid_pid < E_OK) {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return tid_pid;
    }

    /* Start the 10 ms PI controller timer. */
    err = tk_sta_cyc(cyc_pid);

    if (err != E_OK) {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return err;
    }

    tm_printf((UB*)"[B2] PI controller started\n");

    return E_OK;
}
