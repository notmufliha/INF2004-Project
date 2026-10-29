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
#include "task_producer.h"
#include "../subsystem1/subsystem1.h"
#include "../robot_config.h"
#include "b2_motor.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "pid_control.h"
#include "motion_command.h"
#include "motion_self_test.h"
#include "b2_integration.h"

/* Cyclic handler ID for the PI controller. */

static ID gh_cyc_pid = 0;

/* Network/telemetry locks never run in the 10 ms motor loop. */
static void b2_telemetry_task(INT stacd, void *exinf)
{
    motion_status_snapshot_t snapshot;
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        if (motion_get_status(&snapshot) == E_OK)
        {
            b2_integration_publish(&snapshot);
        }
        tk_dly_tsk(100);
    }
}
/* Original subsystem resources. */

#if ROBOT_WHEELS_UP_MOTOR_TEST

/*
 * BENCH TEST ONLY:
 * Both wheels must be raised and the robot securely supported.
 * Keep external motor power disconnected until the firmware
 * has been flashed and the setup checked.
 *
 * This test bypasses normal obstacle-clearance authorization.
 * Never use it with wheels touching the floor.
 */
static void b2_wheels_up_test_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    /* Begin with both motors stopped. */
    motor_stop();

    tm_printf((UB *)"[B2-TEST] Wheels-up test starting\n");

    /* Allow time to observe the startup message. */
    tk_dly_tsk(2000);

    /* Test left motor. */
    tm_printf((UB *)"[B2-TEST] LEFT motor pulse\n");
    motor_left_set(15);
    tk_dly_tsk(300);
    motor_stop();

    tk_dly_tsk(1000);

    /* Test right motor. */
    tm_printf((UB *)"[B2-TEST] RIGHT motor pulse\n");
    motor_right_set(15);
    tk_dly_tsk(300);
    motor_stop();

    /* Finish in the stopped state. */
    motor_stop();
    tm_printf((UB *)"[B2-TEST] Complete - motors stopped\n");

    for (;;)
    {
        motor_stop();
        tk_dly_tsk(1000);
    }
}
#endif

EXPORT ID gh_sem_slots;

EXPORT ID gh_mpf_samples;

/** @brief Initialize resources, motors, encoders, commands and PI timing. */
EXPORT ER subsystem2_init(void)
{
    T_CSEM csem = {0};
    T_CMPF cmpf = {0};
    T_CCYC ccyc = {0};
    ER err;
    err = b2_integration_init();
    if (err != E_OK)
    {
        return err;
    }
    /* Original producer semaphore. */
    csem.sematr  = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem  = S2_POOL_COUNT;
    gh_sem_slots = tk_cre_sem(&csem);

    if (gh_sem_slots < E_OK)
    {
        return (gh_sem_slots);
    }
    /* Original fixed-size memory pool. */
    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz  = sizeof(subsystem2_sample_msg_t);
    gh_mpf_samples = tk_cre_mpf(&cmpf);

    if (gh_mpf_samples < E_OK)
    {
        return (gh_mpf_samples);
    }
    /*
     * Initialize motor PWM.
     * Do not continue if initialization fails.
     */
    err = motor_init();

    if (E_OK != err)
    {
        tm_printf((UB *)"[B2] Motor initialization failed: %d\n", err);
        return (err);
    }
    motor_stop();
    err = b2_safety_init();

    if (E_OK != err)
    {
        return (err);
    }
    err = encoder_init();

    if (E_OK != err)
    {
        b2_integration_fault(MOTION_FAULT_ENCODER);
        return (err);
    }
    pid_init();
    err = motion_command_init();

    if (E_OK != err)
    {
        return (err);
    }
    /*
     * Create the 10 ms PI cyclic handler.
     */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)pid_cyclic_handler;
    ccyc.cyctim = 10;
    ccyc.cycphs = 10;
    gh_cyc_pid = tk_cre_cyc(&ccyc);

    if (gh_cyc_pid < E_OK)
    {
        return (gh_cyc_pid);
    }
    tm_printf((UB *)"[B2] Motion subsystem initialized\n");
    return (E_OK);
}

/** @brief Start subsystem tasks and the PI cyclic handler. */
EXPORT ER subsystem2_start(void)
{
    ID h_task;
    ER err;
    #if ROBOT_WHEELS_UP_MOTOR_TEST
        /* Bench task owns PWM; the PI controller is not started. */
        motor_stop();

        h_task = app_start_task(
            (FP)b2_wheels_up_test_task,
            S2_PRI_PRODUCER,
            TP_PRC2,
            "b2_wheels_up_test_task"
        );

        if (h_task < E_OK)
        {
            motor_stop();
            return (h_task);
        }

        return (E_OK);
    #endif
    #if !ROBOT_MODE
        /* Teaching-demo producer is excluded from real robot mode. */
        h_task = app_start_task(
            subsystem2_producer_task,
            S2_PRI_PRODUCER,
            TP_PRC2,
            "subsystem2_producer_task"
        );

        if (h_task < E_OK)
        {
            b2_request_stop();
            b2_safety_acknowledge_stop();
            return (h_task);
        }
    #endif
    #if ROBOT_ENCODER_DIAGNOSTICS
    /* Optional calibration tasks, disabled during ordinary operation. */
    h_task = app_start_task(
        (FP)encoder_speed_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "encoder_speed_task"
    );

    if (h_task < E_OK)
    {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return (h_task);
    }
    h_task = app_start_task(
        (FP)encoder_test_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "encoder_test_task"
    );

    if (h_task < E_OK)
    {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return (h_task);
    }
    #endif
    /* PI controller task. */
    gh_pid_task = app_start_task(
        (FP)pid_control_task,
        S2_PRI_CONTROL,
        TP_PRC2,
        "pid_control_task"
    );

    if (gh_pid_task < E_OK)
    {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return (gh_pid_task);
    }
    h_task = app_start_task((FP)b2_telemetry_task, S2_PRI_PRODUCER,
        TP_PRC2, "b2_telemetry_task");
    if (h_task < E_OK)
    {
        b2_request_stop();
        return h_task;
    }
    err = tk_sta_cyc(gh_cyc_pid);

    if (E_OK != err)
    {
        b2_request_stop();
        b2_safety_acknowledge_stop();
        return (err);
    }
    return b2_motion_test_start();
}
