#include "motion_self_test.h"
#include "motion_command.h"
#include "encoder_control.h"
#include "b2_motor.h"
#include "../robot_config.h"

#if ROBOT_MOTION_TEST < 0 || ROBOT_MOTION_TEST > 6
#error "ROBOT_MOTION_TEST must be 0..6"
#endif
#if ROBOT_MOTION_TEST && (!ROBOT_ENABLE_DRIVE || ROBOT_WHEELS_UP_MOTOR_TEST)
#error "Motion tests need drive enabled and the raw wheels-up test disabled"
#endif

#if ROBOT_MOTION_TEST
#define B2_TEST_PERIOD_MS 20u
#define B2_TEST_CLEAR_WAIT_MS 30000u
#define B2_TEST_RESULT_WAIT_MS 65000u

static uint32_t b2_test_now(void)
{
    SYSTIM time;
    if (tk_get_tim(&time) != E_OK)
    {
        b2_request_stop();
        return 0u;
    }
    return time.lo;
}

static motion_command_t b2_test_command(UINT test)
{
    motion_command_t command = {0};
    command.target_speed_cps = ROBOT_MOTION_TEST_SPEED_CPS;
    command.safety_generation = b2_safety_stop_generation();
    command.direction = test == 2u ? MOTION_DIR_REVERSE : MOTION_DIR_FORWARD;
    if (test <= 2u)
    {
        command.type = MOTION_CMD_MOVE;
        command.target_distance_counts = (INT)(10.0f *
            ENC_LEFT_COUNTS_PER_REV / (3.14159265f * ENC_WHEEL_DIAMETER_CM));
    }
    else if (test == 3u)
    {
        command.type = MOTION_CMD_DRIVE;
    }
    else
    {
        command.type = MOTION_CMD_TURN;
        command.turn_direction = test == 4u ?
            MOTION_TURN_LEFT : MOTION_TURN_RIGHT;
        command.target_angle_deg = 90;
    }
    return command;
}

static bool b2_test_wait_clear(void)
{
    uint32_t started = b2_test_now();
    uint32_t printed = started - 1000u;
    while ((uint32_t)(b2_test_now() - started) < B2_TEST_CLEAR_WAIT_MS)
    {
        motion_status_snapshot_t status = {0};
        bool allowed = b2_safety_motion_allowed();
        ER error = motion_get_status(&status);
        if (error == E_OK && status.status == MOTION_STATUS_FAULT)
        {
            tm_printf((UB *)"[B2-TEST] controller FAULT=%d; "
                "max_interval=%u ms. Restart after resolving fault.\n",
                status.fault, (UINT)status.maximum_interval_ms);
            return false;
        }
        if (allowed && error == E_OK && status.physically_stopped)
        {
            return true;
        }
        if ((uint32_t)(b2_test_now() - printed) >= 1000u)
        {
            printed = b2_test_now();
            tm_printf((UB *)"[B2-TEST] BLOCKED: B5_permission=%d "
                "standstill=%d status_read=%d state=%d fault=%d\n",
                allowed, status.physically_stopped, error,
                status.status, status.fault);
        }
        tk_dly_tsk(B2_TEST_PERIOD_MS);
    }
    return false;
}

static bool b2_test_wait_result(uint32_t id)
{
    uint32_t started = b2_test_now();
    uint32_t printed = started;
    while ((uint32_t)(b2_test_now() - started) < B2_TEST_RESULT_WAIT_MS)
    {
        motion_status_snapshot_t status;
        if (motion_get_result(id, &status) == E_OK)
        {
            tm_printf((UB *)"[B2-TEST] id=%u result=%d fault=%d "
                "L_mm=%d R_mm=%d stopped=%d\n", (UINT)id,
                status.status, status.fault, (INT)status.left_distance_mm,
                (INT)status.right_distance_mm, status.physically_stopped);
            return status.status == MOTION_STATUS_COMPLETED &&
                status.physically_stopped;
        }
        if ((uint32_t)(b2_test_now() - printed) >= 500u &&
            motion_get_status(&status) == E_OK)
        {
            printed = b2_test_now();
            tm_printf((UB *)"[B2-TEST] waiting id=%u active=%u "
                "state=%d L_cps=%d R_cps=%d\n", (UINT)id,
                (UINT)status.command_id, status.status,
                status.left_speed_cps, status.right_speed_cps);
        }
        tk_dly_tsk(B2_TEST_PERIOD_MS);
    }
    return false;
}

/* Refresh only the DRIVE test; any loss of clearance aborts without restart. */
static bool b2_test_drive(motion_command_t command, uint32_t id)
{
    uint32_t started = b2_test_now();
    while ((uint32_t)(b2_test_now() - started) < 1000u)
    {
        motion_status_snapshot_t result;
        if (!b2_safety_motion_allowed() ||
            command.safety_generation != b2_safety_stop_generation() ||
            motion_get_result(id, &result) == E_OK)
        {
            return false;
        }
        tk_dly_tsk(B2_TEST_PERIOD_MS);
        if (!b2_safety_motion_allowed() ||
            command.safety_generation != b2_safety_stop_generation() ||
            motion_get_result(id, &result) == E_OK ||
            motion_send_command_with_id(&command, &id) != E_OK)
        {
            return false;
        }
    }
    command = (motion_command_t){0};
    command.type = MOTION_CMD_STOP;
    if (motion_send_command_with_id(&command, &id) != E_OK)
    {
        return false;
    }
    return b2_test_wait_result(id);
}

static bool b2_test_run(UINT test)
{
    motion_command_t command;
    uint32_t id;
    tm_printf((UB *)"[B2-TEST] test=%u waiting for fresh B5 clearance\n",
        test);
    if (!b2_test_wait_clear())
    {
        tm_printf((UB *)"[B2-TEST] no clearance/standstill within 30s\n");
        return false;
    }
    command = b2_test_command(test);
    if (motion_send_command_with_id(&command, &id) != E_OK)
    {
        tm_printf((UB *)"[B2-TEST] command submission failed\n");
        return false;
    }
    tm_printf((UB *)"[B2-TEST] START test=%u id=%u speed=%d cps\n",
        test, (UINT)id, command.target_speed_cps);
    return test == 3u ? b2_test_drive(command, id) : b2_test_wait_result(id);
}

static void b2_motion_test_task(INT stacd, void *exinf)
{
    UINT first = ROBOT_MOTION_TEST == 6 ? 1u : ROBOT_MOTION_TEST;
    UINT last = ROBOT_MOTION_TEST == 6 ? 5u : ROBOT_MOTION_TEST;
    UINT test;
    bool passed = true;
    (void)stacd;
    (void)exinf;
    tm_printf((UB *)"[B2-TEST] Armed mode=%d; starts after %u ms. "
        "Clear front/rear/sides; B5 guards front only.\n",
        ROBOT_MOTION_TEST, (UINT)ROBOT_MOTION_TEST_BOOT_MS);
    tk_dly_tsk(ROBOT_MOTION_TEST_BOOT_MS);
    for (test = first; test <= last; test++)
    {
        if (!b2_test_run(test))
        {
            passed = false;
            break;
        }
        tm_printf((UB *)"[B2-TEST] PASS test=%u; pause 3s\n", test);
        tk_dly_tsk(3000);
    }
    b2_request_stop();
    tm_printf((UB *)"[B2-TEST] %s; stopped. Reboot to repeat.\n",
        passed ? "COMPLETE" : "ABORTED");
    for (;;)
    {
        tk_dly_tsk(1000);
    }
}
#endif

ER b2_motion_test_start(void)
{
#if ROBOT_MOTION_TEST
    ID task = app_start_task((FP)b2_motion_test_task, 13,
        TP_PRC2, "b2_motion_test_task");
    return task < E_OK ? task : E_OK;
#else
    return E_OK;
#endif
}
