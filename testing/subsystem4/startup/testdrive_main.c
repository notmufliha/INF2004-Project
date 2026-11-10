/** @file testdrive_main.c
 * @brief Execute B4 motion-test orchestration with deterministic B2/B5 stubs.
 */
#include "s4_motion_test.h"
#include "s4_runtime.h"
#include "imu.h"
#include "../subsystem2/motion_command.h"
#include "../subsystem2/b2_motor.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

volatile ID     gh_imu_test_events = 7;
static uint32_t g_now;
static uint32_t g_start;
static uint32_t g_last_event = UINT32_MAX;
static uint32_t g_commands;
static uint32_t g_stops;
static int      g_case;
extern ER       g_diag_create_error;

/** @brief Run each case in a separate process to verify once-per-boot state. */
int
main (int argc, char ** p_argv)
{
    assert(2 == argc);
    g_case              = atoi(p_argv[1]);
    g_diag_create_error = (14 == g_case) ? E_SYS : E_OK;
    ER error            = s4_motion_test_run();

    if ((0 == g_case) || (7 == g_case))
    {
        assert(E_OK == error);
    }
    else
    {
        assert(E_OK != error);
    }
    assert(1U == g_stops);
    assert(E_OBJ == s4_motion_test_run());
    assert(1U == g_stops);
    assert(g_commands <= 1U);
    if ((8 == g_case) || (9 == g_case) || (14 == g_case))
    {
        assert(0U == g_commands);
    }
    printf("PASS: B4 movement orchestration case %d\n", g_case);
    return 0;
}

/** @brief Deterministic clock, including an injected failure. */
ER
tk_get_tim (SYSTIM * p_time)
{
    p_time->hi = 0U;
    p_time->lo = g_now;
    return (8 == g_case) ? E_SYS : E_OK;
}

/** @brief Advance time without sleeping. */
ER
tk_dly_tsk (TMO delay)
{
    assert(delay >= 0);
    g_now += (uint32_t)delay;
    return E_OK;
}

/** @brief Test tap only; never provide raw or controller queue messages. */
INT
tk_rcv_mbf (ID h_buffer, void * p_data, TMO timeout)
{
    assert((7 == h_buffer) && (TMO_POL == timeout));
    if ((g_last_event == g_now) || ((3 == g_case) && (g_commands > 0U)))
    {
        return E_TMOUT;
    }
    g_last_event          = g_now;
    imu_event_t * p_event = p_data;

    *p_event              = (imu_event_t){0};
    p_event->b_valid      = true;
    p_event->timestamp_ms = g_now;
    if ((g_commands > 0U) && ((g_now - g_start) >= 100U))
    {
        p_event->b_sensor_fault   = (2 == g_case);
        p_event->b_sudden_impact  = (10 == g_case);
        p_event->roll_deg         = (11 == g_case) ? 40.0f : 0.0f;
        p_event->b_hump_completed = (100U == (g_now - g_start));
        p_event->b_height_valid   = (7 != g_case);
        p_event->hump_peak_cm     = 3.0f;
        p_event->highest_hump_cm  = 3.0f;
    }
    return (INT)sizeof(*p_event);
}

/** @brief Shared readiness rejects simulated sensor faults too. */
ER
subsystem4_wait_ready (TMO timeout)
{
    assert(1 == timeout);
    return E_OK;
}

/** @brief Require existing B5 permission, never manufacture it. */
bool
b2_safety_motion_allowed (void)
{
    return (9 != g_case) &&
           !((1 == g_case) && (g_commands > 0U) && ((g_now - g_start) >= 100U));
}

/** @brief Record fail-safe termination on every outcome. */
void
b2_request_stop (void)
{
    ++g_stops;
}

/** @brief Verify public distance API parameters and single submission. */
ER
motion_move_cm (float distance_cm, INT speed_cps, motion_direction_t direction,
                uint32_t * p_id)
{
    assert(50.0f == distance_cm);
    assert(500 == speed_cps);
    assert(MOTION_DIR_FORWARD == direction);
    assert(b2_safety_motion_allowed());
    assert(0U == g_commands);
    ++g_commands;
    g_start = g_now;
    *p_id   = 42U;
    return (5 == g_case) ? E_TMOUT : E_OK;
}

/** @brief Feed fresh standstill and running snapshots or stale/replaced data.
 */
ER
motion_get_status (motion_status_snapshot_t * p_status)
{
    *p_status                     = (motion_status_snapshot_t){0};
    p_status->sample_time_ms      = g_now;
    p_status->control_interval_ms = 10U;
    p_status->physically_stopped  = (0U == g_commands);
    if (g_commands > 0U)
    {
        p_status->status     = MOTION_STATUS_RUNNING;
        p_status->command_id = (12 == g_case) ? 43U : 42U;
        if (4 == g_case)
        {
            p_status->sample_time_ms -= 101U;
        }
    }
    return E_OK;
}

/** @brief Complete, fail or time out the command without a hardware dependency.
 */
ER
motion_get_result (uint32_t id, motion_status_snapshot_t * p_status)
{
    assert(42U == id);
    if (((g_now - g_start) < 2200U) || (6 == g_case))
    {
        return E_OBJ;
    }
    *p_status = (motion_status_snapshot_t){0};
    p_status->status =
        (13 == g_case) ? MOTION_STATUS_CANCELLED : MOTION_STATUS_COMPLETED;
    p_status->physically_stopped = true;
    return E_OK;
}

/** @brief Surface production status messages in test output. */
int
tm_printf (const UB * p_format, ...)
{
    va_list arguments;

    va_start(arguments, p_format);
    int count = vprintf((const char *)p_format, arguments);

    va_end(arguments);
    return count;
}

/*** end of file ***/
