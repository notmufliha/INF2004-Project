/** @file s4_motion_test.c
 * @brief Bounded one-shot forward test with B4 observations and B5 permission.
 */
#include "s4_motion_test.h"
#include "s4_runtime.h"
#include "subsystem4.h"
#include "imu.h"
#include "s4_diag.h"
#include "../subsystem2/motion_command.h"
#include "../subsystem2/b2_motor.h"

#if S4_MOTION_TEST_ACTIVE
#if ROBOT_MOTION_TEST || ROBOT_WHEELS_UP_MOTOR_TEST || !ROBOT_ENABLE_DRIVE
#error Disable B2 motor tests and enable guarded drive for the B4 motion test
#endif
#if (S4_MOTION_TEST_DISTANCE_CM < 1U) || (S4_MOTION_TEST_DISTANCE_CM > 150U)
#error B4 test distance must be 1..150 centimetres
#endif
#if (S4_MOTION_TEST_SPEED_CPS < 1) || (S4_MOTION_TEST_SPEED_CPS > 1000)
#error B4 test speed must be 1..1000 encoder counts per second
#endif

#define S4_TEST_WAIT_MS 30000U
#define S4_TEST_RUN_MS 65000U
#define S4_TEST_FRESH_MS 2000U
#define S4_TEST_STEP_MS 20
#define S4_TEST_TAP_DEPTH 32U

typedef struct
{
    uint32_t last_event_ms;
    uint32_t humps;
    uint32_t heights;
    bool     b_seen;
} s4_motion_test_state_t;

static bool gb_started;

static ER accept_event (s4_motion_test_state_t * p_state,
                        const imu_event_t *      p_event);

static ER read_clock (uint32_t * p_now);
static ER observe_events (s4_motion_test_state_t * p_state, uint32_t now);
static ER read_status (motion_status_snapshot_t * p_status, uint32_t now);
static ER wait_clear (s4_motion_test_state_t * p_state);
static ER follow_motion (s4_motion_test_state_t * p_state, uint32_t command_id);
static void report_height (s4_motion_test_state_t * p_state,
                           const imu_event_t *      p_event);
static void print_diagnostics (void);
#endif

/** @brief Run once from B4 tests after B2/B5 tasks have been started. */
ER
s4_motion_test_run (void)
{
#if S4_MOTION_TEST_ACTIVE
    s4_motion_test_state_t state      = {0};
    uint32_t               command_id = 0U;

    if (gb_started)
    {
        return E_OBJ;
    }

    gb_started = true;
    tm_printf((UB *)"[B4-TEST] Armed: forward %u cm at %d counts/s; "
                    "starts after %u ms. Clear route; B5 guards front only.\n",
              S4_MOTION_TEST_DISTANCE_CM, S4_MOTION_TEST_SPEED_CPS,
              S4_MOTION_TEST_BOOT_MS);

    ER error = tk_dly_tsk(S4_MOTION_TEST_BOOT_MS);

    /* This test owns the APP_TEST tap after s4_test_run's earlier checks.
     * It never consumes the Vehicle Controller queue or raw sensor queue.
     */
    if (E_OK == error)
    {
        error = wait_clear(&state);
    }

    if (E_OK == error)
    {
        state.humps   = 0U;
        state.heights = 0U;
        error         = s4_diag_begin();
    }

    if (E_OK == error)
    {
        error = motion_move_cm((float32_t)S4_MOTION_TEST_DISTANCE_CM,
                               S4_MOTION_TEST_SPEED_CPS, MOTION_DIR_FORWARD,
                               &command_id);
    }

    if (E_OK == error)
    {
        tm_printf((UB *)"[B4-TEST] START id=%u\n", command_id);
        error = follow_motion(&state, command_id);
    }

    b2_request_stop();
    print_diagnostics();
    tm_printf((UB *)"[B4-TEST] %s error=%d humps=%u valid_heights=%u; "
                    "STOP requested. Reboot to repeat.\n",
              (E_OK == error) ? "MOTION PASS" : "ABORTED", error, state.humps,
              state.heights);
    return error;
#else
    return E_OBJ;
#endif
}

#if S4_MOTION_TEST_ACTIVE
/** @brief Freeze collection after requesting STOP and print outside its lock.
 */
static void
print_diagnostics (void)
{
    s4_diag_report_t report = {0};

    if (E_OK == s4_diag_end(&report))
    {
        s4_diag_print(&report);
    }
    else
    {
        tm_printf((UB *)"[B4-DIAG] No motion capture started\n");
    }
}

/** @brief Propagate clock failures instead of fabricating a timestamp. */
static ER
read_clock (uint32_t * p_now)
{
    SYSTIM time  = {0};
    ER     error = tk_get_tim(&time);

    *p_now = time.lo;
    return error;
}

/** @brief Require a recent healthy B2 control snapshot. */
static ER
read_status (motion_status_snapshot_t * p_status, uint32_t now)
{
    ER error = motion_get_status(p_status);

    if (E_OK != error)
    {
        return error;
    }

    if (((now - p_status->sample_time_ms) > 100U) ||
        (0U == p_status->control_interval_ms) ||
        (p_status->control_interval_ms > 100U) ||
        (MOTION_STATUS_FAULT == p_status->status))
    {
        return E_OBJ;
    }

    return E_OK;
}

/** @brief Await standstill, fresh B4 data and existing B5 permission. */
static ER
wait_clear (s4_motion_test_state_t * p_state)
{
    uint32_t start = 0U;
    uint32_t now   = 0U;
    ER       error = read_clock(&start);

    if (E_OK != error)
    {
        return error;
    }

    tm_printf((UB *)"[B4-TEST] Waiting for B5 clearance, B2 standstill "
                    "and fresh B4 readings (30s maximum)\n");

    do
    {
        motion_status_snapshot_t status = {0};

        error = read_clock(&now);

        if (E_OK != error)
        {
            return error;
        }

        error = observe_events(p_state, now);

        if ((E_OK != error) && (E_TMOUT != error))
        {
            return error;
        }

        if ((E_OK == error) && (E_OK == read_status(&status, now)) &&
            (status.physically_stopped) &&
            (MOTION_STATUS_RUNNING != status.status) &&
            (b2_safety_motion_allowed()) && (E_OK == subsystem4_wait_ready(1)))
        {
            return E_OK;
        }

        error = tk_dly_tsk(S4_TEST_STEP_MS);
    } while ((E_OK == error) && ((now - start) < S4_TEST_WAIT_MS));
    return (E_OK == error) ? E_TMOUT : error;
}

/** @brief Monitor motion and abort on permission loss, stale data or faults. */
static ER
follow_motion (s4_motion_test_state_t * p_state, uint32_t command_id)
{
    uint32_t start = 0U;
    uint32_t now   = 0U;
    ER       error = read_clock(&start);

    if (E_OK != error)
    {
        return error;
    }

    do
    {
        motion_status_snapshot_t status = {0};
        motion_status_snapshot_t result = {0};

        error = read_clock(&now);

        if (E_OK != error)
        {
            return error;
        }

        if ((E_OK != observe_events(p_state, now)) ||
            (E_OK != subsystem4_wait_ready(1)) ||
            (E_OK != read_status(&status, now)))
        {
            tm_printf((UB *)"[B4-TEST] Abort: B4/B2 fault or stale data\n");
            return E_OBJ;
        }

        ER result_error = motion_get_result(command_id, &result);

        if (E_OK == result_error)
        {
            tm_printf((UB *)"[B4-TEST] B2 result=%d fault=%d stopped=%d\n",
                      result.status, result.fault, result.physically_stopped);
            return ((MOTION_STATUS_COMPLETED == result.status) &&
                    (result.physically_stopped))
                       ? E_OK
                       : E_OBJ;
        }

        if ((E_OBJ != result_error) || (!b2_safety_motion_allowed()) ||
            ((MOTION_STATUS_RUNNING == status.status) &&
             (command_id != status.command_id)))
        {
            tm_printf((UB *)"[B4-TEST] Abort: B5 permission lost, "
                            "command replaced or result read failed\n");
            return E_OBJ;
        }

        error = tk_dly_tsk(S4_TEST_STEP_MS);
    } while ((E_OK == error) && ((now - start) < S4_TEST_RUN_MS));
    return (E_OK == error) ? E_TMOUT : error;
}

/** @brief Drain a bounded test tap and reject hazards or missing IMU updates.
 */
static ER
observe_events (s4_motion_test_state_t * p_state, uint32_t now)
{
    for (uint32_t index = 0U; index < S4_TEST_TAP_DEPTH; ++index)
    {
        imu_event_t event  = {0};
        INT         length = tk_rcv_mbf(gh_imu_test_events, &event, TMO_POL);

        if (E_TMOUT == length)
        {
            break;
        }

        if ((INT)sizeof(event) != length)
        {
            return E_OBJ;
        }

        ER error = E_OK;

        if ((now - event.timestamp_ms) <= S4_TEST_FRESH_MS)
        {
            error = accept_event(p_state, &event);
        }

        if (E_OK != error)
        {
            return error;
        }
    }

    return ((p_state->b_seen) &&
            ((now - p_state->last_event_ms) <= S4_TEST_FRESH_MS))
               ? E_OK
               : E_TMOUT;
}

/** @brief Reject hazards before updating the last accepted observation. */
static ER
accept_event (s4_motion_test_state_t * p_state, const imu_event_t * p_event)
{
    if ((!p_event->b_valid) || (p_event->b_sensor_fault) ||
        (p_event->b_sudden_impact) ||
        (!imu_numeric_is_finite(p_event->roll_deg)) ||
        (p_event->roll_deg > 30.0f) || (p_event->roll_deg < -30.0f))
    {
        return E_OBJ;
    }

    p_state->b_seen        = true;
    p_state->last_event_ms = p_event->timestamp_ms;
    report_height(p_state, p_event);
    return E_OK;
}

/** @brief Separate traversal detection from accepted height estimation. */
static void
report_height (s4_motion_test_state_t * p_state, const imu_event_t * p_event)
{
    if (p_event->b_hump_completed)
    {
        ++p_state->humps;

        if (p_event->b_height_valid)
        {
            ++p_state->heights;
            tm_printf((UB *)"[B4-TEST] Hump estimate=%d mm highest=%d mm\n",
                      imu_numeric_to_int(p_event->hump_peak_cm * 10.0f),
                      imu_numeric_to_int(p_event->highest_hump_cm * 10.0f));
        }
        else
        {
            tm_printf((UB *)"[B4-TEST] Hump completed; height=N/A\n");
        }
    }
}
#endif

/*** end of file ***/

