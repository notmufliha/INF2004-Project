/** @file s4_test.c
 * @brief Verify B4 kernel objects and fresh analysis events on the Pico.
 */
#include "s4_test.h"
#include "subsystem4.h"
#include "s4_runtime.h"
#include "imu.h"
#if !ROBOT_MODE
#include "task_monitor.h"
#endif
#include "../test_report.h"

#define TEST_MOCK_WAIT_MS 9000U
#define TEST_SENSOR_WAIT_MS 10000U
#define TEST_EVENT_WAIT_MS 100

#if APP_TEST
static void discard_events (void);
static bool event_matches (const imu_event_t * p_event, bool b_wait_hump,
                           bool b_wait_recovery, bool * pb_fault_seen);
static bool wait_event (uint32_t timeout_ms, bool b_wait_hump,
                        bool b_wait_recovery);

/** @brief Check message formats and end-to-end B4 task communication. */
void
s4_test_run (void)
{
    T_RMBF buffer;

    TEST_INIT();
#if !ROBOT_MODE
    TEST_CHECK("S4-U01",
               (E_OK == tk_ref_mbf(gh_summary, &buffer)) &&
                   (S4_SUMMARY_MAX == buffer.maxmsz),
               "message buffer max line = S4_SUMMARY_MAX");

    UW before = task_monitor_line_count();

    tk_dly_tsk(3500);
    TEST_CHECK("INT-02", task_monitor_line_count() >= before + 2U,
               "summary lines flow from subsystem 3 to subsystem 4");
#else
    TEST_CHECK("S4-U17", E_OK == subsystem4_wait_ready(100),
               "real robot startup observes accepted IMU readings");
    TEST_CHECK("S4-U18", (0 == gh_summary) && (0 == gh_monitor_alarm),
               "real robot has no teaching monitor objects");
#endif
    /* Fixed reading size is less than INT_MAX in the verified kernel ABI. */
    TEST_CHECK("S4-U10",
               (E_OK == tk_ref_mbf(gh_imu_data_buffer, &buffer)) &&
                   ((INT)sizeof(imu_data_t) == buffer.maxmsz),
               "IMU message buffer carries a complete reading");
    discard_events();
#if S4_IMU_USE_MOCK
    TEST_CHECK("S4-U11", wait_event(TEST_MOCK_WAIT_MS, true, false),
               "mock readings flow through RTOS tasks to a fresh hump report");
    TEST_CHECK("S4-U14", wait_event(TEST_MOCK_WAIT_MS, false, true),
               "demo outage reports sensor fault followed by valid recovery");
#else
    TEST_CHECK("S4-U16", wait_event(TEST_SENSOR_WAIT_MS, false, false),
               "real GY-511 readings arrive after level calibration");
#endif
    TEST_SUMMARY("subsystem4");
}

/** @brief Discard old observations without stealing acquisition readings. */
static void
discard_events (void)
{
    imu_event_t event;

    while (tk_rcv_mbf(gh_imu_test_events, &event, TMO_POL) > 0)
    {
        /* Drain queued observations before waiting for fresh results. */
    }
}

/** @brief Match a measurement or a fault followed by recovery. */
static bool
event_matches (const imu_event_t * p_event, bool b_wait_hump,
               bool b_wait_recovery, bool * pb_fault_seen)
{
    if (b_wait_recovery)
    {
        if ((p_event->b_sensor_fault) && (!p_event->b_valid))
        {
            *pb_fault_seen = true;
        }

        return (*pb_fault_seen) && (p_event->b_sensor_recovered) &&
               (p_event->b_valid);
    }

    if (b_wait_hump)
    {
        return (p_event->b_valid) && (p_event->b_hump_completed) &&
               (!p_event->b_height_valid);
    }

    return (p_event->b_valid) && (!p_event->b_sensor_fault);
}

/** @brief Wait a bounded interval, checking clock and receive results. */
static bool
wait_event (uint32_t timeout_ms, bool b_wait_hump, bool b_wait_recovery)
{
    SYSTIM      start;
    SYSTIM      now;
    imu_event_t event;
    bool        b_fault_seen = false;

    if (tk_get_tim(&start) < E_OK)
    {
        return false;
    }

    do
    {
        INT length = tk_rcv_mbf(gh_imu_test_events, &event, TEST_EVENT_WAIT_MS);

        /* The fixed event size is below INT_MAX in the kernel ABI. */
        if (((INT)sizeof(event) == length) &&
            (event_matches(&event, b_wait_hump, b_wait_recovery,
                           &b_fault_seen)))
        {
            return true;
        }

        if (tk_get_tim(&now) < E_OK)
        {
            return false;
        }
    } while (now.lo - start.lo < timeout_ms);
    return false;
}
#endif

/*** end of file ***/

