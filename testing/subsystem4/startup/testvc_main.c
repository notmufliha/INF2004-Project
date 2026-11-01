/** @file testvc_main.c
 * @brief Execute the production controller handoff against a bounded queue.
 */
#include "imu_vc.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static imu_notice_t g_queue[IMU_VC_QUEUE_DEPTH];
static uint32_t     g_read;
static uint32_t     g_count;
static ER           g_create_error;
static ER           g_send_error;
static bool         gb_bad_length;
static TMO          g_receive_timeout;

static void test_edges (void);
static void test_overflow (void);
static void test_tracker (void);

/** @brief Check startup errors, safe receive output and all transition tests.
 */
int
main (void)
{
    imu_notice_t notice = {0};
    imu_event_t  event  = {0};

    assert(E_OBJ == imu_vc_publish(&event));
    assert(E_OBJ == imu_vc_receive(&notice, 0));
    assert(E_PAR == imu_vc_publish(NULL));
    assert(E_PAR == imu_vc_receive(NULL, 0));
    assert(E_PAR == imu_vc_receive(&notice, -1));
    g_create_error = E_SYS;
    assert(E_SYS == imu_vc_init());
    g_create_error = E_OK;
    assert(E_OK == imu_vc_init());
    assert(E_OBJ == imu_vc_init());
    notice.events = UINT32_MAX;
    assert(E_TMOUT == imu_vc_receive(&notice, 25));
    assert((0U == notice.events) && (25 == g_receive_timeout));
    gb_bad_length = true;
    assert(E_SYS == imu_vc_receive(&notice, 0));
    assert(0U == notice.events);
    gb_bad_length = false;
    test_edges();
    test_overflow();
    test_tracker();
    printf("PASS: controller edges, bounded queue, recovery, coalescing; "
           "notice=%zu bytes\n",
           sizeof(imu_notice_t));
    return 0;
}

/** @brief Verify edge suppression, invalid tilt handling and hump payloads. */
static void
test_edges (void)
{
    imu_event_t  event  = {0};
    imu_notice_t notice = {0};

    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    assert(0U != (notice.events & IMU_NOTICE_UNAVAILABLE));
    event.timestamp_ms    = 10U;
    event.b_valid         = true;
    event.b_tilt_detected = true;
    event.b_hump_detected = true;
    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    assert((IMU_NOTICE_READY | IMU_NOTICE_TILT_ON | IMU_NOTICE_HUMP_ENTER) ==
           notice.events);
    assert(E_OK == imu_vc_publish(&event));
    assert(E_TMOUT == imu_vc_receive(&notice, 0));
    event.b_valid         = false;
    event.b_tilt_detected = false;
    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    assert(0U != (notice.events & IMU_NOTICE_HUMP_ABORT));
    assert(0U == (notice.events & IMU_NOTICE_TILT_OFF));
    event.b_valid          = true;
    event.b_hump_detected  = false;
    event.b_hump_completed = true;
    event.b_height_valid   = true;
    event.hump_peak_cm     = 2.5f;
    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    assert(0U != (notice.events & IMU_NOTICE_HUMP_DONE));
    assert(0U != (notice.events & IMU_NOTICE_TILT_OFF));
    assert(notice.b_completed_height_valid);
    assert(2.5f == notice.completed_height_cm);
    event.b_hump_completed = false;
    event.timestamp_ms += IMU_NOTICE_PERIOD_MS;
    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    assert(IMU_NOTICE_STATE == notice.events);
}

/** @brief Queue saturation preserves fault/recovery and completion metadata. */
static void
test_overflow (void)
{
    imu_event_t  event  = {0};
    imu_notice_t notice = {0};

    event.b_valid      = true;
    event.timestamp_ms = 2000U;
    for (uint32_t index = 0U; index < IMU_VC_QUEUE_DEPTH; ++index)
    {
        event.b_tilt_detected = !event.b_tilt_detected;
        event.timestamp_ms += 10U;
        assert(E_OK == imu_vc_publish(&event));
    }
    event.b_sensor_fault = true;
    assert(E_TMOUT == imu_vc_publish(&event));
    event.b_sensor_fault  = false;
    event.b_sudden_impact = true;
    assert(E_TMOUT == imu_vc_publish(&event));
    event.b_sudden_impact  = false;
    event.b_hump_completed = true;
    event.b_height_valid   = true;
    event.hump_peak_cm     = 4.0f;
    event.timestamp_ms += 10U;
    assert(E_TMOUT == imu_vc_publish(&event));
    const uint32_t completion_ms = event.timestamp_ms;

    for (uint32_t index = 0U; index < IMU_VC_QUEUE_DEPTH; ++index)
    {
        assert(E_OK == imu_vc_receive(&notice, 0));
    }
    event.b_hump_completed = false;
    event.b_height_valid   = false;
    event.hump_peak_cm     = 0.0f;
    event.timestamp_ms += 10U;
    g_send_error = E_SYS;
    assert(E_SYS == imu_vc_publish(&event));
    g_send_error = E_OK;
    assert(E_OK == imu_vc_publish(&event));
    assert(E_OK == imu_vc_receive(&notice, 0));
    const uint32_t expected = IMU_NOTICE_FAULT | IMU_NOTICE_RECOVERED |
                              IMU_NOTICE_IMPACT | IMU_NOTICE_HUMP_DONE |
                              IMU_NOTICE_OVERFLOW;

    assert(expected == (expected & notice.events));
    assert(4U == notice.failed_sends);
    assert(notice.first_sequence < notice.last_sequence);
    assert(completion_ms == notice.completion_timestamp_ms);
    assert(4.0f == notice.completed_height_cm);
    assert(notice.b_completed_height_valid);
    assert(notice.snapshot.b_valid);
    assert(!notice.snapshot.b_hump_completed);
    assert(event.timestamp_ms == notice.snapshot.timestamp_ms);
    assert(E_OK == imu_vc_publish(&event));
    assert(E_TMOUT == imu_vc_receive(&notice, 0));
}

/** @brief Check heartbeat wraparound and null-safe pure tracker operations. */
static void
test_tracker (void)
{
    imu_notice_tracker_t tracker = {0};
    imu_notice_t         notice  = {0};
    imu_event_t          event   = {0};

    imu_notice_init(NULL);
    imu_notice_track(NULL, &event);
    imu_notice_sent(NULL);
    imu_notice_failed(NULL);
    assert(!imu_notice_peek(NULL, &notice));
    assert(!imu_notice_peek(&tracker, NULL));
    imu_notice_init(&tracker);
    event.b_valid      = true;
    event.timestamp_ms = UINT32_MAX - 499U;
    imu_notice_track(&tracker, &event);
    imu_notice_sent(&tracker);
    event.timestamp_ms = 500U;
    imu_notice_track(&tracker, &event);
    assert(imu_notice_peek(&tracker, &notice));
    assert(IMU_NOTICE_STATE == notice.events);
    imu_notice_sent(&tracker);
    assert(!imu_notice_peek(&tracker, &notice));
}

/* Kernel names intentionally match the production ABI. */
/** @brief Create a bounded copied-message queue with injectable failure. */
ID
tk_cre_mbf (const T_CMBF * p_config)
{
    assert((SZ)sizeof(imu_notice_t) == p_config->maxmsz);
    assert((SZ)(IMU_VC_QUEUE_DEPTH * (sizeof(imu_notice_t) + 4U)) ==
           p_config->bufsz);
    return (E_OK == g_create_error) ? 3 : g_create_error;
}

/** @brief Copy a message without allowing the producer to block. */
ER
tk_snd_mbf (ID h_buffer, const void * p_data, INT length, TMO timeout)
{
    assert((3 == h_buffer) && (TMO_POL == timeout));
    assert((INT)sizeof(imu_notice_t) == length);
    if (E_OK != g_send_error)
    {
        return g_send_error;
    }
    if (IMU_VC_QUEUE_DEPTH == g_count)
    {
        return E_TMOUT;
    }
    memcpy(&g_queue[(g_read + g_count) % IMU_VC_QUEUE_DEPTH], p_data,
           sizeof(imu_notice_t));
    ++g_count;
    return E_OK;
}

/** @brief Pop a copy, or inject an empty queue/malformed message. */
INT
tk_rcv_mbf (ID h_buffer, void * p_data, TMO timeout)
{
    assert(3 == h_buffer);
    g_receive_timeout = timeout;
    if (gb_bad_length)
    {
        return 1;
    }
    if (0U == g_count)
    {
        return E_TMOUT;
    }
    memcpy(p_data, &g_queue[g_read], sizeof(imu_notice_t));
    g_read = (g_read + 1U) % IMU_VC_QUEUE_DEPTH;
    --g_count;
    return (INT)sizeof(imu_notice_t);
}

/*** end of file ***/
