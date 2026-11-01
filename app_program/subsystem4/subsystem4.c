/** @file subsystem4.c
 * @brief Buddy 4 subsystem4 module.
 */
/*
 *  subsystem4.c
 *  Subsystem 4 -- creates IMU demo objects/tasks and legacy monitor.
 *
 *  APIs shown: tk_cre_flg, tk_cre_mbf, tk_cre_alm. */

#include "subsystem4.h"
#include "s4_runtime.h"
#include "imu.h"
#include "task_imu.h"
#include "task_monitor.h"

/* Both payloads fit 64 bytes; aligned storage times either queue depth fits SZ.
 */
typedef uint8_t s4_payload_size_check_t
    [((sizeof(imu_data_t) <= 64U) && (sizeof(imu_event_t) <= 64U)) ? 1 : -1];

volatile ID gh_summary; /* IF-03. */
volatile ID gh_events;  /* IF-03. */
volatile ID gh_monitor_alarm;
volatile ID gh_imu_data_buffer;
#if APP_TEST
volatile ID gh_imu_test_events;
#endif

static SZ message_storage_bytes (uint32_t payload_bytes);

/** @brief Create the B4 queues, event flag and monitor alarm. */
ER
subsystem4_init (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = S4_IMU_QUEUE_DEPTH * message_storage_bytes(sizeof(imu_data_t));
    cmbf.maxmsz        = sizeof(imu_data_t);
    gh_imu_data_buffer = tk_cre_mbf(&cmbf);

    if (gh_imu_data_buffer < E_OK)
    {
        return gh_imu_data_buffer;
    }
#if APP_TEST
    cmbf.bufsz         = 32 * message_storage_bytes(sizeof(imu_event_t));
    cmbf.maxmsz        = sizeof(imu_event_t);
    gh_imu_test_events = tk_cre_mbf(&cmbf);

    if (gh_imu_test_events < E_OK)
    {
        return gh_imu_test_events;
    }
#endif

    /* Event flag: a 32-bit pattern of bits.  Tasks wait for a bit (or a
       combination of bits) to be set. */
    cflg.flgatr  = TA_TFIFO | TA_WMUL; /* allow several waiters. */
    cflg.iflgptn = 0;                  /* all bits clear to begin with. */
    gh_events    = tk_cre_flg(&cflg);

    if (gh_events < E_OK)
    {
        return gh_events;
    }

    /* Message buffer: COPIES variable-length data into a kernel ring
       buffer.  Here it carries short text lines from subsystem 3. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = 4 * S4_SUMMARY_MAX; /* total buffer size, bytes. */
    cmbf.maxmsz = S4_SUMMARY_MAX;     /* largest single message. */
    gh_summary  = tk_cre_mbf(&cmbf);

    if (gh_summary < E_OK)
    {
        return gh_summary;
    }

    /* Alarm handler: called ONCE, a given time after tk_sta_alm().
       The monitor task re-arms it each time round its loop. */
    gh_monitor_alarm = task_monitor_create_alarm();
    return (gh_monitor_alarm < E_OK) ? gh_monitor_alarm : E_OK;
}

/** @brief Start receivers before the sampling task begins publishing. */
ER
subsystem4_start (void)
{
    /* Start receiver first; all objects were created during init. */
    ID h_imu_task = app_start_task(task_imu_analysis_task, S4_PRI_IMU_ANALYSIS,
                                   0, "task_imu_analysis_task");

    if (h_imu_task < E_OK)
    {
        return h_imu_task;
    }

    h_imu_task = app_start_task(task_imu_sampling_task, S4_PRI_IMU_SAMPLE, 0,
                                "task_imu_sampling_task");

    if (h_imu_task < E_OK)
    {
        return h_imu_task;
    }

    /* No affinity: the scheduler may run it on either core. */
    ID h_monitor_task = app_start_task(task_monitor_task, S4_PRI_MONITOR, 0,
                                       "task_monitor_task");
    return (h_monitor_task < E_OK) ? h_monitor_task : E_OK;
}

/** @brief Compute aligned kernel storage for a bounded B4 payload. */
static SZ
message_storage_bytes (uint32_t payload_bytes)
{
    /* Only the fixed, small B4 payload sizes are passed by this module. */
    /* The compile-time payload check bounds this unsigned result to 68. */
    return (SZ)(4U + ((payload_bytes + 3U) & ~3U));
}

/*** end of file ***/

