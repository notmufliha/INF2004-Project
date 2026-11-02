/** @file imu_vc.c
 * @brief Nonblocking single-producer, single-consumer controller handoff.
 */
#include "imu_vc.h"

typedef uint8_t imu_vc_size_check_t[(sizeof(imu_notice_t) <= 128U) ? 1 : -1];

static volatile ID          gh_notice_queue;
static imu_notice_tracker_t g_tracker;

/** @brief Create the private queue before starting B4 tasks. */
ER
imu_vc_init (void)
{
    T_CMBF config = {0};

    if (gh_notice_queue > 0)
    {
        return E_OBJ;
    }

    imu_notice_init(&g_tracker);
    config.mbfatr = TA_TFIFO;
    /* The size check bounds each aligned slot to 132 bytes, fitting SZ. */
    config.bufsz =
        (SZ)(IMU_VC_QUEUE_DEPTH * (4U + ((sizeof(imu_notice_t) + 3U) & ~3U)));
    config.maxmsz   = sizeof(imu_notice_t);
    gh_notice_queue = tk_cre_mbf(&config);
    return (gh_notice_queue < E_OK) ? gh_notice_queue : E_OK;
}

/** @brief Publish from the analysis task only; retain edges if the queue fills.
 */
ER
imu_vc_publish (const imu_event_t * p_event)
{
    imu_notice_t notice = {0};

    if (NULL == p_event)
    {
        return E_PAR;
    }

    if (gh_notice_queue <= 0)
    {
        return E_OBJ;
    }

    imu_notice_track(&g_tracker, p_event);

    if (!imu_notice_peek(&g_tracker, &notice))
    {
        return E_OK;
    }

    ER error = tk_snd_mbf(gh_notice_queue, &notice, sizeof(notice), TMO_POL);

    if (E_OK == error)
    {
        imu_notice_sent(&g_tracker);
    }
    else
    {
        imu_notice_failed(&g_tracker);
    }

    return error;
}

/** @brief Receive a copied notice with polling or a bounded timeout. */
ER
imu_vc_receive (imu_notice_t * p_notice, TMO timeout_ms)
{
    imu_notice_t notice = {0};

    if (NULL == p_notice)
    {
        return E_PAR;
    }

    *p_notice = (imu_notice_t){0};
    if (timeout_ms < 0)
    {
        return E_PAR;
    }

    if (gh_notice_queue <= 0)
    {
        return E_OBJ;
    }

    INT length = tk_rcv_mbf(gh_notice_queue, &notice, timeout_ms);

    if (length < E_OK)
    {
        return length;
    }

    /* The compile-time bound makes this conversion safe. */
    if ((INT)sizeof(notice) != length)
    {
        return E_SYS;
    }

    *p_notice = notice;
    return E_OK;
}

/*** end of file ***/

