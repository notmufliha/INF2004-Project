/** @file imu_notice.c
 * @brief Retain observation edges and completion details through queue
 * pressure.
 */
#include "imu_notice.h"
#include <stddef.h>

static uint32_t quality_edges (imu_notice_tracker_t * p_tracker,
                               const imu_event_t * p_event, bool b_usable);
static uint32_t measurement_edges (imu_notice_tracker_t * p_tracker,
                                   const imu_event_t * p_event, bool b_usable);
static uint32_t tilt_impact_edges (imu_notice_tracker_t * p_tracker,
                                   const imu_event_t *    p_event);

/** @brief Reset the single producer before its task begins publishing. */
void
imu_notice_init (imu_notice_tracker_t * p_tracker)
{
    if (NULL != p_tracker)
    {
        *p_tracker = (imu_notice_tracker_t){0};
    }
}

/** @brief Update current state and retain newly observed transition bits. */
void
imu_notice_track (imu_notice_tracker_t * p_tracker, const imu_event_t * p_event)
{
    if ((NULL == p_tracker) || (NULL == p_event))
    {
        return;
    }

    const bool b_usable = (p_event->b_valid) && (!p_event->b_sensor_fault);
    uint32_t   events   = quality_edges(p_tracker, p_event, b_usable);

    events |= measurement_edges(p_tracker, p_event, b_usable);

    if ((!p_tracker->b_seen) ||
        ((p_event->timestamp_ms - p_tracker->emission_ms) >=
         IMU_NOTICE_PERIOD_MS))
    {
        events |= IMU_NOTICE_STATE;
    }

    p_tracker->pending.snapshot         = *p_event;
    p_tracker->pending.snapshot.b_valid = b_usable;
    p_tracker->b_seen                   = true;

    if (0U != events)
    {
        ++p_tracker->sequence;

        if (0U == p_tracker->pending.events)
        {
            p_tracker->pending.first_sequence     = p_tracker->sequence;
            p_tracker->pending.first_timestamp_ms = p_event->timestamp_ms;
        }

        p_tracker->pending.last_sequence     = p_tracker->sequence;
        p_tracker->pending.last_timestamp_ms = p_event->timestamp_ms;
        p_tracker->pending.events |= events;
        p_tracker->emission_ms = p_event->timestamp_ms;
    }
}

/** @brief Copy a pending notice without consuming it before a successful send.
 */
bool
imu_notice_peek (const imu_notice_tracker_t * p_tracker,
                 imu_notice_t *               p_notice)
{
    if (NULL == p_notice)
    {
        return false;
    }

    *p_notice = (imu_notice_t){0};
    if ((NULL == p_tracker) || (0U == p_tracker->pending.events))
    {
        return false;
    }

    *p_notice = p_tracker->pending;
    return true;
}

/** @brief Clear delivered edges, retaining cumulative counters/current state.
 */
void
imu_notice_sent (imu_notice_tracker_t * p_tracker)
{
    if (NULL != p_tracker)
    {
        p_tracker->pending.events                   = 0U;
        p_tracker->pending.first_sequence           = 0U;
        p_tracker->pending.last_sequence            = 0U;
        p_tracker->pending.completion_timestamp_ms  = 0U;
        p_tracker->pending.completed_height_cm      = 0.0f;
        p_tracker->pending.b_completed_height_valid = false;
    }
}

/** @brief Record a failed attempt and preserve pending edges for retry. */
void
imu_notice_failed (imu_notice_tracker_t * p_tracker)
{
    if ((NULL != p_tracker) && (0U != p_tracker->pending.events))
    {
        ++p_tracker->pending.failed_sends;
        p_tracker->pending.events |= IMU_NOTICE_OVERFLOW;
    }
}

/** @brief Distinguish unknown data, a latched IMU fault and valid recovery. */
static uint32_t
quality_edges (imu_notice_tracker_t * p_tracker, const imu_event_t * p_event,
               bool b_usable)
{
    uint32_t events = 0U;

    if ((!p_tracker->b_seen) || (b_usable != p_tracker->b_valid))
    {
        events |= b_usable ? IMU_NOTICE_READY : IMU_NOTICE_UNAVAILABLE;
    }

    if ((p_event->b_sensor_fault) && (!p_tracker->b_fault))
    {
        events |= IMU_NOTICE_FAULT;
    }

    if (b_usable && ((p_tracker->b_fault) || ((p_event->b_sensor_recovered) &&
                                              (!p_tracker->b_recovered))))
    {
        events |= IMU_NOTICE_RECOVERED;
    }

    p_tracker->b_fault =
        (p_event->b_sensor_fault) || ((p_tracker->b_fault) && (!b_usable));
    p_tracker->b_valid     = b_usable;
    p_tracker->b_recovered = b_usable && (p_event->b_sensor_recovered);
    return events;
}

/** @brief Track hump entry/completion/abort without treating invalid as level.
 */
static uint32_t
measurement_edges (imu_notice_tracker_t * p_tracker,
                   const imu_event_t * p_event, bool b_usable)
{
    uint32_t   events = 0U;
    const bool b_completed =
        b_usable && (p_event->b_hump_completed) && (!p_event->b_sudden_impact);
    const bool b_active = b_usable && (p_event->b_hump_detected) &&
                          (!b_completed) && (!p_event->b_sudden_impact);

    if (b_active && (!p_tracker->b_hump_active))
    {
        events |= IMU_NOTICE_HUMP_ENTER;
    }

    if ((p_tracker->b_hump_active) && (!b_active) && (!b_completed))
    {
        events |= IMU_NOTICE_HUMP_ABORT;
    }

    if (b_completed && (!p_tracker->b_completed))
    {
        events |= IMU_NOTICE_HUMP_DONE;
        p_tracker->pending.completion_timestamp_ms  = p_event->timestamp_ms;
        p_tracker->pending.completed_height_cm      = p_event->hump_peak_cm;
        p_tracker->pending.b_completed_height_valid = p_event->b_height_valid;
    }

    if (b_usable)
    {
        events |= tilt_impact_edges(p_tracker, p_event);
    }

    p_tracker->b_hump_active = b_active;
    p_tracker->b_completed   = b_completed;
    return events;
}

/** @brief Emit tilt changes and one impact edge until a valid sample rearms it.
 */
static uint32_t
tilt_impact_edges (imu_notice_tracker_t * p_tracker,
                   const imu_event_t *    p_event)
{
    uint32_t events = 0U;

    if (p_event->b_tilt_detected)
    {
        if ((!p_tracker->b_tilt_known) || (!p_tracker->b_tilt))
        {
            events |= IMU_NOTICE_TILT_ON;
        }
    }
    else if ((p_tracker->b_tilt_known) && (p_tracker->b_tilt))
    {
        events |= IMU_NOTICE_TILT_OFF;
    }
    else
    {
        /* Initial level data is described by the state snapshot. */
    }

    if ((p_event->b_sudden_impact) && (!p_tracker->b_impact))
    {
        events |= IMU_NOTICE_IMPACT;
    }

    p_tracker->b_tilt_known = true;
    p_tracker->b_tilt       = p_event->b_tilt_detected;
    p_tracker->b_impact     = p_event->b_sudden_impact;
    return events;
}

/*** end of file ***/

