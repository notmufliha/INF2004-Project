/** @file imu_notice.h
 * @brief Typed B4 observations and pure transition/coalescing state.
 */
#ifndef IMU_NOTICE_H
#define IMU_NOTICE_H

#include "imu.h"

#define IMU_NOTICE_READY 0x0001U
#define IMU_NOTICE_HUMP_ENTER 0x0002U
#define IMU_NOTICE_HUMP_DONE 0x0004U
#define IMU_NOTICE_HUMP_ABORT 0x0008U
#define IMU_NOTICE_TILT_ON 0x0010U
#define IMU_NOTICE_TILT_OFF 0x0020U
#define IMU_NOTICE_IMPACT 0x0040U
#define IMU_NOTICE_FAULT 0x0080U
#define IMU_NOTICE_RECOVERED 0x0100U
#define IMU_NOTICE_UNAVAILABLE 0x0200U
#define IMU_NOTICE_STATE 0x0400U
#define IMU_NOTICE_OVERFLOW 0x80000000U
#define IMU_NOTICE_PERIOD_MS 1000U

typedef struct
{
    uint32_t    events;
    uint32_t    first_sequence;
    uint32_t    last_sequence;
    uint32_t    failed_sends;
    uint32_t    first_timestamp_ms;
    uint32_t    last_timestamp_ms;
    uint32_t    completion_timestamp_ms;
    float32_t   completed_height_cm;
    bool        b_completed_height_valid;
    imu_event_t snapshot;
} imu_notice_t;

/* One producer owns this state. Sequence/failure counters wrap modulo 2^32. */
typedef struct
{
    imu_notice_t pending;
    uint32_t     sequence;
    uint32_t     emission_ms;
    bool         b_seen;
    bool         b_valid;
    bool         b_hump_active;
    bool         b_completed;
    bool         b_tilt_known;
    bool         b_tilt;
    bool         b_impact;
    bool         b_fault;
    bool         b_recovered;
} imu_notice_tracker_t;

void imu_notice_init (imu_notice_tracker_t * p_tracker);
void imu_notice_track (imu_notice_tracker_t * p_tracker,
                       const imu_event_t *    p_event);
bool imu_notice_peek (const imu_notice_tracker_t * p_tracker,
                      imu_notice_t *               p_notice);
void imu_notice_sent (imu_notice_tracker_t * p_tracker);
void imu_notice_failed (imu_notice_tracker_t * p_tracker);

#endif

/*** end of file ***/

