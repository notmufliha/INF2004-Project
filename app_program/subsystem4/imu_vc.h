/** @file imu_vc.h
 * @brief B4 observations for one Vehicle Controller consumer.
 */
#ifndef IMU_VC_H
#define IMU_VC_H

#include "../app.h"
#include "imu_notice.h"

#define IMU_VC_QUEUE_DEPTH 8U

ER imu_vc_init (void);
ER imu_vc_publish (const imu_event_t * p_event);
ER imu_vc_receive (imu_notice_t * p_notice, TMO timeout_ms);

#endif

/*** end of file ***/

