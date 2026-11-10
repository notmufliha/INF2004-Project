/** @file imu_odom.h
 * @brief Adapt B2's real encoder counts and timing to B4 readings.
 */
#ifndef IMU_ODOM_H
#define IMU_ODOM_H

#include "imu.h"

void imu_odom_capture (imu_data_t * p_data);
/* Called after a successful acceleration burst, before decoding validity is
 * known. Bound the burst-plus-encoder interval and retain burst-end timestamp.
 */
void imu_odom_pair (imu_data_t * p_data, uint32_t begin_ms, uint32_t end_ms,
                    bool b_time_valid);

#endif

/*** end of file ***/

