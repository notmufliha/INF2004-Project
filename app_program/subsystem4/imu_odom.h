/** @file imu_odom.h
 * @brief Adapt B2's real encoder calibration and counts to B4 readings.
 */
#ifndef IMU_ODOM_H
#define IMU_ODOM_H

#include "imu_height.h"

bool imu_odom_init (imu_height_t * p_height);
void imu_odom_capture (imu_data_t * p_data);

#endif

/*** end of file ***/

