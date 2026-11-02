/** @file task_imu.h
 * @brief Buddy 4 task imu interface.
 */
#ifndef TASK_IMU_H
#define TASK_IMU_H

#include "../app.h"

void task_imu_sampling_task (INT stacd, void * p_exinf);
void task_imu_analysis_task (INT stacd, void * p_exinf);

#endif

/*** end of file ***/

