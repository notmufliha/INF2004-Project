/** @file imu_live.h
 * @brief Latest raw IMU sample for the lift-stop check (Vehicle Controller).
 */
#ifndef IMU_LIVE_H
#define IMU_LIVE_H
#include "imu.h"

typedef struct
{
    float32_t accel_x;  /* m/s^2 */
    float32_t accel_y;
    float32_t accel_z;
    float32_t pitch_deg;
    uint32_t  timestamp_ms;
    bool      b_valid;
} imu_live_t;

/* Sampling task: store one valid reading. */
void imu_live_store (const imu_data_t * p_data);
/* Any task: copy the latest reading (b_valid false before the first). */
void imu_live_get (imu_live_t * p_out);
#endif
/*** end of file ***/
