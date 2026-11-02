/** @file imu_motion.h
 * @brief B4-owned interpretation of the existing B2 motion snapshot API.
 */
#ifndef IMU_MOTION_H
#define IMU_MOTION_H

#include "imu_numeric.h"

#define IMU_MOTION_MAX_AGE_MS 100U

typedef struct
{
    uint32_t  timestamp_ms;
    uint32_t  command_id;
    int32_t   left_speed_cps;
    int32_t   right_speed_cps;
    float32_t left_travel_cm;
    float32_t right_travel_cm;
    float32_t total_travel_cm;
    bool      b_valid;
    bool      b_running;
    bool      b_forward;
    bool      b_reverse;
    bool      b_turning;
    bool      b_stopped;
    bool      b_fault;
} imu_motion_t;

/* Capture only, never submit a command or raise/clear a B2 fault. Failure
 * clears the whole output so callers cannot reuse an older valid snapshot.
 */
bool imu_motion_capture (imu_motion_t * p_motion);
#endif

/*** end of file ***/

