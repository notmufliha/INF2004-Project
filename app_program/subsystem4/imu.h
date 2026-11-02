/** @file imu.h
 * @brief Buddy 4 imu types interface.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_numeric.h"
#include "imu_motion.h"

/* Motion states for Buddy 4. */
typedef enum
{
    IMU_MOTION_STATIONARY = 0,
    IMU_MOTION_ACCELERATING,
    IMU_MOTION_TURNING,
    IMU_MOTION_CLIMBING_HUMP,
    IMU_MOTION_HUMP_PEAK,
    IMU_MOTION_DESCENDING_HUMP,
    IMU_MOTION_SUDDEN_IMPACT,
    IMU_MOTION_UNKNOWN
} imu_motion_state_t;

/* Processed IMU measurements. */
typedef struct
{
    uint32_t timestamp_ms;

    /* Acceleration in m/s^2. */
    float32_t accel_x;
    float32_t accel_y;
    float32_t accel_z;

    /* Estimated tilt angles in degrees. */
    float32_t pitch_deg;
    float32_t roll_deg;

    /* Turn rate in degrees per second. */
    float32_t turn_rate_dps;
    bool      b_valid;
    bool      b_turn_rate_valid;
    float32_t magnetic_x_ut;
    float32_t magnetic_y_ut;
    float32_t magnetic_z_ut;
    bool      b_magnetic_valid;
    float32_t heading_deg;
    bool      b_heading_valid;
    bool      b_heading_calibrated;
    uint32_t  sample_sequence;

    /* Encoder snapshot captured by the sampling task, never fabricated. */
    int32_t      encoder_left;
    int32_t      encoder_right;
    uint32_t     encoder_left_errors;
    uint32_t     encoder_right_errors;
    bool         b_odometry_valid;
    imu_motion_t motion;
} imu_data_t;

/* Motion and terrain analysis results. */
typedef struct
{
    imu_motion_state_t motion_state;
    float32_t          pitch_deg;
    float32_t          roll_deg;
    float32_t          turn_rate_dps;
    bool               b_hump_detected;
    bool b_hump_completed; /* One-reading pulse after a complete traversal. */
    bool b_tilt_detected;
    bool b_valid;
    bool b_height_valid; /* Accepted completed-traversal estimate. */
    bool b_highest_hump_valid;
    bool b_sensor_fault; /* Invalid fault notifications carry valid=false. */
    bool b_sensor_recovered; /* One-notification recovery pulse. */
    bool b_turn_rate_valid;
    float32_t heading_deg;
    bool      b_heading_valid;
    bool      b_heading_calibrated;

    /* Estimated hump height in centimetres. */
    float32_t hump_peak_cm;
    float32_t highest_hump_cm;
    bool      b_sudden_impact;
    uint32_t  timestamp_ms;
} imu_event_t;

#endif /* IMU_H. */

/*** end of file ***/

