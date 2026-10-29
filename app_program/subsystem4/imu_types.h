#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Motion states for Buddy 4 */
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

/* Processed IMU measurements */
typedef struct
{
    uint32_t timestamp_ms;

    /* Acceleration in m/s^2 */
    float accel_x;
    float accel_y;
    float accel_z;

    /* Estimated tilt angles in degrees */
    float pitch_deg;
    float roll_deg;

    /* Turn rate in degrees per second */
    float turn_rate_dps;

    bool valid;
    bool turn_rate_valid;
    float magnetic_x_ut;
    float magnetic_y_ut;
    float magnetic_z_ut;
    bool magnetic_valid;
    float heading_deg;
    bool heading_valid;
    bool heading_calibrated;
} imu_data_t;

/* Motion and terrain analysis results */
typedef struct
{
    imu_motion_state_t motion_state;

    float pitch_deg;
    float roll_deg;
    float turn_rate_dps;

    bool hump_detected;
    bool hump_completed; /* One-reading pulse after a complete traversal. */
    bool tilt_detected;
    bool valid;
    bool height_valid; /* False until a height estimator is implemented. */
    bool sensor_fault; /* Invalid fault notifications carry valid=false. */
    bool sensor_recovered; /* One-notification recovery pulse. */
    bool turn_rate_valid;
    float heading_deg;
    bool heading_valid;
    bool heading_calibrated;

    /* Estimated hump height in centimetres */
    float hump_peak_cm;
    float highest_hump_cm;

    bool sudden_impact;

    uint32_t timestamp_ms;
} imu_event_t;

#endif /* IMU_TYPES_H */
