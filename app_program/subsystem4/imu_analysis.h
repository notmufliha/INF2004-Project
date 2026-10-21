#ifndef IMU_ANALYSIS_H
#define IMU_ANALYSIS_H

#include "imu_types.h"

/* Provisional thresholds: tune using real robot measurements. */
typedef struct
{
    float slope_enter_deg;
    float level_exit_deg;
    float tilt_deg;
    float acceleration_ms2;
    float turn_rate_dps;
    float impact_ms2;
    uint32_t confirmation_samples;
    uint32_t max_gap_ms;
    uint32_t hump_timeout_ms;
} imu_analysis_config_t;

typedef enum
{
    IMU_TERRAIN_LEVEL = 0,
    IMU_TERRAIN_CLIMBING,
    IMU_TERRAIN_PEAK,
    IMU_TERRAIN_DESCENDING
} imu_terrain_state_t;

/* One context per analysis task; no shared global analysis state. */
typedef struct
{
    imu_analysis_config_t config;
    imu_terrain_state_t terrain;
    uint32_t confirmation_count;
    uint32_t last_timestamp_ms;
    uint32_t hump_start_ms;
    bool has_timestamp;
} imu_analysis_t;

void imu_analysis_default_config(imu_analysis_config_t *config);
bool imu_analysis_init(imu_analysis_t *analysis,
                       const imu_analysis_config_t *config);
/* Returns a valid snapshot for each accepted reading, not just transitions.
 * Inputs use robot axes (X forward, Y sideways, Z up), m/s^2 and degrees.
 * Acceleration includes gravity. Pitch is positive when the nose rises.
 * Low acceleration alone cannot distinguish rest from constant-speed travel.
 * NULL, invalid/nonfinite data or bad timing reset the terrain sequence.
 * Height fields remain zero with height_valid=false; zero is not a measurement.
 */
bool imu_analysis_process(imu_analysis_t *analysis, const imu_data_t *data,
                          imu_event_t *event);

#endif
