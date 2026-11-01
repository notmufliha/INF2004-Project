/** @file imu_analysis.h
 * @brief Buddy 4 imu analysis interface.
 */
#ifndef IMU_ANALYSIS_H
#define IMU_ANALYSIS_H

#include "imu.h"

/* Provisional thresholds: tune using real robot measurements. */
typedef struct
{
    float32_t slope_enter_deg;
    float32_t level_exit_deg;
    float32_t tilt_deg;
    float32_t acceleration_ms2;
    float32_t turn_rate_dps;
    float32_t impact_ms2;
    uint32_t  confirmation_samples;
    uint32_t  max_gap_ms;
    uint32_t  hump_timeout_ms;
} imu_analysis_config_t;

typedef enum
{
    IMU_TERRAIN_LEVEL = 0,
    IMU_TERRAIN_CLIMBING,
    IMU_TERRAIN_PEAK,
    IMU_TERRAIN_DESCENDING
} imu_analysis_terrain_t;

/* One context per analysis task; no shared global analysis state. */
typedef struct
{
    imu_analysis_config_t  config;
    imu_analysis_terrain_t terrain;
    uint32_t               confirmation_count;
    uint32_t               last_timestamp_ms;
    uint32_t               hump_start_ms;
    bool                   b_has_timestamp;
} imu_analysis_t;

void imu_analysis_default_config (imu_analysis_config_t * p_config);
bool imu_analysis_init (imu_analysis_t *              p_analysis,
                        const imu_analysis_config_t * p_config);
/* Returns a valid snapshot for each accepted reading, not just transitions.
 * Inputs use robot axes (X forward, Y sideways, Z up), m/s^2 and degrees.
 * Acceleration includes gravity. Pitch is positive when the nose rises.
 * Low acceleration alone cannot distinguish rest from constant-speed travel.
 * NULL, invalid/nonfinite data or bad timing reset the terrain sequence.
 * Height fields remain zero with height_valid=false; zero is not a measurement.
 */
bool imu_analysis_process (imu_analysis_t *   p_analysis,
                           const imu_data_t * p_data, imu_event_t * p_event);

#endif

/*** end of file ***/

