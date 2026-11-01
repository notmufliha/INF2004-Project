/** @file imu_height.h
 * @brief Estimate completed hump height from synchronized pitch and wheel
 * travel.
 */
#ifndef IMU_HEIGHT_H
#define IMU_HEIGHT_H

#include "imu.h"

typedef struct
{
    float32_t left_cm_per_count;
    float32_t right_cm_per_count;
    float32_t max_speed_cm_s;
    float32_t max_roll_deg;
    float32_t max_pitch_deg;
    float32_t level_deg;
    float32_t closure_cm;
    uint32_t  max_gap_ms;
} imu_height_config_t;

typedef struct
{
    imu_height_config_t config;
    int32_t             left_count;
    int32_t             right_count;
    uint32_t            left_errors;
    uint32_t            right_errors;
    uint32_t            timestamp_ms;
    uint32_t            sample_sequence;
    float32_t           pitch_deg;
    float32_t           rise_cm;
    float32_t           peak_cm;
    float32_t           left_cm;
    float32_t           right_cm;
    float32_t           highest_cm;
    bool                b_seeded;
    bool                b_active;
    bool                b_confirmed;
    bool                b_descended;
    bool                b_blocked;
    bool                b_highest_valid;
} imu_height_t;

/* Geometry defaults to zero: callers must supply their encoder calibration. */
void imu_height_default_config (imu_height_config_t * p_config);
bool imu_height_init (imu_height_t *              p_state,
                      const imu_height_config_t * p_config);
/* Raw acquisition pitch avoids filter lag relative to wheel counts. A valid
 * result needs a confirmed full traversal, forward near-straight travel and
 * return close to the original level. Highest persists until reinitialization.
 * Valid means consistency checks passed, not hardware-validated accuracy.
 */
void imu_height_process (imu_height_t * p_state, const imu_data_t * p_data,
                         imu_event_t * p_event);

#endif

/*** end of file ***/

