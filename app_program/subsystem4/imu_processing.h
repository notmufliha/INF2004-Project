/** @file imu_processing.h
 * @brief Buddy 4 imu processing interface.
 */
#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include "imu.h"

typedef struct
{
    float32_t smoothing_alpha; /* Weight per 10 ms; 1 disables smoothing. */
    uint32_t  failure_samples;
    uint32_t  recovery_samples;
    uint32_t  max_gap_ms;
} imu_processing_config_t;

typedef struct
{
    bool b_fault_active;
    bool b_fault_started; /* One-call transition pulses. */
    bool b_recovered;
} imu_processing_health_t;

typedef struct
{
    imu_processing_config_t config;
    imu_data_t              previous;
    uint32_t                last_good_ms;
    uint32_t                failures;
    uint32_t                recovery;
    bool                    b_have_good_time;
    bool                    b_seeded;
    bool                    b_fault_active;
} imu_processing_t;

void imu_processing_default_config (imu_processing_config_t * p_config);
bool imu_processing_init (imu_processing_t *              p_state,
                          const imu_processing_config_t * p_config);
/* Feed each acquisition attempt; NULL represents a missing reading.
 * now_ms uses the same monotonic clock as reading timestamps. On failure the
 * output is invalid and contains no stale measurements. Acceleration passes
 * through unchanged to retain impact spikes. Angles/rate are smoothed.
 * Faults suppress output until enough consecutive valid recovery readings.
 */
bool imu_processing_process (imu_processing_t * p_state,
                             const imu_data_t * p_input, uint32_t now_ms,
                             imu_data_t *              p_output,
                             imu_processing_health_t * p_health);

#endif

/*** end of file ***/

