#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include "imu_types.h"

typedef struct
{
    float smoothing_alpha; /* (0,1], 1 disables smoothing. */
    uint32_t failure_samples;
    uint32_t recovery_samples;
    uint32_t max_gap_ms;
} imu_processing_config_t;

typedef struct
{
    bool fault_active;
    bool fault_started; /* One-call transition pulses. */
    bool recovered;
} imu_health_t;

typedef struct
{
    imu_processing_config_t config;
    imu_data_t previous;
    uint32_t last_good_ms;
    uint32_t failures;
    uint32_t recovery;
    bool have_good_time;
    bool seeded;
    bool fault_active;
} imu_processing_t;

void imu_processing_default_config(imu_processing_config_t *config);
bool imu_processing_init(imu_processing_t *state,
                         const imu_processing_config_t *config);
/* Feed each acquisition attempt; NULL represents a missing reading.
 * now_ms uses the same monotonic clock as reading timestamps. On failure the
 * output is invalid and contains no stale measurements. Acceleration passes
 * through unchanged to retain impact spikes. Angles/rate are smoothed.
 * Faults suppress output until enough consecutive valid recovery readings.
 */
bool imu_processing_process(imu_processing_t *state, const imu_data_t *input,
                            uint32_t now_ms, imu_data_t *output,
                            imu_health_t *health);

#endif
