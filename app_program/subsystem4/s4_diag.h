/** @file s4_diag.h
 * @brief Fixed-size, synchronized diagnostics for one B4 movement attempt.
 */
#ifndef S4_DIAG_H
#define S4_DIAG_H

#include "../app.h"
#include "imu_analysis.h"

#define S4_DIAG_RESET_COUNT 13U

typedef struct
{
    uint32_t  frames;
    uint32_t  raw_samples;
    uint32_t  filtered_samples;
    uint32_t  completions;
    uint32_t  accepted_heights;
    uint32_t  confirmation_samples;
    uint32_t  analysis_resets[S4_DIAG_RESET_COUNT];
    uint32_t  odometry_valid;
    uint32_t  odometry_invalid;
    uint32_t  odometry_reasons[IMU_ODOM_REASON_COUNT];
    uint32_t  odometry_age_samples;
    uint32_t  odometry_encoder_samples;
    uint32_t  odometry_context_samples;
    uint32_t  odometry_before_samples;
    uint32_t  max_odometry_age_ms;
    uint32_t  max_odometry_encoder_ms;
    uint32_t  max_odometry_context_ms;
    uint32_t  max_odometry_before_ms;
    uint32_t  acceleration_read_samples;
    uint32_t  magnetic_read_samples;
    uint32_t  max_acceleration_read_ms;
    uint32_t  max_magnetic_read_ms;
    uint32_t  magnetic_deferred_frames;
    float32_t raw_min_deg;
    float32_t raw_max_deg;
    float32_t filtered_min_deg;
    float32_t filtered_max_deg;
    float32_t raw_roll_deg;
    float32_t filtered_roll_deg;
    uint32_t  roll_candidates;
    float32_t roll_enter_deg;
    float32_t roll_level_deg;
    uint32_t  roll_stages;
} s4_diag_report_t;

/* One test-task controller; analysis is the only record producer.
 * Begin lazily creates a FastLock before publishing collection readiness.
 * No lock is held while printing. End copies and freezes the report.
 */
ER   s4_diag_begin (void);
void s4_diag_record (const imu_data_t * p_raw, const imu_data_t * p_filtered,
                     const imu_analysis_t * p_analysis,
                     const imu_event_t *    p_event);
ER   s4_diag_end (s4_diag_report_t * p_report);
void s4_diag_print (const s4_diag_report_t * p_report);

#endif

/*** end of file ***/

