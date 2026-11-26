/** @file imu_analysis.h
 * @brief Buddy 4 imu analysis interface.
 */
#ifndef IMU_ANALYSIS_H
#define IMU_ANALYSIS_H

#include "imu.h"

/* Provisional thresholds: tune using real robot measurements. */
typedef struct
{
    float32_t tilt_deg;
    float32_t acceleration_ms2;
    float32_t turn_rate_dps;
    float32_t impact_ms2;
    uint32_t  confirmation_samples;
    uint32_t  max_gap_ms;
    uint32_t  hump_timeout_ms;
    float32_t roll_enter_deg; /* Provisional one-wheel excursion threshold. */
    float32_t roll_level_deg; /* Hysteresis for baseline and return to level. */
    float32_t roll_trend_deg; /* Meaningful absolute-roll change, degrees. */
    float32_t wheel_track_mm; /* Tyre centre spacing; one-wheel model. */
    /* Flat-ground rejection.  Accelerometer roll also sees sideways
     * acceleration: steering jerks give short spikes, curves a steady
     * offset.  A hump must hold roll_enter_deg for roll_enter_ms (0 = the
     * two-sample rule only), and cannot start while the wheel speeds differ
     * by more than roll_curve_ratio of the faster wheel (0 = no check). */
    uint32_t  roll_enter_ms;
    float32_t roll_curve_ratio;
    /* Height accuracy.  The peak used for the height is the trimmed mean of
     * the last roll_peak_samples unfiltered accelerometer rolls (the top and
     * bottom third dropped), so the jolt as a wheel hits the hump edge does
     * not count as roll.  1 = single-sample peak.  1..IMU_ROLL_PEAK_MAX. */
    uint32_t roll_peak_samples;
    /* Jerk rejection.  The GY-511 has no gyroscope, so a level car pushed
     * sideways reads as tilted: a steering jerk (wheel speed difference
     * changing) or a speed jerk (mean wheel speed changing) can look like a
     * hump on flat ground.  Samples where, over the last IMU_JERK_WINDOW
     * readings, the difference changed by more than roll_steer_jerk_cps or
     * the mean by more than roll_speed_jerk_cps are skipped: they neither
     * count toward a hump nor enter the height estimate.  0 = no check. */
    uint32_t roll_steer_jerk_cps;
    uint32_t roll_speed_jerk_cps;
} imu_analysis_config_t;

#define IMU_ROLL_PEAK_MAX 9U
#define IMU_JERK_WINDOW 8U

typedef enum
{
    IMU_TERRAIN_LEVEL = 0,
    IMU_TERRAIN_CLIMBING,
    IMU_TERRAIN_PEAK,
    IMU_TERRAIN_DESCENDING
} imu_analysis_terrain_t;

typedef enum
{
    IMU_ROLL_UNARMED = 0,
    IMU_ROLL_READY,
    IMU_ROLL_ACTIVE
} imu_analysis_roll_t;

#define IMU_ANALYSIS_RESET_INVALID 0x01U
#define IMU_ANALYSIS_RESET_TIMING 0x02U
#define IMU_ANALYSIS_RESET_IMPACT 0x04U
#define IMU_ANALYSIS_RESET_TIMEOUT 0x08U
#define IMU_ANALYSIS_RESET_MOTION 0x10U
#define IMU_ANALYSIS_RESET_ODOMETRY 0x20U
#define IMU_ANALYSIS_RESET_STALE 0x40U
#define IMU_ANALYSIS_RESET_B2_FAULT 0x80U
#define IMU_ANALYSIS_RESET_REVERSE 0x100U
#define IMU_ANALYSIS_RESET_TURNING 0x200U
#define IMU_ANALYSIS_RESET_STOPPED 0x400U
#define IMU_ANALYSIS_RESET_NOT_FORWARD 0x800U
#define IMU_ANALYSIS_RESET_NO_PROGRESS 0x1000U
#define IMU_ANALYSIS_NO_PROGRESS_MS 200U
#define IMU_ROLL_CONFIRM_SAMPLES 2U

/* One context per analysis task; no shared global analysis state. */
typedef struct
{
    imu_analysis_config_t  config;
    uint32_t               last_timestamp_ms;
    uint32_t               reset_reasons; /* Current process call only. */
    int32_t                left_count;
    int32_t                right_count;
    uint32_t               last_progress_ms;
    uint32_t               roll_count;
    uint32_t               roll_start_ms;
    float32_t              roll_anchor_deg;
    uint32_t               roll_trend_count;
    uint32_t               roll_stable_count;
    float32_t              roll_baseline_deg;
    float32_t              roll_peak_deg;
    float32_t              roll_window[IMU_ROLL_PEAK_MAX];
    uint32_t               roll_window_count;
    uint32_t               roll_window_next;
    float32_t              highest_hump_cm;
    int32_t                jerk_left[IMU_JERK_WINDOW];  /* counts/s */
    int32_t                jerk_right[IMU_JERK_WINDOW]; /* counts/s */
    uint32_t               jerk_next;
    uint32_t               jerk_count;
    imu_analysis_roll_t    roll_state : 2;
    imu_analysis_terrain_t roll_terrain : 2;
    int8_t                 roll_direction; /* -1, 0, +1 */
    bool                   b_has_timestamp : 1;
    bool                   b_require_motion : 1;
    bool                   b_have_counts : 1;
    bool                   b_have_progress : 1;
    bool                   b_roll_positive : 1;
    bool                   b_highest_hump_valid : 1;
} imu_analysis_t;

void imu_analysis_default_config (imu_analysis_config_t * p_config);
bool imu_analysis_init (
    imu_analysis_t * p_analysis, const imu_analysis_config_t * p_config);
/* Returns a valid snapshot for each accepted reading, not just transitions.
 * Inputs use robot axes (X forward, Y sideways, Z up), m/s^2 and degrees.
 * Acceleration includes gravity. Pitch is positive when the nose rises.
 * Low acceleration alone cannot distinguish rest from constant-speed travel.
 * NULL, invalid/nonfinite data or bad timing reset the terrain sequence.
 * Hump and climb/peak/descent events use roll only; pitch remains measured.
 * Forward movement, a level baseline, two same-side excursion readings and
 * confirmation_samples returning level are required. Completed height uses
 * wheel_track_mm * sin(peak roll relative to the level baseline). This assumes
 * one wheel stays on level ground and both tyres maintain contact.
 * Brief no-pulse intervals hold the state without advancing confirmation;
 * 200ms without progress and invalid movement context cancel the episode.
 */
bool imu_analysis_process (imu_analysis_t * p_analysis,
    const imu_data_t *                      p_data,
    imu_event_t *                           p_event);

#endif

/*** end of file ***/

