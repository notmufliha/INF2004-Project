/** @file imu_analysis.c
 * @brief Buddy 4 imu analysis module.
 */
#include "imu_analysis.h"

#include <math.h>
#include <stddef.h>

#define IMU_GRAVITY_MS2 (9.81f)
#define IMU_DEGREES_TO_RADIANS (0.01745329252f)

static void reset_sequence (imu_analysis_t * p_analysis, uint32_t reason);
static void reset_roll (imu_analysis_t * p_analysis);
static void expire_roll (imu_analysis_t * p_analysis, uint32_t now_ms);
static void update_roll_motion (imu_analysis_t * p_analysis, float32_t roll);
static void track_roll_height (imu_analysis_t * p_analysis,
                               const imu_data_t * p_data, bool b_level);
static void complete_height (imu_analysis_t * p_analysis,
                             imu_event_t * p_event);
static void update_roll (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                         imu_event_t * p_event);
static void analyse_terrain (imu_analysis_t * p_analysis,
                             const imu_data_t * p_data, imu_event_t * p_event);
static bool stopped_context (const imu_data_t * p_data);
static bool forward_travel (imu_analysis_t * p_analysis,
                            const imu_data_t * p_data);
static uint32_t movement_reason (const imu_analysis_t * p_analysis,
                                 const imu_data_t * p_data);

static void classify_motion (const imu_analysis_t * p_analysis,
                             const imu_data_t * p_data, imu_event_t * p_event,
                             float32_t magnitude);

/** @brief Supply provisional motion and terrain thresholds. */
void
imu_analysis_default_config (imu_analysis_config_t * p_config)
{
    if (p_config != NULL)
    {
        *p_config = (imu_analysis_config_t){8.0f, 1.0f, 15.0f, 6.0f,
                                            3U, 100U, 10000U,
                                            4.0f, 2.0f, 0.5f, 110.0f};
    }
}

/** @brief Validate thresholds and reset the terrain context. */
bool
imu_analysis_init (imu_analysis_t *              p_analysis,
                   const imu_analysis_config_t * p_config)
{
    if ((NULL == p_analysis) || (NULL == p_config) ||
        (!isfinite(p_config->tilt_deg)) ||
        (!isfinite(p_config->acceleration_ms2)) ||
        (!isfinite(p_config->turn_rate_dps)) ||
        (!isfinite(p_config->impact_ms2)) ||
        (!isfinite(p_config->roll_enter_deg)) ||
        (!isfinite(p_config->roll_level_deg)) ||
        (!isfinite(p_config->roll_trend_deg)) ||
        (!isfinite(p_config->wheel_track_mm)) ||
        (p_config->wheel_track_mm <= 0.0f) ||
        (p_config->roll_trend_deg <= 0.0f) ||
        (p_config->roll_trend_deg > p_config->roll_enter_deg) ||
        (p_config->roll_level_deg < 0.0f) ||
        (p_config->roll_enter_deg <= p_config->roll_level_deg) ||
        (p_config->roll_enter_deg >= 90.0f) ||
        (p_config->tilt_deg <= 0.0f) || (p_config->acceleration_ms2 <= 0.0f) ||
        (p_config->turn_rate_dps <= 0.0f) ||
        (p_config->impact_ms2 <= p_config->acceleration_ms2) ||
        (0U == p_config->confirmation_samples) ||
        (0U == p_config->max_gap_ms) ||
        (p_config->hump_timeout_ms <= p_config->max_gap_ms) ||
        (p_config->hump_timeout_ms >= UINT32_MAX / 2U))
    {
        return false;
    }

    *p_analysis        = (imu_analysis_t){0};
    p_analysis->config = *p_config;
    p_analysis->b_require_motion = true;
    return true;
}

/** @brief Validate a reading and publish motion and terrain status. */
bool
imu_analysis_process (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                      imu_event_t * p_event)
{
    uint32_t  elapsed;
    float32_t magnitude;

    if (NULL != p_analysis)
    {
        p_analysis->reset_reasons = 0U;
    }

    if (p_event != NULL)
    {
        *p_event              = (imu_event_t){0};
        p_event->motion_state = IMU_MOTION_UNKNOWN;
    }

    if ((NULL == p_analysis) || (NULL == p_event) || (NULL == p_data) ||
        (!p_data->b_valid) || (!isfinite(p_data->accel_x)) ||
        (!isfinite(p_data->accel_y)) || (!isfinite(p_data->accel_z)) ||
        (!isfinite(p_data->pitch_deg)) || (!isfinite(p_data->roll_deg)) ||
        (!isfinite(p_data->turn_rate_dps)))
    {
        if (p_analysis != NULL)
        {
            reset_sequence(p_analysis, IMU_ANALYSIS_RESET_INVALID);
            p_analysis->b_has_timestamp = false;
            p_analysis->b_have_counts = false;
        }

        return false;
    }

    magnitude =
        hypotf(hypotf(p_data->accel_x, p_data->accel_y), p_data->accel_z);

    if (!isfinite(magnitude))
    {
        reset_sequence(p_analysis, IMU_ANALYSIS_RESET_INVALID);
        p_analysis->b_has_timestamp = false;
        return false;
    }

    if (p_analysis->b_has_timestamp)
    {
        elapsed = p_data->timestamp_ms - p_analysis->last_timestamp_ms;

        if ((0U == elapsed) || (elapsed > p_analysis->config.max_gap_ms))
        {
            reset_sequence(p_analysis, IMU_ANALYSIS_RESET_TIMING);
            p_analysis->b_has_timestamp = false;
            p_analysis->b_have_counts = false;
            return false;
        }
    }

    p_analysis->last_timestamp_ms = p_data->timestamp_ms;
    p_analysis->b_has_timestamp   = true;
    p_event->timestamp_ms         = p_data->timestamp_ms;
    p_event->pitch_deg            = p_data->pitch_deg;
    p_event->roll_deg             = p_data->roll_deg;
    p_event->turn_rate_dps        = p_data->turn_rate_dps;
    p_event->b_turn_rate_valid    = p_data->b_turn_rate_valid;
    p_event->heading_deg          = p_data->heading_deg;
    p_event->b_heading_valid      = p_data->b_heading_valid;
    p_event->b_heading_calibrated = p_data->b_heading_calibrated;
    p_event->b_valid              = true;
    p_event->highest_hump_cm = p_analysis->highest_hump_cm;
    p_event->b_highest_hump_valid = p_analysis->b_highest_hump_valid;
    p_event->b_tilt_detected =
        (fabsf(p_data->pitch_deg) >= p_analysis->config.tilt_deg) ||
        (fabsf(p_data->roll_deg) >= p_analysis->config.tilt_deg);
    /* Compare magnitude with gravity: a stationary tilted sensor is not impact.
     * This is a provisional detector, not gravity-compensated acceleration.
     */
    p_event->b_sudden_impact =
        fabsf(magnitude - IMU_GRAVITY_MS2) >= p_analysis->config.impact_ms2;

    if (p_event->b_sudden_impact)
    {
        reset_sequence(p_analysis, IMU_ANALYSIS_RESET_IMPACT);
        p_event->motion_state = IMU_MOTION_SUDDEN_IMPACT;
        return true;
    }

    analyse_terrain(p_analysis, p_data, p_event);
    p_event->highest_hump_cm = p_analysis->highest_hump_cm;
    p_event->b_highest_hump_valid = p_analysis->b_highest_hump_valid;
    classify_motion(p_analysis, p_data, p_event, magnitude);
    return true;
}

/** @brief Advance on travel, hold briefly, and reject bad movement context. */
static bool
forward_travel (imu_analysis_t * p_analysis, const imu_data_t * p_data)
{
    const uint32_t reason = movement_reason(p_analysis, p_data);

    if (0U != reason)
    {
        reset_sequence(p_analysis, reason);
        p_analysis->b_have_counts = false;
        p_analysis->b_have_progress = false;
        return false;
    }

    if (!p_analysis->b_have_counts)
    {
        p_analysis->left_count = p_data->encoder_left;
        p_analysis->right_count = p_data->encoder_right;
        p_analysis->last_progress_ms = p_data->timestamp_ms;
        p_analysis->b_have_counts = true;
        p_analysis->b_have_progress = false;
        return false;
    }

    const bool b_travel =
        ((p_data->encoder_left > p_analysis->left_count) ||
         (p_data->encoder_right > p_analysis->right_count));

    p_analysis->left_count = p_data->encoder_left;
    p_analysis->right_count = p_data->encoder_right;
    if (b_travel)
    {
        p_analysis->last_progress_ms = p_data->timestamp_ms;
        p_analysis->b_have_progress = true;
    }
    else if (p_data->timestamp_ms - p_analysis->last_progress_ms >=
             IMU_ANALYSIS_NO_PROGRESS_MS)
    {
        reset_sequence(p_analysis, IMU_ANALYSIS_RESET_NO_PROGRESS);
        p_analysis->b_have_progress = false;
    }
    else
    {
        /* Keep the candidate while fresh forward feedback awaits a pulse. */
    }

    return (b_travel) && (p_analysis->b_have_progress);
}

/** @brief Name the first failed B4 interpretation of paired B2 feedback. */
static uint32_t
movement_reason (const imu_analysis_t * p_analysis,
                 const imu_data_t * p_data)
{
    if (!p_data->b_odometry_valid)
    {
        return IMU_ANALYSIS_RESET_ODOMETRY;
    }
    if (!p_data->motion.b_valid)
    {
        return IMU_ANALYSIS_RESET_MOTION;
    }
    if (p_data->timestamp_ms - p_data->motion.timestamp_ms >
        IMU_MOTION_MAX_AGE_MS)
    {
        return IMU_ANALYSIS_RESET_STALE;
    }
    if (p_data->motion.b_fault)
    {
        return IMU_ANALYSIS_RESET_B2_FAULT;
    }
    if ((p_data->motion.b_reverse) ||
        ((p_analysis->b_have_counts) &&
         ((p_data->encoder_left < p_analysis->left_count) ||
          (p_data->encoder_right < p_analysis->right_count))))
    {
        return IMU_ANALYSIS_RESET_REVERSE;
    }
    if (p_data->motion.b_turning)
    {
        return IMU_ANALYSIS_RESET_TURNING;
    }
    if (p_data->motion.b_stopped)
    {
        return IMU_ANALYSIS_RESET_STOPPED;
    }
    if (!p_data->motion.b_forward)
    {
        return IMU_ANALYSIS_RESET_NOT_FORWARD;
    }
    return 0U;
}

/** @brief Discard the incomplete roll crossing after a failed check. */
static void
reset_sequence (imu_analysis_t * p_analysis, uint32_t reason)
{
    p_analysis->reset_reasons |= reason;
    reset_roll(p_analysis);
}

/** @brief Reset roll recognition until a new stable baseline is observed. */
static void
reset_roll (imu_analysis_t * p_analysis)
{
    p_analysis->roll_state = IMU_ROLL_UNARMED;
    p_analysis->roll_count = 0U;
    p_analysis->roll_start_ms = 0U;
    p_analysis->b_roll_positive = false;
    p_analysis->roll_terrain = IMU_TERRAIN_LEVEL;
    p_analysis->roll_anchor_deg = 0.0f;
    p_analysis->roll_direction = 0;
    p_analysis->roll_trend_count = 0U;
    p_analysis->roll_stable_count = 0U;
    p_analysis->roll_baseline_deg = 0.0f;
    p_analysis->roll_peak_deg = 0.0f;
}

/** @brief Bound an active roll episode even while encoder progress pauses. */
static void
expire_roll (imu_analysis_t * p_analysis, uint32_t now_ms)
{
    if ((IMU_ROLL_ACTIVE == p_analysis->roll_state) &&
        (now_ms - p_analysis->roll_start_ms >=
         p_analysis->config.hump_timeout_ms))
    {
        p_analysis->reset_reasons |= IMU_ANALYSIS_RESET_TIMEOUT;
        reset_roll(p_analysis);
    }
}

/** @brief Confirm trends in absolute roll with a noise deadband. */
static void
update_roll_motion (imu_analysis_t * p_analysis, float32_t roll)
{
    const float32_t delta = roll - p_analysis->roll_anchor_deg;

    if (fabsf(delta) >= p_analysis->config.roll_trend_deg)
    {
        const int8_t direction = delta > 0.0f ? 1 : -1;
        p_analysis->roll_trend_count =
            direction == p_analysis->roll_direction
                ? p_analysis->roll_trend_count + 1U : 1U;
        p_analysis->roll_direction = direction;
        p_analysis->roll_anchor_deg = roll;
        p_analysis->roll_stable_count = 0U;
        if (p_analysis->roll_trend_count >= IMU_ROLL_CONFIRM_SAMPLES)
        {
            p_analysis->roll_terrain = direction > 0
                ? IMU_TERRAIN_CLIMBING : IMU_TERRAIN_DESCENDING;
            p_analysis->roll_trend_count = IMU_ROLL_CONFIRM_SAMPLES;
        }
    }
    else
    {
        if (p_analysis->roll_stable_count <
            p_analysis->config.confirmation_samples)
        {
            ++p_analysis->roll_stable_count;
        }
        if (p_analysis->roll_stable_count >=
            p_analysis->config.confirmation_samples)
        {
            p_analysis->roll_terrain =
                IMU_TERRAIN_CLIMBING == p_analysis->roll_terrain
                    ? IMU_TERRAIN_PEAK : p_analysis->roll_terrain;
            p_analysis->roll_direction = 0;
            p_analysis->roll_trend_count = 0U;
        }
    }

    /* A short return can step directly into the level band. Two progressing
     * level samples confirm the drop even without two separate angle steps.
     */
    if ((roll <= p_analysis->config.roll_level_deg) &&
        (p_analysis->roll_count >= IMU_ROLL_CONFIRM_SAMPLES))
    {
        p_analysis->roll_terrain = IMU_TERRAIN_DESCENDING;
    }
}

/** @brief Track baseline and excursion independently of detector decisions. */
static void
track_roll_height (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                   bool b_level)
{
    if (IMU_ROLL_UNARMED == p_analysis->roll_state)
    {
        if (b_level)
        {
            p_analysis->roll_baseline_deg +=
                (p_data->roll_deg - p_analysis->roll_baseline_deg) /
                (float32_t)(p_analysis->roll_count + 1U);
        }
        else
        {
            p_analysis->roll_baseline_deg = 0.0f;
        }
    }
    else
    {
        const float32_t excursion = fabsf(p_data->roll_deg -
                                          p_analysis->roll_baseline_deg);
        if ((IMU_ROLL_READY == p_analysis->roll_state) &&
            ((0U == p_analysis->roll_count) ||
             ((p_data->roll_deg >= 0.0f) != p_analysis->b_roll_positive)))
        {
            p_analysis->roll_peak_deg = excursion;
        }
        else
        {
            p_analysis->roll_peak_deg =
                fmaxf(p_analysis->roll_peak_deg, excursion);
        }
    }
}

/** @brief Estimate wheel height difference after a one-wheel crossing. */
static void
complete_height (imu_analysis_t * p_analysis, imu_event_t * p_event)
{
    if (p_analysis->roll_peak_deg <= 90.0f)
    {
        p_event->hump_peak_cm =
            (p_analysis->config.wheel_track_mm / 10.0f) *
            sinf(p_analysis->roll_peak_deg * IMU_DEGREES_TO_RADIANS);
        p_event->b_height_valid = true;
        p_analysis->highest_hump_cm =
            fmaxf(p_analysis->highest_hump_cm, p_event->hump_peak_cm);
        p_analysis->b_highest_hump_valid = true;
    }
}

/** @brief Confirm either-side roll excursions with a stable return to level. */
static void
update_roll (imu_analysis_t * p_analysis, const imu_data_t * p_data,
             imu_event_t * p_event)
{
    const float32_t roll = fabsf(p_data->roll_deg);
    const bool b_level = roll <= p_analysis->config.roll_level_deg;

    if (!p_analysis->b_require_motion)
    {
        reset_roll(p_analysis);
        return;
    }

    track_roll_height(p_analysis, p_data, b_level);
    switch (p_analysis->roll_state)
    {
        case IMU_ROLL_UNARMED:
            p_analysis->roll_count = b_level
                ? p_analysis->roll_count + 1U : 0U;
            if (p_analysis->roll_count >=
                p_analysis->config.confirmation_samples)
            {
                p_analysis->roll_state = IMU_ROLL_READY;
                p_analysis->roll_count = 0U;
            }
        break;
        case IMU_ROLL_READY:
            if (roll >= p_analysis->config.roll_enter_deg)
            {
                const bool b_positive = p_data->roll_deg >= 0.0f;
                p_analysis->roll_count =
                    b_positive == p_analysis->b_roll_positive
                        ? p_analysis->roll_count + 1U : 1U;
                p_analysis->b_roll_positive = b_positive;
            }
            else
            {
                p_analysis->roll_count = 0U;
            }
            if (p_analysis->roll_count >= IMU_ROLL_CONFIRM_SAMPLES)
            {
                p_analysis->roll_state = IMU_ROLL_ACTIVE;
                p_analysis->roll_start_ms = p_data->timestamp_ms;
                p_analysis->roll_count = 0U;
                p_analysis->roll_terrain = IMU_TERRAIN_CLIMBING;
                p_analysis->roll_anchor_deg = roll;
                p_analysis->roll_direction = 0;
                p_analysis->roll_trend_count = 0U;
                p_analysis->roll_stable_count = 0U;
            }
        break;
        case IMU_ROLL_ACTIVE:
            if ((roll >= p_analysis->config.roll_enter_deg) &&
                ((p_data->roll_deg >= 0.0f) != p_analysis->b_roll_positive))
            {
                /* Large opposite-side motion before level is not one hump. */
                reset_roll(p_analysis);
                return;
            }
            p_analysis->roll_count = b_level
                ? p_analysis->roll_count + 1U : 0U;
            update_roll_motion(p_analysis, roll);
            if (p_analysis->roll_count >=
                p_analysis->config.confirmation_samples)
            {
                p_analysis->roll_state = IMU_ROLL_READY;
                p_analysis->roll_count = 0U;
                p_event->b_hump_completed = true;
                complete_height(p_analysis, p_event);
                p_analysis->roll_terrain = IMU_TERRAIN_LEVEL;
            }
        break;
        default:
            reset_roll(p_analysis);
        break;
    }
}

/** @brief Advance only the roll crossing when forward travel is usable. */
static void
analyse_terrain (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                 imu_event_t * p_event)
{
    const bool b_advance = (p_analysis->b_require_motion) &&
                           (forward_travel(p_analysis, p_data));
    expire_roll(p_analysis, p_data->timestamp_ms);
    if (b_advance)
    {
        update_roll(p_analysis, p_data, p_event);
    }
    p_event->b_hump_detected =
        (IMU_ROLL_ACTIVE == p_analysis->roll_state) ||
        (p_event->b_hump_completed);
}

/** @brief Choose the motion state from terrain and dynamic thresholds. */
static void
classify_motion (const imu_analysis_t * p_analysis, const imu_data_t * p_data,
                 imu_event_t * p_event, float32_t magnitude)
{
    switch (p_analysis->roll_terrain)
    {
        case IMU_TERRAIN_CLIMBING:
            p_event->motion_state = IMU_MOTION_CLIMBING_HUMP;
        break;
        case IMU_TERRAIN_PEAK:
            p_event->motion_state = IMU_MOTION_HUMP_PEAK;
        break;
        case IMU_TERRAIN_DESCENDING:
            p_event->motion_state = IMU_MOTION_DESCENDING_HUMP;
        break;
        default:

            if ((p_data->b_turn_rate_valid) &&
                (fabsf(p_data->turn_rate_dps) >=
                 p_analysis->config.turn_rate_dps))
            {
                p_event->motion_state = IMU_MOTION_TURNING;
            }
            else if ((fabsf(magnitude - IMU_GRAVITY_MS2) >=
                      p_analysis->config.acceleration_ms2) ||
                     (((!p_event->b_tilt_detected) &&
                       (hypotf(p_data->accel_x, p_data->accel_y) >=
                        p_analysis->config.acceleration_ms2))))
            {
                p_event->motion_state = IMU_MOTION_ACCELERATING;
            }
            else
            {
                p_event->motion_state = stopped_context(p_data)
                                            ? IMU_MOTION_STATIONARY
                                            : IMU_MOTION_UNKNOWN;
            }
        break;
    }
}

/** @brief Confirm rest only from fresh, nonfaulted B2 physical-stop feedback.
 */
static bool
stopped_context (const imu_data_t * p_data)
{
    return (p_data->motion.b_valid) && (p_data->motion.b_stopped) &&
           (!p_data->motion.b_fault) && (!p_data->motion.b_turning) &&
           (!p_data->motion.b_reverse) &&
           ((p_data->timestamp_ms - p_data->motion.timestamp_ms) <=
            IMU_MOTION_MAX_AGE_MS);
}

/*** end of file ***/

