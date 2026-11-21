/** @file imu_analysis.c
 * @brief Buddy 4 imu analysis module.
 */
#include "imu_analysis.h"
#include "../subsystem2/encoder_numeric.h"

#include <math.h>
#include <stddef.h>

#define IMU_GRAVITY_MS2 (9.81f)
#define IMU_DEGREES_TO_RADIANS (0.01745329252f)
#define IMU_RADIANS_TO_DEGREES (57.2957795131f)
/* Level baseline follows 1/16 of each level sample's difference. */
#define IMU_ROLL_BASELINE_FOLLOW (16.0f)

static void reset_sequence (imu_analysis_t * p_analysis, uint32_t reason);
static void reset_roll (imu_analysis_t * p_analysis);
static void expire_roll (imu_analysis_t * p_analysis, uint32_t now_ms);
static void update_roll_motion (imu_analysis_t * p_analysis, float32_t roll);
static void track_roll_height (imu_analysis_t * p_analysis,
                               const imu_data_t * p_data, bool b_level);
static float32_t push_roll_window (imu_analysis_t * p_analysis,
                                   float32_t excursion);
static void complete_height (imu_analysis_t * p_analysis,
                             imu_event_t * p_event);
static bool curving (const imu_analysis_t * p_analysis,
                     const imu_data_t * p_data);
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
                                            4.0f, 2.0f, 0.5f, 110.0f,
                                            80U, 0.3f, 7U};
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
        (!isfinite(p_config->roll_curve_ratio)) ||
        (p_config->roll_curve_ratio < 0.0f) ||
        (p_config->roll_curve_ratio > 1.0f) ||
        (p_config->roll_enter_ms >= p_config->hump_timeout_ms) ||
        (0U == p_config->roll_peak_samples) ||
        (p_config->roll_peak_samples > IMU_ROLL_PEAK_MAX) ||
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
        ((encoder_count_delta(p_data->encoder_left, p_analysis->left_count) > 0) ||
         (encoder_count_delta(p_data->encoder_right, p_analysis->right_count) > 0));

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
         ((encoder_count_delta(p_data->encoder_left, p_analysis->left_count) < 0) ||
          (encoder_count_delta(p_data->encoder_right, p_analysis->right_count) < 0))))
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
    p_analysis->roll_window_count = 0U;
    p_analysis->roll_window_next = 0U;
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
    /* Unfiltered roll: the angle smoothing spreads a jolt over the following
     * samples, where the window below could no longer drop it. */
    const float32_t raw_roll =
        atan2f(p_data->accel_y, p_data->accel_z) * IMU_RADIANS_TO_DEGREES;

    if (IMU_ROLL_UNARMED == p_analysis->roll_state)
    {
        if (b_level)
        {
            p_analysis->roll_baseline_deg +=
                (raw_roll - p_analysis->roll_baseline_deg) /
                (float32_t)(p_analysis->roll_count + 1U);
            /* Level history, so a hump straight after arming still has
             * enough samples around an edge jolt to drop it. */
            (void)push_roll_window(
                p_analysis, fabsf(raw_roll - p_analysis->roll_baseline_deg));
        }
        else
        {
            p_analysis->roll_baseline_deg = 0.0f;
            p_analysis->roll_window_count = 0U;
            p_analysis->roll_window_next = 0U;
        }
    }
    else
    {
        const float32_t excursion =
            fabsf(raw_roll - p_analysis->roll_baseline_deg);
        const float32_t estimate = push_roll_window(p_analysis, excursion);

        if ((IMU_ROLL_READY == p_analysis->roll_state) &&
            ((0U == p_analysis->roll_count) ||
             ((p_data->roll_deg >= 0.0f) != p_analysis->b_roll_positive)))
        {
            p_analysis->roll_peak_deg = estimate;
            if ((b_level) && (0U == p_analysis->roll_count))
            {
                /* Level driving between humps: follow slow attitude drift
                 * so the next height is measured from the current floor. */
                p_analysis->roll_baseline_deg +=
                    (raw_roll - p_analysis->roll_baseline_deg) /
                    IMU_ROLL_BASELINE_FOLLOW;
            }
        }
        else
        {
            p_analysis->roll_peak_deg =
                fmaxf(p_analysis->roll_peak_deg, estimate);
        }
    }
}

/** @brief Add an excursion; return the trimmed mean of the recent window. */
static float32_t
push_roll_window (imu_analysis_t * p_analysis, float32_t excursion)
{
    const uint32_t size = p_analysis->config.roll_peak_samples;
    float32_t      sorted[IMU_ROLL_PEAK_MAX];
    float32_t      sum = 0.0f;
    uint32_t       count;
    uint32_t       drop;

    p_analysis->roll_window[p_analysis->roll_window_next] = excursion;
    p_analysis->roll_window_next = (p_analysis->roll_window_next + 1U) % size;
    if (p_analysis->roll_window_count < size)
    {
        ++p_analysis->roll_window_count;
    }
    count = p_analysis->roll_window_count;

    /* Insertion sort: at most IMU_ROLL_PEAK_MAX values. */
    for (uint32_t index = 0U; index < count; ++index)
    {
        const float32_t value = p_analysis->roll_window[index];
        uint32_t        slot = index;

        while ((slot > 0U) && (sorted[slot - 1U] > value))
        {
            sorted[slot] = sorted[slot - 1U];
            --slot;
        }
        sorted[slot] = value;
    }

    /* Drop a third from each end: 7 samples keep the middle 3. */
    drop = count / 3U;
    for (uint32_t index = drop; index < count - drop; ++index)
    {
        sum += sorted[index];
    }
    return sum / (float32_t)(count - (2U * drop));
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

/** @brief True while the wheel speeds show a curve, whose sideways
 *         acceleration reads as roll on the accelerometer. */
static bool
curving (const imu_analysis_t * p_analysis, const imu_data_t * p_data)
{
    const int32_t left = p_data->motion.left_speed_cps;
    const int32_t right = p_data->motion.right_speed_cps;
    const int32_t fast = left > right ? left : right;
    const int32_t diff = left > right ? left - right : right - left;

    return (p_analysis->config.roll_curve_ratio > 0.0f) && (fast > 0) &&
           ((float32_t)diff >
            p_analysis->config.roll_curve_ratio * (float32_t)fast);
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
            if ((roll >= p_analysis->config.roll_enter_deg) &&
                (!curving(p_analysis, p_data)))
            {
                const bool b_positive = p_data->roll_deg >= 0.0f;
                p_analysis->roll_count =
                    ((0U != p_analysis->roll_count) &&
                     (b_positive == p_analysis->b_roll_positive))
                        ? p_analysis->roll_count + 1U : 1U;
                p_analysis->b_roll_positive = b_positive;
                if (1U == p_analysis->roll_count)
                {
                    p_analysis->roll_start_ms = p_data->timestamp_ms;
                }
            }
            else
            {
                p_analysis->roll_count = 0U;
            }
            /* Steering jerks tilt the accelerometer for a few samples; a
             * wheel on a hump stays tilted while it climbs. */
            if ((p_analysis->roll_count >= IMU_ROLL_CONFIRM_SAMPLES) &&
                (p_data->timestamp_ms - p_analysis->roll_start_ms >=
                 p_analysis->config.roll_enter_ms))
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

