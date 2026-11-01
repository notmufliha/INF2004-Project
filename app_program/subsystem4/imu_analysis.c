/** @file imu_analysis.c
 * @brief Buddy 4 imu analysis module.
 */
#include "imu_analysis.h"

#include <math.h>
#include <stddef.h>

#define IMU_GRAVITY_MS2 (9.81f)

static void reset_sequence (imu_analysis_t * p_analysis);

static bool confirmed (imu_analysis_t * p_analysis, bool b_condition);

static void update_terrain (imu_analysis_t *   p_analysis,
                            const imu_data_t * p_data, imu_event_t * p_event);

static void classify_motion (const imu_analysis_t * p_analysis,
                             const imu_data_t * p_data, imu_event_t * p_event,
                             float32_t magnitude);

/** @brief Supply provisional motion and terrain thresholds. */
void
imu_analysis_default_config (imu_analysis_config_t * p_config)
{
    if (p_config != NULL)
    {
        *p_config = (imu_analysis_config_t){8.0f, 3.0f, 8.0f, 1.0f,  15.0f,
                                            6.0f, 3U,   100U, 10000U};
    }
}

/** @brief Validate thresholds and reset the terrain context. */
bool
imu_analysis_init (imu_analysis_t *              p_analysis,
                   const imu_analysis_config_t * p_config)
{
    if ((NULL == p_analysis) || (NULL == p_config) ||
        (!isfinite(p_config->slope_enter_deg)) ||
        (!isfinite(p_config->level_exit_deg)) ||
        (!isfinite(p_config->tilt_deg)) ||
        (!isfinite(p_config->acceleration_ms2)) ||
        (!isfinite(p_config->turn_rate_dps)) ||
        (!isfinite(p_config->impact_ms2)) ||
        (p_config->level_exit_deg < 0.0f) ||
        (p_config->slope_enter_deg <= p_config->level_exit_deg) ||
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
    return true;
}

/** @brief Validate a reading and publish motion and terrain status. */
bool
imu_analysis_process (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                      imu_event_t * p_event)
{
    uint32_t  elapsed;
    float32_t magnitude;

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
            reset_sequence(p_analysis);
            p_analysis->b_has_timestamp = false;
        }

        return false;
    }

    magnitude =
        hypotf(hypotf(p_data->accel_x, p_data->accel_y), p_data->accel_z);

    if (!isfinite(magnitude))
    {
        reset_sequence(p_analysis);
        p_analysis->b_has_timestamp = false;
        return false;
    }

    if (p_analysis->b_has_timestamp)
    {
        elapsed = p_data->timestamp_ms - p_analysis->last_timestamp_ms;

        if ((0U == elapsed) || (elapsed > p_analysis->config.max_gap_ms))
        {
            reset_sequence(p_analysis);
            p_analysis->b_has_timestamp = false;
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
        reset_sequence(p_analysis);
        p_event->motion_state = IMU_MOTION_SUDDEN_IMPACT;
        return true;
    }

    update_terrain(p_analysis, p_data, p_event);
    classify_motion(p_analysis, p_data, p_event, magnitude);
    return true;
}

/** @brief Discard an incomplete hump traversal. */
static void
reset_sequence (imu_analysis_t * p_analysis)
{
    p_analysis->terrain            = IMU_TERRAIN_LEVEL;
    p_analysis->confirmation_count = 0U;
}

/** @brief Require consecutive qualifying readings before a transition. */
static bool
confirmed (imu_analysis_t * p_analysis, bool b_condition)
{
    if (!b_condition)
    {
        p_analysis->confirmation_count = 0U;
        return false;
    }

    p_analysis->confirmation_count++;

    if (p_analysis->confirmation_count >=
        p_analysis->config.confirmation_samples)
    {
        p_analysis->confirmation_count = 0U;
        return true;
    }

    return false;
}

/** @brief Advance or time out the climb, peak and descent sequence. */
static void
update_terrain (imu_analysis_t * p_analysis, const imu_data_t * p_data,
                imu_event_t * p_event)
{
    if ((p_analysis->terrain != IMU_TERRAIN_LEVEL) &&
        (p_data->timestamp_ms - p_analysis->hump_start_ms >=
         p_analysis->config.hump_timeout_ms))
    {
        reset_sequence(p_analysis);
    }

    bool b_level =
        fabsf(p_data->pitch_deg) <= p_analysis->config.level_exit_deg;
    bool b_descending =
        p_data->pitch_deg <= -p_analysis->config.slope_enter_deg;

    switch (p_analysis->terrain)
    {
        case IMU_TERRAIN_LEVEL:

            if (confirmed(p_analysis, p_data->pitch_deg >=
                                          p_analysis->config.slope_enter_deg))
            {
                p_analysis->terrain       = IMU_TERRAIN_CLIMBING;
                p_analysis->hump_start_ms = p_data->timestamp_ms;
            }
        break;
        case IMU_TERRAIN_CLIMBING:

            if (confirmed(p_analysis, b_level || b_descending))
            {
                p_analysis->terrain =
                    b_descending ? IMU_TERRAIN_DESCENDING : IMU_TERRAIN_PEAK;
            }
        break;
        case IMU_TERRAIN_PEAK:

            if (confirmed(p_analysis, b_descending))
            {
                p_analysis->terrain = IMU_TERRAIN_DESCENDING;
            }
        break;
        case IMU_TERRAIN_DESCENDING:

            if (confirmed(p_analysis, b_level))
            {
                reset_sequence(p_analysis);
                p_event->b_hump_completed = true;
            }
        break;
        default:
            reset_sequence(p_analysis);
        break;
    }

    p_event->b_hump_detected = (p_analysis->terrain != IMU_TERRAIN_LEVEL) ||
                               (p_event->b_hump_completed);
}

/** @brief Choose the motion state from terrain and dynamic thresholds. */
static void
classify_motion (const imu_analysis_t * p_analysis, const imu_data_t * p_data,
                 imu_event_t * p_event, float32_t magnitude)
{
    switch (p_analysis->terrain)
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
                /* IMU alone cannot establish that the vehicle is stationary. */
                p_event->motion_state = IMU_MOTION_UNKNOWN;
            }
        break;
    }
}

/*** end of file ***/

