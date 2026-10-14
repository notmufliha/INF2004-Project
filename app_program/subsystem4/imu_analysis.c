#include "imu_analysis.h"

#include <math.h>
#include <stddef.h>

#define IMU_GRAVITY_MS2 (9.81f)

static void reset_sequence(imu_analysis_t *analysis)
{
    analysis->terrain = IMU_TERRAIN_LEVEL;
    analysis->confirmation_count = 0U;
}

void imu_analysis_default_config(imu_analysis_config_t *config)
{
    if (config != NULL)
    {
        *config = (imu_analysis_config_t){8.0f, 3.0f, 8.0f, 1.0f,
                                        15.0f, 6.0f, 3U, 100U, 10000U};
    }
}

bool imu_analysis_init(imu_analysis_t *analysis,
                       const imu_analysis_config_t *config)
{
    if ((analysis == NULL) || (config == NULL) ||
        !isfinite(config->slope_enter_deg) ||
        !isfinite(config->level_exit_deg) || !isfinite(config->tilt_deg) ||
        !isfinite(config->acceleration_ms2) ||
        !isfinite(config->turn_rate_dps) || !isfinite(config->impact_ms2) ||
        (config->level_exit_deg < 0.0f) ||
        (config->slope_enter_deg <= config->level_exit_deg) ||
        (config->tilt_deg <= 0.0f) || (config->acceleration_ms2 <= 0.0f) ||
        (config->turn_rate_dps <= 0.0f) ||
        (config->impact_ms2 <= config->acceleration_ms2) ||
        (config->confirmation_samples == 0U) || (config->max_gap_ms == 0U) ||
        (config->hump_timeout_ms <= config->max_gap_ms) ||
        (config->hump_timeout_ms >= UINT32_MAX / 2U))
    {
        return false;
    }
    *analysis = (imu_analysis_t){0};
    analysis->config = *config;
    return true;
}

static bool confirmed(imu_analysis_t *analysis, bool condition)
{
    if (!condition)
    {
        analysis->confirmation_count = 0U;
        return false;
    }
    analysis->confirmation_count++;
    if (analysis->confirmation_count >= analysis->config.confirmation_samples)
    {
        analysis->confirmation_count = 0U;
        return true;
    }
    return false;
}

bool imu_analysis_process(imu_analysis_t *analysis, const imu_data_t *data,
                          imu_event_t *event)
{
    uint32_t elapsed;
    float magnitude;
    bool level;
    bool descending;

    if (event != NULL)
    {
        *event = (imu_event_t){0};
        event->motion_state = IMU_MOTION_UNKNOWN;
    }
    if ((analysis == NULL) || (event == NULL) || (data == NULL) ||
        !data->valid || !isfinite(data->accel_x) ||
        !isfinite(data->accel_y) || !isfinite(data->accel_z) ||
        !isfinite(data->pitch_deg) || !isfinite(data->roll_deg) ||
        !isfinite(data->turn_rate_dps))
    {
        if (analysis != NULL)
        {
            reset_sequence(analysis);
            analysis->has_timestamp = false;
        }
        return false;
    }
    magnitude = hypotf(hypotf(data->accel_x, data->accel_y), data->accel_z);
    if (!isfinite(magnitude))
    {
        reset_sequence(analysis);
        analysis->has_timestamp = false;
        return false;
    }
    if (analysis->has_timestamp)
    {
        elapsed = data->timestamp_ms - analysis->last_timestamp_ms;
        if ((elapsed == 0U) || (elapsed > analysis->config.max_gap_ms))
        {
            reset_sequence(analysis);
            analysis->has_timestamp = false;
            return false;
        }
    }
    analysis->last_timestamp_ms = data->timestamp_ms;
    analysis->has_timestamp = true;
    event->timestamp_ms = data->timestamp_ms;
    event->pitch_deg = data->pitch_deg;
    event->roll_deg = data->roll_deg;
    event->turn_rate_dps = data->turn_rate_dps;
    event->valid = true;
    event->tilt_detected = (fabsf(data->pitch_deg) >= analysis->config.tilt_deg) ||
                          (fabsf(data->roll_deg) >= analysis->config.tilt_deg);
    /* Compare magnitude with gravity: a stationary tilted sensor is not impact.
     * This is a provisional detector, not gravity-compensated acceleration.
     */
    event->sudden_impact = fabsf(magnitude - IMU_GRAVITY_MS2) >=
                          analysis->config.impact_ms2;
    if (event->sudden_impact)
    {
        reset_sequence(analysis);
        event->motion_state = IMU_MOTION_SUDDEN_IMPACT;
        return true;
    }
    if ((analysis->terrain != IMU_TERRAIN_LEVEL) &&
        (data->timestamp_ms - analysis->hump_start_ms >=
         analysis->config.hump_timeout_ms))
    {
        reset_sequence(analysis);
    }
    level = fabsf(data->pitch_deg) <= analysis->config.level_exit_deg;
    descending = data->pitch_deg <= -analysis->config.slope_enter_deg;
    switch (analysis->terrain)
    {
        case IMU_TERRAIN_LEVEL:
            if (confirmed(analysis, data->pitch_deg >= analysis->config.slope_enter_deg))
            {
                analysis->terrain = IMU_TERRAIN_CLIMBING;
                analysis->hump_start_ms = data->timestamp_ms;
            }
            break;
        case IMU_TERRAIN_CLIMBING:
            if (confirmed(analysis, level || descending))
            {
                analysis->terrain = descending ? IMU_TERRAIN_DESCENDING : IMU_TERRAIN_PEAK;
            }
            break;
        case IMU_TERRAIN_PEAK:
            if (confirmed(analysis, descending))
            {
                analysis->terrain = IMU_TERRAIN_DESCENDING;
            }
            break;
        case IMU_TERRAIN_DESCENDING:
            if (confirmed(analysis, level))
            {
                reset_sequence(analysis);
                event->hump_completed = true;
            }
            break;
        default:
            reset_sequence(analysis);
            break;
    }
    event->hump_detected = (analysis->terrain != IMU_TERRAIN_LEVEL) ||
                          event->hump_completed;
    switch (analysis->terrain)
    {
        case IMU_TERRAIN_CLIMBING:
            event->motion_state = IMU_MOTION_CLIMBING_HUMP;
            break;
        case IMU_TERRAIN_PEAK:
            event->motion_state = IMU_MOTION_HUMP_PEAK;
            break;
        case IMU_TERRAIN_DESCENDING:
            event->motion_state = IMU_MOTION_DESCENDING_HUMP;
            break;
        default:
            if (fabsf(data->turn_rate_dps) >= analysis->config.turn_rate_dps)
            {
                event->motion_state = IMU_MOTION_TURNING;
            }
            else if (fabsf(magnitude - IMU_GRAVITY_MS2) >= analysis->config.acceleration_ms2 ||
                     (!event->tilt_detected &&
                      hypotf(data->accel_x, data->accel_y) >= analysis->config.acceleration_ms2))
            {
                event->motion_state = IMU_MOTION_ACCELERATING;
            }
            else
            {
                /* IMU alone cannot establish that the vehicle is stationary. */
                event->motion_state = IMU_MOTION_UNKNOWN;
            }
            break;
    }
    return true;
}
