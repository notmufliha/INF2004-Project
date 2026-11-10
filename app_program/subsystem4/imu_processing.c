/** @file imu_processing.c
 * @brief Buddy 4 imu processing module.
 */
#include "imu_processing.h"
#include <math.h>
#include <stddef.h>

static float32_t smooth_angle (float32_t old, float32_t next, float32_t alpha);

static bool reading_is_valid (const imu_data_t * p_input, uint32_t now_ms);

static bool check_timing (imu_processing_t * p_state, uint32_t now_ms,
                          bool * pb_valid);

static void record_failure (imu_processing_t *        p_state,
                            imu_processing_health_t * p_health);

static bool filter_reading (imu_processing_t * p_state,
                            const imu_data_t * p_input, imu_data_t * p_output);

/** @brief Supply filter, fault and recovery defaults. */
void
imu_processing_default_config (imu_processing_config_t * p_config)
{
    if (p_config != NULL)
    {
        *p_config = (imu_processing_config_t){0.25f, 3U, 5U, 100U};
    }
}

/** @brief Validate configuration and reset filter history. */
bool
imu_processing_init (imu_processing_t *              p_state,
                     const imu_processing_config_t * p_config)
{
    if ((NULL == p_state) || (NULL == p_config) ||
        (!isfinite(p_config->smoothing_alpha)) ||
        (p_config->smoothing_alpha <= 0.0f) ||
        (p_config->smoothing_alpha > 1.0f) ||
        (0U == p_config->failure_samples) ||
        (0U == p_config->recovery_samples) || (0U == p_config->max_gap_ms) ||
        (p_config->max_gap_ms >= UINT32_MAX / 2U))
    {
        return false;
    }

    *p_state        = (imu_processing_t){0};
    p_state->config = *p_config;
    return true;
}

/** @brief Validate timing, track health and filter a fresh reading. */
bool
imu_processing_process (imu_processing_t * p_state, const imu_data_t * p_input,
                        uint32_t now_ms, imu_data_t * p_output,
                        imu_processing_health_t * p_health)
{
    if (p_output != NULL)
    {
        *p_output              = (imu_data_t){0};
        p_output->timestamp_ms = now_ms;
    }

    if (p_health != NULL)
    {
        *p_health = (imu_processing_health_t){0};
    }

    if ((NULL == p_state) || (NULL == p_output) || (NULL == p_health))
    {
        return false;
    }

    bool b_valid        = reading_is_valid(p_input, now_ms);
    bool b_timing_fault = check_timing(p_state, now_ms, &b_valid);

    if (b_timing_fault)
    {
        if (!p_state->b_fault_active)
        {
            p_health->b_fault_started = true;
        }

        p_state->b_fault_active = true;
        p_state->b_seeded       = false;
        p_state->recovery       = 0U;
    }

    if (!b_valid)
    {
        record_failure(p_state, p_health);
        return false;
    }

    p_state->last_good_ms     = now_ms;
    p_state->b_have_good_time = true;
    p_state->failures         = 0U;

    if (p_state->b_fault_active)
    {
        p_state->recovery++;

        if (p_state->recovery < p_state->config.recovery_samples)
        {
            p_health->b_fault_active = true;
            return false;
        }

        p_state->b_fault_active = false;
        p_state->recovery       = 0U;
        p_health->b_recovered   = true;
    }

    if (!filter_reading(p_state, p_input, p_output))
    {
        record_failure(p_state, p_health);
        *p_output              = (imu_data_t){0};
        p_output->timestamp_ms = now_ms;
        return false;
    }

    return true;
}

/** @brief Interpolate along the shortest path across the angle boundary. */
static float32_t
smooth_angle (float32_t old, float32_t next, float32_t alpha)
{
    /* Follow the shortest path across the +/-180 degree boundary. */
    float32_t delta = next - old;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }

    if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    float32_t result = old + (alpha * delta);

    if (result > 180.0f)
    {
        result -= 360.0f;
    }

    if (result < -180.0f)
    {
        result += 360.0f;
    }

    return result;
}

/** @brief Check reading freshness, finite values and angle bounds. */
static bool
reading_is_valid (const imu_data_t * p_input, uint32_t now_ms)
{
    return (p_input != NULL) && (p_input->b_valid) &&
           (p_input->timestamp_ms == now_ms) && (isfinite(p_input->accel_x)) &&
           (isfinite(p_input->accel_y)) && (isfinite(p_input->accel_z)) &&
           (isfinite(p_input->pitch_deg)) && (isfinite(p_input->roll_deg)) &&
           (isfinite(p_input->turn_rate_dps)) &&
           (fabsf(p_input->pitch_deg) <= 180.0f) &&
           (fabsf(p_input->roll_deg) <= 180.0f);
}

/** @brief Detect duplicate, backward or missing-reading intervals. */
static bool
check_timing (imu_processing_t * p_state, uint32_t now_ms, bool * pb_valid)
{
    bool     b_timing_fault = false;
    uint32_t elapsed;

    if (p_state->b_have_good_time)
    {
        elapsed = now_ms - p_state->last_good_ms;

        if ((0U == elapsed) || (elapsed >= (UINT32_MAX / 2U)))
        {
            *pb_valid      = false;
            b_timing_fault = true;
            /* Rebase only for subsequent recovery, never use this reading. */
            p_state->b_have_good_time = false;
        }
        else if (elapsed > p_state->config.max_gap_ms)
        {
            b_timing_fault = true;
        }
        else
        {
            /* The elapsed interval is valid; retain the existing state. */
        }
    }

    return b_timing_fault;
}

/** @brief Count failures and latch a sensor fault at the threshold. */
static void
record_failure (imu_processing_t * p_state, imu_processing_health_t * p_health)
{

    if (p_state->failures < p_state->config.failure_samples)
    {
        p_state->failures++;
    }

    p_state->b_seeded = false;
    p_state->recovery = 0U;

    if (p_state->failures >= p_state->config.failure_samples)
    {
        if (!p_state->b_fault_active)
        {
            p_health->b_fault_started = true;
        }

        p_state->b_fault_active = true;
    }

    p_health->b_fault_active = p_state->b_fault_active;
}

/** @brief Smooth finite angles and rate while retaining acceleration. */
static bool
filter_reading (imu_processing_t * p_state, const imu_data_t * p_input,
                imu_data_t * p_output)
{
    *p_output = *p_input;
    if (p_state->b_seeded)
    {
        /* Preserve the configured response per 10 ms, not per arrival.
         * Accepted intervals are positive and bounded by max_gap_ms.
         */
        const uint32_t elapsed =
            p_input->timestamp_ms - p_state->previous.timestamp_ms;
        const float32_t alpha =
            1.0f - powf(1.0f - p_state->config.smoothing_alpha,
                        (float32_t)elapsed / 10.0f);

        p_output->pitch_deg =
            smooth_angle(p_state->previous.pitch_deg, p_input->pitch_deg,
                         alpha);
        p_output->roll_deg =
            smooth_angle(p_state->previous.roll_deg, p_input->roll_deg,
                         alpha);
        /* Convex combination avoids overflow in (next - old). */
        if ((p_input->b_turn_rate_valid) &&
            (p_state->previous.b_turn_rate_valid))
        {
            p_output->turn_rate_dps =
                (1.0f - alpha) *
                    p_state->previous.turn_rate_dps +
                alpha * p_input->turn_rate_dps;
        }
    }

    if ((!isfinite(p_output->pitch_deg)) || (!isfinite(p_output->roll_deg)) ||
        (!isfinite(p_output->turn_rate_dps)))
    {
        return false;
    }

    p_state->previous = *p_output;
    p_state->b_seeded = true;
    return true;
}

/*** end of file ***/

