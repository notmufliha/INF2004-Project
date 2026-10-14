#include "imu_processing.h"
#include <math.h>
#include <stddef.h>

void imu_processing_default_config(imu_processing_config_t *config)
{
    if (config != NULL) {
        *config = (imu_processing_config_t){0.25f, 3U, 5U, 100U};
    }
}

bool imu_processing_init(imu_processing_t *state,
                         const imu_processing_config_t *config)
{
    if (state == NULL || config == NULL ||
        !isfinite(config->smoothing_alpha) || config->smoothing_alpha <= 0.0f ||
        config->smoothing_alpha > 1.0f || config->failure_samples == 0U ||
        config->recovery_samples == 0U || config->max_gap_ms == 0U ||
        config->max_gap_ms >= UINT32_MAX / 2U) {
        return false;
    }
    *state = (imu_processing_t){0};
    state->config = *config;
    return true;
}

static float smooth_angle(float old, float next, float alpha)
{
    /* Follow the shortest path across the +/-180 degree boundary. */
    float delta = next - old;
    if (delta > 180.0f) { delta -= 360.0f; }
    if (delta < -180.0f) { delta += 360.0f; }
    float result = old + alpha * delta;
    if (result > 180.0f) { result -= 360.0f; }
    if (result < -180.0f) { result += 360.0f; }
    return result;
}

bool imu_processing_process(imu_processing_t *state, const imu_data_t *input,
                            uint32_t now_ms, imu_data_t *output,
                            imu_health_t *health)
{
    bool valid;
    bool timing_fault = false;
    uint32_t elapsed;
    if (output != NULL) {
        *output = (imu_data_t){0};
        output->timestamp_ms = now_ms;
    }
    if (health != NULL) { *health = (imu_health_t){0}; }
    if (state == NULL || output == NULL || health == NULL) { return false; }

    valid = input != NULL && input->valid &&
            input->timestamp_ms == now_ms &&
            isfinite(input->accel_x) && isfinite(input->accel_y) &&
            isfinite(input->accel_z) && isfinite(input->pitch_deg) &&
            isfinite(input->roll_deg) && isfinite(input->turn_rate_dps) &&
            fabsf(input->pitch_deg) <= 180.0f && fabsf(input->roll_deg) <= 180.0f;
    if (state->have_good_time) {
        elapsed = now_ms - state->last_good_ms;
        if (elapsed == 0U || elapsed >= UINT32_MAX / 2U) {
            valid = false;
            timing_fault = true;
            /* Rebase only for subsequent recovery, never use this reading. */
            state->have_good_time = false;
        } else if (elapsed > state->config.max_gap_ms) {
            timing_fault = true;
        }
    }
    if (timing_fault) {
        if (!state->fault_active) { health->fault_started = true; }
        state->fault_active = true;
        state->seeded = false;
        state->recovery = 0U;
    }
    if (!valid) {
        if (state->failures < state->config.failure_samples) { state->failures++; }
        state->seeded = false;
        state->recovery = 0U;
        if (state->failures >= state->config.failure_samples) {
            if (!state->fault_active) { health->fault_started = true; }
            state->fault_active = true;
        }
        health->fault_active = state->fault_active;
        return false;
    }
    state->last_good_ms = now_ms;
    state->have_good_time = true;
    state->failures = 0U;
    if (state->fault_active) {
        state->recovery++;
        if (state->recovery < state->config.recovery_samples) {
            health->fault_active = true;
            return false;
        }
        state->fault_active = false;
        state->recovery = 0U;
        health->recovered = true;
    }
    *output = *input;
    if (state->seeded) {
        output->pitch_deg = smooth_angle(state->previous.pitch_deg, input->pitch_deg,
                                         state->config.smoothing_alpha);
        output->roll_deg = smooth_angle(state->previous.roll_deg, input->roll_deg,
                                        state->config.smoothing_alpha);
        /* Convex combination avoids overflow in (next - old). */
        output->turn_rate_dps = (1.0f - state->config.smoothing_alpha) *
                               state->previous.turn_rate_dps +
                               state->config.smoothing_alpha * input->turn_rate_dps;
    }
    state->previous = *output;
    state->seeded = true;
    return true;
}
