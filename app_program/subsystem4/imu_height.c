/** @file imu_height.c
 * @brief Integrate vertical wheel-path displacement over a confirmed hump.
 */
#include "imu_height.h"
#include <math.h>
#include <stddef.h>

#define HEIGHT_DEG_TO_RAD 0.01745329252f
#define HEIGHT_TICK_LIMIT 4096
#define HEIGHT_MAX_TRAVEL_CM 200.0f
#define HEIGHT_MIN_PEAK_CM 0.2f

static void cancel_sequence (imu_height_t * p_state);
static void seed_reading (imu_height_t * p_state, const imu_data_t * p_data);
static bool sample_is_valid (const imu_height_t * p_state,
                             const imu_data_t *   p_data);
static bool wheel_step (const imu_height_t * p_state, const imu_data_t * p_data,
                        float32_t * p_left, float32_t * p_right);
static void integrate_step (imu_height_t * p_state, const imu_data_t * p_data,
                            float32_t left_cm, float32_t right_cm);
static void finish_height (imu_height_t * p_state, imu_event_t * p_event);
static bool motion_allows_height (const imu_data_t * p_data);

/** @brief Supply conservative consistency limits without assuming geometry. */
void
imu_height_default_config (imu_height_config_t * p_config)
{
    if (NULL != p_config)
    {
        *p_config = (imu_height_config_t){0.0f, 0.0f, 100.0f, 10.0f, 45.0f,
                                          3.0f, 1.0f, 25U,    false};
    }
}

/** @brief Validate calibration and reset the estimator and session maximum. */
bool
imu_height_init (imu_height_t * p_state, const imu_height_config_t * p_config)
{
    if ((NULL == p_state) || (NULL == p_config) ||
        (!isfinite(p_config->left_cm_per_count)) ||
        (!isfinite(p_config->right_cm_per_count)) ||
        (!isfinite(p_config->max_speed_cm_s)) ||
        (!isfinite(p_config->max_roll_deg)) ||
        (!isfinite(p_config->max_pitch_deg)) ||
        (!isfinite(p_config->level_deg)) || (!isfinite(p_config->closure_cm)) ||
        (p_config->left_cm_per_count <= 0.0f) ||
        (p_config->right_cm_per_count <= 0.0f) ||
        (p_config->max_speed_cm_s <= 0.0f) ||
        (p_config->max_roll_deg <= 0.0f) || (p_config->max_roll_deg >= 45.0f) ||
        (p_config->level_deg <= 0.0f) ||
        (p_config->max_pitch_deg <= p_config->level_deg) ||
        (p_config->max_pitch_deg >= 90.0f) || (p_config->closure_cm <= 0.0f) ||
        (0U == p_config->max_gap_ms) || (p_config->max_gap_ms > 1000U))
    {
        return false;
    }

    *p_state        = (imu_height_t){0};
    p_state->config = *p_config;
    return true;
}

/** @brief Advance an estimate only while both sensor streams are trustworthy.
 */
void
imu_height_process (imu_height_t * p_state, const imu_data_t * p_data,
                    imu_event_t * p_event)
{
    float32_t left_cm  = 0.0f;
    float32_t right_cm = 0.0f;

    if (NULL == p_event)
    {
        return;
    }

    p_event->b_height_valid       = false;
    p_event->hump_peak_cm         = 0.0f;
    p_event->highest_hump_cm      = 0.0f;
    p_event->b_highest_hump_valid = false;

    if (NULL == p_state)
    {
        return;
    }

    p_event->highest_hump_cm      = p_state->highest_cm;
    p_event->b_highest_hump_valid = p_state->b_highest_valid;

    if ((!p_event->b_valid) || (p_event->b_sudden_impact) ||
        (!sample_is_valid(p_state, p_data)))
    {
        cancel_sequence(p_state);
        p_state->b_seeded = false;
        return;
    }

    if ((!p_state->b_seeded) ||
        (!wheel_step(p_state, p_data, &left_cm, &right_cm)))
    {
        cancel_sequence(p_state);
        seed_reading(p_state, p_data);
        return;
    }

    if ((fabsf(p_data->pitch_deg) <= p_state->config.level_deg) &&
        (!p_event->b_hump_detected) && (!p_event->b_hump_completed))
    {
        cancel_sequence(p_state);
        p_state->b_blocked = false;
    }

    if ((p_state->b_confirmed) && (!p_event->b_hump_detected) &&
        (!p_event->b_hump_completed))
    {
        cancel_sequence(p_state);
    }

    if ((!p_state->b_blocked) && (!p_state->b_active) &&
        (p_data->pitch_deg > p_state->config.level_deg))
    {
        p_state->b_active = true;
    }

    if (p_state->b_active)
    {
        p_state->b_confirmed =
            (p_state->b_confirmed) || (p_event->b_hump_detected);
        integrate_step(p_state, p_data, left_cm, right_cm);

        if (p_event->b_hump_completed)
        {
            finish_height(p_state, p_event);
            cancel_sequence(p_state);
        }
    }

    seed_reading(p_state, p_data);
}

/** @brief Invalidate the whole current traversal while retaining the maximum.
 */
static void
cancel_sequence (imu_height_t * p_state)
{
    p_state->b_active    = false;
    p_state->b_confirmed = false;
    p_state->b_descended = false;
    p_state->b_blocked   = true;
    p_state->rise_cm     = 0.0f;
    p_state->peak_cm     = 0.0f;
    p_state->left_cm     = 0.0f;
    p_state->right_cm    = 0.0f;
}

/** @brief Store the count/pitch baseline without fabricating a missing step. */
static void
seed_reading (imu_height_t * p_state, const imu_data_t * p_data)
{
    p_state->left_count      = p_data->encoder_left;
    p_state->right_count     = p_data->encoder_right;
    p_state->left_errors     = p_data->encoder_left_errors;
    p_state->right_errors    = p_data->encoder_right_errors;
    p_state->timestamp_ms    = p_data->timestamp_ms;
    p_state->sample_sequence = p_data->sample_sequence;
    p_state->pitch_deg       = p_data->pitch_deg;
    p_state->b_seeded        = true;
}

/** @brief Reject missing odometry, extreme tilt and strongly dynamic readings.
 */
static bool
sample_is_valid (const imu_height_t * p_state, const imu_data_t * p_data)
{
    if ((NULL == p_data) || (!p_data->b_valid) || (!p_data->b_odometry_valid) ||
        (!isfinite(p_data->pitch_deg)) || (!isfinite(p_data->roll_deg)) ||
        (!isfinite(p_data->accel_x)) || (!isfinite(p_data->accel_y)) ||
        (!isfinite(p_data->accel_z)) ||
        (fabsf(p_data->pitch_deg) > p_state->config.max_pitch_deg) ||
        (fabsf(p_data->roll_deg) > p_state->config.max_roll_deg))
    {
        return false;
    }

    if ((p_state->config.b_require_motion) && (!motion_allows_height(p_data)))
    {
        return false;
    }

    const float32_t norm =
        hypotf(hypotf(p_data->accel_x, p_data->accel_y), p_data->accel_z);

    return (isfinite(norm)) && (fabsf(norm - 9.81f) <= 1.5f);
}

/** @brief Convert bounded forward deltas and reject gaps/errors/pivoting. */
static bool
wheel_step (const imu_height_t * p_state, const imu_data_t * p_data,
            float32_t * p_left, float32_t * p_right)
{
    const uint32_t elapsed = p_data->timestamp_ms - p_state->timestamp_ms;

    /* int32_t counts widen exactly; subtraction cannot overflow int64_t. */
    const int64_t left  = (int64_t)p_data->encoder_left - p_state->left_count;
    const int64_t right = (int64_t)p_data->encoder_right - p_state->right_count;

    if ((0U == elapsed) || (elapsed > p_state->config.max_gap_ms) ||
        (p_data->sample_sequence != p_state->sample_sequence + 1U) ||
        (p_data->encoder_left_errors != p_state->left_errors) ||
        (p_data->encoder_right_errors != p_state->right_errors) || (left < 0) ||
        (right < 0) || (left > HEIGHT_TICK_LIMIT) ||
        (right > HEIGHT_TICK_LIMIT))
    {
        return false;
    }

    /* Validated counts 0..4096 and milliseconds 1..1000 fit binary32 exactly.
     */
    *p_left  = (float32_t)left * p_state->config.left_cm_per_count;
    *p_right = (float32_t)right * p_state->config.right_cm_per_count;
    const float32_t mean = (*p_left + *p_right) * 0.5f;
    const float32_t limit =
        p_state->config.max_speed_cm_s * ((float32_t)elapsed / 1000.0f);

    if ((p_state->config.b_require_motion) && (p_data->motion.b_stopped) &&
        (mean > 0.05f))
    {
        /* More than encoder quantization contradicts a stopped snapshot. */
        return false;
    }

    return (isfinite(mean)) && (isfinite(limit)) && (mean <= limit) &&
           (fabsf(*p_left - *p_right) <= fmaxf(0.1f, mean * 0.25f));
}

/** @brief Integrate ds*sin(pitch) using the mean of adjacent pitch sines. */
static void
integrate_step (imu_height_t * p_state, const imu_data_t * p_data,
                float32_t left_cm, float32_t right_cm)
{
    /* Wheel travel follows the road; chassis pitch must approximate its slope.
     * Starting before terrain confirmation retains the first rising samples.
     */
    const float32_t mean = (left_cm + right_cm) * 0.5f;
    const float32_t sine = (sinf(p_state->pitch_deg * HEIGHT_DEG_TO_RAD) +
                            sinf(p_data->pitch_deg * HEIGHT_DEG_TO_RAD)) *
                           0.5f;

    if ((p_data->pitch_deg < -p_state->config.level_deg) && (mean > 0.0f))
    {
        p_state->b_descended = true;
    }

    p_state->rise_cm += mean * sine;
    p_state->left_cm += left_cm;
    p_state->right_cm += right_cm;
    p_state->peak_cm = fmaxf(p_state->peak_cm, p_state->rise_cm);

    if ((!isfinite(p_state->rise_cm)) ||
        (p_state->rise_cm < -p_state->config.closure_cm) ||
        (p_state->left_cm + p_state->right_cm > 2.0f * HEIGHT_MAX_TRAVEL_CM))
    {
        cancel_sequence(p_state);
    }
}

/** @brief Accept only closed, confirmed traversals with consistent wheels. */
static void
finish_height (imu_height_t * p_state, imu_event_t * p_event)
{
    const float32_t closure = fminf(p_state->config.closure_cm,
                                    fmaxf(0.05f, p_state->peak_cm * 0.25f));
    const float32_t travel  = (p_state->left_cm + p_state->right_cm) * 0.5f;

    if ((!p_state->b_confirmed) || (!p_state->b_descended) ||
        (p_state->b_blocked) || (p_state->peak_cm < HEIGHT_MIN_PEAK_CM) ||
        (travel < 0.5f) || (fabsf(p_state->rise_cm) > closure) ||
        (fabsf(p_state->left_cm - p_state->right_cm) >
         fmaxf(0.2f, travel * 0.1f)))
    {
        return;
    }

    p_state->highest_cm      = fmaxf(p_state->highest_cm, p_state->peak_cm);
    p_state->b_highest_valid = true;
    p_event->hump_peak_cm    = p_state->peak_cm;
    p_event->highest_hump_cm = p_state->highest_cm;
    p_event->b_height_valid  = true;
    p_event->b_highest_hump_valid = true;
}

/** @brief Use paired fresh B2 context without substituting its wheel counts. */
static bool
motion_allows_height (const imu_data_t * p_data)
{
    const imu_motion_t * p_motion = &p_data->motion;

    return (p_motion->b_valid) && (!p_motion->b_fault) &&
           (!p_motion->b_reverse) && (!p_motion->b_turning) &&
           ((p_motion->b_forward) || (p_motion->b_stopped)) &&
           ((p_data->timestamp_ms - p_motion->timestamp_ms) <=
            IMU_MOTION_MAX_AGE_MS);
}

/*** end of file ***/

