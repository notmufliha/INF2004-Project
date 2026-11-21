/** @file imu_lift.c
 * @brief Buddy 4 lift detection (see imu_lift.h).
 */
#include "imu_lift.h"

#include <math.h>
#include <stddef.h>

static bool accept_sample (imu_lift_t * p_lift, const imu_data_t * p_data,
                           float32_t * p_magnitude, float32_t * p_dt_s);
static void integrate (imu_lift_t * p_lift, float32_t excess, float32_t dt_s);
static void learn_baseline (imu_lift_t * p_lift, float32_t excess,
                            float32_t dt_s);
static void detect (imu_lift_t * p_lift, float32_t dt_s, uint32_t now_ms);
static void restart_integration (imu_lift_t * p_lift);
static void expire_hold (imu_lift_t * p_lift, uint32_t now_ms);

/** @brief Supply provisional lift thresholds. */
void
imu_lift_default_config (imu_lift_config_t * p_config)
{
    if (NULL != p_config)
    {
        /* A hand pick-up reaches ~0.4-1 m/s and rises 100-200 mm.  Host
         * sweep (testlift_main.c): no false lift in 60 s of +-2 m/s^2
         * broadband vibration on every axis; typical/gentle pick-ups
         * caught.  Short low lifts are left to the line-sensor rule. */
        *p_config = (imu_lift_config_t){0.25f, 0.03f, 0.40f, 0.80f, 2.0f,
                                        1000U, 40U, 100U};
    }
}

/** @brief Validate thresholds and start without a learned baseline. */
bool
imu_lift_init (imu_lift_t * p_lift, const imu_lift_config_t * p_config)
{
    if ((NULL == p_lift) || (NULL == p_config) ||
        (!isfinite(p_config->rise_speed_mps)) ||
        (!isfinite(p_config->rise_m)) ||
        (!isfinite(p_config->speed_leak_s)) ||
        (!isfinite(p_config->rise_leak_s)) ||
        (!isfinite(p_config->baseline_s)) ||
        (p_config->rise_speed_mps <= 0.0f) || (p_config->rise_m <= 0.0f) ||
        (p_config->speed_leak_s <= 0.0f) || (p_config->rise_leak_s <= 0.0f) ||
        (p_config->baseline_s <= 0.0f) || (0U == p_config->hold_ms) ||
        (0U == p_config->max_gap_ms))
    {
        return false;
    }

    *p_lift        = (imu_lift_t){0};
    p_lift->config = *p_config;
    return true;
}

/** @brief Integrate one sample; detect, hold and re-arm the lift flag. */
bool
imu_lift_step (imu_lift_t * p_lift, const imu_data_t * p_data)
{
    float32_t magnitude = 0.0f;
    float32_t dt_s      = 0.0f;

    if ((NULL == p_lift) || (NULL == p_data))
    {
        return false;
    }

    p_lift->b_was_lifted = p_lift->b_lifted;
    expire_hold(p_lift, p_data->timestamp_ms);
    if (accept_sample(p_lift, p_data, &magnitude, &dt_s))
    {
        integrate(p_lift, magnitude - p_lift->baseline_ms2, dt_s);
        learn_baseline(p_lift, magnitude - p_lift->baseline_ms2, dt_s);
        detect(p_lift, dt_s, p_data->timestamp_ms);
    }
    return p_lift->b_lifted;
}

/** @brief True on the sample that detected the current lift. */
bool
imu_lift_started (const imu_lift_t * p_lift, const imu_data_t * p_data)
{
    return (NULL != p_lift) && (NULL != p_data) && (p_lift->b_lifted) &&
           (p_lift->lifted_ms == p_data->timestamp_ms);
}

/** @brief True when the last step changed the lifted flag either way. */
bool
imu_lift_changed (const imu_lift_t * p_lift)
{
    return (NULL != p_lift) && (p_lift->b_lifted != p_lift->b_was_lifted);
}

/** @brief Validate timing and data; settle the 1 g baseline first.
 *         True when the sample should be integrated. */
static bool
accept_sample (imu_lift_t * p_lift, const imu_data_t * p_data,
               float32_t * p_magnitude, float32_t * p_dt_s)
{
    uint32_t elapsed;

    if ((!p_data->b_valid) || (!isfinite(p_data->accel_x)) ||
        (!isfinite(p_data->accel_y)) || (!isfinite(p_data->accel_z)))
    {
        restart_integration(p_lift);
        return false;
    }

    *p_magnitude = sqrtf((p_data->accel_x * p_data->accel_x) +
                         (p_data->accel_y * p_data->accel_y) +
                         (p_data->accel_z * p_data->accel_z));
    if (!isfinite(*p_magnitude))
    {
        restart_integration(p_lift);
        return false;
    }

    elapsed = p_data->timestamp_ms - p_lift->last_ms;
    if ((!p_lift->b_have_sample) || (0U == elapsed) ||
        (elapsed > p_lift->config.max_gap_ms))
    {
        restart_integration(p_lift);
        p_lift->b_have_sample = true;
        p_lift->last_ms       = p_data->timestamp_ms;
        return false;
    }
    p_lift->last_ms = p_data->timestamp_ms;

    /* First IMU_LIFT_SETTLE_SAMPLES: the 1 g baseline is their average and
     * nothing is detected.  (Starting from one noisy sample left a fixed
     * offset that integrated into a false lift or hid real ones.) */
    if (p_lift->settle_count < IMU_LIFT_SETTLE_SAMPLES)
    {
        ++p_lift->settle_count;
        p_lift->baseline_ms2 += (*p_magnitude - p_lift->baseline_ms2) /
                                (float32_t)p_lift->settle_count;
        return false;
    }

    /* Elapsed is 1..max_gap_ms and converts to binary32 exactly. */
    *p_dt_s = (float32_t)elapsed / 1000.0f;
    return true;
}

/** @brief Leaky upward speed and rise from the excess over 1 g. */
static void
integrate (imu_lift_t * p_lift, float32_t excess, float32_t dt_s)
{
    p_lift->speed_mps +=
        (excess * dt_s) -
        (p_lift->speed_mps * dt_s / p_lift->config.speed_leak_s);
    p_lift->rise_m +=
        (p_lift->speed_mps * dt_s) -
        (p_lift->rise_m * dt_s / p_lift->config.rise_leak_s);
}

/** @brief Follow slow 1 g changes while nothing moves the car vertically.
 *
 * Learning only on the way down dragged the baseline low at every bump;
 * at most IMU_LIFT_LEARN_CLIP_MS2 per sample so a jolt cannot move it.
 */
static void
learn_baseline (imu_lift_t * p_lift, float32_t excess, float32_t dt_s)
{
    float32_t step = excess;

    if ((p_lift->b_lifted) ||
        (fabsf(p_lift->speed_mps) >= (p_lift->config.rise_speed_mps * 0.5f)))
    {
        return;
    }
    if (step > IMU_LIFT_LEARN_CLIP_MS2)
    {
        step = IMU_LIFT_LEARN_CLIP_MS2;
    }
    else if (step < -IMU_LIFT_LEARN_CLIP_MS2)
    {
        step = -IMU_LIFT_LEARN_CLIP_MS2;
    }
    else
    {
        /* Small differences are learned as they are. */
    }
    p_lift->baseline_ms2 += step * dt_s / p_lift->config.baseline_s;
}

/** @brief Lifted: rise plus a recent upward speed peak, for confirm_ms. */
static void
detect (imu_lift_t * p_lift, float32_t dt_s, uint32_t now_ms)
{
    /* The rise lags the speed (speed peaks while the hand accelerates the
     * car, the rise only builds up as it slows): compare the rise with the
     * recent upward speed peak, which fades at IMU_LIFT_PEAK_FADE_MPS2. */
    p_lift->recent_speed_mps -= IMU_LIFT_PEAK_FADE_MPS2 * dt_s;
    if (p_lift->speed_mps > p_lift->recent_speed_mps)
    {
        p_lift->recent_speed_mps = p_lift->speed_mps;
    }

    if ((p_lift->recent_speed_mps < p_lift->config.rise_speed_mps) ||
        (p_lift->rise_m < p_lift->config.rise_m))
    {
        p_lift->b_rising = false;
        return;
    }
    if (!p_lift->b_rising)
    {
        p_lift->b_rising  = true;
        p_lift->rising_ms = now_ms;
    }
    if ((!p_lift->b_lifted) &&
        ((now_ms - p_lift->rising_ms) >= p_lift->config.confirm_ms))
    {
        p_lift->b_lifted       = true;
        p_lift->lifted_ms      = now_ms;
        p_lift->peak_speed_mps = p_lift->recent_speed_mps;
        p_lift->peak_rise_m    = p_lift->rise_m;
    }
}

/** @brief Forget motion after invalid or late data; keep the baseline. */
static void
restart_integration (imu_lift_t * p_lift)
{
    p_lift->b_have_sample    = false;
    p_lift->speed_mps        = 0.0f;
    p_lift->rise_m           = 0.0f;
    p_lift->recent_speed_mps = 0.0f;
    p_lift->b_rising         = false;
}

/** @brief End the lifted report after hold_ms and re-arm detection. */
static void
expire_hold (imu_lift_t * p_lift, uint32_t now_ms)
{
    if ((p_lift->b_lifted) &&
        ((now_ms - p_lift->lifted_ms) >= p_lift->config.hold_ms))
    {
        p_lift->b_lifted = false;
        restart_integration(p_lift);
    }
}

/*** end of file ***/

