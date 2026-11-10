/** @file imu_mock.c
 * @brief Buddy 4 imu mock module.
 */
#include "imu_mock.h"

#include <stddef.h>
#include <math.h>

/* Acceleration due to gravity (m/s^2). */
#define IMU_GRAVITY_MS2 (9.81f)

/* Simulated sampling interval. */
#define IMU_MOCK_PERIOD_MS (10U)

static imu_mock_scenario_t g_current_scenario;
static uint32_t            g_mock_timestamp_ms;
static int32_t             g_mock_counts;

static void apply_scenario (imu_data_t * p_data);

/** @brief Reset the mock clock and select the stationary scenario. */
void
imu_mock_init (void)
{
    g_current_scenario  = IMU_MOCK_STATIONARY;
    g_mock_timestamp_ms = 0U;
    g_mock_counts = 0;
}

/** @brief Select the next synthetic motion or fault scenario. */
void
imu_mock_set_scenario (imu_mock_scenario_t scenario)
{
    g_current_scenario = scenario;
}

/** @brief Produce one synthetic reading and advance the mock clock. */
bool
imu_mock_read (imu_data_t * p_data)
{
    if (NULL == p_data)
    {
        return false;
    }

    /* Default stationary measurements. */
    *p_data              = (imu_data_t){0};
    p_data->timestamp_ms = g_mock_timestamp_ms;

    p_data->accel_x = 0.0f;
    p_data->accel_y = 0.0f;
    p_data->accel_z = IMU_GRAVITY_MS2;

    p_data->pitch_deg     = 0.0f;
    p_data->roll_deg      = 0.0f;
    p_data->turn_rate_dps = 0.0f;

    p_data->b_valid = true;
    /* Explicit synthetic travel is confined to this mock driver. */
    p_data->b_odometry_valid = true;
    p_data->encoder_left = p_data->encoder_right = g_mock_counts;
    p_data->motion.b_valid = true;
    p_data->motion.b_forward = true;
    p_data->motion.timestamp_ms = g_mock_timestamp_ms;
    p_data->b_turn_rate_valid =
        true; /* Mock explicitly supplies a synthetic rate. */
    p_data->b_magnetic_valid     = false;
    p_data->b_heading_valid      = false;
    p_data->b_heading_calibrated = false;
    p_data->magnetic_x_ut        = 0.0f;
    p_data->magnetic_y_ut        = 0.0f;
    p_data->magnetic_z_ut        = 0.0f;
    p_data->heading_deg          = 0.0f;

    apply_scenario(p_data);

    if (fabsf(p_data->roll_deg) > 0.0f)
    {
        p_data->accel_y = IMU_GRAVITY_MS2 *
            sinf(p_data->roll_deg * 0.01745329252f);
        p_data->accel_z = IMU_GRAVITY_MS2 *
            cosf(p_data->roll_deg * 0.01745329252f);
    }
    if ((p_data->motion.b_forward) && (g_mock_counts < INT32_MAX))
    {
        ++g_mock_counts;
    }
    g_mock_timestamp_ms += IMU_MOCK_PERIOD_MS;

    return p_data->b_valid;
}

/** @brief Apply the selected synthetic sensor and movement scenario. */
static void
apply_scenario (imu_data_t * p_data)
{
    switch (g_current_scenario)
    {
        case IMU_MOCK_STATIONARY:
            /* Robot is stationary on level ground. */
            p_data->motion.b_forward = false;
            p_data->motion.b_stopped = true;
        break;

        case IMU_MOCK_ACCELERATING:
            /* Robot accelerating forward. */
            p_data->accel_x = 2.0f;
        break;

        case IMU_MOCK_TURNING:
            /* Robot turning at 45 degrees/second. */
            p_data->turn_rate_dps = 45.0f;
            p_data->motion.b_forward = false;
            p_data->motion.b_turning = true;
        break;

        case IMU_MOCK_CLIMBING_HUMP:
            /* One side rises during the scripted one-second ascent. */
            p_data->roll_deg = 4.0f +
                8.0f * (float32_t)(g_mock_timestamp_ms % 1000U) / 1000.0f;
        break;

        case IMU_MOCK_HUMP_PEAK:
            /* Robot reaching the top of a hump. */
            p_data->roll_deg = 12.0f;
        break;

        case IMU_MOCK_DESCENDING_HUMP:
            /* The same side lowers toward level without changing sign. */
            p_data->roll_deg = 12.0f *
                (1.0f - (float32_t)(g_mock_timestamp_ms % 1000U) / 1000.0f);
        break;

        case IMU_MOCK_SUDDEN_IMPACT:
            /* Simulated sudden acceleration spike. */
            p_data->accel_x = 15.0f;
        break;

        case IMU_MOCK_NOISY_LEVEL:
            p_data->pitch_deg =
                ((g_mock_timestamp_ms / IMU_MOCK_PERIOD_MS) % 2U) ? 2.0f
                                                                  : -2.0f;
            p_data->roll_deg = -p_data->pitch_deg;
        break;
        case IMU_MOCK_READ_FAILURE:
            p_data->b_valid = false;
        break;
        case IMU_MOCK_NONFINITE:
            p_data->pitch_deg = NAN;
        break;
        default:
            p_data->b_valid = false;
        break;
    }

}

/*** end of file ***/

