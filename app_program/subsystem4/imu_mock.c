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

/** @brief Reset the mock clock and select the stationary scenario. */
void
imu_mock_init (void)
{
    g_current_scenario  = IMU_MOCK_STATIONARY;
    g_mock_timestamp_ms = 0U;
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
    p_data->timestamp_ms = g_mock_timestamp_ms;

    p_data->accel_x = 0.0f;
    p_data->accel_y = 0.0f;
    p_data->accel_z = IMU_GRAVITY_MS2;

    p_data->pitch_deg     = 0.0f;
    p_data->roll_deg      = 0.0f;
    p_data->turn_rate_dps = 0.0f;

    p_data->b_valid = true;
    p_data->b_turn_rate_valid =
        true; /* Mock explicitly supplies a synthetic rate. */
    p_data->b_magnetic_valid     = false;
    p_data->b_heading_valid      = false;
    p_data->b_heading_calibrated = false;
    p_data->magnetic_x_ut        = 0.0f;
    p_data->magnetic_y_ut        = 0.0f;
    p_data->magnetic_z_ut        = 0.0f;
    p_data->heading_deg          = 0.0f;

    switch (g_current_scenario)
    {
        case IMU_MOCK_STATIONARY:
            /* Robot is stationary on level ground. */
        break;

        case IMU_MOCK_ACCELERATING:
            /* Robot accelerating forward. */
            p_data->accel_x = 2.0f;
        break;

        case IMU_MOCK_TURNING:
            /* Robot turning at 45 degrees/second. */
            p_data->turn_rate_dps = 45.0f;
        break;

        case IMU_MOCK_CLIMBING_HUMP:
            /* Robot tilted upward. */
            p_data->pitch_deg = 12.0f;
        break;

        case IMU_MOCK_HUMP_PEAK:
            /* Robot reaching the top of a hump. */
            p_data->pitch_deg = 0.0f;
        break;

        case IMU_MOCK_DESCENDING_HUMP:
            /* Robot tilted downward. */
            p_data->pitch_deg = -12.0f;
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

    g_mock_timestamp_ms += IMU_MOCK_PERIOD_MS;

    return p_data->b_valid;
}

/*** end of file ***/

