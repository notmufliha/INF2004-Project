#include "imu_mock.h"

#include <stddef.h>
#include <math.h>

/* Acceleration due to gravity (m/s^2) */
#define IMU_GRAVITY_MS2 (9.81f)

/* Simulated sampling interval */
#define IMU_MOCK_PERIOD_MS (10U)

static imu_mock_scenario_t current_scenario;
static uint32_t mock_timestamp_ms;

void imu_mock_init(void)
{
    current_scenario = IMU_MOCK_STATIONARY;
    mock_timestamp_ms = 0U;
}

void imu_mock_set_scenario(imu_mock_scenario_t scenario)
{
    current_scenario = scenario;
}

bool imu_mock_read(imu_data_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    /* Default stationary measurements */
    data->timestamp_ms = mock_timestamp_ms;

    data->accel_x = 0.0f;
    data->accel_y = 0.0f;
    data->accel_z = IMU_GRAVITY_MS2;

    data->pitch_deg = 0.0f;
    data->roll_deg = 0.0f;
    data->turn_rate_dps = 0.0f;

    data->valid = true;
    data->turn_rate_valid = true; /* Mock explicitly supplies a synthetic rate. */
    data->magnetic_valid = false;
    data->heading_valid = false;
    data->heading_calibrated = false;
    data->magnetic_x_ut = 0.0f;
    data->magnetic_y_ut = 0.0f;
    data->magnetic_z_ut = 0.0f;
    data->heading_deg = 0.0f;

    switch (current_scenario)
    {
        case IMU_MOCK_STATIONARY:
            /* Robot is stationary on level ground */
            break;

        case IMU_MOCK_ACCELERATING:
            /* Robot accelerating forward */
            data->accel_x = 2.0f;
            break;

        case IMU_MOCK_TURNING:
            /* Robot turning at 45 degrees/second */
            data->turn_rate_dps = 45.0f;
            break;

        case IMU_MOCK_CLIMBING_HUMP:
            /* Robot tilted upward */
            data->pitch_deg = 12.0f;
            break;

        case IMU_MOCK_HUMP_PEAK:
            /* Robot reaching the top of a hump */
            data->pitch_deg = 0.0f;
            break;

        case IMU_MOCK_DESCENDING_HUMP:
            /* Robot tilted downward */
            data->pitch_deg = -12.0f;
            break;

        case IMU_MOCK_SUDDEN_IMPACT:
            /* Simulated sudden acceleration spike */
            data->accel_x = 15.0f;
            break;

        case IMU_MOCK_NOISY_LEVEL:
            data->pitch_deg = ((mock_timestamp_ms / IMU_MOCK_PERIOD_MS) % 2U)
                              ? 2.0f : -2.0f;
            data->roll_deg = -data->pitch_deg;
            break;
        case IMU_MOCK_READ_FAILURE:
            data->valid = false;
            break;
        case IMU_MOCK_NONFINITE:
            data->pitch_deg = NAN;
            break;
        default:
            data->valid = false;
            break;
    }

    mock_timestamp_ms += IMU_MOCK_PERIOD_MS;

    return data->valid;
}
