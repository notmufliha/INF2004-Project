#ifndef IMU_MOCK_H
#define IMU_MOCK_H

#include "imu_types.h"

/* Simulated robot motion scenarios */
typedef enum
{
    IMU_MOCK_STATIONARY = 0,
    IMU_MOCK_ACCELERATING,
    IMU_MOCK_TURNING,
    IMU_MOCK_CLIMBING_HUMP,
    IMU_MOCK_HUMP_PEAK,
    IMU_MOCK_DESCENDING_HUMP,
    IMU_MOCK_SUDDEN_IMPACT,
    IMU_MOCK_NOISY_LEVEL,
    IMU_MOCK_READ_FAILURE,
    IMU_MOCK_NONFINITE
} imu_mock_scenario_t;

/* Initialise the mock IMU */
void imu_mock_init(void);

/* Select the motion scenario to simulate */
void imu_mock_set_scenario(imu_mock_scenario_t scenario);

/* Generate one simulated IMU reading */
bool imu_mock_read(imu_data_t *data);

#endif /* IMU_MOCK_H */
