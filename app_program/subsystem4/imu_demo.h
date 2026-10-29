#ifndef IMU_DEMO_H
#define IMU_DEMO_H

#include "imu_mock.h"

/* Pure sequence selection, shared by Pico task and host test. Cycle: 8 s. */
imu_mock_scenario_t imu_demo_scenario(uint32_t elapsed_ms);

#endif
