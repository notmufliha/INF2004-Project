#include "imu_demo.h"

imu_mock_scenario_t imu_demo_scenario(uint32_t elapsed_ms)
{
    switch ((elapsed_ms % 8000U) / 1000U)
    {
        case 1U: return IMU_MOCK_ACCELERATING;
        case 2U: return IMU_MOCK_TURNING;
        case 3U: return IMU_MOCK_CLIMBING_HUMP;
        case 4U: return IMU_MOCK_HUMP_PEAK;
        case 5U: return IMU_MOCK_DESCENDING_HUMP;
        case 7U: return IMU_MOCK_SUDDEN_IMPACT;
        default: return IMU_MOCK_STATIONARY;
    }
}
