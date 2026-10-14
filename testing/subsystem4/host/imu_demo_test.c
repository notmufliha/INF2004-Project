#include <assert.h>
#include <stdio.h>
#include "imu_analysis.h"
#include "imu_demo.h"

int main(void)
{
    imu_analysis_t analysis;
    imu_analysis_config_t config;
    imu_data_t data;
    imu_event_t event;
    unsigned int completions = 0U;
    unsigned int impacts = 0U;
    bool climbing = false, peak = false, descending = false;

    imu_mock_init();
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
    assert(imu_demo_scenario(7999U) == IMU_MOCK_SUDDEN_IMPACT);
    assert(imu_demo_scenario(8000U) == IMU_MOCK_STATIONARY);
    for (uint32_t ms = 0U; ms < 16000U; ms += 10U) {
        imu_mock_set_scenario(imu_demo_scenario(ms));
        assert(imu_mock_read(&data));
        data.timestamp_ms = ms;
        assert(imu_analysis_process(&analysis, &data, &event));
        climbing |= event.motion_state == IMU_MOTION_CLIMBING_HUMP;
        peak |= event.motion_state == IMU_MOTION_HUMP_PEAK;
        descending |= event.motion_state == IMU_MOTION_DESCENDING_HUMP;
        completions += event.hump_completed ? 1U : 0U;
        impacts += event.sudden_impact ? 1U : 0U;
        assert(!event.height_valid);
    }
    assert(climbing && peak && descending);
    assert(completions == 2U);
    assert(impacts == 200U);
    puts("PASS: repeating demo produces two ordered humps and impact intervals");
    return 0;
}
