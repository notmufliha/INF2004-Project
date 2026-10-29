#include <assert.h>
#include <stdio.h>
#include "imu_analysis.h"
#include "imu_demo.h"
#include "imu_processing.h"

int main(void)
{
    imu_analysis_t analysis;
    imu_analysis_config_t config;
    imu_data_t data;
    imu_data_t processed;
    imu_processing_t processing;
    imu_processing_config_t processing_config;
    imu_health_t health;
    imu_event_t event;
    unsigned int completions = 0U;
    unsigned int impacts = 0U;
    unsigned int faults = 0U, recoveries = 0U;
    bool climbing = false, peak = false, descending = false;

    imu_mock_init();
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
    imu_processing_default_config(&processing_config);
    assert(imu_processing_init(&processing, &processing_config));
    assert(imu_demo_scenario(7999U) == IMU_MOCK_SUDDEN_IMPACT);
    assert(imu_demo_scenario(8000U) == IMU_MOCK_STATIONARY);
    for (uint32_t ms = 0U; ms < 16000U; ms += 10U) {
        imu_mock_set_scenario(imu_demo_scenario(ms));
        data.valid = imu_mock_read(&data);
        data.timestamp_ms = ms;
        bool usable = imu_processing_process(&processing, &data, ms, &processed, &health);
        faults += health.fault_started ? 1U : 0U;
        recoveries += health.recovered ? 1U : 0U;
        assert(imu_analysis_process(&analysis, &processed, &event) == usable);
        if (!usable) { continue; }
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
    assert(faults == 2U && recoveries == 2U);
    puts("PASS: filtered demo produces two humps, impact intervals and fault recoveries");
    return 0;
}
