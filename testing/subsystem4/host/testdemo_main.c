/** @file testdemo_main.c
 * @brief Buddy 4 testdemo main module.
 */
#include "testdemo_main.h"
#include <assert.h>
#include <stdio.h>
#include "imu_analysis.h"
#include "imu_demo.h"
#include "imu_processing.h"

#define TEST_DURATION_MS 16000U
#define TEST_PERIOD_MS 10U

/** @brief Main. */
int
main (void)
{
    imu_analysis_t          analysis;
    imu_analysis_config_t   config;
    imu_data_t              data;
    imu_data_t              processed;
    imu_processing_t        processing;
    imu_processing_config_t processing_config;
    imu_processing_health_t health;
    imu_event_t             event;
    uint32_t                completions  = 0U;
    uint32_t                impacts      = 0U;
    uint32_t                faults       = 0U;
    uint32_t                recoveries   = 0U;
    bool                    b_climbing   = false;
    bool                    b_peak       = false;
    bool                    b_descending = false;

    imu_mock_init();
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
    imu_processing_default_config(&processing_config);
    assert(imu_processing_init(&processing, &processing_config));
    assert(IMU_MOCK_SUDDEN_IMPACT == imu_demo_scenario(7999U));
    assert(IMU_MOCK_STATIONARY == imu_demo_scenario(8000U));

    for (uint32_t elapsed_ms = 0U; elapsed_ms < TEST_DURATION_MS;
         elapsed_ms += TEST_PERIOD_MS)
    {
        imu_mock_set_scenario(imu_demo_scenario(elapsed_ms));
        data.b_valid      = imu_mock_read(&data);
        data.timestamp_ms = elapsed_ms;

        bool b_usable = imu_processing_process(&processing, &data, elapsed_ms,
                                               &processed, &health);

        faults += health.b_fault_started ? 1U : 0U;
        recoveries += health.b_recovered ? 1U : 0U;
        assert(imu_analysis_process(&analysis, &processed, &event) == b_usable);

        if (b_usable)
        {
            b_climbing =
                b_climbing || (IMU_MOTION_CLIMBING_HUMP == event.motion_state);
            b_peak = b_peak || (IMU_MOTION_HUMP_PEAK == event.motion_state);
            b_descending = b_descending ||
                           (IMU_MOTION_DESCENDING_HUMP == event.motion_state);
            completions += event.b_hump_completed ? 1U : 0U;
            impacts += event.b_sudden_impact ? 1U : 0U;
            assert(!event.b_height_valid);
        }
    }

    assert(b_climbing && b_peak && b_descending);
    assert(2U == completions);
    assert(200U == impacts);
    assert((2U == faults) && (2U == recoveries));
    puts("PASS: filtered demo produces two humps, impact intervals and fault "
         "recoveries");
    return 0;
}

/*** end of file ***/

