#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "imu_analysis.h"
#include "imu_mock.h"

static imu_analysis_t analysis;
static imu_analysis_config_t config;

static void reset(void)
{
    imu_mock_init();
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
}

static imu_event_t feed(imu_mock_scenario_t scenario, unsigned int count)
{
    imu_data_t data;
    imu_event_t event = {0};
    imu_mock_set_scenario(scenario);
    for (unsigned int i = 0U; i < count; ++i)
    {
        assert(imu_mock_read(&data));
        assert(imu_analysis_process(&analysis, &data, &event));
        assert(event.valid);
        assert(!event.height_valid);
        assert(event.hump_peak_cm == 0.0f);
        assert(event.highest_hump_cm == 0.0f);
    }
    return event;
}

static void test_classification(void)
{
    reset();
    assert(feed(IMU_MOCK_STATIONARY, 1U).motion_state == IMU_MOTION_UNKNOWN);
    assert(feed(IMU_MOCK_ACCELERATING, 1U).motion_state == IMU_MOTION_ACCELERATING);
    assert(feed(IMU_MOCK_TURNING, 1U).motion_state == IMU_MOTION_TURNING);
    assert(feed(IMU_MOCK_SUDDEN_IMPACT, 1U).sudden_impact);
    puts("PASS: motion classification and unavailable height");
}

static void test_hump(void)
{
    imu_event_t event;
    reset();
    assert(!feed(IMU_MOCK_HUMP_PEAK, 5U).hump_detected);
    assert(!feed(IMU_MOCK_CLIMBING_HUMP, 2U).hump_detected);
    assert(feed(IMU_MOCK_CLIMBING_HUMP, 1U).motion_state == IMU_MOTION_CLIMBING_HUMP);
    assert(feed(IMU_MOCK_HUMP_PEAK, 3U).motion_state == IMU_MOTION_HUMP_PEAK);
    assert(feed(IMU_MOCK_DESCENDING_HUMP, 3U).motion_state == IMU_MOTION_DESCENDING_HUMP);
    event = feed(IMU_MOCK_STATIONARY, 3U);
    assert(event.hump_completed && event.hump_detected);
    assert(!feed(IMU_MOCK_STATIONARY, 1U).hump_completed);
    /* A second hump may skip level readings at the peak. */
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(feed(IMU_MOCK_STATIONARY, 3U).hump_completed);
    puts("PASS: ordered traversal, single report and consecutive humps");
}

static void test_noise_and_impact(void)
{
    reset();
    for (unsigned int i = 0U; i < 10U; ++i)
    {
        assert(!feed(IMU_MOCK_CLIMBING_HUMP, 1U).hump_detected);
        assert(!feed(IMU_MOCK_STATIONARY, 1U).hump_detected);
    }
    assert(!feed(IMU_MOCK_DESCENDING_HUMP, 5U).hump_detected);
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(feed(IMU_MOCK_SUDDEN_IMPACT, 1U).motion_state == IMU_MOTION_SUDDEN_IMPACT);
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(!feed(IMU_MOCK_STATIONARY, 3U).hump_completed);
    puts("PASS: noise rejection, descent without climb and impact interruption");
}

static void test_validation(void)
{
    imu_data_t data;
    imu_event_t event;
    reset();
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(imu_mock_read(&data));
    data.valid = false;
    assert(!imu_analysis_process(&analysis, &data, &event));
    assert(!event.valid && event.motion_state == IMU_MOTION_UNKNOWN);
    data.valid = true;
    data.pitch_deg = NAN;
    assert(!imu_analysis_process(&analysis, &data, &event));
    assert(!imu_analysis_process(&analysis, NULL, &event));
    assert(!imu_analysis_process(NULL, &data, &event));
    assert(!imu_analysis_process(&analysis, &data, NULL));
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(!feed(IMU_MOCK_STATIONARY, 3U).hump_completed);
    config.confirmation_samples = 0U;
    assert(!imu_analysis_init(&analysis, &config));
    puts("PASS: invalid readings, null arguments and invalid configuration");
}

static void test_timing(void)
{
    imu_data_t data;
    imu_event_t event;
    reset();
    assert(imu_mock_read(&data));
    data.timestamp_ms = UINT32_MAX - 4U;
    assert(imu_analysis_process(&analysis, &data, &event));
    data.timestamp_ms = 5U;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(!imu_analysis_process(&analysis, &data, &event));
    data.timestamp_ms = 15U;
    assert(imu_analysis_process(&analysis, &data, &event));
    data.timestamp_ms = 14U;
    assert(!imu_analysis_process(&analysis, &data, &event));
    data.timestamp_ms = 20U;
    assert(imu_analysis_process(&analysis, &data, &event));
    data.timestamp_ms += config.max_gap_ms + 1U;
    assert(!imu_analysis_process(&analysis, &data, &event));
    reset();
    config.hump_timeout_ms = 120U;
    assert(imu_analysis_init(&analysis, &config));
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(!feed(IMU_MOCK_HUMP_PEAK, 12U).hump_detected);
    puts("PASS: timestamp wrap, duplicate/backward readings, gaps and timeout");
}

static void test_tilt_and_hysteresis(void)
{
    imu_data_t data;
    imu_event_t event;
    reset();
    assert(imu_mock_read(&data));
    /* Gravity expressed in robot axes for a stationary tilted sensor. */
    data.pitch_deg = 30.0f;
    data.accel_x = 4.905f;
    data.accel_z = 8.4957f;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(event.tilt_detected && !event.sudden_impact);
    assert(event.motion_state != IMU_MOTION_ACCELERATING);
    reset();
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(imu_mock_read(&data));
    data.pitch_deg = 5.0f; /* Between entry and exit thresholds. */
    for (unsigned int i = 0U; i < 5U; ++i)
    {
        assert(imu_analysis_process(&analysis, &data, &event));
        assert(event.motion_state == IMU_MOTION_CLIMBING_HUMP);
        data.timestamp_ms += 10U;
    }
    puts("PASS: tilted gravity and slope hysteresis");
}

int main(void)
{
    test_classification();
    test_hump();
    test_noise_and_impact();
    test_validation();
    test_timing();
    test_tilt_and_hysteresis();
    puts("All IMU analysis tests passed.");
    return 0;
}
