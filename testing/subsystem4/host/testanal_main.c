/** @file testanal_main.c
 * @brief Buddy 4 testanal main module.
 */
#include "testanal_main.h"
#include "imu_test_support.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "imu_analysis.h"
#include "imu_mock.h"

#define TEST_NOISE_CYCLES 10U
#define TEST_HYSTERESIS_SAMPLES 5U

static imu_analysis_t        g_analysis;
static imu_analysis_config_t g_config;

static void reset (void);

static imu_event_t feed (imu_mock_scenario_t scenario, uint32_t count);

static void test_classification (void);

static void test_hump (void);

static void test_noise_and_impact (void);

static void test_validation (void);

static void test_timing (void);

static void test_tilt_and_hysteresis (void);

/** @brief Main. */
int
main (void)
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

/** @brief Reset. */
static void
reset (void)
{
    imu_mock_init();
    imu_analysis_default_config(&g_config);
    assert(imu_analysis_init(&g_analysis, &g_config));
}

/** @brief Feed. */
static imu_event_t
feed (imu_mock_scenario_t scenario, uint32_t count)
{
    imu_data_t  data;
    imu_event_t event = {0};

    imu_mock_set_scenario(scenario);

    for (uint32_t index = 0U; index < count; ++index)
    {
        assert(imu_mock_read(&data));
        assert(imu_analysis_process(&g_analysis, &data, &event));
        assert(event.b_valid);
        assert(!event.b_height_valid);
        assert(imu_test_support_near(event.hump_peak_cm, 0.0f));
        assert(imu_test_support_near(event.highest_hump_cm, 0.0f));
    }

    return event;
}

/** @brief Test classification. */
static void
test_classification (void)
{
    reset();
    assert(IMU_MOTION_UNKNOWN == feed(IMU_MOCK_STATIONARY, 1U).motion_state);
    assert(IMU_MOTION_ACCELERATING ==
           feed(IMU_MOCK_ACCELERATING, 1U).motion_state);
    assert(IMU_MOTION_TURNING == feed(IMU_MOCK_TURNING, 1U).motion_state);
    assert(feed(IMU_MOCK_SUDDEN_IMPACT, 1U).b_sudden_impact);
    puts("PASS: motion classification and unavailable height");
}

/** @brief Test hump. */
static void
test_hump (void)
{
    imu_event_t event;

    reset();
    assert(!feed(IMU_MOCK_HUMP_PEAK, 5U).b_hump_detected);
    assert(!feed(IMU_MOCK_CLIMBING_HUMP, 2U).b_hump_detected);
    assert(IMU_MOTION_CLIMBING_HUMP ==
           feed(IMU_MOCK_CLIMBING_HUMP, 1U).motion_state);
    assert(IMU_MOTION_HUMP_PEAK == feed(IMU_MOCK_HUMP_PEAK, 3U).motion_state);
    assert(IMU_MOTION_DESCENDING_HUMP ==
           feed(IMU_MOCK_DESCENDING_HUMP, 3U).motion_state);
    event = feed(IMU_MOCK_STATIONARY, 3U);
    assert((event.b_hump_completed) && (event.b_hump_detected));
    assert(!feed(IMU_MOCK_STATIONARY, 1U).b_hump_completed);
    /* A second hump may skip level readings at the peak. */
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(feed(IMU_MOCK_STATIONARY, 3U).b_hump_completed);
    puts("PASS: ordered traversal, single report and consecutive humps");
}

/** @brief Test noise and impact. */
static void
test_noise_and_impact (void)
{
    reset();

    for (uint32_t index = 0U; index < TEST_NOISE_CYCLES; ++index)
    {
        assert(!feed(IMU_MOCK_CLIMBING_HUMP, 1U).b_hump_detected);
        assert(!feed(IMU_MOCK_STATIONARY, 1U).b_hump_detected);
    }

    assert(!feed(IMU_MOCK_DESCENDING_HUMP, 5U).b_hump_detected);
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(IMU_MOTION_SUDDEN_IMPACT ==
           feed(IMU_MOCK_SUDDEN_IMPACT, 1U).motion_state);
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(!feed(IMU_MOCK_STATIONARY, 3U).b_hump_completed);
    puts(
        "PASS: noise rejection, descent without climb and impact interruption");
}

/** @brief Test validation. */
static void
test_validation (void)
{
    imu_data_t  data;
    imu_event_t event;

    reset();
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(imu_mock_read(&data));
    data.b_valid = false;
    assert(!imu_analysis_process(&g_analysis, &data, &event));
    assert((!event.b_valid) && (IMU_MOTION_UNKNOWN == event.motion_state));
    data.b_valid   = true;
    data.pitch_deg = NAN;
    assert(!imu_analysis_process(&g_analysis, &data, &event));
    assert(!imu_analysis_process(&g_analysis, NULL, &event));
    assert(!imu_analysis_process(NULL, &data, &event));
    assert(!imu_analysis_process(&g_analysis, &data, NULL));
    feed(IMU_MOCK_DESCENDING_HUMP, 3U);
    assert(!feed(IMU_MOCK_STATIONARY, 3U).b_hump_completed);
    g_config.confirmation_samples = 0U;
    assert(!imu_analysis_init(&g_analysis, &g_config));
    puts("PASS: invalid readings, null arguments and invalid configuration");
}

/** @brief Test timing. */
static void
test_timing (void)
{
    imu_data_t  data;
    imu_event_t event;

    reset();
    assert(imu_mock_read(&data));
    data.timestamp_ms = UINT32_MAX - 4U;
    assert(imu_analysis_process(&g_analysis, &data, &event));
    data.timestamp_ms = 5U;
    assert(imu_analysis_process(&g_analysis, &data, &event));
    assert(!imu_analysis_process(&g_analysis, &data, &event));
    data.timestamp_ms = 15U;
    assert(imu_analysis_process(&g_analysis, &data, &event));
    data.timestamp_ms = 14U;
    assert(!imu_analysis_process(&g_analysis, &data, &event));
    data.timestamp_ms = 20U;
    assert(imu_analysis_process(&g_analysis, &data, &event));
    data.timestamp_ms += g_config.max_gap_ms + 1U;
    assert(!imu_analysis_process(&g_analysis, &data, &event));
    reset();
    g_config.hump_timeout_ms = 120U;
    assert(imu_analysis_init(&g_analysis, &g_config));
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(!feed(IMU_MOCK_HUMP_PEAK, 12U).b_hump_detected);
    puts("PASS: timestamp wrap, duplicate/backward readings, gaps and timeout");
}

/** @brief Test tilt and hysteresis. */
static void
test_tilt_and_hysteresis (void)
{
    imu_data_t  data;
    imu_event_t event;

    reset();
    assert(imu_mock_read(&data));
    /* Gravity expressed in robot axes for a stationary tilted sensor. */
    data.pitch_deg = 30.0f;
    data.accel_x   = 4.905f;
    data.accel_z   = 8.4957f;
    assert(imu_analysis_process(&g_analysis, &data, &event));
    assert((event.b_tilt_detected) && (!event.b_sudden_impact));
    assert(event.motion_state != IMU_MOTION_ACCELERATING);
    reset();
    feed(IMU_MOCK_CLIMBING_HUMP, 3U);
    assert(imu_mock_read(&data));
    data.pitch_deg = 5.0f; /* Between entry and exit thresholds. */

    for (uint32_t index = 0U; index < TEST_HYSTERESIS_SAMPLES; ++index)
    {
        assert(imu_analysis_process(&g_analysis, &data, &event));
        assert(IMU_MOTION_CLIMBING_HUMP == event.motion_state);
        data.timestamp_ms += 10U;
    }

    puts("PASS: tilted gravity and slope hysteresis");
}

/*** end of file ***/

