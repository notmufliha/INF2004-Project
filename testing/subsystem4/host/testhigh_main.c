/** @file testhigh_main.c
 * @brief Verify height integration against analytic and rejected traversals.
 */
#include "testhigh_main.h"
#include "imu_height.h"
#include "imu_analysis.h"
#include "imu_processing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_ASCENT_STEPS 20U
#define TEST_SHORT_STEPS 10U
#define TEST_LONG_STEPS 30U
#define TEST_BAD_STEPS 8U

static imu_height_t g_height;
static imu_data_t   g_data;
static imu_event_t  g_event;
static bool         gb_require_motion;

static void reset (void);
static void prepare_reading (float32_t pitch, int32_t counts);
static void feed (float32_t pitch, int32_t counts, bool b_detected,
                  bool b_completed);
static void profile (uint32_t steps);
static void test_rejections (void);
static void test_pipeline (void);
static void test_guards (void);
static void test_motion_context (void);

/** @brief Check analytic heights, session maximum and failure recovery. */
int
main (void)
{
    reset();
    profile(TEST_ASCENT_STEPS);
    assert(g_event.b_height_valid);
    assert(fabsf(g_event.hump_peak_cm - 10.0f) < 0.001f);
    assert(g_event.b_highest_hump_valid);
    profile(TEST_SHORT_STEPS);
    assert(g_event.b_height_valid);
    assert(fabsf(g_event.hump_peak_cm - 5.0f) < 0.001f);
    assert(fabsf(g_event.highest_hump_cm - 10.0f) < 0.001f);
    profile(TEST_LONG_STEPS);
    assert(g_event.b_height_valid);
    assert(fabsf(g_event.highest_hump_cm - 15.0f) < 0.001f);
    g_data.b_odometry_valid = false;
    feed(0.0f, 0, false, false);
    assert(!g_event.b_height_valid);
    assert(g_event.b_highest_hump_valid);
    assert(fabsf(g_event.highest_hump_cm - 15.0f) < 0.001f);
    puts("PASS: analytic heights, consecutive humps and persistent maximum");
    test_rejections();
    test_pipeline();
    test_guards();
    test_motion_context();
    return 0;
}

/** @brief Start level with geometry where 100 counts represents one cm. */
static void
reset (void)
{
    imu_height_config_t config;

    imu_height_default_config(&config);
    assert(!imu_height_init(&g_height, &config));
    config.left_cm_per_count  = 0.01f;
    config.right_cm_per_count = 0.01f;
    config.b_require_motion   = gb_require_motion;
    assert(imu_height_init(&g_height, &config));
    g_data                  = (imu_data_t){0};
    g_data.b_valid          = true;
    g_data.b_odometry_valid = true;
    feed(0.0f, 0, false, false);
    feed(0.0f, 0, false, false);
}

/** @brief Generate paired counts and gravity-only tilt without analysis. */
static void
prepare_reading (float32_t pitch, int32_t counts)
{
    g_data.timestamp_ms += 10U;
    g_data.sample_sequence++;

    if (gb_require_motion)
    {
        g_data.motion              = (imu_motion_t){0};
        g_data.motion.timestamp_ms = g_data.timestamp_ms;
        g_data.motion.b_valid      = true;
        g_data.motion.b_forward    = counts > 0;
        g_data.motion.b_stopped    = 0 == counts;
    }

    g_data.encoder_left += counts;
    g_data.encoder_right += counts;
    g_data.pitch_deg = pitch;
    g_data.accel_x   = -9.81f * sinf(pitch * 0.01745329252f);
    g_data.accel_z   = 9.81f * cosf(pitch * 0.01745329252f);
    g_event          = (imu_event_t){0};
    g_event.b_valid  = true;
}

/** @brief Move a deterministic pair of encoders and generate gravity-only tilt.
 */
static void
feed (float32_t pitch, int32_t counts, bool b_detected, bool b_completed)
{
    prepare_reading(pitch, counts);
    g_event.b_hump_detected  = b_detected;
    g_event.b_hump_completed = b_completed;
    imu_height_process(&g_height, &g_data, &g_event);
}

/** @brief Traverse equal 30-degree slopes with level transitions at both ends.
 */
static void
profile (uint32_t steps)
{
    feed(0.0f, 0, false, false);
    feed(0.0f, 0, false, false);

    for (uint32_t index = 0U; index < steps; ++index)
    {
        /* Delayed detection must not lose the first two rising samples. */
        feed(30.0f, 100, index >= 2U, false);
    }

    feed(0.0f, 100, true, false);

    for (uint32_t index = 0U; index < steps; ++index)
    {
        feed(-30.0f, 100, true, false);
    }

    feed(0.0f, 100, true, true);
}

/** @brief Invalidate errors, reversals, pivots, gaps, jumps and poor closure.
 */
static void
test_rejections (void)
{
    for (uint32_t scenario = 0U; scenario < TEST_BAD_STEPS; ++scenario)
    {
        reset();
        feed(30.0f, 100, true, false);

        switch (scenario)
        {
            case 0U:
                g_data.encoder_left_errors++;
            break;
            case 1U:
                g_data.b_odometry_valid = false;
            break;
            case 2U:
                g_data.timestamp_ms += 101U;
            break;
            case 3U:
                g_data.encoder_left -= 200;
                g_data.encoder_right -= 200;
            break;
            case 4U:
                g_data.encoder_left += 1000;
            break;
            case 5U:
                g_data.encoder_left = INT32_MIN;
            break;
            case 6U:
                g_data.roll_deg = 20.0f;
            break;
            default:
                g_data.b_valid = false;
            break;
        }

        feed(30.0f, 100, true, false);
        g_data.b_odometry_valid = true;
        g_data.b_valid          = true;
        g_data.roll_deg         = 0.0f;
        feed(-30.0f, 100, true, false);
        feed(0.0f, 100, true, true);
        assert(!g_event.b_height_valid);
        assert(!g_event.b_highest_hump_valid);
        profile(TEST_SHORT_STEPS);
        assert(g_event.b_height_valid);
    }

    reset();
    feed(30.0f, 100, true, false);
    feed(30.0f, 100, true, false);
    feed(0.0f, 100, true, true);
    assert(!g_event.b_height_valid);
    reset();
    feed(30.0f, 0, true, false);
    feed(-30.0f, 0, true, false);
    feed(0.0f, 0, true, true);
    assert(!g_event.b_height_valid);
    puts("PASS: traversal faults, recovery, stationary tilt and closure "
         "rejection");
}

/** @brief Run the actual terrain classifier together with the height module. */
static void
test_pipeline (void)
{
    imu_analysis_t          analysis;
    imu_analysis_config_t   config;
    imu_processing_t        processing;
    imu_processing_config_t processing_config;
    imu_processing_health_t health;
    imu_data_t              processed;

    reset();
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
    imu_processing_default_config(&processing_config);
    assert(imu_processing_init(&processing, &processing_config));

    const uint32_t total_samples = 70U;
    bool           b_completed   = false;

    for (uint32_t index = 0U; index < total_samples; ++index)
    {
        float32_t pitch = 0.0f;

        if ((index >= 5U) && (index < 25U))
        {
            pitch = 30.0f;
        }
        else if ((index >= 30U) && (index < 50U))
        {
            pitch = -30.0f;
        }
        else
        {
            /* Level before, between and after the two slopes. */
        }

        prepare_reading(pitch, 100);
        assert(imu_processing_process(&processing, &g_data, g_data.timestamp_ms,
                                      &processed, &health));
        assert(imu_analysis_process(&analysis, &processed, &g_event));
        imu_height_process(&g_height, &g_data, &g_event);

        if (g_event.b_hump_completed)
        {
            assert(g_event.b_height_valid);
            assert(fabsf(g_event.hump_peak_cm - 10.0f) < 0.001f);
            b_completed = true;
        }
    }

    assert(b_completed);
    puts(
        "PASS: actual terrain classifier produces a validated height estimate");
}

/** @brief Check unavailable inputs, clock wrap and interruption by impact. */
static void
test_guards (void)
{
    reset();
    g_data.timestamp_ms = UINT32_MAX - 50U;
    profile(TEST_ASCENT_STEPS);
    assert(g_event.b_height_valid);
    assert(fabsf(g_event.hump_peak_cm - 10.0f) < 0.001f);
    reset();
    feed(30.0f, 100, true, false);
    g_data.sample_sequence++;
    feed(30.0f, 100, true, false);
    feed(-30.0f, 100, true, false);
    feed(0.0f, 100, true, true);
    assert(!g_event.b_height_valid);
    reset();
    feed(30.0f, 100, true, false);
    prepare_reading(30.0f, 100);
    g_event.b_hump_detected = true;
    g_event.b_sudden_impact = true;
    imu_height_process(&g_height, &g_data, &g_event);
    feed(-30.0f, 100, true, false);
    feed(0.0f, 100, true, true);
    assert(!g_event.b_height_valid);
    profile(TEST_SHORT_STEPS);
    assert(g_event.b_height_valid);
    prepare_reading(NAN, 100);
    imu_height_process(&g_height, &g_data, &g_event);
    assert(!g_event.b_height_valid);
    assert(g_event.b_highest_hump_valid);
    imu_height_process(NULL, &g_data, &g_event);
    assert(!g_event.b_height_valid);
    assert(!g_event.b_highest_hump_valid);
    imu_height_process(&g_height, NULL, &g_event);
    imu_height_process(&g_height, &g_data, NULL);
    puts("PASS: impact, nonfinite/null guards and timestamp wrap");
    printf("INFO: reading=%zu event=%zu height-context=%zu bytes\n",
           sizeof(imu_data_t), sizeof(imu_event_t), sizeof(imu_height_t));
}

/** @brief Reject unsafe/unavailable B2 context without losing session maxima.
 */
static void
test_motion_context (void)
{
    const uint32_t scenarios = 6U;

    gb_require_motion = true;

    for (uint32_t scenario = 0U; scenario < scenarios; ++scenario)
    {
        reset();
        profile(TEST_ASCENT_STEPS);
        assert(g_event.b_height_valid);
        feed(30.0f, 100, true, false);
        prepare_reading(30.0f, 100);

        switch (scenario)
        {
            case 0U:
                g_data.motion.b_valid = false;
            break;
            case 1U:
                g_data.motion.timestamp_ms -= IMU_MOTION_MAX_AGE_MS + 1U;
            break;
            case 2U:
                g_data.motion.b_reverse = true;
                g_data.motion.b_forward = false;
            break;
            case 3U:
                g_data.motion.b_turning = true;
                g_data.motion.b_forward = false;
            break;
            case 4U:
                g_data.motion.b_fault = true;
            break;
            default:
                g_data.motion.b_forward = false;
                g_data.motion.b_stopped = true;
            break;
        }

        g_event.b_hump_detected = true;
        imu_height_process(&g_height, &g_data, &g_event);
        feed(-30.0f, 100, true, false);
        feed(0.0f, 100, true, true);
        assert(!g_event.b_height_valid);
        assert(g_event.b_highest_hump_valid);
        assert(fabsf(g_event.highest_hump_cm - 10.0f) < 0.001f);
        profile(TEST_SHORT_STEPS);
        assert(g_event.b_height_valid);
    }

    gb_require_motion = false;
    puts("PASS: B2 freshness/direction/fault gating and height recovery");
}

/*** end of file ***/

