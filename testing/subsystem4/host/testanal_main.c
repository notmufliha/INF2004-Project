/** @file testanal_main.c
 * @brief Verify roll-only hump and motion analysis with paired travel.
 */
#include "testanal_main.h"
#include "imu_test_support.h"
#include "imu_analysis.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_SHORT_BASELINE 4U
#define TEST_SHORT_SETTLE 3U
#define TEST_SHORT_NOISE 8U
#define TEST_ROLL_SETTLED 6U

static imu_analysis_t g_analysis;
static imu_analysis_config_t g_config;
static imu_data_t g_short_data;

static void short_reset (void);
static imu_event_t short_feed (float32_t pitch, uint32_t count);
static imu_event_t roll_feed (float32_t roll, uint32_t count);
static imu_event_t hold_feed (float32_t pitch, uint32_t elapsed);
static void test_general_motion (void);
static void test_validation_and_timing (void);
static void test_progress_pause (void);
static void test_movement_reasons (void);
static void test_roll_crossings (void);
static void test_roll_guards (void);
static void test_roll_pitch_independence (void);
static void test_roll_motion_trends (void);

/** @brief Run classification, timing, movement and roll regressions. */
int
main (void)
{
    test_general_motion();
    test_validation_and_timing();
    test_progress_pause();
    test_movement_reasons();
    test_roll_crossings();
    test_roll_guards();
    test_roll_pitch_independence();
    test_roll_motion_trends();
    puts("All roll-only IMU analysis tests passed.");
    return 0;
}

/** @brief Retain acceleration, rate, impact, tilt and pitch measurements. */
static void
test_general_motion (void)
{
    short_reset();
    g_short_data.motion.b_forward = false;
    g_short_data.motion.b_stopped = true;
    assert(IMU_MOTION_STATIONARY == hold_feed(0.0f, 10U).motion_state);
    short_reset();
    g_short_data.accel_x = 2.0f;
    assert(IMU_MOTION_ACCELERATING == hold_feed(0.0f, 10U).motion_state);
    g_short_data.b_turn_rate_valid = true;
    g_short_data.turn_rate_dps = 45.0f;
    assert(IMU_MOTION_TURNING == hold_feed(0.0f, 10U).motion_state);
    g_short_data.accel_x = 15.0f;
    assert(hold_feed(0.0f, 10U).b_sudden_impact);
    short_reset();
    imu_event_t event = short_feed(25.0f, 1U);
    assert(event.b_tilt_detected);
    assert(imu_test_support_near(event.pitch_deg, 25.0f));
    assert(!event.b_hump_detected);
    assert(IMU_MOTION_CLIMBING_HUMP != event.motion_state);
    puts("PASS: stationary, acceleration, rate, impact and retained pitch");
}

/** @brief Invalid samples and timing cannot preserve a crossing. */
static void
test_validation_and_timing (void)
{
    imu_event_t event;
    short_reset();
    g_short_data.timestamp_ms = UINT32_MAX - 30U;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    assert(!imu_analysis_process(&g_analysis, &g_short_data, &event));
    assert(IMU_ANALYSIS_RESET_TIMING == g_analysis.reset_reasons);
    assert(IMU_ROLL_UNARMED == g_analysis.roll_state);
    roll_feed(0.0f, 1U);
    --g_short_data.timestamp_ms;
    assert(!imu_analysis_process(&g_analysis, &g_short_data, &event));
    roll_feed(0.0f, 1U);
    g_short_data.timestamp_ms += g_config.max_gap_ms + 1U;
    assert(!imu_analysis_process(&g_analysis, &g_short_data, &event));
    g_short_data.pitch_deg = NAN;
    assert(!imu_analysis_process(&g_analysis, &g_short_data, &event));
    assert(!imu_analysis_process(&g_analysis, NULL, &event));
    assert(!imu_analysis_process(NULL, &g_short_data, &event));
    assert(!imu_analysis_process(&g_analysis, &g_short_data, NULL));
    g_config.confirmation_samples = 0U;
    assert(!imu_analysis_init(&g_analysis, &g_config));
    puts("PASS: null/nonfinite, duplicate/backward/gap timing and wrap");
}

/** @brief Start an encoder-backed detector with gravity-only readings. */
static void
short_reset (void)
{
    imu_analysis_default_config(&g_config);
    assert(imu_analysis_init(&g_analysis, &g_config));
    g_analysis.b_require_motion = true;
    g_short_data = (imu_data_t){0};
    g_short_data.b_valid = true;
    g_short_data.accel_z = 9.81f;
    g_short_data.b_odometry_valid = true;
    g_short_data.motion.b_valid = true;
    g_short_data.motion.b_forward = true;
}

/** @brief Feed short slopes at 20ms intervals with paired forward travel. */
static imu_event_t
short_feed (float32_t pitch, uint32_t count)
{
    imu_event_t event = {0};

    for (uint32_t index = 0U; index < count; ++index)
    {
        g_short_data.timestamp_ms += 20U;
        g_short_data.motion.timestamp_ms = g_short_data.timestamp_ms;
        g_short_data.encoder_left += 10;
        g_short_data.encoder_right += 10;
        g_short_data.pitch_deg = pitch;
        g_short_data.accel_x = -9.81f * sinf(pitch * 0.01745329252f);
        g_short_data.accel_y = 9.81f * cosf(pitch * 0.01745329252f) *
            sinf(g_short_data.roll_deg * 0.01745329252f);
        g_short_data.accel_z = 9.81f * cosf(pitch * 0.01745329252f) *
            cosf(g_short_data.roll_deg * 0.01745329252f);
        assert(imu_analysis_process(&g_analysis, &g_short_data, &event));
    }

    return event;
}

/** @brief A fresh forward snapshot can legitimately contain no new pulses. */
static imu_event_t
hold_feed (float32_t pitch, uint32_t elapsed)
{
    imu_event_t event;

    g_short_data.timestamp_ms += elapsed;
    g_short_data.motion.timestamp_ms = g_short_data.timestamp_ms;
    g_short_data.pitch_deg = pitch;
    assert(imu_analysis_process(&g_analysis, &g_short_data, &event));
    return event;
}

/** @brief Holds preserve stages but cannot confirm slope or completion. */
static void
test_progress_pause (void)
{
    short_reset();
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    hold_feed(-6.0f, 50U);
    hold_feed(-6.0f, 50U);
    assert(IMU_TERRAIN_CLIMBING == g_analysis.roll_terrain);
    assert(0U == g_analysis.reset_reasons);
    roll_feed(4.0f, 1U);
    roll_feed(3.0f, 1U);
    assert(IMU_TERRAIN_DESCENDING == g_analysis.roll_terrain);
    g_short_data.roll_deg = 0.0f;
    assert(!hold_feed(0.0f, 50U).b_hump_completed);
    assert(!hold_feed(0.0f, 50U).b_hump_completed);
    assert(roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);

    short_reset();
    g_short_data.timestamp_ms = UINT32_MAX - 150U;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    hold_feed(6.0f, 99U);
    hold_feed(6.0f, 100U);
    assert(IMU_TERRAIN_CLIMBING == g_analysis.roll_terrain);
    assert(0U == g_analysis.reset_reasons);
    hold_feed(6.0f, 1U);
    assert(IMU_TERRAIN_LEVEL == g_analysis.roll_terrain);
    assert(IMU_ANALYSIS_RESET_NO_PROGRESS == g_analysis.reset_reasons);
    roll_feed(-6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    assert(!roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);

    short_reset();
    g_analysis.config.hump_timeout_ms = 120U;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    hold_feed(6.0f, 100U);
    assert(IMU_TERRAIN_CLIMBING == g_analysis.roll_terrain);
    hold_feed(6.0f, 20U);
    assert(IMU_ANALYSIS_RESET_TIMEOUT == g_analysis.reset_reasons);
    puts("PASS: no-pulse holds, resumed crossing, 199/200ms timeout, wrap "
         "and terrain timeout during hold");
}

/** @brief Genuine bad movement context still cancels with a precise cause. */
static void
test_movement_reasons (void)
{
    const uint32_t reasons[] = {
        IMU_ANALYSIS_RESET_ODOMETRY, IMU_ANALYSIS_RESET_MOTION,
        IMU_ANALYSIS_RESET_STALE, IMU_ANALYSIS_RESET_B2_FAULT,
        IMU_ANALYSIS_RESET_REVERSE, IMU_ANALYSIS_RESET_TURNING,
        IMU_ANALYSIS_RESET_STOPPED, IMU_ANALYSIS_RESET_NOT_FORWARD};
    const uint32_t cases = sizeof(reasons) / sizeof(reasons[0]);

    for (uint32_t index = 0U; index < cases; ++index)
    {
        short_reset();
        roll_feed(0.0f, TEST_SHORT_BASELINE);
        roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
        g_short_data.timestamp_ms += 10U;
        g_short_data.motion.timestamp_ms = g_short_data.timestamp_ms;
        switch (index)
        {
            case 0U: g_short_data.b_odometry_valid = false;
            break;
            case 1U: g_short_data.motion.b_valid = false;
            break;
            case 2U:
                g_short_data.motion.timestamp_ms -=
                    IMU_MOTION_MAX_AGE_MS + 1U;
            break;
            case 3U: g_short_data.motion.b_fault = true;
            break;
            case 4U: --g_short_data.encoder_left;
            break;
            case 5U: g_short_data.motion.b_turning = true;
            break;
            case 6U: g_short_data.motion.b_stopped = true;
            break;
            default: g_short_data.motion.b_forward = false;
            break;
        }
        imu_event_t event;
        assert(imu_analysis_process(&g_analysis, &g_short_data, &event));
        assert(IMU_TERRAIN_LEVEL == g_analysis.roll_terrain);
        assert(reasons[index] == g_analysis.reset_reasons);
        assert(!event.b_hump_completed);
    }
    puts("PASS: separate odometry, context, stale, fault, reverse, turn, "
         "stopped and not-forward cancellations");
}

/** @brief Set sideways tilt while retaining paired forward travel. */
static imu_event_t
roll_feed (float32_t roll, uint32_t count)
{
    g_short_data.roll_deg = roll;
    return short_feed(0.0f, count);
}

/** @brief Both wheel sides require an excursion and stable return, once. */
static void
test_roll_crossings (void)
{
    const float32_t sides[] = {1.0f, -1.0f};
    const uint32_t cases = sizeof(sides) / sizeof(sides[0]);

    for (uint32_t index = 0U; index < cases; ++index)
    {
        short_reset();
            roll_feed(0.0f, TEST_SHORT_BASELINE);
        assert(!roll_feed(sides[index] * 6.0f, 1U).b_hump_detected);
        assert(roll_feed(sides[index] * 6.0f, 1U).b_hump_detected);
        assert(IMU_ROLL_ACTIVE == g_analysis.roll_state);
        assert(!roll_feed(sides[index] * 6.0f, 2U).b_hump_completed);
        assert(!roll_feed(sides[index] * 3.0f, 2U).b_hump_completed);
        assert(!roll_feed(0.0f, 2U).b_hump_completed);
        imu_event_t event = roll_feed(0.0f, 1U);
        assert(event.b_hump_completed);
        assert(!event.b_height_valid);
        assert(!roll_feed(0.0f, TEST_ROLL_SETTLED).b_hump_completed);
        roll_feed(-sides[index] * 6.0f, IMU_ROLL_CONFIRM_SAMPLES);
        assert(roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    }
    short_reset();
    g_analysis.config.roll_enter_deg = 8.0f;
    g_analysis.config.roll_level_deg = 1.0f;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    assert(!roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES).b_hump_detected);
    assert(roll_feed(9.0f, IMU_ROLL_CONFIRM_SAMPLES).b_hump_detected);
    assert(!roll_feed(2.0f, TEST_SHORT_SETTLE).b_hump_completed);
    assert(roll_feed(1.0f, TEST_SHORT_SETTLE).b_hump_completed);
    puts("PASS: either-wheel roll, hysteresis, stable return, single event, "
         "rearm and unavailable height");
}

/** @brief Noise, missing baselines, pauses and faults cannot fake crossings. */
static void
test_roll_guards (void)
{
    short_reset();
    assert(!roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES).b_hump_detected);
    assert(!roll_feed(0.0f, TEST_SHORT_BASELINE).b_hump_completed);
    for (uint32_t index = 0U; index < TEST_SHORT_NOISE; ++index)
    {
        assert(!roll_feed(6.0f, 1U).b_hump_detected);
        assert(!roll_feed(-6.0f, 1U).b_hump_detected);
        assert(!roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    }
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    roll_feed(-6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    assert(!roll_feed(0.0f, TEST_SHORT_BASELINE).b_hump_completed);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    g_short_data.roll_deg = 0.0f;
    assert(!hold_feed(0.0f, 50U).b_hump_completed);
    assert(!hold_feed(0.0f, 50U).b_hump_completed);
    assert(IMU_ROLL_ACTIVE == g_analysis.roll_state);
    assert(roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);

    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    g_short_data.motion.b_fault = true;
    assert(!roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    g_config.roll_level_deg = g_config.roll_enter_deg;
    assert(!imu_analysis_init(&g_analysis, &g_config));
    g_config.roll_level_deg = NAN;
    assert(!imu_analysis_init(&g_analysis, &g_config));
    assert(IMU_ROLL_UNARMED == g_analysis.roll_state);
    g_short_data.motion.b_fault = false;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    g_short_data.b_valid = false;
    imu_event_t event;
    assert(!imu_analysis_process(&g_analysis, &g_short_data, &event));
    assert(IMU_ROLL_UNARMED == g_analysis.roll_state);

    short_reset();
    g_short_data.motion.b_stopped = true;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    assert(!roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES).b_hump_detected);
    assert(!roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);

    short_reset();
    g_analysis.config.hump_timeout_ms = 120U;
    g_short_data.timestamp_ms = UINT32_MAX - 150U;
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    hold_feed(0.0f, 100U);
    hold_feed(0.0f, 20U);
    assert(IMU_ROLL_UNARMED == g_analysis.roll_state);
    assert(IMU_ANALYSIS_RESET_TIMEOUT == g_analysis.reset_reasons);
    assert(!roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    puts("PASS: roll noise, baseline, encoder hold, fault, invalid sample, "
         "stationary rejection and timeout across wrap");
}

/** @brief Pitch measurements do not override motion labels in roll mode. */
static void
test_roll_pitch_independence (void)
{
    short_reset();
    roll_feed(0.0f, TEST_SHORT_BASELINE);
    assert(IMU_MOTION_CLIMBING_HUMP !=
           short_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES).motion_state);
    short_feed(0.0f, TEST_SHORT_SETTLE);
    assert(IMU_MOTION_DESCENDING_HUMP !=
           short_feed(-6.0f, IMU_ROLL_CONFIRM_SAMPLES).motion_state);
    assert(imu_test_support_near(-6.0f,
                                short_feed(-6.0f, 1U).pitch_deg));
    assert(!short_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);

    roll_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    short_feed(6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    short_feed(0.0f, TEST_SHORT_SETTLE);
    short_feed(-6.0f, IMU_ROLL_CONFIRM_SAMPLES);
    assert(!short_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    assert(IMU_ROLL_ACTIVE == g_analysis.roll_state);
    assert(roll_feed(0.0f, TEST_SHORT_SETTLE).b_hump_completed);
    puts("PASS: pitch readings retained without overriding roll motion; "
         "pitch-only changes cannot emit or clear a roll event");
}

/** @brief Absolute roll determines trends for either wheel without chatter. */
static void
test_roll_motion_trends (void)
{
    const float32_t sides[] = {1.0f, -1.0f};
    const uint32_t cases = sizeof(sides) / sizeof(sides[0]);

    for (uint32_t index = 0U; index < cases; ++index)
    {
        const float32_t side = sides[index];
        short_reset();
            roll_feed(0.0f, TEST_SHORT_BASELINE);
        assert(IMU_MOTION_CLIMBING_HUMP ==
               roll_feed(side * 6.0f, IMU_ROLL_CONFIRM_SAMPLES).motion_state);
        assert(IMU_MOTION_CLIMBING_HUMP ==
               roll_feed(side * 5.0f, 1U).motion_state);
        assert(IMU_MOTION_CLIMBING_HUMP ==
               roll_feed(side * 6.0f, 1U).motion_state);
        assert(IMU_MOTION_CLIMBING_HUMP ==
               roll_feed(side * 5.0f, 1U).motion_state);
        roll_feed(side * 7.0f, 1U);
        assert(IMU_MOTION_CLIMBING_HUMP ==
               roll_feed(side * 8.0f, 1U).motion_state);
        assert(IMU_MOTION_HUMP_PEAK ==
               roll_feed(side * 8.0f, TEST_SHORT_SETTLE).motion_state);
        assert(IMU_MOTION_HUMP_PEAK ==
               roll_feed(side * 7.0f, 1U).motion_state);
        assert(IMU_MOTION_DESCENDING_HUMP ==
               roll_feed(side * 6.0f, 1U).motion_state);
        assert(IMU_MOTION_DESCENDING_HUMP ==
               roll_feed(side * 6.2f, 1U).motion_state);
        assert(IMU_MOTION_DESCENDING_HUMP ==
               roll_feed(side * 5.9f, 1U).motion_state);
        assert(IMU_MOTION_DESCENDING_HUMP ==
               roll_feed(side * 6.1f, 1U).motion_state);
        roll_feed(0.0f, 1U);
        assert(IMU_MOTION_DESCENDING_HUMP == roll_feed(0.0f, 1U).motion_state);
        assert(roll_feed(0.0f, 1U).b_hump_completed);

        roll_feed(side * 6.0f, IMU_ROLL_CONFIRM_SAMPLES);
        assert(!roll_feed(0.0f, 1U).b_hump_completed);
        assert(IMU_MOTION_DESCENDING_HUMP == roll_feed(0.0f, 1U).motion_state);
        assert(roll_feed(0.0f, 1U).b_hump_completed);
    }
    puts("PASS: either-side roll climb/peak/descent, trend confirmation, "
         "deadband noise rejection and short return to level");
    g_config.roll_trend_deg = 0.0f;
    assert(!imu_analysis_init(&g_analysis, &g_config));
}

/*** end of file ***/

