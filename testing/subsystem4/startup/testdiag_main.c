/** @file testdiag_main.c
 * @brief Verify diagnostic reports against actual roll detection and acquisition code.
 */
#include "s4_diag.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

extern ER       g_diag_create_error;
extern uint32_t g_diag_lock_depth;

static imu_data_t     g_raw;
static imu_data_t     g_filtered;
static imu_analysis_t g_analysis;
static imu_event_t    g_event;

static void prepare (void);
static void feed (float32_t pitch);
static void test_ranges (void);
static void test_filtered_ranges (void);
static void test_resets (void);
static void test_odometry (void);
static void test_roll_report (void);

/** @brief Check lifecycle failures, actual rejection reasons and frozen ranges.
 */
int
main (void)
{
    s4_diag_report_t report = {0};

    assert(E_OBJ == s4_diag_end(&report));
    assert(E_PAR == s4_diag_end(NULL));
    s4_diag_record(NULL, NULL, NULL, NULL);
    s4_diag_print(NULL);
    g_diag_create_error = E_SYS;
    assert(E_SYS == s4_diag_begin());
    g_diag_create_error = E_OK;
    test_ranges();
    test_filtered_ranges();
    test_resets();
    test_odometry();
    test_roll_report();
    assert(E_OK == s4_diag_begin());
    assert(E_OK == s4_diag_end(&report));
    assert(0U == report.frames);
    s4_diag_print(&report);
    assert(0U == g_diag_lock_depth);
    printf("PASS: B4 diagnostics, actual classifier/reset reasons, "
           "frozen capture; report=%zu bytes\n",
           sizeof(report));
    return 0;
}

/** @brief Set synthetic calibrated wheel travel and a known level baseline. */
static void
prepare (void)
{
    imu_analysis_config_t analysis_config;

    imu_analysis_default_config(&analysis_config);
    assert(imu_analysis_init(&g_analysis, &analysis_config));
    g_raw                  = (imu_data_t){0};
    g_raw.b_valid          = true;
    g_raw.b_odometry_valid = true;
    g_raw.accel_z          = 9.81f;
    g_raw.motion.b_valid = true;
    g_raw.motion.b_forward = true;
}

/** @brief Advance actual roll detection, then aggregate. */
static void
feed (float32_t pitch)
{
    g_raw.timestamp_ms += 10U;
    g_raw.motion.timestamp_ms = g_raw.timestamp_ms;
    ++g_raw.sample_sequence;
    g_raw.encoder_left += 10;
    g_raw.encoder_right += 10;
    g_raw.pitch_deg = pitch;
    g_filtered      = g_raw;
    (void)imu_analysis_process(&g_analysis, &g_filtered, &g_event);
    s4_diag_record(&g_raw, &g_filtered, &g_analysis, &g_event);
}

/** @brief Pitch extrema remain available without producing hump events. */
static void
test_ranges (void)
{
    static const float32_t p_pitch[] = {0.0f,   0.0f, 10.0f, 10.0f,  10.0f,
                                        0.0f,   0.0f, 0.0f,  -10.0f, -10.0f,
                                        -10.0f, 0.0f, 0.0f,  0.0f};
    s4_diag_report_t       report    = {0};

    prepare();
    assert(E_OK == s4_diag_begin());
    assert(E_OBJ == s4_diag_begin());
    for (uint32_t index = 0U; index < 14U; ++index)
    {
        feed(p_pitch[index]);
    }
    assert(E_OK == s4_diag_end(&report));
    assert(14U == report.frames);
    assert(14U == report.raw_samples);
    assert(-10.0f == report.raw_min_deg);
    assert(10.0f == report.filtered_max_deg);
    assert(0U == report.completions);
    assert(0U == report.accepted_heights);
    s4_diag_print(&report);
    feed(45.0f); /* Capturing has stopped: this must not leak into the next run.
                  */
    assert(E_OBJ == s4_diag_end(&report));
    assert(0U == report.frames);
}

/** @brief Raw/filtered extrema and invalid measurements stay distinct. */
static void
test_filtered_ranges (void)
{
    s4_diag_report_t report;

    prepare();
    assert(E_OK == s4_diag_begin());
    feed(0.0f);
    g_raw.timestamp_ms += 10U;
    g_raw.motion.timestamp_ms = g_raw.timestamp_ms;
    g_raw.pitch_deg = 30.0f;
    g_raw.roll_deg = 23.0f;
    g_filtered = g_raw;
    g_filtered.pitch_deg = 4.0f;
    g_filtered.roll_deg = 5.0f;
    assert(imu_analysis_process(&g_analysis, &g_filtered, &g_event));
    s4_diag_record(&g_raw, &g_filtered, &g_analysis, &g_event);
    g_raw.pitch_deg = NAN;
    s4_diag_record(&g_raw, &g_filtered, &g_analysis, &g_event);
    assert(E_OK == s4_diag_end(&report));
    assert(30.0f == report.raw_max_deg);
    assert(4.0f == report.filtered_max_deg);
    assert(23.0f == report.raw_roll_deg);
    assert(5.0f == report.filtered_roll_deg);
    s4_diag_print(&report);
}

/** @brief Verify input, timing, impact and timeout reasons come from the
 * detector. */
static void
test_resets (void)
{
    s4_diag_report_t report = {0};

    prepare();
    g_analysis.config.hump_timeout_ms = 200U;
    assert(E_OK == s4_diag_begin());
    for (uint32_t index = 0U; index < 4U; ++index)
    {
        feed(0.0f);
    }
    g_raw.roll_deg = 10.0f;
    for (uint32_t index = 0U; index < 25U; ++index)
    {
        feed(0.0f);
    }
    g_raw.roll_deg = 0.0f;
    g_raw.timestamp_ms += 110U;
    feed(0.0f);
    assert(IMU_ANALYSIS_RESET_TIMING == g_analysis.reset_reasons);
    g_raw.b_valid = false;
    feed(0.0f);
    assert(IMU_ANALYSIS_RESET_INVALID == g_analysis.reset_reasons);
    g_raw.b_valid = true;
    g_raw.accel_z = 20.0f;
    feed(0.0f);
    assert(IMU_ANALYSIS_RESET_IMPACT == g_analysis.reset_reasons);
    g_raw.accel_z = 9.81f;
    g_analysis.b_require_motion = true;
    g_raw.motion.b_valid = false;
    feed(0.0f);
    assert(IMU_ANALYSIS_RESET_MOTION == g_analysis.reset_reasons);
    assert(E_OK == s4_diag_end(&report));
    assert(1U == report.analysis_resets[0]);
    assert(1U == report.analysis_resets[1]);
    assert(1U == report.analysis_resets[2]);
    assert(1U == report.analysis_resets[3]);
    assert(1U == report.analysis_resets[4]);
    assert(3U == report.confirmation_samples);
    s4_diag_print(&report);
}

/** @brief Preserve overlapping causes and timing validity independently. */
static void
test_odometry (void)
{
    s4_diag_report_t report = {0};

    prepare();
    assert(E_OK == s4_diag_begin());
    g_raw.b_odometry_valid    = false;
    g_raw.odometry_reasons    = IMU_ODOM_REASON_AGE | IMU_ODOM_REASON_LEFT;
    g_raw.odometry_age_ms     = 20U;
    g_raw.odometry_encoder_ms = 2U;
    g_raw.odometry_context_ms = 3U;
    g_raw.odometry_time_valid =
        IMU_ODOM_TIME_ALL | IMU_READ_TIME_ACCEL | IMU_READ_TIME_MAG;
    g_raw.acceleration_read_ms = 20U;
    g_raw.magnetic_read_ms     = 30U;
    feed(0.0f);
    g_raw.b_odometry_valid = true;
    g_raw.odometry_reasons = IMU_ODOM_REASON_PROBE;
    g_raw.odometry_age_ms  = 5U;
    g_raw.odometry_time_valid =
        IMU_ODOM_TIME_AGE | IMU_ODOM_TIME_CONTEXT | IMU_READ_MAG_DEFERRED;
    feed(0.0f);
    g_raw.b_odometry_valid    = false;
    g_raw.odometry_reasons    = IMU_ODOM_REASON_CLOCK;
    g_raw.odometry_time_valid = 0U;
    g_raw.odometry_context_ms =
        UINT16_MAX; /* Must not count an unknown value. */
    feed(0.0f);
    g_raw.odometry_reasons    = IMU_ODOM_REASON_AGE;
    g_raw.odometry_age_ms     = UINT16_MAX;
    g_raw.odometry_encoder_ms = 1U;
    g_raw.odometry_context_ms = 1U;
    g_raw.odometry_time_valid = IMU_ODOM_TIME_ALL;
    feed(0.0f);
    assert(E_OK == s4_diag_end(&report));
    assert(1U == report.odometry_valid);
    assert(3U == report.odometry_invalid);
    assert(1U == report.odometry_reasons[1]);
    assert(2U == report.odometry_reasons[3]);
    assert(1U == report.odometry_reasons[4]);
    assert(1U == report.odometry_reasons[5]);
    assert(3U == report.odometry_age_samples);
    assert(2U == report.odometry_encoder_samples);
    assert(3U == report.odometry_context_samples);
    assert(1U == report.odometry_before_samples);
    assert(15U == report.max_odometry_before_ms);
    assert(UINT16_MAX == report.max_odometry_age_ms);
    assert(2U == report.max_odometry_encoder_ms);
    assert(3U == report.max_odometry_context_ms);
    assert(1U == report.acceleration_read_samples);
    assert(1U == report.magnetic_read_samples);
    assert(20U == report.max_acceleration_read_ms);
    assert(30U == report.max_magnetic_read_ms);
    assert(1U == report.magnetic_deferred_frames);
    s4_diag_print(&report);
}

/** @brief Printing must occur after releasing the diagnostic lock. */
int
tm_printf (const UB * p_format, ...)
{
    va_list arguments;

    assert(0U == g_diag_lock_depth);
    va_start(arguments, p_format);
    int count = vprintf((const char *)p_format, arguments);

    va_end(arguments);
    return count;
}

/** @brief Actual roll classification is reported without pitch stages. */
static void
test_roll_report (void)
{
    const float32_t profile[] = {0.0f, 0.0f, 0.0f, 0.0f, 8.0f,
                                8.0f, 0.0f, 0.0f, 0.0f};
    const uint32_t samples = sizeof(profile) / sizeof(profile[0]);
    s4_diag_report_t report;

    prepare();
    g_analysis.b_require_motion = true;
    assert(E_OK == s4_diag_begin());
    for (uint32_t index = 0U; index < samples; ++index)
    {
        g_raw.timestamp_ms += 20U;
        g_raw.motion.timestamp_ms = g_raw.timestamp_ms;
        g_raw.motion.b_valid = true;
        g_raw.motion.b_forward = true;
        g_raw.encoder_left += 10;
        g_raw.encoder_right += 10;
        g_raw.roll_deg = profile[index];
        g_raw.accel_y = 9.81f * sinf(profile[index] * 0.01745329252f);
        g_raw.accel_z = 9.81f * cosf(profile[index] * 0.01745329252f);
        g_filtered = g_raw;
        assert(imu_analysis_process(&g_analysis, &g_filtered, &g_event));
        assert(g_event.b_height_valid == g_event.b_hump_completed);
        s4_diag_record(&g_raw, &g_filtered, &g_analysis, &g_event);
    }
    assert(E_OK == s4_diag_end(&report));
    assert(1U == report.roll_candidates);
    assert(1U == report.completions);
    assert(1U == report.accepted_heights);
    assert(0U != (report.roll_stages & (1U << IMU_TERRAIN_CLIMBING)));
    assert(0U != (report.roll_stages & (1U << IMU_TERRAIN_DESCENDING)));
    s4_diag_print(&report);
}

/*** end of file ***/
