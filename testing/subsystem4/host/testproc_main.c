/** @file testproc_main.c
 * @brief Buddy 4 testproc main module.
 */
#include "testproc_main.h"
#include "imu_test_support.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "imu_processing.h"
#include "imu_analysis.h"

#define TEST_RECOVERY_PENDING 4U

static imu_processing_t        g_state;
static imu_processing_config_t g_config;
static imu_data_t              g_raw;
static imu_data_t              g_out;
static imu_processing_health_t g_health;

static void reset (void);

static bool step (void);

static void test_filter (void);
static void test_faults (void);
static void test_elapsed_filter (void);

/** @brief Main. */
int
main (void)
{
    test_filter();
    test_faults();
    test_elapsed_filter();
    puts("All IMU processing tests passed.");
    return 0;
}

/** @brief Verify filter response, angle wrapping and impact preservation. */
static void
test_filter (void)
{
    reset();
    assert(step());
    g_raw.pitch_deg     = 12.0f;
    g_raw.roll_deg      = -12.0f;
    g_raw.turn_rate_dps = 40.0f;
    g_raw.accel_x       = 15.0f;
    assert(step());
    assert((imu_test_support_near(g_out.pitch_deg, 3.0f)) &&
           (imu_test_support_near(g_out.roll_deg, -3.0f)));
    assert((imu_test_support_near(g_out.turn_rate_dps, 10.0f)) &&
           (imu_test_support_near(g_out.accel_x, 15.0f)));

    imu_analysis_t        analysis;
    imu_analysis_config_t analysis_config;
    imu_event_t           event;

    imu_analysis_default_config(&analysis_config);
    assert(imu_analysis_init(&analysis, &analysis_config));
    assert((imu_analysis_process(&analysis, &g_out, &event)) &&
           (event.b_sudden_impact));
    puts("PASS: smoothing preserves an immediate impact spike");

    reset();
    g_raw.pitch_deg = 179.0f;
    assert(step());
    g_raw.pitch_deg = -179.0f;
    assert(step());
    assert(g_out.pitch_deg > 179.0f);
    puts("PASS: angles wrap along the shortest path");
}

/** @brief Verify fault latching, recovery and timestamp validation. */
static void
test_faults (void)
{
    reset();
    assert(step());
    g_raw.b_valid = false;
    assert((!step()) && (!g_health.b_fault_active) && (!g_out.b_valid));
    assert((!step()) && (!g_health.b_fault_active));
    assert((!step()) && (g_health.b_fault_started) &&
           (g_health.b_fault_active));
    assert((!step()) && (!g_health.b_fault_started) &&
           (g_health.b_fault_active));
    g_raw.b_valid = true;

    for (uint32_t index = 0U; index < TEST_RECOVERY_PENDING; ++index)
    {
        assert((!step()) && (g_health.b_fault_active));
    }

    g_raw.pitch_deg = 20.0f;
    assert((step()) && (g_health.b_recovered) && (!g_health.b_fault_active));
    assert(imu_test_support_near(
        g_out.pitch_deg, 20.0f)); /* Reseeded, no stale filter history. */
    assert((step()) && (!g_health.b_recovered));
    puts("PASS: fault latching, recovery suppression and filter reseeding");

    reset();
    assert(step());
    g_raw.timestamp_ms += 101U;
    assert((!step()) && (g_health.b_fault_started) &&
           (g_health.b_fault_active));
    assert(!step());
    g_raw.b_valid = false;
    assert(!step()); /* Interrupt recovery. */
    g_raw.b_valid = true;

    for (uint32_t index = 0U; index < TEST_RECOVERY_PENDING; ++index)
    {
        assert(!step());
    }

    assert((step()) && (g_health.b_recovered));
    assert(!imu_processing_process(&g_state, NULL, g_raw.timestamp_ms + 10U,
                                   &g_out, &g_health));
    assert(!g_out.b_valid);
    assert(!imu_processing_process(&g_state, &g_raw, g_raw.timestamp_ms, &g_out,
                                   &g_health));
    assert(g_health.b_fault_started); /* Duplicate timestamp. */
    puts("PASS: missing data, gaps, duplicate timestamps and interrupted "
         "recovery");

    reset();
    g_raw.pitch_deg = NAN;
    assert(!step());
    g_raw.pitch_deg = 181.0f;
    assert(!step());
    g_raw.pitch_deg = 0.0f;
    g_raw.accel_x   = INFINITY;
    assert((!step()) && (g_health.b_fault_started));
    g_config.smoothing_alpha = 0.0f;
    assert(!imu_processing_init(&g_state, &g_config));
    assert(!imu_processing_process(NULL, &g_raw, 0U, &g_out, &g_health));
    puts("PASS: nonfinite readings, angle bounds and invalid configuration");

    reset();
    g_raw.timestamp_ms = UINT32_MAX - 20U;
    assert(step());
    assert(step());
    assert(step()); /* Clock wraps normally. */
    g_raw.timestamp_ms -= 20U;
    assert((!step()) && (g_health.b_fault_started));
    puts("PASS: timestamp wrap and backward-clock fault");
}

/** @brief Reset. */
static void
reset (void)
{
    imu_processing_default_config(&g_config);
    assert(imu_processing_init(&g_state, &g_config));
    g_raw                   = (imu_data_t){0};
    g_raw.b_valid           = true;
    g_raw.b_turn_rate_valid = true;
    g_raw.accel_z           = 9.81f;
}

/** @brief Step. */
static bool
step (void)
{
    g_raw.timestamp_ms += 10U;
    return imu_processing_process(&g_state, &g_raw, g_raw.timestamp_ms, &g_out,
                                  &g_health);
}

/** @brief Equal elapsed time has equal response despite arrival intervals. */
static void
test_elapsed_filter (void)
{
    reset();
    assert(step());
    g_raw.pitch_deg = 12.0f;
    assert(step());
    assert(step());
    const float32_t regular = g_out.pitch_deg;

    reset();
    assert(step());
    g_raw.pitch_deg = 12.0f;
    g_raw.timestamp_ms += 10U;
    assert(step());
    assert(imu_test_support_near(regular, g_out.pitch_deg));
    puts("PASS: elapsed-time filter response at 10ms and 20ms intervals");
}

/*** end of file ***/

