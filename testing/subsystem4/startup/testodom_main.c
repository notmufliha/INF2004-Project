/** @file testodom_main.c
 * @brief Test capture timing, error races and shared B2 wheel calibration.
 */
#include "testodom_main.h"
#include "imu_odom.h"
#include "../../../app_program/subsystem2/encoder_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t g_now_ms = 105U;
static uint32_t g_error_calls;
static uint32_t g_count_calls;
static bool     gb_error_race;
static ER       g_clock_error;

/** @brief Execute the actual acquisition adapter without a board. */
int
main (void)
{
    imu_height_t height;
    imu_data_t   data = {0};

    assert(imu_odom_init(&height));
    assert(fabsf(height.config.left_cm_per_count * ENC_LEFT_COUNTS_PER_REV -
                 ENC_WHEEL_DIAMETER_CM * 3.14159265f) < 0.001f);
    assert(fabsf(height.config.right_cm_per_count * ENC_RIGHT_COUNTS_PER_REV -
                 ENC_WHEEL_DIAMETER_CM * 3.14159265f) < 0.001f);
    data.b_valid      = true;
    data.timestamp_ms = 100U;
    imu_odom_capture(&data);
    assert(data.b_odometry_valid);
    assert((1000 == data.encoder_left) && (2000 == data.encoder_right));
    assert((2U == data.encoder_left_errors) &&
           (3U == data.encoder_right_errors));
    assert(105U == data.timestamp_ms);
    gb_error_race = true;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    gb_error_race = false;
    g_now_ms += 20U;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    g_clock_error = E_OBJ;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    data.b_valid               = false;

    const uint32_t prior_calls = g_count_calls;

    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(prior_calls == g_count_calls);
    imu_odom_capture(NULL);
    puts("PASS: real odometry adapter, B2 geometry, error races and stale "
         "capture");
    return 0;
}

/* These stubs retain the external kernel/B2 ABI names and exact-width types. */

/** @brief Provide a deterministic acquisition clock or kernel error. */
ER
tk_get_tim (SYSTIM * p_time)
{
    p_time->hi = 0U;
    p_time->lo = g_now_ms;
    return g_clock_error;
}

/** @brief Return one paired real-interface count snapshot. */
void
encoder_get_counts (INT * p_left, INT * p_right)
{
    *p_left  = 1000;
    *p_right = 2000;
    g_count_calls++;
}

/** @brief Inject a transition error between the before/after error snapshots.
 */
void
encoder_get_invalid_counts (UW * p_left, UW * p_right)
{
    g_error_calls++;
    *p_left  = ((gb_error_race) && (0U == (g_error_calls % 2U))) ? 3U : 2U;
    *p_right = 3U;
}

/*** end of file ***/

