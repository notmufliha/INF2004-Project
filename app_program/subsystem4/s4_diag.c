/** @file s4_diag.c
 * @brief Aggregate sample-rate evidence without per-sample serial output.
 */
#include "s4_diag.h"
#include "s4_runtime.h"

#if APP_TEST
static FastLock         g_lock;
static volatile bool    gb_ready;
static bool             gb_active;
static s4_diag_report_t g_report;
static bool             gb_roll_active;

static const char * gp_odometry_names[IMU_ODOM_REASON_COUNT] = {
    "invalid_IMU_input",           "left_error_counter_changed",
    "right_error_counter_changed", "acquisition_over_limit",
    "final_clock_failure",         "diagnostic_clock_failure"};

static const char * gp_reset_names[S4_DIAG_RESET_COUNT] = {
    "invalid_input", "timing_gap", "impact", "terrain_timeout",
    "B2_context_invalid", "odometry_unavailable", "B2_context_stale",
    "B2_fault", "reverse_context_or_encoder_step", "turning", "stopped",
    "not_forward", "no_encoder_progress_200ms"};
static void increment (uint32_t * p_count);
static void record_angles (const imu_data_t * p_raw,
                           const imu_data_t * p_filtered);
static void record_terrain (const imu_analysis_t * p_analysis,
                            const imu_event_t *    p_event);
static void record_reasons (uint32_t mask, uint32_t * p_counts, uint32_t count);
static void print_angles (const s4_diag_report_t * p_report);
static void print_reasons (const s4_diag_report_t * p_report);
static void update_range (float32_t value, uint32_t * p_samples,
                          float32_t * p_min, float32_t * p_max);
static void record_odometry (const imu_data_t * p_raw);
static void record_sensor_reads (const imu_data_t * p_raw);
static void record_timing (uint32_t value, uint32_t * p_samples,
                           uint32_t * p_max);
static void print_odometry (const s4_diag_report_t * p_report);
static void print_timing (const char * p_name, uint32_t maximum,
                          uint32_t samples);
#endif

/** @brief Start a clean capture before submitting the single MOVE command. */
ER
s4_diag_begin (void)
{
#if APP_TEST
    if (!gb_ready)
    {
        ER error = CreateLock(&g_lock, (const UB *)"B4 diagnostic");

        if (E_OK != error)
        {
            return error;
        }

        gb_ready = true;
    }

    Lock(&g_lock);

    if (gb_active)
    {
        Unlock(&g_lock);
        return E_OBJ;
    }

    g_report      = (s4_diag_report_t){0};
    gb_roll_active = false;
    gb_active     = true;
    Unlock(&g_lock);
    return E_OK;
#else
    return E_OBJ;
#endif
}

/** @brief Observe every processed frame, even when classification is invalid.
 */
void
s4_diag_record (const imu_data_t * p_raw, const imu_data_t * p_filtered,
                const imu_analysis_t * p_analysis,
                const imu_event_t * p_event)
{
#if APP_TEST
    if ((!gb_ready) || (NULL == p_raw) || (NULL == p_filtered) ||
        (NULL == p_analysis) || (NULL == p_event))
    {
        return;
    }

    Lock(&g_lock);

    if (gb_active)
    {
        increment(&g_report.frames);
        record_angles(p_raw, p_filtered);
        record_odometry(p_raw);
        record_sensor_reads(p_raw);
        record_terrain(p_analysis, p_event);
        record_reasons(p_analysis->reset_reasons, g_report.analysis_resets,
                       S4_DIAG_RESET_COUNT);
    }

    Unlock(&g_lock);
#else
    (void)p_raw;
    (void)p_filtered;
    (void)p_analysis;
    (void)p_event;
#endif
}

/** @brief Freeze the collection before printing; failed reads clear output. */
ER
s4_diag_end (s4_diag_report_t * p_report)
{
    if (NULL == p_report)
    {
        return E_PAR;
    }

    *p_report = (s4_diag_report_t){0};
#if APP_TEST
    if (!gb_ready)
    {
        return E_OBJ;
    }

    Lock(&g_lock);

    if (!gb_active)
    {
        Unlock(&g_lock);
        return E_OBJ;
    }

    gb_active = false;
    *p_report = g_report;
    Unlock(&g_lock);
    return E_OK;
#else
    return E_OBJ;
#endif
}

/** @brief Print a frozen run once, with integer tenths of degrees. */
void
s4_diag_print (const s4_diag_report_t * p_report)
{
#if APP_TEST
    if (NULL == p_report)
    {
        return;
    }

    tm_printf((UB *)"[B4-DIAG] Frames=%u raw_valid=%u filtered_valid=%u\n",
              p_report->frames, p_report->raw_samples,
              p_report->filtered_samples);
    print_angles(p_report);

    if (0U == p_report->frames)
    {
        tm_printf((UB *)"[B4-DIAG] No analysis frames captured; "
                        "detection evidence unavailable\n");
        return;
    }

    tm_printf((UB *)"[B4-DIAG] Hump mode=ROLL candidates=%u; "
                    "enter=%d level=%d tenths deg; height=N/A\n",
              p_report->roll_candidates,
              imu_numeric_to_int(p_report->roll_enter_deg * 10.0f),
              imu_numeric_to_int(p_report->roll_level_deg * 10.0f));
    tm_printf((UB *)"[B4-DIAG] Roll stages climb=%d crest=%d descent=%d\n",
              0U != (p_report->roll_stages & (1U << IMU_TERRAIN_CLIMBING)),
              0U != (p_report->roll_stages & (1U << IMU_TERRAIN_PEAK)),
              0U != (p_report->roll_stages & (1U << IMU_TERRAIN_DESCENDING)));
    tm_printf((UB *)"[B4-DIAG] Hump events completed=%u height_accepted=%u\n",
              p_report->completions, p_report->accepted_heights);
    tm_printf((UB *)"[B4-DIAG] Roll confirmation enter=%u level=%u samples\n",
              IMU_ROLL_CONFIRM_SAMPLES, p_report->confirmation_samples);
    print_odometry(p_report);
    print_reasons(p_report);
#else
    (void)p_report;
#endif
}

#if APP_TEST
/** @brief Saturate fixed-size counters instead of wrapping a long run. */
static void
increment (uint32_t * p_count)
{
    if (*p_count < UINT32_MAX)
    {
        ++(*p_count);
    }
}

/** @brief Seed extrema from the first valid sample, including negative peaks.
 */
static void
update_range (float32_t value, uint32_t * p_samples, float32_t * p_min,
              float32_t * p_max)
{
    if ((0U == *p_samples) || (value < *p_min))
    {
        *p_min = value;
    }

    if ((0U == *p_samples) || (value > *p_max))
    {
        *p_max = value;
    }

    increment(p_samples);
}

/** @brief Store sensor and filter ranges separately, excluding invalid angles.
 */
static void
record_angles (const imu_data_t * p_raw, const imu_data_t * p_filtered)
{
    if ((p_raw->b_valid) && (imu_numeric_is_finite(p_raw->pitch_deg)) &&
        (imu_numeric_is_finite(p_raw->roll_deg)))
    {
        update_range(p_raw->pitch_deg, &g_report.raw_samples,
                     &g_report.raw_min_deg, &g_report.raw_max_deg);

        float32_t roll =
            (p_raw->roll_deg < 0.0f) ? -p_raw->roll_deg : p_raw->roll_deg;

        if (roll > g_report.raw_roll_deg)
        {
            g_report.raw_roll_deg = roll;
        }
    }

    if ((p_filtered->b_valid) &&
        (imu_numeric_is_finite(p_filtered->pitch_deg)) &&
        (imu_numeric_is_finite(p_filtered->roll_deg)))
    {
        update_range(p_filtered->pitch_deg, &g_report.filtered_samples,
                     &g_report.filtered_min_deg, &g_report.filtered_max_deg);

        float32_t roll = (p_filtered->roll_deg < 0.0f) ? -p_filtered->roll_deg
                                                       : p_filtered->roll_deg;

        if (roll > g_report.filtered_roll_deg)
        {
            g_report.filtered_roll_deg = roll;
        }
    }
}

/** @brief Record reached stages and actual classifier-input threshold streaks.
 */
static void
record_terrain (const imu_analysis_t * p_analysis, const imu_event_t * p_event)
{
    g_report.roll_enter_deg = p_analysis->config.roll_enter_deg;
    g_report.roll_level_deg = p_analysis->config.roll_level_deg;
    if ((uint32_t)p_analysis->roll_terrain <=
        (uint32_t)IMU_TERRAIN_DESCENDING)
    {
        g_report.roll_stages |= 1U << (uint32_t)p_analysis->roll_terrain;
    }
    const bool b_roll_active = IMU_ROLL_ACTIVE == p_analysis->roll_state;
    if ((b_roll_active) && (!gb_roll_active))
    {
        increment(&g_report.roll_candidates);
    }
    gb_roll_active = b_roll_active;
    g_report.confirmation_samples = p_analysis->config.confirmation_samples;

    if (p_event->b_hump_completed)
    {
        increment(&g_report.completions);
    }

    if (p_event->b_height_valid)
    {
        increment(&g_report.accepted_heights);
    }
}

/** @brief Count failed checks per sample, not distinct humps or hardware
 * faults. */
static void
record_reasons (uint32_t mask, uint32_t * p_counts, uint32_t count)
{
    for (uint32_t index = 0U; index < count; ++index)
    {
        if (0U != (mask & (1U << index)))
        {
            increment(&p_counts[index]);
        }
    }
}

/** @brief Separate failed capture conditions; more than one may affect a frame.
 */
static void
record_odometry (const imu_data_t * p_raw)
{
    if (p_raw->b_odometry_valid)
    {
        increment(&g_report.odometry_valid);
    }
    else
    {
        increment(&g_report.odometry_invalid);
    }

    record_reasons(p_raw->odometry_reasons, g_report.odometry_reasons,
                   IMU_ODOM_REASON_COUNT);

    if (0U != (p_raw->odometry_time_valid & IMU_ODOM_TIME_AGE))
    {
        record_timing(p_raw->odometry_age_ms, &g_report.odometry_age_samples,
                      &g_report.max_odometry_age_ms);
    }

    if (0U != (p_raw->odometry_time_valid & IMU_ODOM_TIME_ENCODER))
    {
        record_timing(p_raw->odometry_encoder_ms,
                      &g_report.odometry_encoder_samples,
                      &g_report.max_odometry_encoder_ms);
    }

    if (0U != (p_raw->odometry_time_valid & IMU_ODOM_TIME_CONTEXT))
    {
        record_timing(p_raw->odometry_context_ms,
                      &g_report.odometry_context_samples,
                      &g_report.max_odometry_context_ms);
    }

    const uint32_t phases =
        (uint32_t)p_raw->odometry_encoder_ms + p_raw->odometry_context_ms;

    /* Derive this per frame, not by subtracting unrelated phase maxima.
     * Ignore saturated/unknown measurements when taking the difference.
     */
    if ((IMU_ODOM_TIME_ALL ==
         (p_raw->odometry_time_valid & IMU_ODOM_TIME_ALL)) &&
        (UINT16_MAX != p_raw->odometry_age_ms) &&
        (UINT16_MAX != p_raw->odometry_encoder_ms) &&
        (UINT16_MAX != p_raw->odometry_context_ms) &&
        (p_raw->odometry_age_ms >= phases))
    {
        record_timing(p_raw->odometry_age_ms - phases,
                      &g_report.odometry_before_samples,
                      &g_report.max_odometry_before_ms);
    }
}

/** @brief Record sensor I2C phases independently of encoder pairing. */
static void
record_sensor_reads (const imu_data_t * p_raw)
{
    if (0U != (p_raw->odometry_time_valid & IMU_READ_MAG_DEFERRED))
    {
        increment(&g_report.magnetic_deferred_frames);
    }

    if (0U != (p_raw->odometry_time_valid & IMU_READ_TIME_ACCEL))
    {
        record_timing(p_raw->acceleration_read_ms,
                      &g_report.acceleration_read_samples,
                      &g_report.max_acceleration_read_ms);
    }

    if (0U != (p_raw->odometry_time_valid & IMU_READ_TIME_MAG))
    {
        record_timing(p_raw->magnetic_read_ms, &g_report.magnetic_read_samples,
                      &g_report.max_magnetic_read_ms);
    }
}

/** @brief Store a timing maximum only when its validity flag is present. */
static void
record_timing (uint32_t value, uint32_t * p_samples, uint32_t * p_max)
{
    increment(p_samples);

    if (value > *p_max)
    {
        *p_max = value;
    }
}

/** @brief Print acquisition causes and phase timing after the run has stopped.
 */
static void
print_odometry (const s4_diag_report_t * p_report)
{
    tm_printf((UB *)"[B4-DIAG] Odometry valid=%u invalid=%u; "
                    "acquisition limit=%u ms\n",
              p_report->odometry_valid, p_report->odometry_invalid,
              S4_IMU_PERIOD_MS);

    for (uint32_t index = 0U; index < IMU_ODOM_REASON_COUNT; ++index)
    {
        if (0U != p_report->odometry_reasons[index])
        {
            tm_printf((UB *)"[B4-DIAG] Odom check %s=%u\n",
                      gp_odometry_names[index],
                      p_report->odometry_reasons[index]);
        }
    }

    print_timing("accel_encoder_window", p_report->max_odometry_age_ms,
                 p_report->odometry_age_samples);
    print_timing("before_encoder_in_window", p_report->max_odometry_before_ms,
                 p_report->odometry_before_samples);
    print_timing("encoder_capture", p_report->max_odometry_encoder_ms,
                 p_report->odometry_encoder_samples);
    print_timing("B2_context", p_report->max_odometry_context_ms,
                 p_report->odometry_context_samples);
    print_timing("acceleration_I2C", p_report->max_acceleration_read_ms,
                 p_report->acceleration_read_samples);
    print_timing("magnetic_I2C", p_report->max_magnetic_read_ms,
                 p_report->magnetic_read_samples);
    tm_printf((UB *)"[B4-DIAG] Magnetic reads deferred=%u frames\n",
              p_report->magnetic_deferred_frames);
}

/** @brief Zero milliseconds is valid at clock resolution; missing data is N/A.
 */
static void
print_timing (const char * p_name, uint32_t maximum, uint32_t samples)
{
    if (0U == samples)
    {
        tm_printf((UB *)"[B4-DIAG] Timing %s=N/A\n", p_name);
    }
    else
    {
        tm_printf((UB *)"[B4-DIAG] Timing %s max=%u ms samples=%u\n", p_name,
                  maximum, samples);
    }
}

/** @brief Label missing data instead of treating unobserved extrema as zero. */
static void
print_angles (const s4_diag_report_t * p_report)
{
    if (0U != p_report->raw_samples)
    {
        tm_printf((UB *)"[B4-DIAG] Raw pitch min=%d max=%d; "
                        "max_abs_roll=%d (tenths deg)\n",
                  imu_numeric_to_int(p_report->raw_min_deg * 10.0f),
                  imu_numeric_to_int(p_report->raw_max_deg * 10.0f),
                  imu_numeric_to_int(p_report->raw_roll_deg * 10.0f));
    }
    else
    {
        tm_printf((UB *)"[B4-DIAG] Raw angles=N/A\n");
    }

    if (0U != p_report->filtered_samples)
    {
        tm_printf((UB *)"[B4-DIAG] Filtered pitch min=%d max=%d; "
                        "max_abs_roll=%d (tenths deg)\n",
                  imu_numeric_to_int(p_report->filtered_min_deg * 10.0f),
                  imu_numeric_to_int(p_report->filtered_max_deg * 10.0f),
                  imu_numeric_to_int(p_report->filtered_roll_deg * 10.0f));
    }
    else
    {
        tm_printf((UB *)"[B4-DIAG] Filtered angles=N/A\n");
    }
}

/** @brief Only print nonzero reasons; flat-ground input checks are included. */
static void
print_reasons (const s4_diag_report_t * p_report)
{
    tm_printf((UB *)"[B4-DIAG] Counts below are failed checks per sample, "
                    "not distinct humps. Only nonzero counts printed.\n");

    for (uint32_t index = 0U; index < S4_DIAG_RESET_COUNT; ++index)
    {
        if (0U != p_report->analysis_resets[index])
        {
            tm_printf((UB *)"[B4-DIAG] Detection reset %s=%u\n",
                      gp_reset_names[index], p_report->analysis_resets[index]);
        }
    }


}
#endif

/*** end of file ***/

