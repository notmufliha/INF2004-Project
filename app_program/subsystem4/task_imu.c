/* NOTE: Each UB pointer cast below adapts a string/byte buffer to the
 * kernel console API. Character bytes are preserved and formats are not
 * changed.
 */
/** @file task_imu.c
 * @brief Buddy 4 task imu module.
 */
#include "task_imu.h"
#include "subsystem4.h"
#include "s4_runtime.h"
#include "imu_analysis.h"
#include "imu_processing.h"
#include "../subsystem1/subsystem1.h"
#if S4_IMU_USE_MOCK
#include "imu_demo.h"
#else
#include "gy511_i2c.h"
#endif

#define IMU_AXIS_COUNT 3

typedef uint8_t
    imu_kernel_integer_check_t[(sizeof(INT) == sizeof(int32_t)) ? 1 : -1];

#if !S4_IMU_USE_MOCK
typedef struct
{
    gy511_t   sensor;
    bool      b_initialised;
    bool      b_calibrated;
    bool      b_attempted;
    uint32_t  retry_ms;
    uint32_t  last_data_ms;
    uint32_t  count;
    uint32_t  failures;
    float32_t mean[3];
    float32_t variance_sum[3];
} imu_hardware_state_t;

#endif

/* Only this task owns sensor acquisition and I2C0. */

/* Short integer-only lines match the other buddies' console format. */

static INT console_integer (float32_t value);
static H telemetry_integer (float32_t value);

#if !S4_IMU_USE_MOCK
static bool initialise_hardware (imu_hardware_state_t * p_state,
                                 uint32_t               now_ms);
#endif

#if !S4_IMU_USE_MOCK
static void calibrate_hardware (imu_hardware_state_t * p_state,
                                imu_data_t *           p_data);
#endif

#if !S4_IMU_USE_MOCK
static bool hardware_read (uint32_t now_ms, imu_data_t * p_data);
#endif

static const char * motion_name (imu_motion_state_t state);

static void print_readings (const imu_data_t *  p_data,
                            const imu_event_t * p_event);

static bool receive_reading (imu_data_t * p_data, TMO timeout_ms);

static void report_fault (const imu_processing_health_t * p_health,
                          uint32_t                        timestamp_ms);

static void report_event (const imu_data_t *  p_data,
                          const imu_event_t * p_event, bool * pb_have_previous,
                          imu_motion_state_t * p_previous, UW * p_last_report);

static bool publish_reading (const imu_data_t * p_data, bool b_publish,
                             UW * p_dropped);

/** @brief Acquire timestamped readings on a deadline without blocking. */
void
task_imu_sampling_task (INT stacd, void * p_exinf)
{
    imu_data_t data;
    SYSTIM     now;
#if S4_IMU_USE_MOCK
    UW origin;
#endif
    UW deadline;
    UW dropped = 0U;
    ER error_code;

    /* The kernel task arguments are unused; these casts discard no data. */
    (void)stacd;
    (void)p_exinf;

#if S4_IMU_USE_MOCK
    imu_mock_init();
#endif
    error_code = tk_get_tim(&now);

    if (error_code < E_OK)
    {
        CHECK(error_code, "IMU clock");
        tk_ext_tsk();
        return;
    }
#if S4_IMU_USE_MOCK
    origin = now.lo;
    tm_printf((UB *)"[B4] IMU starting: MOCK input, target period=10 ms\n");
#else
    tm_printf((UB *)"[B4] IMU starting: REAL GY-511, target period=10 ms\n");
#endif
    deadline = now.lo;

    for (;;)
    {
        error_code = tk_get_tim(&now);

        if (error_code < E_OK)
        {
            CHECK(error_code, "IMU clock");
            tk_ext_tsk();
            return;
        }

        {
            data = (imu_data_t){0};
#if S4_IMU_USE_MOCK
            imu_mock_set_scenario(imu_demo_scenario(now.lo - origin));
            data.b_valid = imu_mock_read(&data);

            bool b_publish = true;
#else
            bool b_publish = hardware_read(now.lo, &data);
#endif
            /* Use actual RTOS time, not the mock's synthetic sample count. */
            data.timestamp_ms = now.lo;

            if (!publish_reading(&data, b_publish, &dropped))
            {
                tk_ext_tsk();
                return;
            }
        }

        deadline += S4_IMU_PERIOD_MS;
        error_code = tk_get_tim(&now);

        if (error_code < E_OK)
        {
            CHECK(error_code, "IMU clock");
            tk_ext_tsk();
            return;
        }

        /* Deadline scheduling avoids adding processing time to each period.
         * On overrun skip catch-up bursts and yield for a full period. */
        /* Modular unsigned intervals avoid an out-of-range signed cast. */
        if ((0U == (deadline - now.lo)) ||
            ((deadline - now.lo) >= (UINT32_MAX / 2U)))
        {
            deadline = now.lo + S4_IMU_PERIOD_MS;
        }

        /* The scheduling branch bounds the delay to a positive short interval.
         */
        error_code = tk_dly_tsk((RELTIM)(deadline - now.lo));

        if (error_code < E_OK)
        {
            CHECK(error_code, "IMU delay");
            tk_ext_tsk();
            return;
        }
    }
}

/** @brief Receive readings, track health and report motion events. */
void
task_imu_analysis_task (INT stacd, void * p_exinf)
{
    imu_analysis_t          analysis;
    imu_analysis_config_t   config;
    imu_data_t              data;
    imu_data_t              processed;
    imu_processing_t        processing;
    imu_processing_config_t processing_config;
    imu_processing_health_t health;
    imu_event_t             event;
    imu_motion_state_t      previous        = IMU_MOTION_UNKNOWN;
    bool                    b_have_previous = false;
    bool                    b_warned        = false;
    UW                      last_report     = 0U;

    /* The kernel task arguments are unused; these casts discard no data. */
    (void)stacd;
    (void)p_exinf;

    imu_analysis_default_config(&config);
    imu_processing_default_config(&processing_config);

    if (!imu_processing_init(&processing, &processing_config))
    {
        tm_printf((UB *)"[B4] ERROR: processing configuration failed\n");
        tk_ext_tsk();
        return;
    }

    if (!imu_analysis_init(&analysis, &config))
    {
        tm_printf((UB *)"[B4] ERROR: analysis configuration failed\n");
        tk_ext_tsk();
        return;
    }

    for (;;)
    {
        /* Successful processing_init bounds max_gap_ms below INT32_MAX. */
        if (!receive_reading(&data, (TMO)processing_config.max_gap_ms))
        {
            tk_ext_tsk();
            return;
        }

        bool b_usable = imu_processing_process(
            &processing, &data, data.timestamp_ms, &processed, &health);

        report_fault(&health, data.timestamp_ms);
        /* An invalid processed reading resets any incomplete terrain sequence.
         */
        if ((!imu_analysis_process(&analysis, &processed, &event)) ||
            (!b_usable))
        {
            if (!b_warned)
            {
                tm_printf((UB *)"[B4] Invalid reading/timing: terrain sequence "
                                "reset\n");
            }

            b_warned        = true;
            b_have_previous = false;
        }
        else
        {
            b_warned                 = false;
            event.b_sensor_recovered = health.b_recovered;

            if (health.b_recovered)
            {
                tm_printf((UB *)"[B4] Sensor recovered: readings valid\n");
                fault_clear(FAULT_IMU);
            }

            if ((!b_have_previous) || (health.b_recovered))
            {
                CHECK(tk_set_flg(gh_events, S4_IMU_READY_FLAG), "IMU ready");
            }

            report_event(&data, &event, &b_have_previous, &previous,
                         &last_report);
        }
    }
}

/** @brief Publish without blocking; count dropped readings on a full queue. */
static bool
publish_reading (const imu_data_t * p_data, bool b_publish, UW * p_dropped)
{
    ER error_code = E_OK;

    if (b_publish)
    {
        error_code =
            tk_snd_mbf(gh_imu_data_buffer, p_data, sizeof(*p_data), TMO_POL);
    }

    if (E_TMOUT == error_code)
    {
        ++(*p_dropped);

        if ((1U == *p_dropped) || (0U == (*p_dropped % 100U)))
        {
            tm_printf((UB *)"[B4] Queue full: dropped readings=%u\n",
                      *p_dropped);
        }
    }
    else if (error_code < E_OK)
    {
        CHECK(error_code, "IMU send");
        return false;
    }
    else
    {
        /* Delivery succeeded or no fresh reading was available. */
    }

    return true;
}

/** @brief Clamp a telemetry value before narrowing to the signed 16-bit ABI. */
static H
telemetry_integer (float32_t value)
{
    int32_t converted = imu_numeric_to_int(value);

    if (converted > INT16_MAX)
    {
        converted = INT16_MAX;
    }
    else if (converted < INT16_MIN)
    {
        converted = INT16_MIN;
    }
    else
    {
        /* The converted value already fits the telemetry range. */
    }

    /* The bounds above ensure the value fits the kernel's signed 16-bit H. */
    return (H)converted;
}

/** @brief Convert a sensor value to the kernel console integer ABI. */
static INT
console_integer (float32_t value)
{
    /* The checked int32_t result fits the verified 32-bit kernel INT ABI. */
    return (INT)imu_numeric_to_int(value);
}

#if !S4_IMU_USE_MOCK
/** @brief Retry sensor startup and reset calibration history. */
static bool
initialise_hardware (imu_hardware_state_t * p_state, uint32_t now_ms)
{

    if ((p_state->b_attempted) && (now_ms - p_state->retry_ms < 1000U))
    {
        return true;
    }

    p_state->b_attempted = true;
    p_state->retry_ms    = now_ms;

    gy511_bus_t    bus;
    gy511_config_t config;

    gy511_default_config(&config);
    /* Configure signed axis mapping and magnetic calibration here after
     * checking the installed orientation. Defaults: sensor XYZ=robot
     * XYZ.
     */
    if ((!gy511_i2c_open(&bus)) ||
        (!gy511_init(&p_state->sensor, &bus, &config)))
    {
        tm_printf((UB *)"[B4] GY-511 not ready: check power/wiring; retry "
                        "in 1 s\n");
        return true;
    }

    p_state->b_initialised = true;
    p_state->b_calibrated  = false;
    p_state->count = p_state->failures = 0U;

    for (int32_t index = 0; index < IMU_AXIS_COUNT; ++index)
    {
        p_state->mean[index] = p_state->variance_sum[index] = 0.0f;
    }

    p_state->last_data_ms = now_ms;
    tm_printf((UB *)"[B4] GY-511 connected on I2C0 (GP4/GP5)\n");
    tm_printf((UB *)"[B4] Calibration starting: keep level/still for 200 "
                    "samples\n");
    return true;
}
#endif

#if !S4_IMU_USE_MOCK
/** @brief Collect level, still samples and reject unstable calibration. */
static void
calibrate_hardware (imu_hardware_state_t * p_state, imu_data_t * p_data)
{
    if (p_state->b_calibrated)
    {
        return;
    }

    p_data->b_valid = false;

    float32_t sample_accel[3] = {p_data->accel_x, p_data->accel_y,
                                 p_data->accel_z};

    ++p_state->count;

    for (int32_t index = 0; index < IMU_AXIS_COUNT; ++index)
    {
        float32_t delta = sample_accel[index] - p_state->mean[index];

        /* Calibration count is 1..200 and fits binary32 exactly. */
        p_state->mean[index] += delta / (float32_t)p_state->count;
        p_state->variance_sum[index] +=
            delta * (sample_accel[index] - p_state->mean[index]);
    }

    if (p_state->count != 200U)
    {
        return;
    }

    bool b_stable = true;

    for (int32_t index = 0; index < IMU_AXIS_COUNT; ++index)
    {
        if ((!imu_numeric_is_finite(p_state->mean[index])) ||
            (!imu_numeric_is_finite(p_state->variance_sum[index])) ||
            (p_state->variance_sum[index] / 199.0f > 0.04f))
        {
            b_stable = false;
        }
    }

    if (b_stable &&
        (gy511_set_level_calibration(&p_state->sensor, p_state->mean)))
    {
        p_state->b_calibrated = true;
        tm_printf((UB *)"[B4] Level calibration complete\n");
        tm_printf((UB *)"[B4] Magnetic calibration pending; turn rate "
                        "unavailable\n");
    }
    else
    {
        tm_printf((UB *)"[B4] Calibration rejected: keep level/still; "
                        "retrying\n");
    }

    p_state->count = 0U;

    for (int32_t index = 0; index < IMU_AXIS_COUNT; ++index)
    {
        p_state->mean[index] = p_state->variance_sum[index] = 0.0f;
    }
}
#endif

#if !S4_IMU_USE_MOCK
/** @brief Acquire a fresh reading or report an unavailable sensor. */
static bool
hardware_read (uint32_t now_ms, imu_data_t * p_data)
{
    static imu_hardware_state_t state;

    *p_data              = (imu_data_t){0};
    p_data->timestamp_ms = now_ms;

    if (!state.b_initialised)
    {
        return initialise_hardware(&state, now_ms);
    }

    gy511_result_t result = gy511_read(&state.sensor, now_ms, p_data);

    if (GY511_NO_DATA == result)
    {
        if (now_ms - state.last_data_ms <= 100U)
        {
            return false;
        }

        result = GY511_ERROR;
    }

    if (GY511_ERROR == result)
    {
        p_data->b_valid = false;
        state.count     = 0U;

        for (int32_t index = 0; index < IMU_AXIS_COUNT; ++index)
        {
            state.mean[index] = state.variance_sum[index] = 0.0f;
        }

        ++state.failures;

        if (state.failures >= 10U)
        {
            state.b_initialised = false;
        }

        return true;
    }

    state.last_data_ms = now_ms;
    state.failures     = 0U;
    calibrate_hardware(&state, p_data);
    return true;
}
#endif

/** @brief Return the console label for a motion classification. */
static const char *
motion_name (imu_motion_state_t state)
{
    switch (state)
    {
        case IMU_MOTION_STATIONARY:
            return "stationary";
        case IMU_MOTION_ACCELERATING:
            return "accelerating";
        case IMU_MOTION_TURNING:
            return "turning";
        case IMU_MOTION_CLIMBING_HUMP:
            return "climbing";
        case IMU_MOTION_HUMP_PEAK:
            return "peak";
        case IMU_MOTION_DESCENDING_HUMP:
            return "descending";
        case IMU_MOTION_SUDDEN_IMPACT:
            return "suspected-impact";
        default:
            return "unknown/quiet";
    }
}

/** @brief Print measurements with their units and availability. */
static void
print_readings (const imu_data_t * p_data, const imu_event_t * p_event)
{
    tm_printf((UB *)"[B4] Tilt Pitch=%d Roll=%d deg Detected=%s\n",
              console_integer(p_event->pitch_deg),
              console_integer(p_event->roll_deg),
              p_event->b_tilt_detected ? "yes" : "no");
    tm_printf((UB *)"[B4] Accel X=%d Y=%d Z=%d mg\n",
              console_integer(p_data->accel_x * 1000.0f / 9.81f),
              console_integer(p_data->accel_y * 1000.0f / 9.81f),
              console_integer(p_data->accel_z * 1000.0f / 9.81f));

    if (p_event->b_heading_valid)
    {
        tm_printf((UB *)"[B4] Heading=%d deg Calibrated=%s\n",
                  console_integer(p_event->heading_deg),
                  p_event->b_heading_calibrated ? "yes" : "no");
    }
    else
    {
        tm_printf((UB *)"[B4] Heading=N/A\n");
    }

    if (p_event->b_turn_rate_valid)
    {
        tm_printf((UB *)"[B4] TurnRate=%d deg/s\n",
                  console_integer(p_event->turn_rate_dps));
    }
    else
    {
        tm_printf((UB *)"[B4] TurnRate=N/A\n");
    }
#ifndef NDEBUG
#if S4_IMU_LOG_RAW_MAG
    if (p_data->b_magnetic_valid)
    {
        tm_printf((UB *)"[B4] Mag X=%d Y=%d Z=%d uT\n",
                  console_integer(p_data->magnetic_x_ut),
                  console_integer(p_data->magnetic_y_ut),
                  console_integer(p_data->magnetic_z_ut));
    }
    else
    {
        tm_printf((UB *)"[B4] Mag=N/A\n");
    }
#endif
#endif
}

/** @brief Receive a full sample or timestamp an invalid timeout sample. */
static bool
receive_reading (imu_data_t * p_data, TMO timeout_ms)
{
    INT    length;
    SYSTIM now;

    length = tk_rcv_mbf(gh_imu_data_buffer, p_data, timeout_ms);

    if ((length < E_OK) && (length != E_TMOUT))
    {
        CHECK(length, "IMU receive");
        return false;
    }

    if (length != (INT)sizeof(*p_data)) /* Fixed B4 message sizes fit the kernel
                                           signed size type. */
    {
        ER error_code = tk_get_tim(&now);

        if (error_code < E_OK)
        {
            CHECK(error_code, "IMU receive clock");
            return false;
        }

        *p_data              = (imu_data_t){0};
        p_data->timestamp_ms = now.lo;
    }

    return true;
}

/** @brief Report a newly latched fault and notify the test observer. */
static void
report_fault (const imu_processing_health_t * p_health, uint32_t timestamp_ms)
{
    (void)
        timestamp_ms; /* Used only by the test-event tap in APP_TEST builds. */

    if (p_health->b_fault_started)
    {
        tm_printf((UB *)"[B4] Sensor fault/unavailable: readings suppressed\n");
        CHECK(tk_clr_flg(gh_events, ~S4_IMU_READY_FLAG), "IMU not ready");
        fault_raise(FAULT_IMU);
        /* Discard the best-effort queue result; the shared fault stays set
         * even if the error notification cannot be queued. */
        (void)comm_report_error(ERR_S4_IMU_FAULT, 0U);
#if APP_TEST
        imu_event_t fault = {0};

        fault.motion_state   = IMU_MOTION_UNKNOWN;
        fault.timestamp_ms   = timestamp_ms;
        fault.b_sensor_fault = true;

        ER error_code =
            tk_snd_mbf(gh_imu_test_events, &fault, sizeof(fault), TMO_POL);

        if ((error_code < E_OK) && (error_code != E_TMOUT))
        {
            CHECK(error_code, "IMU fault event");
        }
#endif
    }
}

/** @brief Report motion changes, periodic values and hump completion. */
static void
report_event (const imu_data_t * p_data, const imu_event_t * p_event,
              bool * pb_have_previous, imu_motion_state_t * p_previous,
              UW * p_last_report)
{
    bool b_report_due = (!(*pb_have_previous)) ||
                        (p_event->b_sensor_recovered) ||
                        (p_event->timestamp_ms - (*p_last_report) >= 1000U);

    if ((!(*pb_have_previous)) || (p_event->motion_state != (*p_previous)) ||
        (p_event->b_hump_completed) || b_report_due)
    {
        if ((!(*pb_have_previous)) || (p_event->motion_state != (*p_previous)))
        {
            tm_printf((UB *)"[B4] Motion=%s\n",
                      motion_name(p_event->motion_state));
        }

        if (b_report_due)
        {
            print_readings(p_data, p_event);
            (*p_last_report) = p_event->timestamp_ms;
        }

        if (p_event->b_hump_completed)
        {
            tm_printf((UB *)"[B4] Hump completed: height=N/A\n");
            /* Unavailable heights remain explicitly invalid in B1. Discard
             * the best-effort queue result to avoid retrying in analysis. */
            (void)comm_report_hump(
                p_event->b_height_valid ? TRUE : FALSE,
                telemetry_integer(p_event->hump_peak_cm * 10.0f),
                telemetry_integer(p_event->highest_hump_cm * 10.0f));
        }

        /* Motion values are the shared 0..7 enum; the byte cast is exact. */
        comm_set_imu((UB)p_event->motion_state,
                     telemetry_integer(p_event->pitch_deg * 10.0f));
#if APP_TEST
        /* Test observer can lag without blocking analysis. */
        ER error_code =
            tk_snd_mbf(gh_imu_test_events, p_event, sizeof(*p_event), TMO_POL);

        if ((error_code < E_OK) && (error_code != E_TMOUT))
        {
            CHECK(error_code, "IMU test event");
        }
#endif
        (*p_previous)       = p_event->motion_state;
        (*pb_have_previous) = true;
    }
}

/*** end of file ***/

