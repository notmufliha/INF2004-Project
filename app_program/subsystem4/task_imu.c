#include "subsystem4.h"
#include "imu_analysis.h"
#include "imu_demo.h"
#include "imu_processing.h"
#include "gy511_i2c.h"

#if !S4_IMU_USE_MOCK
static bool hardware_read(uint32_t now_ms, imu_data_t *data)
{
    static gy511_t sensor;
    static bool initialised, calibrated, attempted;
    static uint32_t retry_ms, last_data_ms;
    static unsigned int count, failures;
    static float mean[3], m2[3];
    *data = (imu_data_t){0};
    data->timestamp_ms = now_ms;
    if (!initialised) {
        if (attempted && now_ms - retry_ms < 1000U) { return true; }
        attempted = true;
        retry_ms = now_ms;
        gy511_bus_t bus;
        gy511_config_t config;
        gy511_default_config(&config);
        /* Configure signed axis mapping and magnetic calibration here after
         * checking the installed orientation. Defaults: sensor XYZ=robot XYZ. */
        if (!gy511_i2c_open(&bus) || !gy511_init(&sensor, &bus, &config)) {
            tm_printf((UB*)"[B4] GY-511 not ready: check power/wiring; retry in 1 s\n");
            return true;
        }
        initialised = true;
        calibrated = false;
        count = failures = 0U;
        for (int i = 0; i < 3; ++i) { mean[i] = m2[i] = 0.0f; }
        last_data_ms = now_ms;
        tm_printf((UB*)"[B4] GY-511 connected on I2C0 (GP4/GP5)\n");
        tm_printf((UB*)"[B4] Calibration starting: keep level/still for 200 samples\n");
        return true;
    }
    gy511_result_t result = gy511_read(&sensor, now_ms, data);
    if (result == GY511_NO_DATA) {
        if (now_ms - last_data_ms <= 100U) { return false; }
        result = GY511_ERROR;
    }
    if (result == GY511_ERROR) {
        data->valid = false;
        count = 0U;
        for (int i = 0; i < 3; ++i) { mean[i] = m2[i] = 0.0f; }
        if (++failures >= 10U) { initialised = false; }
        return true;
    }
    last_data_ms = now_ms;
    failures = 0U;
    if (!calibrated) {
        float a[3] = {data->accel_x, data->accel_y, data->accel_z};
        ++count;
        for (int i = 0; i < 3; ++i) {
            float delta = a[i] - mean[i];
            mean[i] += delta / (float)count;
            m2[i] += delta * (a[i] - mean[i]);
        }
        if (count == 200U) {
            bool stable = true;
            for (int i = 0; i < 3; ++i) {
                if (m2[i] / 199.0f > 0.04f) { stable = false; }
            }
            if (stable && gy511_set_level_calibration(&sensor, mean)) {
                calibrated = true;
                tm_printf((UB*)"[B4] Level calibration complete\n");
                tm_printf((UB*)"[B4] Magnetic calibration pending; turn rate unavailable\n");
            } else {
                tm_printf((UB*)"[B4] Calibration rejected: keep level/still; retrying\n");
            }
            count = 0U;
            for (int i = 0; i < 3; ++i) { mean[i] = m2[i] = 0.0f; }
        }
        data->valid = false;
    }
    return true;
}
#endif

/* Only this task owns sensor acquisition and I2C0. */
EXPORT void imu_sampling_task(INT stacd, void *exinf)
{
    imu_data_t data;
    SYSTIM now;
#if S4_IMU_USE_MOCK
    UW origin;
#endif
    UW deadline;
    UW dropped = 0U;
    ER er;
    (void)stacd;
    (void)exinf;

#if S4_IMU_USE_MOCK
    imu_mock_init();
#endif
    er = tk_get_tim(&now);
    if (er < E_OK) {
        CHECK(er, "IMU clock");
        tk_ext_tsk();
        return;
    }
#if S4_IMU_USE_MOCK
    origin = now.lo;
    tm_printf((UB*)"[B4] IMU starting: MOCK input, target period=10 ms\n");
#else
    tm_printf((UB*)"[B4] IMU starting: REAL GY-511, target period=10 ms\n");
#endif
    deadline = now.lo;
    while (1) {
        er = tk_get_tim(&now);
        if (er < E_OK) {
            CHECK(er, "IMU clock");
            tk_ext_tsk();
            return;
        }
        {
            data = (imu_data_t){0};
#if S4_IMU_USE_MOCK
            imu_mock_set_scenario(imu_demo_scenario(now.lo - origin));
            data.valid = imu_mock_read(&data);
            bool publish = true;
#else
            bool publish = hardware_read(now.lo, &data);
#endif
            /* Use actual RTOS time, not the mock's synthetic sample count. */
            data.timestamp_ms = now.lo;
            er = publish ? tk_snd_mbf(mbf_imu_data, &data, sizeof(data), TMO_POL) : E_OK;
            if (er == E_TMOUT) {
                /* Never block acquisition on a full queue; drop newest. */
                if (++dropped == 1U || dropped % 100U == 0U) {
                    tm_printf((UB*)"[B4] Queue full: dropped readings=%u\n", dropped);
                }
            } else if (er < E_OK) {
                CHECK(er, "IMU send");
                tk_ext_tsk();
                return;
            }
        }
        deadline += S4_IMU_PERIOD_MS;
        er = tk_get_tim(&now);
        if (er < E_OK) {
            CHECK(er, "IMU clock");
            tk_ext_tsk();
            return;
        }
        /* Deadline scheduling avoids adding processing time to each period.
         * On overrun skip catch-up bursts and yield for a full period. */
        if ((W)(deadline - now.lo) <= 0) {
            deadline = now.lo + S4_IMU_PERIOD_MS;
        }
        er = tk_dly_tsk((RELTIM)(deadline - now.lo));
        if (er < E_OK) {
            CHECK(er, "IMU delay");
            tk_ext_tsk();
            return;
        }
    }
}

static const char *motion_name(imu_motion_state_t state)
{
    switch (state) {
        case IMU_MOTION_STATIONARY: return "stationary";
        case IMU_MOTION_ACCELERATING: return "accelerating";
        case IMU_MOTION_TURNING: return "turning";
        case IMU_MOTION_CLIMBING_HUMP: return "climbing";
        case IMU_MOTION_HUMP_PEAK: return "peak";
        case IMU_MOTION_DESCENDING_HUMP: return "descending";
        case IMU_MOTION_SUDDEN_IMPACT: return "suspected-impact";
        default: return "unknown/quiet";
    }
}

/* Short integer-only lines match the other buddies' console format. */
static void print_readings(const imu_data_t *data, const imu_event_t *event)
{
    tm_printf((UB*)"[B4] Tilt Pitch=%d Roll=%d deg Detected=%s\n",
              (INT)event->pitch_deg, (INT)event->roll_deg,
              event->tilt_detected ? "yes" : "no");
    tm_printf((UB*)"[B4] Accel X=%d Y=%d Z=%d mg\n",
              (INT)(data->accel_x * 1000.0f / 9.81f),
              (INT)(data->accel_y * 1000.0f / 9.81f),
              (INT)(data->accel_z * 1000.0f / 9.81f));
    if (event->heading_valid) {
        tm_printf((UB*)"[B4] Heading=%d deg Calibrated=%s\n",
                  (INT)event->heading_deg, event->heading_calibrated ? "yes" : "no");
    } else {
        tm_printf((UB*)"[B4] Heading=N/A\n");
    }
    if (event->turn_rate_valid) {
        tm_printf((UB*)"[B4] TurnRate=%d deg/s\n", (INT)event->turn_rate_dps);
    } else {
        tm_printf((UB*)"[B4] TurnRate=N/A\n");
    }
#if S4_IMU_LOG_RAW_MAG
    if (data->magnetic_valid) {
        tm_printf((UB*)"[B4] Mag X=%d Y=%d Z=%d uT\n",
                  (INT)data->magnetic_x_ut, (INT)data->magnetic_y_ut,
                  (INT)data->magnetic_z_ut);
    } else {
        tm_printf((UB*)"[B4] Mag=N/A\n");
    }
#endif
}

EXPORT void imu_analysis_task(INT stacd, void *exinf)
{
    imu_analysis_t analysis;
    imu_analysis_config_t config;
    imu_data_t data;
    imu_data_t processed;
    imu_processing_t processing;
    imu_processing_config_t processing_config;
    imu_health_t health;
    SYSTIM now;
    imu_event_t event;
    imu_motion_state_t previous = IMU_MOTION_UNKNOWN;
    bool have_previous = false;
    bool warned = false;
    UW last_report = 0U;
    INT length;
    (void)stacd;
    (void)exinf;

    imu_analysis_default_config(&config);
    imu_processing_default_config(&processing_config);
    if (!imu_processing_init(&processing, &processing_config)) {
        tm_printf((UB*)"[B4] ERROR: processing configuration failed\n");
        tk_ext_tsk();
        return;
    }
    if (!imu_analysis_init(&analysis, &config)) {
        tm_printf((UB*)"[B4] ERROR: analysis configuration failed\n");
        tk_ext_tsk();
        return;
    }
    while (1) {
        length = tk_rcv_mbf(mbf_imu_data, &data, processing_config.max_gap_ms);
        if (length < E_OK && length != E_TMOUT) {
            CHECK(length, "IMU receive");
            tk_ext_tsk();
            return;
        }
        if (length != (INT)sizeof(data)) {
            ER er = tk_get_tim(&now);
            if (er < E_OK) {
                CHECK(er, "IMU receive clock");
                tk_ext_tsk();
                return;
            }
            data = (imu_data_t){0};
            data.timestamp_ms = now.lo;
        }
        bool usable = imu_processing_process(&processing, &data, data.timestamp_ms,
                                              &processed, &health);
        if (health.fault_started) {
            tm_printf((UB*)"[B4] Sensor fault/unavailable: readings suppressed\n");
#if APP_TEST
            imu_event_t fault = {0};
            fault.motion_state = IMU_MOTION_UNKNOWN;
            fault.timestamp_ms = data.timestamp_ms;
            fault.sensor_fault = true;
            ER er = tk_snd_mbf(mbf_imu_test_events, &fault, sizeof(fault), TMO_POL);
            if (er < E_OK && er != E_TMOUT) { CHECK(er, "IMU fault event"); }
#endif
        }
        /* An invalid processed reading resets any incomplete terrain sequence. */
        if (!imu_analysis_process(&analysis, &processed, &event) || !usable) {
            if (!warned) {
                tm_printf((UB*)"[B4] Invalid reading/timing: terrain sequence reset\n");
            }
            warned = true;
            have_previous = false;
            continue;
        }
        warned = false;
        event.sensor_recovered = health.recovered;
        if (health.recovered) {
            tm_printf((UB*)"[B4] Sensor recovered: readings valid\n");
        }
        bool report_due = !have_previous || health.recovered ||
                          event.timestamp_ms - last_report >= 1000U;
        if (!have_previous || event.motion_state != previous || event.hump_completed || report_due) {
            if (!have_previous || event.motion_state != previous) {
                tm_printf((UB*)"[B4] Motion=%s\n", motion_name(event.motion_state));
            }
            if (report_due) {
                print_readings(&data, &event);
                last_report = event.timestamp_ms;
            }
            if (event.hump_completed) {
                tm_printf((UB*)"[B4] Hump completed: height=N/A\n");
            }
#if APP_TEST
            /* Test observer can lag without blocking analysis. */
            ER er = tk_snd_mbf(mbf_imu_test_events, &event, sizeof(event), TMO_POL);
            if (er < E_OK && er != E_TMOUT) {
                CHECK(er, "IMU test event");
            }
#endif
            previous = event.motion_state;
            have_previous = true;
        }
    }
}
