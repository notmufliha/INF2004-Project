#include "subsystem4.h"
#include "imu_analysis.h"
#include "imu_demo.h"

/* Only this task owns the mock. Replace acquisition with a real driver later. */
EXPORT void imu_sampling_task(INT stacd, void *exinf)
{
    imu_data_t data;
    SYSTIM now;
    UW origin;
    UW deadline;
    UW dropped = 0U;
    ER er;
    (void)stacd;
    (void)exinf;

    imu_mock_init();
    er = tk_get_tim(&now);
    if (er < E_OK) {
        CHECK(er, "IMU clock");
        tk_ext_tsk();
        return;
    }
    origin = now.lo;
    deadline = origin;
    tm_printf((UB*)"IMU: MOCK INPUT, 10 ms target, no motor commands\n");
    while (1) {
        er = tk_get_tim(&now);
        if (er < E_OK) {
            CHECK(er, "IMU clock");
            tk_ext_tsk();
            return;
        }
        imu_mock_set_scenario(imu_demo_scenario(now.lo - origin));
        if (imu_mock_read(&data)) {
            /* Use actual RTOS time, not the mock's synthetic sample count. */
            data.timestamp_ms = now.lo;
            er = tk_snd_mbf(mbf_imu_data, &data, sizeof(data), TMO_POL);
            if (er == E_TMOUT) {
                /* Never block acquisition on a full queue; drop newest. */
                if (++dropped == 1U || dropped % 100U == 0U) {
                    tm_printf((UB*)"IMU: dropped readings=%u\n", dropped);
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

EXPORT void imu_analysis_task(INT stacd, void *exinf)
{
    imu_analysis_t analysis;
    imu_analysis_config_t config;
    imu_data_t data;
    imu_event_t event;
    imu_motion_state_t previous = IMU_MOTION_UNKNOWN;
    bool have_previous = false;
    bool warned = false;
    INT length;
    (void)stacd;
    (void)exinf;

    imu_analysis_default_config(&config);
    if (!imu_analysis_init(&analysis, &config)) {
        tm_printf((UB*)"IMU: analysis configuration failed\n");
        tk_ext_tsk();
        return;
    }
    while (1) {
        length = tk_rcv_mbf(mbf_imu_data, &data, TMO_FEVR);
        if (length < E_OK) {
            CHECK(length, "IMU receive");
            tk_ext_tsk();
            return;
        }
        if (length != (INT)sizeof(data)) {
            data.valid = false;
        }
        if (!imu_analysis_process(&analysis, &data, &event)) {
            if (!warned) {
                tm_printf((UB*)"IMU: invalid reading/timing; terrain sequence reset\n");
            }
            warned = true;
            have_previous = false;
            continue;
        }
        warned = false;
        if (!have_previous || event.motion_state != previous || event.hump_completed) {
            /* Integer output: T-Monitor does not require float formatting. */
            tm_printf((UB*)"IMU MOCK: t=%u state=%s pitch=%d roll=%d turn=%d tilt=%d\n",
                      (UW)event.timestamp_ms, motion_name(event.motion_state),
                      (INT)event.pitch_deg, (INT)event.roll_deg,
                      (INT)event.turn_rate_dps, (INT)event.tilt_detected);
            if (event.hump_completed) {
                tm_printf((UB*)"IMU MOCK: hump completed; height unavailable\n");
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
