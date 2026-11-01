/*
 *    subsystem2_test.c
 *    On-target tests for subsystem 2.  Built only with: make TEST=1
 *    Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2_test.h"
#include "subsystem2.h"
#include "task_producer.h"
#include "motion_command.h"
#include "motion_profile.h"
#include "../test_report.h"

#define S2_TEST_SAMPLE_COUNT 40U

#if APP_TEST

EXPORT void subsystem2_test(void)
{
    T_RSEM    rsem;
    T_RMPF    rmpf;
    UW seq;
    INT value;
    INT bad = 0;
    motion_profile_t profile;
    motion_command_t invalid = {0};
    motion_status_snapshot_t snapshot;
    TEST_INIT();
    /* S2-U01: every sample value is inside the documented 0..100 range */
    for (seq = 0; seq < S2_TEST_SAMPLE_COUNT; seq++)
    {
        value = subsystem2_make_value(seq);
        if (value < 0 || value > 100)
        {
            bad++;
        }
    }
    TEST_CHECK("S2-U01", 0 == bad, "sample values stay within 0..100");
    /* S2-U02: flow control never over- or under-counts */
    TEST_CHECK("S2-U02",
        E_OK == tk_ref_sem(gh_sem_slots, &rsem) &&
        rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
        E_OK == tk_ref_mpf(gh_mpf_samples, &rmpf) &&
        rmpf.frbcnt <= S2_POOL_COUNT,
    "free slots and free blocks within pool size");
    profile = motion_profile_targets(20.0f, 0.0f, 10.0f,
        0.0f, 100.0f, false, true);
    TEST_CHECK("S2-U03", profile.left_speed < profile.right_speed,
        "straight correction slows leading wheel");
    profile = motion_profile_targets(20.0f, 0.0f, 100.0f,
        20.0f, 100.0f, true, true);
    TEST_CHECK("S2-U04", profile.left_reached && !profile.right_reached &&
        profile.left_speed == 0.0f && profile.right_speed > 0.0f,
        "one reached wheel cannot complete MOVE");
    invalid.type = MOTION_CMD_DRIVE;
    invalid.direction = MOTION_DIR_FORWARD;
    invalid.target_speed_cps = 1000;
    invalid.steering_permille = 1001;
    TEST_CHECK("S2-U05", motion_send_command(&invalid) == E_PAR,
        "invalid steering rejected without motion");
    TEST_CHECK("S2-U06", motion_get_status(&snapshot) == E_OK,
        "motion status available");
    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
