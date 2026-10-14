/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	UW	before;
    imu_event_t event;
    SYSTIM start, now;
    BOOL completed = FALSE;
    INT len;

	TEST_INIT();

	/* S4-U01: the message buffer accepts the documented line length */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");

	/* INT-02 (integration, IF-03): summary lines from subsystem 3 reach
	   the monitor.  One line per 5 samples = one per 1.5 s. */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");

    TEST_CHECK("S4-U10",
               tk_ref_mbf(mbf_imu_data, &rmbf) == E_OK &&
               rmbf.maxmsz == sizeof(imu_data_t),
               "IMU message buffer carries a complete reading");
    /* Drain old observations, then require a fresh completion within a cycle.
     * Test only reads the event tap, never steals sampling-task readings. */
    while (tk_rcv_mbf(mbf_imu_test_events, &event, TMO_POL) > 0) {
    }
    tk_get_tim(&start);
    do {
        len = tk_rcv_mbf(mbf_imu_test_events, &event, 100);
        if (len == sizeof(event) && event.valid && event.hump_completed &&
            !event.height_valid) {
            completed = TRUE;
            break;
        }
        tk_get_tim(&now);
    } while (now.lo - start.lo < 9000U);
    TEST_CHECK("S4-U11", completed,
               "mock readings flow through RTOS tasks to a fresh hump report");
	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
