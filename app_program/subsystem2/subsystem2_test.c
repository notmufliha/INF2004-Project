/*
 *    subsystem2_test.c
 *    On-target tests for subsystem 2.  Built only with: make TEST=1
 *    Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2_test.h"
#include "subsystem2.h"
#include "task_producer.h"
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
    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
