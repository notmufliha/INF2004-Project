/** @file testlink_main.c
 * @brief Host unit tests for Subsystem 1's reconnect policy (s1_link.c).
 *
 * Boundary tests in the Lecture 6 style: every limit is tested just
 * below and exactly at its threshold, plus out-of-range inputs.  Test
 * IDs (S1-H02..H05) trace to NFR-B1-6 / NFR-B1-7 / FR-B1-7 in
 * testing/subsystem1/REQUIREMENTS_TRACE.md.
 *
 * Exit status 0 = all checks passed.
 */

#include "testlink_main.h"
#include <stdio.h>
#include "../../../app_program/subsystem1/s1_link.h"

static int32_t g_checks   = 0;
static int32_t g_failures = 0;

static void expect (const char * p_id, bool b_ok, const char * p_what);
static void test_rejoin (void);
static void test_progress (void);
static void test_backoff (void);
static void test_broker_choice (void);

/** @brief Run every check; exit status 0 = all passed. */
int
main (void)
{
    test_rejoin();
    test_progress();
    test_backoff();
    test_broker_choice();

    printf("s1_link: %d checks, %d failed\n", (int)g_checks, (int)g_failures);

    return ((0 == g_failures) ? 0 : 1);
}

/** @brief Count one check; print the ID and a reason if it failed. */
static void
expect (const char * p_id, bool b_ok, const char * p_what)
{
    g_checks++;
    if (!b_ok)
    {
        g_failures++;
        printf("FAIL %s: %s\n", p_id, p_what);
    }
}

/** @brief S1-H02: re-join after S1_LINK_REJOIN_MS for DOWN and failures.
 *
 * DOWN (0) is the regression case: the AP vanished (hotspot off), which
 * the old rule ignored, so the car never re-joined.
 */
static void
test_rejoin (void)
{
    static const int32_t lost[] =
    {
        S1_LINK_STATUS_DOWN, S1_LINK_STATUS_FAIL, S1_LINK_STATUS_NONET,
        S1_LINK_STATUS_BADAUTH, -9, 7 /* unknown values: treat as lost */
    };
    uint32_t index;

    for (index = 0U; index < (sizeof(lost) / sizeof(lost[0])); index++)
    {
        expect("S1-H02", S1_LINK_WIFI_WAIT ==
                   s1_link_wifi_action(lost[index], S1_LINK_REJOIN_MS - 1U),
               "no re-join just before 5 s");
        expect("S1-H02", S1_LINK_WIFI_REJOIN ==
                   s1_link_wifi_action(lost[index], S1_LINK_REJOIN_MS),
               "re-join at exactly 5 s");
        expect("S1-H02", S1_LINK_WIFI_REJOIN ==
                   s1_link_wifi_action(lost[index], UINT32_MAX),
               "re-join after a very long outage");
    }
}

/** @brief S1-H03: UP never acts; JOIN/NOIP only reset once stuck. */
static void
test_progress (void)
{
    static const int32_t busy[] = {S1_LINK_STATUS_JOIN, S1_LINK_STATUS_NOIP};
    uint32_t             index;

    expect("S1-H03", S1_LINK_WIFI_WAIT ==
               s1_link_wifi_action(S1_LINK_STATUS_UP, 0U), "UP: wait");
    expect("S1-H03", S1_LINK_WIFI_WAIT ==
               s1_link_wifi_action(S1_LINK_STATUS_UP, UINT32_MAX),
           "UP: never re-join, however long");

    for (index = 0U; index < (sizeof(busy) / sizeof(busy[0])); index++)
    {
        expect("S1-H03", S1_LINK_WIFI_WAIT ==
                   s1_link_wifi_action(busy[index], S1_LINK_REJOIN_MS),
               "joining: do not interrupt at 5 s");
        expect("S1-H03", S1_LINK_WIFI_WAIT ==
                   s1_link_wifi_action(busy[index], S1_LINK_STUCK_MS - 1U),
               "joining: still waiting just before 20 s");
        expect("S1-H03", S1_LINK_WIFI_RESET ==
                   s1_link_wifi_action(busy[index], S1_LINK_STUCK_MS),
               "joining: reset at exactly 20 s");
    }
}

/** @brief S1-H04: MQTT backoff 1, 2, 4, 8, 16, 16 s, then OFFLINE. */
static void
test_backoff (void)
{
    static const uint32_t want[] =
    {
        1000U, 1000U, 2000U, 4000U, 8000U, 16000U, 16000U
    };
    uint32_t attempt;

    for (attempt = 0U; attempt <= S1_LINK_RETRY_LIMIT; attempt++)
    {
        expect("S1-H04", want[attempt] == s1_link_backoff_ms(attempt),
               "backoff delay for attempt 0..6");
        expect("S1-H04", !s1_link_backoff_done(attempt),
               "attempts 0..6 are retries");
    }

    expect("S1-H04", s1_link_backoff_done(S1_LINK_RETRY_LIMIT + 1U),
           "attempt 7 goes OFFLINE");
    expect("S1-H04", S1_LINK_OFFLINE_MS ==
               s1_link_backoff_ms(S1_LINK_RETRY_LIMIT + 1U),
           "OFFLINE pause is 60 s");
    expect("S1-H04", S1_LINK_OFFLINE_MS == s1_link_backoff_ms(UINT32_MAX),
           "huge attempt count does not overflow");
}

/** @brief S1-H05: a failed DNS lookup falls back to the last address. */
static void
test_broker_choice (void)
{
    expect("S1-H05", S1_LINK_BROKER_RESOLVED ==
               s1_link_broker_choice(true, false), "DNS ok, no cache");
    expect("S1-H05", S1_LINK_BROKER_RESOLVED ==
               s1_link_broker_choice(true, true), "DNS ok wins over cache");
    expect("S1-H05", S1_LINK_BROKER_CACHED ==
               s1_link_broker_choice(false, true), "DNS failed: use cache");
    expect("S1-H05", S1_LINK_BROKER_RETRY ==
               s1_link_broker_choice(false, false), "nothing: back off");
}

/*** end of file ***/

