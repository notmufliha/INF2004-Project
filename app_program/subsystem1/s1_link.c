/** @file s1_link.c
 * @brief Subsystem 1 reconnect policy (see s1_link.h).
 *
 * Why the WiFi rule covers DOWN as well as the failure codes: when the
 * access point disappears (hotspot switched off, out of range) the
 * cyw43 driver handles the DISASSOC event by clearing its join state,
 * so the link reports DOWN (0), not a negative failure code.  The old
 * rule only re-joined on negative codes, so the car never came back
 * after the hotspot returned until it was power-cycled.
 */

#include "s1_link.h"

/* link_status: CYW43_LINK_xxx value from the driver.  elapsed_ms: time
 * since that status was first seen, or since the last join request. */
/** @brief Decide whether the WiFi task should re-join the access point. */
s1_link_wifi_action_t
s1_link_wifi_action (int32_t link_status, uint32_t elapsed_ms)
{
    s1_link_wifi_action_t action = S1_LINK_WIFI_WAIT;

    if (S1_LINK_STATUS_UP == link_status)
    {
        action = S1_LINK_WIFI_WAIT;
    }
    else if ((S1_LINK_STATUS_JOIN == link_status) ||
             (S1_LINK_STATUS_NOIP == link_status))
    {
        /* Still making progress: only intervene once it is stuck. */
        if (elapsed_ms >= S1_LINK_STUCK_MS)
        {
            action = S1_LINK_WIFI_RESET;
        }
    }
    else
    {
        /* DOWN, FAIL, NONET, BADAUTH, or a value this code does not
         * know: the association is over, so ask for a new one. */
        if (elapsed_ms >= S1_LINK_REJOIN_MS)
        {
            action = S1_LINK_WIFI_REJOIN;
        }
    }

    return (action);
}

/* attempt: 1 for the first retry after a failure, 2 for the next...
 * (0 counts as 1).  Returns 1000, 2000, 4000, 8000, 16000, 16000 ms for
 * attempts 1..6, then S1_LINK_OFFLINE_MS once the retries are used up. */
/** @brief Delay before MQTT connection attempt number @p attempt. */
uint32_t
s1_link_backoff_ms (uint32_t attempt)
{
    uint32_t delay_ms = S1_LINK_RETRY_FIRST_MS;
    uint32_t step;

    if (s1_link_backoff_done(attempt))
    {
        delay_ms = S1_LINK_OFFLINE_MS;
    }
    else
    {
        /* Double per attempt, stopping at the ceiling.  The loop runs
         * at most S1_LINK_RETRY_LIMIT times, so it cannot overflow. */
        for (step = 1U; step < attempt; step++)
        {
            if (delay_ms < S1_LINK_RETRY_MAX_MS)
            {
                delay_ms = delay_ms * 2U;
            }
        }

        if (delay_ms > S1_LINK_RETRY_MAX_MS)
        {
            delay_ms = S1_LINK_RETRY_MAX_MS;
        }
    }

    return (delay_ms);
}

/** @brief True once @p attempt is past the retry limit (go OFFLINE). */
bool
s1_link_backoff_done (uint32_t attempt)
{
    return (attempt > S1_LINK_RETRY_LIMIT);
}

/* A failed lookup (no DNS server from DHCP yet, a lost answer, or a
 * timeout) must not stop the car reconnecting: the last address that
 * worked is used instead, if there is one (NFR-B1-7). */
/** @brief Pick the broker address after a DNS lookup has finished. */
s1_link_broker_t
s1_link_broker_choice (bool b_dns_ok, bool b_have_cached)
{
    s1_link_broker_t choice = S1_LINK_BROKER_RETRY;

    if (b_dns_ok)
    {
        choice = S1_LINK_BROKER_RESOLVED;
    }
    else if (b_have_cached)
    {
        choice = S1_LINK_BROKER_CACHED;
    }
    else
    {
        choice = S1_LINK_BROKER_RETRY;
    }

    return (choice);
}

/*** end of file ***/

