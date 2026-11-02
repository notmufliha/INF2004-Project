/** @file s1_link.h
 * @brief Subsystem 1 reconnect policy: when to re-join WiFi, how long to
 * back off between MQTT attempts, and which broker address to use when
 * a DNS lookup fails.
 *
 * Pure functions with no kernel, lwIP or cyw43 calls, so the policy is
 * unit-tested on the laptop (testing/subsystem1/host/test_s1_link.c)
 * and shared by the two kernel-side callers:
 *
 *   lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c  (WiFi re-join)
 *   lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c   (MQTT backoff, DNS)
 *
 * RULE: this header includes <stdint.h> and <stdbool.h> only.  The two
 * callers live in the pico-sdk/lwIP namespace and must never see
 * tk/tkernel.h (see lib/liblwip/sysdepend/pico_rp2040/include/arch/cc.h).
 *
 * Requirements: FR-B1-7 (reconnect), NFR-B1-6 (WiFi re-join timing),
 * NFR-B1-7 (DNS failure must not stop reconnection).  See
 * testing/subsystem1/REQUIREMENTS_TRACE.md.
 */

#ifndef S1_LINK_H
#define S1_LINK_H

#include <stdint.h>
#include <stdbool.h>

/* WiFi link status, same values as CYW43_LINK_xxx in cyw43.h.  The
 * cyw43 port checks at compile time that the two sets still match. */
#define S1_LINK_STATUS_BADAUTH  (-3)
#define S1_LINK_STATUS_NONET    (-2)
#define S1_LINK_STATUS_FAIL     (-1)
#define S1_LINK_STATUS_DOWN     (0)
#define S1_LINK_STATUS_JOIN     (1)
#define S1_LINK_STATUS_NOIP     (2)
#define S1_LINK_STATUS_UP       (3)

/* Re-join a failed or dropped association after this long (ms). */
#define S1_LINK_REJOIN_MS       5000U

/* A join that has neither finished nor failed after this long (ms) is
 * treated as stuck: leave and join again.  Covers an AP that stops
 * answering mid-handshake and a DHCP server that never replies. */
#define S1_LINK_STUCK_MS        20000U

/* MQTT reconnect: 1, 2, 4, 8, 16, 16 s, then a long OFFLINE pause. */
#define S1_LINK_RETRY_FIRST_MS  1000U
#define S1_LINK_RETRY_MAX_MS    16000U
#define S1_LINK_RETRY_LIMIT     6U
#define S1_LINK_OFFLINE_MS      60000U

/* Longest wait for one DNS answer before the fallback is used (ms). */
#define S1_LINK_DNS_TIMEOUT_MS  5000U

/** @brief What the WiFi service task should do on this poll. */
typedef enum
{
    S1_LINK_WIFI_WAIT = 0, /* nothing: link up, or still in progress */
    S1_LINK_WIFI_REJOIN,   /* issue a new join request               */
    S1_LINK_WIFI_RESET     /* leave the AP, then join again          */
} s1_link_wifi_action_t;

/** @brief Which broker address the MQTT client should connect to. */
typedef enum
{
    S1_LINK_BROKER_RESOLVED = 0, /* the address DNS just returned     */
    S1_LINK_BROKER_CACHED,       /* the last address that worked      */
    S1_LINK_BROKER_RETRY         /* none usable: back off, retry DNS  */
} s1_link_broker_t;

s1_link_wifi_action_t s1_link_wifi_action (int32_t  link_status,
                                           uint32_t elapsed_ms);
uint32_t              s1_link_backoff_ms (uint32_t attempt);
bool                  s1_link_backoff_done (uint32_t attempt);
s1_link_broker_t      s1_link_broker_choice (bool b_dns_ok,
                                             bool b_have_cached);

#endif /* S1_LINK_H */

/*** end of file ***/

