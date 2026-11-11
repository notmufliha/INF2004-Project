/** @file mqtt_utk.c
 * @brief MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1),
 * owned by Subsystem 1 (WiFi & Telemetry).
 *
 * Runs entirely on the single task that already owns the CYW43439 and
 * the rest of lwIP (docs/RELEASE.md safety rule 6): cyw43_utk_task()
 * calls mqtt_utk_start() on every poll while the link is up.  mqtt.c's
 * keepalive timer and the pump/retry/DNS timers below all ride the
 * sys_check_timeouts() polling that task already does for DHCP and TCP.
 *
 * Behaviour (Buddy 1 statechart, testing/subsystem1/DESIGN_B1.md):
 *   - A PUMP timer drains subsystem 1's outgoing queue
 *     (comm_bridge_tx_pop()) and publishes each frame on
 *     <prefix>/telemetry | event | status | cmd/ack.
 *   - Subscribes to <prefix>/cmd and hands every command to subsystem 1
 *     (comm_bridge_rx_command()).
 *   - Retained "online" status on connect plus a Last Will "offline".
 *   - Reconnect with backoff 1, 2, 4, 8, 16, 16 s, then a 60 s OFFLINE
 *     pause (policy in app_program/subsystem1/s1_link.c).
 *   - Keepalive 10 s, so a lost broker is noticed in about 15 s.
 *   - WiFi loss closes the session at once (mqtt_utk_link_down()).
 *   - Optional broker host name (MQTT_BROKER_HOST): looked up again on
 *     every connection attempt, with its own timeout, and falling back
 *     to the last address that worked if DNS fails.
 *
 * DNS ON RECONNECT -- the two traps this file avoids:
 *   1. lwIP can call the DNS callback INSIDE dns_gethostbyname(), before
 *      it returns ERR_INPROGRESS: it does so when no DNS server is set,
 *      which is exactly the state just after a WiFi re-join, before DHCP
 *      has finished.  So the timeout is armed and the "pending" flag set
 *      BEFORE the call, and finish_lookup() runs only once per lookup.
 *   2. A slow answer can arrive after the timeout already gave up, or
 *      after WiFi dropped again.  Every lookup carries a generation
 *      number; answers for an older generation are ignored.
 *
 * This file does not include tk/tkernel.h or any cyw43 header, so it is
 * not subject to the size_t workaround; tm/tmonitor.h alone is
 * namespace-safe here, and comm_bridge.h / s1_link.h are stdint-only.
 */

#include "mqtt_utk.h"
#include <stdbool.h>
#include <stdint.h>
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "../../../../app_program/subsystem1/comm_bridge.h"
#include "../../../../app_program/subsystem1/s1_link.h"
#include <tm/tmonitor.h>

#ifndef MQTT_TOPIC_PREFIX
#define MQTT_TOPIC_PREFIX COMM_TOPIC_PREFIX_DEFAULT
#endif

#define MQTT_UTK_KEEPALIVE_S  10U /* broker loss noticed in ~1.5x this */
#define MQTT_UTK_PUMP_MS      20U /* how often the TX queue is drained  */
#define MQTT_UTK_PUMP_BURST   4U  /* max publishes per pump             */
#define MQTT_UTK_QOS_AT_MOST  0U
#define MQTT_UTK_QOS_AT_LEAST 1U
#define MQTT_UTK_RETAIN       1U

static mqtt_client_t * gp_client        = NULL;
static ip_addr_t       g_broker_addr;
static ip_addr_t       g_cached_addr;   /* last address that worked     */
static bool            gb_have_cached   = false;
static int32_t         g_state          = COMM_MQTT_IDLE;
static uint32_t        g_retries        = 0U;
static uint32_t        g_dns_generation = 0U; /* tags each lookup        */
static bool            gb_dns_pending   = false;

/* Full topic names, built once from MQTT_TOPIC_PREFIX */
static char g_topic_tx[COMM_TOPIC_COUNT][COMM_TOPIC_MAX];
static char g_topic_cmd[COMM_TOPIC_MAX];
static bool gb_topics_built = false;

/* One outgoing frame waiting to be published, kept across a failed
 * attempt so a full output buffer delays it instead of losing it. */
static uint8_t g_pend_payload[COMM_PAYLOAD_MAX];
static int32_t g_pend_len    = 0;
static uint8_t g_pend_topic  = 0U;
static uint8_t g_pend_retain = 0U;
static bool    gb_pend_valid = false;

/* Inbound staging: one topic callback, then data fragments up to the
 * one flagged MQTT_DATA_FLAG_LAST.  +1 byte marks "too long". */
static bool     gb_rx_is_cmd = false;
static uint8_t  g_rx_data[COMM_CMD_MAX + 1];
static uint16_t g_rx_len = 0U;

static void set_state (int32_t new_state);
static void build_topics (void);
static void join_topic (char * p_dst, const char * p_left,
                        const char * p_right);
static bool str_equal (const char * p_left, const char * p_right);
static void schedule_retry (void);
static void retry_timeout (void * p_arg);
static void seed_cached_address (void);
static void begin_attempt (void);
static void connect_broker (void);
#ifdef MQTT_BROKER_HOST
static void resolve_broker (void);
static void dns_found_cb (const char * p_name, const ip_addr_t * p_addr,
                          void * p_arg);
#endif
static void dns_timeout (void * p_arg);
static void finish_lookup (const ip_addr_t * p_addr);
static void incoming_publish_cb (void * p_arg, const char * p_topic,
                                 u32_t total_len);
static void incoming_data_cb (void * p_arg, const u8_t * p_data,
                              u16_t len, u8_t flags);
static void subscribe_done_cb (void * p_arg, err_t result);
static void publish_done_cb (void * p_arg, err_t result);
static bool publish_pending (void);
static void pump_timeout (void * p_arg);
static void connection_cb (mqtt_client_t * p_client, void * p_arg,
                           mqtt_connection_status_t status);

/** @brief Connect if idle (see mqtt_utk.h); called on every WiFi poll. */
void
mqtt_utk_start (void)
{
    if (COMM_MQTT_IDLE == g_state)
    {
        if (!gb_topics_built)
        {
            build_topics();
            seed_cached_address();
        }

        if (NULL == gp_client)
        {
            gp_client = mqtt_client_new();
        }

        if (NULL == gp_client)
        {
            tm_printf((UB *)"mqtt: mqtt_client_new() failed\n");
            schedule_retry();
        }
        else
        {
            begin_attempt();
        }
    }
}

/** @brief WiFi left UP: close the session and reset (see mqtt_utk.h). */
void
mqtt_utk_link_down (void)
{
    sys_untimeout(retry_timeout, NULL);
    sys_untimeout(pump_timeout, NULL);
    sys_untimeout(dns_timeout, NULL);
    g_dns_generation++; /* any answer still on its way is now stale */
    gb_dns_pending = false;

    if (NULL != gp_client)
    {
        /* mqtt.c marks the client disconnected BEFORE closing, so this
         * does not call connection_cb() and cannot double-schedule. */
        mqtt_disconnect(gp_client);
    }

    if (COMM_MQTT_IDLE != g_state)
    {
        tm_printf((UB *)"mqtt: WiFi lost, session closed\n");
    }

    g_retries = 0U;
    set_state(COMM_MQTT_IDLE);
}

/** @brief Record a state change and tell subsystem 1. */
static void
set_state (int32_t new_state)
{
    g_state = new_state;
    comm_bridge_mqtt_state(new_state);
}

/** @brief Build "<prefix>/<suffix>" for every topic, once. */
static void
build_topics (void)
{
    static const char * const suffix[COMM_TOPIC_COUNT] = COMM_TOPIC_SUFFIXES;
    uint32_t                  topic;

    for (topic = 0U; topic < (uint32_t)COMM_TOPIC_COUNT; topic++)
    {
        join_topic(g_topic_tx[topic], MQTT_TOPIC_PREFIX, suffix[topic]);
    }

    join_topic(g_topic_cmd, MQTT_TOPIC_PREFIX, COMM_TOPIC_CMD_SUFFIX);
    gb_topics_built = true;
}

/** @brief p_dst = p_left + "/" + p_right, NUL-terminated, cut to fit. */
static void
join_topic (char * p_dst, const char * p_left, const char * p_right)
{
    uint32_t used = 0U;

    while (('\0' != *p_left) && (used < (COMM_TOPIC_MAX - 1U)))
    {
        p_dst[used] = *p_left;
        used++;
        p_left++;
    }

    if (used < (COMM_TOPIC_MAX - 1U))
    {
        p_dst[used] = '/';
        used++;
    }

    while (('\0' != *p_right) && (used < (COMM_TOPIC_MAX - 1U)))
    {
        p_dst[used] = *p_right;
        used++;
        p_right++;
    }

    p_dst[used] = '\0';
}

/** @brief True if the two NUL-terminated strings are identical. */
static bool
str_equal (const char * p_left, const char * p_right)
{
    while (('\0' != *p_left) && (*p_left == *p_right))
    {
        p_left++;
        p_right++;
    }

    return (*p_left == *p_right);
}

/** @brief Wait before the next attempt (s1_link_backoff_ms() policy). */
static void
schedule_retry (void)
{
    uint32_t delay_ms;

    sys_untimeout(retry_timeout, NULL);
    g_retries++;
    delay_ms = s1_link_backoff_ms(g_retries);

    if (s1_link_backoff_done(g_retries))
    {
        g_retries = 0U;
        set_state(COMM_MQTT_OFFLINE);
        tm_printf((UB *)"mqtt: broker unreachable, OFFLINE for %ds\n",
                  (int)(delay_ms / 1000U));
    }
    else
    {
        set_state(COMM_MQTT_BACKOFF);
        tm_printf((UB *)"mqtt: retry %d in %dms\n", (int)g_retries,
                  (int)delay_ms);
    }

    sys_timeout(delay_ms, retry_timeout, NULL);
}

/** @brief Backoff over: the next mqtt_utk_start() tries again. */
static void
retry_timeout (void * p_arg)
{
    (void)p_arg;
    set_state(COMM_MQTT_IDLE);
}

/** @brief Use MQTT_BROKER_IP as the first fallback address, if valid. */
static void
seed_cached_address (void)
{
    if (0 != ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_cached_addr)))
    {
        IP_SET_TYPE(&g_cached_addr, IPADDR_TYPE_V4);
        gb_have_cached = true;
    }
}

/** @brief Resolve the host name, or connect to the configured IP. */
static void
begin_attempt (void)
{
#ifdef MQTT_BROKER_HOST
    resolve_broker();
#else
    if (gb_have_cached)
    {
        ip_addr_copy(g_broker_addr, g_cached_addr);
        connect_broker();
    }
    else
    {
        /* Config error (bad MQTT_BROKER_IP): retrying cannot help, so
         * stay OFFLINE until the next link change. */
        tm_printf((UB *)"mqtt: bad MQTT_BROKER_IP \"%s\"\n", MQTT_BROKER_IP);
        set_state(COMM_MQTT_OFFLINE);
    }
#endif
}

/** @brief Open the MQTT session to g_broker_addr. */
static void
connect_broker (void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             err;

    client_info.client_id   = MQTT_CLIENT_ID;
    client_info.keep_alive  = MQTT_UTK_KEEPALIVE_S;
#ifdef MQTT_USERNAME
    client_info.client_user = MQTT_USERNAME;
    client_info.client_pass = MQTT_PASSWORD;
#endif
    client_info.will_topic  = g_topic_tx[COMM_TOPIC_STATUS];
    client_info.will_msg    = COMM_STATUS_OFFLINE;
    client_info.will_qos    = MQTT_UTK_QOS_AT_LEAST;
    client_info.will_retain = MQTT_UTK_RETAIN;

    tm_printf((UB *)"mqtt: connecting to %s:%d as \"%s\" ...\n",
              ipaddr_ntoa(&g_broker_addr), (int)MQTT_BROKER_PORT,
              MQTT_CLIENT_ID);
    set_state(COMM_MQTT_CONNECTING);
    err = mqtt_client_connect(gp_client, &g_broker_addr, MQTT_BROKER_PORT,
                              connection_cb, NULL, &client_info);

    if (ERR_OK != err)
    {
        tm_printf((UB *)"mqtt: mqtt_client_connect() failed, err=%d\n",
                  (int)err);
        schedule_retry();
    }
}

#ifdef MQTT_BROKER_HOST
/** @brief Start a DNS lookup of MQTT_BROKER_HOST (see file header). */
static void
resolve_broker (void)
{
    ip_addr_t resolved;
    err_t     err;

    g_dns_generation++;
    gb_dns_pending = true;
    set_state(COMM_MQTT_RESOLVING);

    /* Armed BEFORE the call: the callback may run inside it (trap 1). */
    sys_untimeout(dns_timeout, NULL);
    sys_timeout(S1_LINK_DNS_TIMEOUT_MS, dns_timeout, NULL);
    tm_printf((UB *)"mqtt: resolving %s via DNS %s ...\n", MQTT_BROKER_HOST,
              ipaddr_ntoa(dns_getserver(0U)));

    /* The generation number travels as the callback argument; the
     * cast is undone in dns_found_cb() and only ever compared. */
    err = dns_gethostbyname(MQTT_BROKER_HOST, &resolved, dns_found_cb,
                            (void *)(uintptr_t)g_dns_generation);

    if (ERR_OK == err)
    {
        finish_lookup(&resolved); /* cached, or a numeric address */
    }
    else if (ERR_INPROGRESS == err)
    {
        /* The answer, a failure or the timeout arrives later (or has
         * already arrived, see trap 1): nothing more to do here. */
    }
    else
    {
        tm_printf((UB *)"mqtt: dns_gethostbyname() err=%d\n", (int)err);
        finish_lookup(NULL);
    }
}

/** @brief lwIP DNS answer (address, or NULL for "not found"). */
static void
dns_found_cb (const char * p_name, const ip_addr_t * p_addr, void * p_arg)
{
    (void)p_name;

    /* Same cast as in resolve_broker(): a tag, never dereferenced. */
    if (g_dns_generation == (uint32_t)(uintptr_t)p_arg)
    {
        finish_lookup(p_addr);
    }
}
#endif /* MQTT_BROKER_HOST */

/** @brief No DNS answer in time: give up on this lookup (trap 2). */
static void
dns_timeout (void * p_arg)
{
    (void)p_arg;

    if (gb_dns_pending)
    {
        g_dns_generation++; /* a late answer must not connect twice */
        tm_printf((UB *)"mqtt: DNS timeout after %dms\n",
                  (int)S1_LINK_DNS_TIMEOUT_MS);
        finish_lookup(NULL);
    }
}

/** @brief End of a lookup: connect, fall back, or back off (once). */
static void
finish_lookup (const ip_addr_t * p_addr)
{
    s1_link_broker_t choice;

    if (gb_dns_pending)
    {
        gb_dns_pending = false;
        sys_untimeout(dns_timeout, NULL);
        choice = s1_link_broker_choice(NULL != p_addr, gb_have_cached);

        switch (choice)
        {
            case S1_LINK_BROKER_RESOLVED:
                ip_addr_copy(g_broker_addr, *p_addr);
                connect_broker();
            break;

            case S1_LINK_BROKER_CACHED:
                ip_addr_copy(g_broker_addr, g_cached_addr);
                tm_printf((UB *)"mqtt: DNS failed, using last address\n");
                connect_broker();
            break;

            default:
                tm_printf((UB *)"mqtt: DNS failed, no address to try\n");
                schedule_retry();
            break;
        }
    }
}

/** @brief Start of an inbound publish: remember if it is our topic. */
static void
incoming_publish_cb (void * p_arg, const char * p_topic, u32_t total_len)
{
    (void)p_arg;
    (void)total_len;

    /* mqtt.c's topic string is only valid during this call. */
    gb_rx_is_cmd = str_equal(p_topic, g_topic_cmd);
    g_rx_len     = 0U;
}

/** @brief Inbound data; the last fragment completes one command. */
static void
incoming_data_cb (void * p_arg, const u8_t * p_data, u16_t len, u8_t flags)
{
    u16_t index;

    (void)p_arg;

    for (index = 0U; index < len; index++)
    {
        if (g_rx_len < (u16_t)sizeof(g_rx_data))
        {
            g_rx_data[g_rx_len] = p_data[index];
            g_rx_len++;
        }
    }

    if (0U != (flags & MQTT_DATA_FLAG_LAST))
    {
        if (gb_rx_is_cmd)
        {
            /* Validated (and printed) by subsystem 1's comm task. */
            comm_bridge_rx_command(g_rx_data, g_rx_len);
        }

        g_rx_len = 0U;
    }
}

/** @brief Result of subscribing to the command topic. */
static void
subscribe_done_cb (void * p_arg, err_t result)
{
    (void)p_arg;

    if (ERR_OK == result)
    {
        tm_printf((UB *)"mqtt: subscribed to %s\n", g_topic_cmd);
    }
    else
    {
        tm_printf((UB *)"mqtt: subscribe failed, err=%d\n", (int)result);
    }
}

/** @brief Broker confirmation (QoS 1) or failure of one publish. */
static void
publish_done_cb (void * p_arg, err_t result)
{
    (void)p_arg;

    if (ERR_OK != result)
    {
        tm_printf((UB *)"mqtt: publish not confirmed, err=%d\n", (int)result);
    }
}

/** @brief Publish the pending frame.  True = output buffer full: stop.
 *
 * Telemetry and status go out at QoS 0 (a newer one is always coming);
 * events and command acks at QoS 1 (the broker must confirm them).
 */
static bool
publish_pending (void)
{
    bool  b_stop = false;
    u8_t  qos    = MQTT_UTK_QOS_AT_MOST;
    err_t err;

    if (g_pend_topic >= (uint8_t)COMM_TOPIC_COUNT)
    {
        gb_pend_valid = false; /* corrupt record: drop it */
    }
    else
    {
        if (((uint8_t)COMM_TOPIC_EVENT == g_pend_topic) ||
            ((uint8_t)COMM_TOPIC_ACK == g_pend_topic))
        {
            qos = MQTT_UTK_QOS_AT_LEAST;
        }

        /* g_pend_len is 0..COMM_PAYLOAD_MAX (comm_bridge_tx_pop()). */
        err = mqtt_publish(gp_client, g_topic_tx[g_pend_topic],
                           g_pend_payload, (u16_t)g_pend_len, qos,
                           g_pend_retain, publish_done_cb, NULL);

        if (ERR_OK == err)
        {
            gb_pend_valid = false;
        }
        else if (ERR_MEM == err)
        {
            b_stop = true; /* same frame again on the next pump */
        }
        else
        {
            tm_printf((UB *)"mqtt: publish failed, err=%d, dropped\n",
                      (int)err);
            gb_pend_valid = false;
        }
    }

    return (b_stop);
}

/** @brief Drain up to MQTT_UTK_PUMP_BURST frames, then re-arm. */
static void
pump_timeout (void * p_arg)
{
    uint32_t count  = 0U;
    bool     b_stop = false;

    (void)p_arg;

    while ((!b_stop) && (count < MQTT_UTK_PUMP_BURST) &&
           (NULL != gp_client) && (0U != mqtt_client_is_connected(gp_client)))
    {
        count++;

        if (!gb_pend_valid)
        {
            g_pend_len = comm_bridge_tx_pop(&g_pend_topic, &g_pend_retain,
                                            g_pend_payload,
                                            (uint16_t)sizeof(g_pend_payload));
            gb_pend_valid = (g_pend_len >= 0);
            b_stop        = !gb_pend_valid; /* queue empty */
        }

        if (gb_pend_valid)
        {
            b_stop = publish_pending();
        }
    }

    sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
}

/** @brief CONNACK received, or the session failed or dropped. */
static void
connection_cb (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    err_t err;

    (void)p_client;
    (void)p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_retries = 0U;
        sys_untimeout(retry_timeout, NULL);
        ip_addr_copy(g_cached_addr, g_broker_addr); /* known good */
        gb_have_cached = true;
        tm_printf((UB *)"mqtt: connected to %s\n",
                  ipaddr_ntoa(&g_broker_addr));

        /* Callbacks before subscribing, or the first command has
         * nowhere to go. */
        mqtt_set_inpub_callback(gp_client, incoming_publish_cb,
                                incoming_data_cb, NULL);
        err = mqtt_subscribe(gp_client, g_topic_cmd, MQTT_UTK_QOS_AT_LEAST,
                             subscribe_done_cb, NULL);
        if (ERR_OK != err)
        {
            tm_printf((UB *)"mqtt: mqtt_subscribe() failed, err=%d\n",
                      (int)err);
        }

        /* Retained, so a dashboard opened later still sees "online".
         * The broker replaces it with the Last Will if we vanish. */
        (void)mqtt_publish(gp_client, g_topic_tx[COMM_TOPIC_STATUS],
                           COMM_STATUS_ONLINE, sizeof(COMM_STATUS_ONLINE) - 1U,
                           MQTT_UTK_QOS_AT_LEAST, MQTT_UTK_RETAIN,
                           publish_done_cb, NULL);

        /* Exactly one pump chain alive at a time. */
        sys_untimeout(pump_timeout, NULL);
        sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
        set_state(COMM_MQTT_CONNECTED);
    }
    else
    {
        /* Refused, timed out or dropped; mqtt.c has already closed the
         * TCP connection.  Wait, then try again. */
        sys_untimeout(pump_timeout, NULL);
        tm_printf((UB *)"mqtt: connection lost/refused (status %d)\n",
                  (int)status);
        schedule_retry();
    }
}

/*** end of file ***/

