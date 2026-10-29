/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() on every poll
 *	while the link is up.  mqtt.c's own keepalive/ping/retry timers, and
 *	the pump/retry timers below, ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	Changes for the Group 25 robot car (Subsystem 1, WiFi & Telemetry):
 *
 *	  - The demo heartbeat is replaced by a PUMP timer that drains the
 *	    application's outgoing queue (comm_bridge_tx_pop(), implemented
 *	    in app_program/subsystem1/comm_bridge.c) and publishes each
 *	    message on <prefix>/telemetry | event | status | cmd/ack.
 *	  - Subscribes to <prefix>/cmd and hands every command to the
 *	    application (comm_bridge_rx_command()).
 *	  - Retained "online" status on connect plus a Last Will "offline",
 *	    so any dashboard can tell whether the car is reachable.
 *	  - Reconnect with exponential backoff (1, 2, 4, 8, 16, 16 s), then
 *	    a 60 s OFFLINE pause, as in the Buddy 1 statechart.
 *	  - Keepalive cut from 60 s to 10 s so a lost broker is noticed in
 *	    ~15 s instead of ~90 s.
 *
 *	The application never calls into this file; it only fills/empties
 *	kernel message buffers, which this file reads with TMO_POL through
 *	the comm_bridge_xxx() functions.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here,
 *	and comm_bridge.h is <stdint.h>-only by design.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/comm_bridge.h"
#include <tm/tmonitor.h>

#ifndef MQTT_TOPIC_PREFIX
#define MQTT_TOPIC_PREFIX	COMM_TOPIC_PREFIX_DEFAULT
#endif

#define MQTT_UTK_KEEPALIVE_S	10	/* broker loss noticed in ~1.5x this */
#define MQTT_UTK_PUMP_MS	20	/* how often the TX queue is drained */
#define MQTT_UTK_PUMP_BURST	4	/* max publishes per pump            */
#define MQTT_UTK_RETRY_FIRST_MS	1000	/* first retry delay, then doubled   */
#define MQTT_UTK_RETRY_MAX_MS	16000	/* backoff ceiling                   */
#define MQTT_UTK_MAX_RETRIES	6	/* then OFFLINE                      */
#define MQTT_UTK_OFFLINE_MS	60000	/* OFFLINE pause before trying again */

#define QOS_AT_MOST_ONCE	0
#define QOS_AT_LEAST_ONCE	1

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static int32_t		state = COMM_MQTT_IDLE;
static uint32_t		retries;

/* Full topic names, built once from MQTT_TOPIC_PREFIX */
static char		topic_tx[COMM_TOPIC_COUNT][COMM_TOPIC_MAX];
static char		topic_cmd[COMM_TOPIC_MAX];
static u8_t		topics_built;

/* One outgoing message waiting to be published (kept across a failed
 * attempt so a full output buffer delays it instead of losing it) */
static uint8_t		pend_payload[COMM_PAYLOAD_MAX];
static int32_t		pend_len;
static uint8_t		pend_topic;
static uint8_t		pend_retain;
static u8_t		pend_valid;

/* Inbound message staging.  A publish arrives as one topic callback
 * followed by one or more data callbacks, so the topic has to be kept
 * until the fragment carrying MQTT_DATA_FLAG_LAST shows up. */
static u8_t		rx_is_cmd;
static uint8_t		rx_data[COMM_CMD_MAX + 1];	/* +1: "too long" marker */
static u16_t		rx_len;

static void pump_timeout(void *arg);
static void retry_timeout(void *arg);

/* ------------------------------------------------------------------ */

static void set_state(int32_t s)
{
	state = s;
	comm_bridge_mqtt_state(s);
}

/* dst = a + "/" + b, always NUL-terminated, cut if too long */
static void join_topic(char *dst, const char *a, const char *b)
{
	u16_t n = 0;

	while ((*a != '\0') && (n < (COMM_TOPIC_MAX - 1))) {
		dst[n++] = *a++;
	}
	if (n < (COMM_TOPIC_MAX - 1)) {
		dst[n++] = '/';
	}
	while ((*b != '\0') && (n < (COMM_TOPIC_MAX - 1))) {
		dst[n++] = *b++;
	}
	dst[n] = '\0';
}

static u8_t str_equal(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return (*a == *b) ? 1 : 0;
}

static void build_topics(void)
{
	static const char *const suffix[COMM_TOPIC_COUNT] = COMM_TOPIC_SUFFIXES;
	int i;

	for (i = 0; i < COMM_TOPIC_COUNT; i++) {
		join_topic(topic_tx[i], MQTT_TOPIC_PREFIX, suffix[i]);
	}
	join_topic(topic_cmd, MQTT_TOPIC_PREFIX, COMM_TOPIC_CMD_SUFFIX);
	topics_built = 1;
}

/* Failed or lost connection: wait 1, 2, 4, 8, 16, 16 s between attempts,
 * then go OFFLINE for a minute and start the sequence again. */
static void schedule_retry(void)
{
	uint32_t delay;

	sys_untimeout(retry_timeout, NULL);
	retries++;
	if (retries > MQTT_UTK_MAX_RETRIES) {
		retries = 0;
		delay = MQTT_UTK_OFFLINE_MS;
		set_state(COMM_MQTT_OFFLINE);
		tm_printf((UB*)"mqtt: broker unreachable, OFFLINE for %ds\n",
			  (int)(delay / 1000));
	} else {
		delay = MQTT_UTK_RETRY_FIRST_MS << (retries - 1);
		if (delay > MQTT_UTK_RETRY_MAX_MS) {
			delay = MQTT_UTK_RETRY_MAX_MS;
		}
		set_state(COMM_MQTT_BACKOFF);
		tm_printf((UB*)"mqtt: retry %d in %dms\n",
			  (int)retries, (int)delay);
	}
	sys_timeout(delay, retry_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	set_state(COMM_MQTT_IDLE);	/* next poll's mqtt_utk_start() connects */
}

/* ------------------------------------------------------------------ */
/* Receiving commands                                                  */

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- just remember whether it's ours. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	(void)tot_len;
	rx_is_cmd = str_equal(topic, topic_cmd);
	rx_len = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t i;

	(void)arg;

	for (i = 0; i < len; i++) {
		if (rx_len < sizeof(rx_data)) {
			rx_data[rx_len++] = data[i];
		}
	}

	if ((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if (rx_is_cmd) {
			/* Validated (and printed) by subsystem 1's comm_task */
			comm_bridge_rx_command(rx_data, rx_len);
		}
		rx_len = 0;
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if (result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to %s\n", topic_cmd);
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

/* ------------------------------------------------------------------ */
/* Sending                                                             */

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish not confirmed, err=%d\n", (int)result);
	}
}

/* Drains up to MQTT_UTK_PUMP_BURST queued messages, then re-arms itself.
 * Telemetry/status go out at QoS 0 (a newer one is always coming);
 * events and command acks at QoS 1 (the broker must confirm them). */
static void pump_timeout(void *arg)
{
	int	i;
	err_t	err;

	(void)arg;

	for (i = 0; (i < MQTT_UTK_PUMP_BURST) && mqtt_client_is_connected(client); i++) {
		u8_t qos;

		if (!pend_valid) {
			pend_len = comm_bridge_tx_pop(&pend_topic, &pend_retain,
						      pend_payload,
						      (uint16_t)sizeof(pend_payload));
			if (pend_len < 0) {
				break;			/* queue empty */
			}
			pend_valid = 1;
		}
		if (pend_topic >= COMM_TOPIC_COUNT) {
			pend_valid = 0;			/* corrupt record: drop */
			continue;
		}

		qos = ((pend_topic == COMM_TOPIC_EVENT) || (pend_topic == COMM_TOPIC_ACK))
		      ? QOS_AT_LEAST_ONCE : QOS_AT_MOST_ONCE;
		err = mqtt_publish(client, topic_tx[pend_topic], pend_payload,
				   (u16_t)pend_len, qos, pend_retain,
				   publish_done_cb, NULL);
		if (err == ERR_OK) {
			pend_valid = 0;
		} else if (err == ERR_MEM) {
			break;		/* output buffer full: same message next pump */
		} else {
			tm_printf((UB*)"mqtt: publish failed, err=%d, dropped\n", (int)err);
			pend_valid = 0;
		}
	}

	sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
}

/* ------------------------------------------------------------------ */
/* Connection                                                          */

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if (status == MQTT_CONNECT_ACCEPTED) {
		err_t err;

		retries = 0;
		sys_untimeout(retry_timeout, NULL);
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks before subscribing, or the first command has
		 * nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		err = mqtt_subscribe(client, topic_cmd, QOS_AT_LEAST_ONCE,
				     subscribe_done_cb, NULL);
		if (err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n", (int)err);
		}

		/* Retained, so a dashboard opened later still sees "online".
		 * Replaced by the broker with the Last Will if we vanish. */
		(void)mqtt_publish(client, topic_tx[COMM_TOPIC_STATUS],
				   COMM_STATUS_ONLINE, sizeof(COMM_STATUS_ONLINE) - 1,
				   QOS_AT_LEAST_ONCE, 1, publish_done_cb, NULL);

		/* Exactly one pump chain alive at a time */
		sys_untimeout(pump_timeout, NULL);
		sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);

		set_state(COMM_MQTT_CONNECTED);
	} else {
		/* Refused, timed out or dropped.  mqtt.c has already closed
		 * the TCP connection; wait, then try again. */
		tm_printf((UB*)"mqtt: connection lost/refused (status %d)\n", (int)status);
		schedule_retry();
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	/* Called on every poll while WiFi is up: only act when idle */
	if (state != COMM_MQTT_IDLE) {
		return;
	}

	if (!topics_built) {
		build_topics();
	}

	if (client == NULL) {
		client = mqtt_client_new();
		if (client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			schedule_retry();
			return;
		}
	}

	if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP \"" MQTT_BROKER_IP "\"\n");
		set_state(COMM_MQTT_OFFLINE);	/* config error: never retry */
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id   = MQTT_CLIENT_ID;
	client_info.keep_alive  = MQTT_UTK_KEEPALIVE_S;
#ifdef MQTT_USERNAME
	client_info.client_user = MQTT_USERNAME;
	client_info.client_pass = MQTT_PASSWORD;
#endif
	client_info.will_topic  = topic_tx[COMM_TOPIC_STATUS];
	client_info.will_msg    = COMM_STATUS_OFFLINE;
	client_info.will_qos    = QOS_AT_LEAST_ONCE;
	client_info.will_retain = 1;

	tm_printf((UB*)"mqtt: connecting to " MQTT_BROKER_IP ":%d as \""
		  MQTT_CLIENT_ID "\" ...\n", (int)MQTT_BROKER_PORT);
	set_state(COMM_MQTT_CONNECTING);
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if (err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		schedule_retry();
	}
}
