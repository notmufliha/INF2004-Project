/*
 *	comm_bridge.h
 *	Subsystem 1 -- the contract between the WiFi & Telemetry subsystem
 *	(app_program/subsystem1/) and the kernel's MQTT client
 *	(lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c).
 *
 *	WHY THIS FILE EXISTS
 *	--------------------
 *	Only the kernel's WiFi service task (cyw43_utk_task) may call cyw43,
 *	lwIP or MQTT functions (TEAM.md section 3).  So subsystem 1 never
 *	touches the network itself.  Instead:
 *
 *	  app tasks --comm_report_xxx()--> [mbf_events] --> comm_task (S1)
 *	  comm_task packs binary frames  --> [mbf_tx]   --> cyw43_utk_task
 *	  cyw43_utk_task publishes it over MQTT (mqtt_utk.c pump timer)
 *
 *	  broker --> cyw43_utk_task --comm_bridge_rx_command()--> [mbf_rx]
 *	  comm_task validates it   --> [mbf_remote_cmd] --> Vehicle Controller
 *
 *	The three comm_bridge_xxx() functions below are implemented by
 *	subsystem 1 (comm_bridge.c) and CALLED BY mqtt_utk.c, on the WiFi
 *	service task.  They use only TMO_POL kernel calls, so the network
 *	task can never be blocked by the application.
 *
 *	RULE: this header may include <stdint.h> only.  mqtt_utk.c lives in
 *	the lwIP/pico-sdk "namespace" and must never see tk/tkernel.h (see the
 *	comment in lib/liblwip/sysdepend/pico_rp2040/include/arch/cc.h).
 */

#ifndef COMM_BRIDGE_H
#define COMM_BRIDGE_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 *  Topics.  Full topic = <prefix>/<suffix>.  The prefix comes from
 *  MQTT_TOPIC_PREFIX in config/wifi_secrets.h, or this default.
 * --------------------------------------------------------------------- */
#define COMM_TOPIC_PREFIX_DEFAULT	"inf2004/g25/car"

typedef enum {
	COMM_TOPIC_TELEMETRY = 0,	/* periodic robot state         */
	COMM_TOPIC_EVENT,		/* barcode / hump / obstacle... */
	COMM_TOPIC_STATUS,		/* heartbeat + online/offline   */
	COMM_TOPIC_ACK,			/* reply to every command       */
	COMM_TOPIC_COUNT		/* fits in 2 bits               */
} COMM_TOPIC_ID;

/* Same order as COMM_TOPIC_ID */
#define COMM_TOPIC_SUFFIXES	{ "telemetry", "event", "status", "cmd/ack" }
#define COMM_TOPIC_CMD_SUFFIX	"cmd"		/* the one we subscribe to */

#define COMM_TOPIC_MAX		64	/* bytes, incl. NUL, full topic   */
#define COMM_PAYLOAD_MAX	32	/* bytes, one binary frame (largest: telemetry 28) */
#define COMM_CMD_MAX		3	/* bytes, one received command    */

/* Online / Last-Will message: 1 byte, the first byte of the status frame
 * (wire version 1 in bits 7..5, online flag in bit 4).  Never 0x00:
 * lwIP takes the will message's length with strlen(). */
#define COMM_STATUS_ONLINE	"\x30"
#define COMM_STATUS_OFFLINE	"\x20"

/* ---------------------------------------------------------------------
 *  MQTT connection state, as reported by mqtt_utk.c
 * --------------------------------------------------------------------- */
typedef enum {
	COMM_MQTT_IDLE = 0,	/* not started / waiting for WiFi        */
	COMM_MQTT_CONNECTING,	/* CONNECT sent, waiting for CONNACK     */
	COMM_MQTT_CONNECTED,	/* session up, subscribed to cmd topic   */
	COMM_MQTT_BACKOFF,	/* failed, waiting before the next retry */
	COMM_MQTT_OFFLINE	/* retries exhausted, long wait          */
} COMM_MQTT_STATE;

/* ---------------------------------------------------------------------
 *  Implemented in subsystem1/comm_bridge.c.  Called ONLY from the
 *  kernel's WiFi service task.  None of them ever blocks.
 * --------------------------------------------------------------------- */

/* Take the next queued outgoing message.
 *   returns the payload length (0..payload_max), or -1 if nothing queued.
 *   *topic_id  which COMM_TOPIC_ID to publish on
 *   *retain    1 = ask the broker to keep it as the topic's last value */
int32_t comm_bridge_tx_pop(uint8_t *topic_id, uint8_t *retain,
			   uint8_t *payload, uint16_t payload_max);

/* One complete message arrived on <prefix>/cmd */
void comm_bridge_rx_command(const uint8_t *data, uint16_t len);

/* The MQTT client changed state (a COMM_MQTT_STATE value) */
void comm_bridge_mqtt_state(int32_t state);

#endif /* COMM_BRIDGE_H */
