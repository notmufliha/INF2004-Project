/** @file comm_bridge.h
 * @brief The contract between Subsystem 1 (app_program/subsystem1/) and
 * the kernel's MQTT client (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c).
 *
 * Only the kernel's WiFi service task (cyw43_utk_task) may call cyw43,
 * lwIP or MQTT functions (TEAM.md section 3), so subsystem 1 never
 * touches the network itself:
 *
 *   app tasks --comm_report_xxx()--> [events mbf] --> comm task (D1.2)
 *   comm task packs binary frames --> [tx mbf]    --> cyw43_utk_task
 *   cyw43_utk_task publishes them (mqtt_utk.c pump timer)       (I1.5)
 *
 *   broker --> cyw43_utk_task --comm_bridge_rx_command()--> [rx mbf]
 *   comm task validates --> [mbf_remote_cmd] --> Vehicle Controller
 *
 * The comm_bridge_tx_pop/rx_command/mqtt_state hooks are implemented in
 * comm_bridge.c and CALLED BY mqtt_utk.c on the WiFi service task.
 * They only use TMO_POL kernel calls, so the network task can never be
 * blocked by the application.
 *
 * RULE: this header includes <stdint.h> and <stdbool.h> only, because
 * mqtt_utk.c lives in the lwIP/pico-sdk namespace and must never see
 * tk/tkernel.h (see lib/liblwip/sysdepend/pico_rp2040/include/arch/cc.h).
 */

#ifndef COMM_BRIDGE_H
#define COMM_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

/* Full topic = <prefix>/<suffix>.  The prefix comes from
 * MQTT_TOPIC_PREFIX in config/wifi_secrets.h, or this default. */
#define COMM_TOPIC_PREFIX_DEFAULT "inf2004/g25/car"

/** @brief Outgoing topic (2 bits in the tx record header). */
typedef enum
{
    COMM_TOPIC_TELEMETRY = 0, /* periodic robot state         */
    COMM_TOPIC_EVENT,         /* barcode / hump / obstacle... */
    COMM_TOPIC_STATUS,        /* heartbeat + online/offline   */
    COMM_TOPIC_ACK,           /* reply to every command       */
    COMM_TOPIC_COUNT
} comm_bridge_topic_t;

/* Same order as comm_bridge_topic_t */
#define COMM_TOPIC_SUFFIXES   { "telemetry", "event", "status", "cmd/ack" }
#define COMM_TOPIC_CMD_SUFFIX "cmd" /* the one topic we subscribe to */

#define COMM_TOPIC_MAX   64U /* bytes incl. NUL, full topic name   */
#define COMM_PAYLOAD_MAX 32U /* bytes, one frame (telemetry: 28)   */
#define COMM_CMD_MAX     3U  /* bytes, one received command        */

/* Online / Last-Will message: 1 byte, the first byte of the status
 * frame (wire version 1 in bits 7..5, online flag in bit 4).  Never
 * 0x00: lwIP takes the will message's length with strlen(). */
#define COMM_STATUS_ONLINE  "\x30"
#define COMM_STATUS_OFFLINE "\x20"

/** @brief MQTT connection state, as reported by mqtt_utk.c. */
typedef enum
{
    COMM_MQTT_IDLE = 0,   /* not started / waiting for WiFi        */
    COMM_MQTT_CONNECTING, /* CONNECT sent, waiting for CONNACK     */
    COMM_MQTT_CONNECTED,  /* session up, subscribed to cmd topic   */
    COMM_MQTT_BACKOFF,    /* failed, waiting before the next retry */
    COMM_MQTT_OFFLINE,    /* retries used up, long wait            */
    COMM_MQTT_RESOLVING   /* DNS lookup of MQTT_BROKER_HOST         */
} comm_bridge_mqtt_t;

/* Take the next queued outgoing frame.  Returns its length
 * (0..payload_max) or -1 if nothing is queued; *p_topic_id is a
 * comm_bridge_topic_t, *p_retain 1 = broker keeps it as last value. */
int32_t comm_bridge_tx_pop (uint8_t * p_topic_id, uint8_t * p_retain,
                            uint8_t * p_payload, uint16_t payload_max);

/* One complete message arrived on <prefix>/cmd. */
void comm_bridge_rx_command (const uint8_t * p_data, uint16_t len);

/* The MQTT client changed state (a comm_bridge_mqtt_t value). */
void comm_bridge_mqtt_state (int32_t state);

/* Used by subsystem 1's comm task only, never by mqtt_utk.c: put one
 * frame in the outgoing queue without blocking.  false = dropped. */
bool comm_bridge_tx_enqueue (comm_bridge_topic_t topic, bool b_retain,
                             const uint8_t * p_payload, uint32_t len);

/* Commands lost because the receive queue was full (wrapping count). */
uint16_t comm_bridge_rx_dropped (void);

#endif /* COMM_BRIDGE_H */

/*** end of file ***/

