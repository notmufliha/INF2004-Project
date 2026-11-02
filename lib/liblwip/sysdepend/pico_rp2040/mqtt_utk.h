/** @file mqtt_utk.h
 * @brief MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1).
 *
 * Owned by Subsystem 1 (WiFi & Telemetry).  Both functions run only on
 * the kernel's WiFi service task, cyw43_utk_task() in cyw43_utk.c.
 *
 * Once connected, messages queued by subsystem 1 are published and
 * commands on <MQTT_TOPIC_PREFIX>/cmd are passed to it: see
 * app_program/subsystem1/comm_bridge.h for the contract.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/* Called on every poll while the WiFi link is CYW43_LINK_UP.  Only acts
 * when the client is idle: it then looks up MQTT_BROKER_HOST (if
 * configured) and connects.  While resolving, connecting, connected or
 * waiting out a retry backoff it returns at once, so calling it every
 * 2 ms is fine. */
void mqtt_utk_start (void);

/* Called once when the WiFi link leaves CYW43_LINK_UP.  Closes the MQTT
 * session at once (instead of waiting ~15 s for the keepalive to time
 * out), cancels any DNS lookup or retry in progress and resets the
 * backoff, so the next mqtt_utk_start() after the link returns connects
 * immediately. */
void mqtt_utk_link_down (void);

#endif /* MQTT_UTK_H */

/*** end of file ***/

