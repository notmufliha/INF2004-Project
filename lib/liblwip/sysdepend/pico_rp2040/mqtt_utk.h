/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Called from cyw43_utk_task() on every poll while the link is
 * CYW43_LINK_UP (see cyw43_utk.c).  Only does something when the client
 * is idle: it then connects to MQTT_BROKER_IP:MQTT_BROKER_PORT
 * (config/wifi_secrets.h).  While connecting, connected, or waiting out a
 * retry backoff it returns immediately, so calling it every 2 ms is fine.
 *
 * Once connected, messages queued by subsystem 1 are published and
 * commands on <MQTT_TOPIC_PREFIX>/cmd are passed to it -- see
 * app_program/subsystem1/comm_bridge.h for the contract.
 */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */
