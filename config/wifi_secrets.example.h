/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 *
 *	The Pico W radio is 2.4 GHz only.  A phone hotspot must have
 *	"maximise compatibility" / 2.4 GHz turned on, and WPA2 (not WPA3-only).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip). */

/* IP address of the computer running the MQTT broker (Mosquitto).
 * Must be a numeric IPv4 address on the SAME network as the Pico. */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883

/* Must be unique per car on the broker */
#define MQTT_CLIENT_ID   "inf2004-g25-car"

/* All topics start with this (Subsystem 1, app_program/subsystem1/MQTT.md):
 *   <prefix>/telemetry  <prefix>/event  <prefix>/status
 *   <prefix>/cmd        <prefix>/cmd/ack                                 */
#define MQTT_TOPIC_PREFIX "inf2004/g25/car"

/* Uncomment only if your broker needs a login */
/* #define MQTT_USERNAME "user" */
/* #define MQTT_PASSWORD "pass" */

#endif /* WIFI_SECRETS_H */
