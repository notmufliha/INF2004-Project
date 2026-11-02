/** @file s1_private.h
 * @brief Definitions shared ONLY between Subsystem 1's own modules.
 * Other subsystems must not include this; they use subsystem1.h.
 *
 * Kernel objects (created in subsystem1.c) and who uses them:
 *   gh_s1_event_mbf  app tasks -> comm task   comm_pack_event_t
 *   gh_s1_tx_mbf     comm task -> WiFi task   topic + frame (variable)
 *   gh_s1_rx_mbf     WiFi task -> comm task   3-byte command (variable)
 *   mbf_remote_cmd   comm task -> VC          comm_remote_cmd_t (I1.3)
 *   gh_s1_telem_mtx  guards g_s1_telem and g_s1_stats.evt_dropped
 *   gh_s1_comm_flg   wakes the comm task as soon as something is queued
 *   gh_s1_led_cyc    paces the status LED
 *   flg_faults       the shared fault bitmask (fault.h)
 */

#ifndef S1_PRIVATE_H
#define S1_PRIVATE_H

#include "subsystem1.h"
#include "comm_bridge.h"
#include "comm_pack.h"

/* Kernel object handles */
extern ID gh_s1_comm_task; /* the WiFi & Telemetry task         */
extern ID gh_s1_led_cyc;   /* LED pattern tick                  */
extern ID gh_s1_event_mbf; /* app tasks -> comm task, events    */
extern ID gh_s1_tx_mbf;    /* comm task -> WiFi task, frames    */
extern ID gh_s1_rx_mbf;    /* WiFi task -> comm task, commands  */
extern ID gh_s1_telem_mtx; /* protects the telemetry snapshot   */
extern ID gh_s1_comm_flg;  /* wakes the comm task early         */

/* gh_s1_comm_flg bits */
#define FLG_COMM_EVENT   0x01U /* something in the events queue  */
#define FLG_COMM_RX      0x02U /* something in the receive queue */
#define FLG_COMM_LINK    0x04U /* MQTT state changed             */
#define FLG_COMM_ALL     (FLG_COMM_EVENT | FLG_COMM_RX | FLG_COMM_LINK)

/* Queue sizes (binary frames: was 3072 + 288 + 832 bytes of buffers) */
#define EVENT_QUEUE_LEN  16U  /* events buffered                    */
#define TX_RECORD_HDR    1U   /* topic (2 bits) + retain (1 bit)    */
#define TX_RECORD_MAX    (TX_RECORD_HDR + COMM_PAYLOAD_MAX)
#define TX_BUF_SIZE      640U /* ~17 telemetry or ~40 event frames  */
#define RX_QUEUE_LEN     8U   /* commands buffered                  */
#define RX_RECORD_MAX    (COMM_CMD_MAX + 1U) /* +1 = "too long"     */
#define MBF_MSG_OVERHEAD 8U   /* kernel header + alignment          */
#define RX_BUF_SIZE      (RX_QUEUE_LEN * (RX_RECORD_MAX + MBF_MSG_OVERHEAD))
#define TELEM_LOCK_TMO   5    /* ms, longest wait for the mutex     */

/* Shared state.  The two link states are 3-bit values kept in whole
 * bytes: each has ONE writer and is read by other tasks without a lock,
 * and a byte store is atomic on the RP2040 while a bit-field store is a
 * read-modify-write of the whole word. */
extern volatile uint8_t g_s1_mqtt_state; /* comm_bridge_mqtt_t, bridge  */
extern volatile uint8_t g_s1_link_state; /* comm_link_state_t, comm task */
extern comm_stats_t     g_s1_stats;
extern comm_telemetry_t g_s1_telem;      /* guarded by gh_s1_telem_mtx  */

#endif /* S1_PRIVATE_H */

/*** end of file ***/

