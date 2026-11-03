/** @file s1_comm.c
 * @brief Subsystem 1 WiFi & Telemetry task (design element D1.2).
 *
 * Implements the Buddy 1 subsystem statechart:
 *
 *    INITIALISING -> CONNECTING_WIFI -> CONNECTING_MQTT -> CONNECTED
 *                         ^                   ^               |
 *                         |   RECONNECTING <--+---------------+ (lost)
 *                         |        |
 *                         +---- OFFLINE  (WiFi down > 30 s, or MQTT
 *                                         retries used up)
 *
 * The radio and the MQTT session are driven by the kernel's WiFi task
 * (cyw43_utk_task + mqtt_utk.c).  This task only OBSERVES their state
 * and does all application-level work, at low priority:
 *
 *   every loop (<= 50 ms):
 *     - work out the link state, report changes        (FR-B1-1/2/7)
 *     - keep FAULT_WIFI_LOSS in the shared mask in step
 *     - validate received commands, forward/ack them    (FR-B1-5)
 *     - pack queued events into binary frames           (FR-B1-4)
 *   every telemetry period while CONNECTED: telemetry   (FR-B1-3)
 *   every COMM_HEARTBEAT_MS while CONNECTED: heartbeat  (FR-B1-6)
 *
 * Telemetry is PAUSED while not connected (it would only be stale when
 * it arrived).  Events are still queued, up to the size of the tx
 * queue, and flushed on reconnect.  Nothing here ever stops the robot:
 * a network fault only affects telemetry and commands (NFR-B1-1).
 *
 * APIs shown: tk_wai_flg (timeout), tk_rcv_mbf/tk_snd_mbf TMO_POL,
 *             tk_set_flg/tk_clr_flg (fault mask), tk_get_otm
 */

#include "s1_comm.h"
#include "s1_private.h"
#include "comm_names.h"

#if TM_MQTT
#include "cyw43_utk.h" /* cyw43_utk_get_status(): safe from any task */
#include "s1_link.h"   /* S1_LINK_STATUS_UP == CYW43_LINK_UP          */
#endif

#if S1_POLL_B2_ENCODERS
#include "../subsystem2/encoder_control.h" /* encoder_get_counts() */
#endif

#define COMM_LOOP_MS    50   /* longest sleep between checks       */
#define MS_PER_DS       100U
#define TIME_DS_MASK    0xFFFFFU /* 20-bit time stamp              */
#define MS_PER_S        1000

static uint8_t  g_telem_period_ds = COMM_TELEM_PERIOD_DS;
static uint16_t g_telem_seq       = 0U;
static uint8_t  g_event_seq       = 0U;
static uint16_t g_heartbeat_seq   = 0U;
static bool     gb_ever_connected = false;
#if TM_MQTT
static uint32_t g_wifi_down_since = 0U; /* 0 = WiFi is up */
#endif
static uint8_t  g_frame[COMM_PAYLOAD_MAX]; /* this task only */

static uint32_t          time_ds (uint32_t now_ms);
static comm_link_state_t compute_state (uint32_t now_ms);
static void              update_wifi_fault (comm_link_state_t state);
static void              on_state_change (comm_link_state_t old_state,
                                          comm_link_state_t new_state,
                                          uint32_t now_ms);
static void              send_frame (int32_t len, comm_bridge_topic_t topic);
static void              publish_status (uint32_t now_ms);
static void              publish_telemetry (uint32_t now_ms);
static void              publish_event (const comm_pack_event_t * p_event);
static void              send_ack (const comm_pack_cmd_t * p_cmd,
                                   comm_pack_result_t result);
static void              forward_command (const comm_pack_cmd_t * p_cmd);
static void              handle_command (const uint8_t * p_data,
                                         uint32_t len, uint32_t now_ms);
static void              drain_commands (uint32_t now_ms);
static void              drain_events (void);
#if S1_POLL_B2_ENCODERS
static void              poll_encoders (uint32_t now_ms);
#endif

/** @brief The WiFi & Telemetry task: one pass per wake-up, forever. */
void
s1_comm_task (INT stacd, void * p_exinf)
{
    comm_link_state_t state = COMM_INITIALISING;
    comm_link_state_t next;
    uint32_t          last_telem;
    uint32_t          last_heartbeat;
    uint32_t          now_ms;
    UINT              pattern;

    (void)stacd;
    (void)p_exinf;

    tm_printf((UB *)"s1: WiFi & Telemetry task started%s\n",
              (0 != S1_STANDALONE_DEMO) ? " (STANDALONE DEMO MODE)" : "");
#if !TM_MQTT
    tm_printf((UB *)"s1: built without NET=lwip MQTT=1 -> no network, "
                    "events are printed only\n");
#endif

    last_telem     = s1_comm_now_ms();
    last_heartbeat = last_telem;

    for (;;)
    {
        /* Sleep until something is queued, or at most COMM_LOOP_MS.
         * E_TMOUT is the normal "nothing happened" case. */
        (void)tk_wai_flg(gh_s1_comm_flg, FLG_COMM_ALL, TWF_ORW | TWF_BITCLR,
                         &pattern, COMM_LOOP_MS);
        now_ms = s1_comm_now_ms();

        next = compute_state(now_ms);
        if (next != state)
        {
            on_state_change(state, next, now_ms);
            if (COMM_CONNECTED == next)
            {
                last_telem     = now_ms;
                last_heartbeat = now_ms;
            }
            state = next;
        }

        drain_commands(now_ms);
        drain_events();

        if (COMM_CONNECTED == state)
        {
            if ((now_ms - last_telem) >=
                ((uint32_t)g_telem_period_ds * MS_PER_DS))
            {
                last_telem = now_ms;
                publish_telemetry(now_ms);
            }

            if ((now_ms - last_heartbeat) >= COMM_HEARTBEAT_MS)
            {
                last_heartbeat = now_ms;
                publish_status(now_ms);
            }
        }
    }
}

/** @brief Milliseconds since boot (kernel operating time). */
uint32_t
s1_comm_now_ms (void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim); /* operating time: never set back */

    return ((uint32_t)tim.lo);
}

/** @brief 20-bit time stamp in 0.1 s units. */
static uint32_t
time_ds (uint32_t now_ms)
{
    return ((now_ms / MS_PER_DS) & TIME_DS_MASK);
}

/** @brief Link state from the WiFi and MQTT state (the statechart). */
static comm_link_state_t
compute_state (uint32_t now_ms)
{
    comm_link_state_t state = COMM_DISABLED; /* no NET=lwip MQTT=1 */
#if TM_MQTT
    T_CYW43_UTK_STATUS radio;

    cyw43_utk_get_status(&radio);

    if (0U == radio.init_complete)
    {
        state = COMM_INITIALISING;
    }
    else if (0U == radio.ready)
    {
        state = COMM_DISABLED; /* radio hardware failed */
    }
    else if (S1_LINK_STATUS_UP != radio.link_status)
    {
        /* The WiFi task re-joins by itself (s1_link_wifi_action()). */
        if (0U == g_wifi_down_since)
        {
            g_wifi_down_since = (0U != now_ms) ? now_ms : 1U;
        }

        if ((now_ms - g_wifi_down_since) >= COMM_WIFI_OFFLINE_MS)
        {
            state = COMM_OFFLINE;
        }
        else
        {
            state = gb_ever_connected ? COMM_RECONNECTING
                                      : COMM_CONNECTING_WIFI;
        }
    }
    else
    {
        g_wifi_down_since = 0U;

        switch (g_s1_mqtt_state)
        {
            case COMM_MQTT_CONNECTED:
                state = COMM_CONNECTED;
            break;

            case COMM_MQTT_BACKOFF:
                state = COMM_RECONNECTING;
            break;

            case COMM_MQTT_OFFLINE:
                state = COMM_OFFLINE;
            break;

            default: /* IDLE, RESOLVING, CONNECTING */
                state = gb_ever_connected ? COMM_RECONNECTING
                                          : COMM_CONNECTING_MQTT;
            break;
        }
    }
#else
    (void)now_ms;
#endif

    return (state);
}

/** @brief Keep FAULT_WIFI_LOSS in step: set once a link we had is lost
 * (or we gave up), cleared when connected again. */
static void
update_wifi_fault (comm_link_state_t state)
{
    if (COMM_CONNECTED == state)
    {
        fault_clear(FAULT_WIFI_LOSS);
    }
    else if ((COMM_OFFLINE == state) ||
             (gb_ever_connected && (COMM_RECONNECTING == state)))
    {
        fault_raise(FAULT_WIFI_LOSS);
    }
    else
    {
        /* Still connecting for the first time: not a fault yet. */
    }
}

/** @brief Publish a link change: counters, fault bit, console, status. */
static void
on_state_change (comm_link_state_t old_state, comm_link_state_t new_state,
                 uint32_t now_ms)
{
    if (COMM_CONNECTED == new_state)
    {
        if (gb_ever_connected)
        {
            g_s1_stats.reconnects++;
        }
        gb_ever_connected = true;
    }

    /* comm_link_state_t values are 0..6: they fit in a byte. */
    g_s1_link_state = (uint8_t)new_state;
    update_wifi_fault(new_state);
    tm_printf((UB *)"s1: link %s -> %s  faults=0x%02x\n",
              comm_names_link(old_state), comm_names_link(new_state),
              (INT)fault_get());

    if (COMM_CONNECTED == new_state)
    {
        publish_status(now_ms); /* "I'm back" */
    }
}

/** @brief Queue one packed frame.  A packing overflow cannot happen with
 * the fixed layouts, but would be dropped and counted, never sent. */
static void
send_frame (int32_t len, comm_bridge_topic_t topic)
{
    if (len < 0)
    {
        g_s1_stats.tx_dropped++;
        tm_printf((UB *)"s1: frame too long for topic %d, dropped\n",
                  (INT)topic);
    }
    else if ((uint8_t)COMM_DISABLED != g_s1_link_state)
    {
        /* len is 0..COMM_PAYLOAD_MAX here. */
        (void)comm_bridge_tx_enqueue(topic, false, g_frame, (uint32_t)len);
    }
    else
    {
        /* No network in this build: nobody would ever send it. */
    }
}

/** @brief Publish a heartbeat/status frame (23 bytes, FR-B1-6). */
static void
publish_status (uint32_t now_ms)
{
    comm_pack_status_t status = {0};
#if TM_MQTT
    T_CYW43_UTK_STATUS radio;

    cyw43_utk_get_status(&radio);
    status.ip_addr = radio.ip_addr;
#endif

    comm_get_stats(&status.stats);
    g_heartbeat_seq++;
    status.link     = g_s1_link_state;
    status.b_demo   = (0 != S1_STANDALONE_DEMO);
    status.seq      = g_heartbeat_seq;
    status.time_ds  = time_ds(now_ms);
    status.telem_ds = g_telem_period_ds;
    status.faults   = fault_get();

    send_frame(comm_pack_status(g_frame, COMM_PAYLOAD_MAX, &status),
               COMM_TOPIC_STATUS);

    tm_printf((UB *)"s1: heartbeat #%d  link=%s  faults=0x%02x  tx=%d "
                    "drop=%d rx=%d rej=%d\n",
              (INT)g_heartbeat_seq,
              comm_names_link((comm_link_state_t)g_s1_link_state),
              (INT)status.faults, (INT)status.stats.tx_queued,
              (INT)status.stats.tx_dropped, (INT)status.stats.rx_ok,
              (INT)status.stats.rx_rejected);
}

/** @brief Publish the telemetry snapshot (28 bytes, FR-B1-3). */
static void
publish_telemetry (uint32_t now_ms)
{
    comm_telemetry_t telem = {0};

#if S1_POLL_B2_ENCODERS
    poll_encoders(now_ms);
#endif
    comm_get_telemetry(&telem); /* consistent copy, under the mutex */
    g_telem_seq++;
    send_frame(comm_pack_telemetry(g_frame, COMM_PAYLOAD_MAX, g_telem_seq,
                                   time_ds(now_ms), fault_get(), &telem),
               COMM_TOPIC_TELEMETRY);
}

/** @brief Print one queued event and publish it (5..9 bytes, FR-B1-4). */
static void
publish_event (const comm_pack_event_t * p_event)
{
    g_event_seq++;

    switch (p_event->type)
    {
        case EVT_BARCODE:
            tm_printf((UB *)"s1: EVENT barcode '%c' -> %s\n",
                      (INT)((UINT)COMM_BARCODE_FIRST - 1U + p_event->val_a),
                      comm_names_nav((comm_nav_cmd_t)p_event->val_e));
        break;

        case EVT_HUMP:
            tm_printf((UB *)"s1: EVENT hump %d mm (peak %d mm, valid %d)\n",
                      (INT)p_event->val_a, (INT)p_event->val_b,
                      (INT)p_event->flag);
        break;

        case EVT_OBSTACLE:
            tm_printf((UB *)"s1: EVENT obstacle at %d cm, width %d -> %s\n",
                      (INT)p_event->val_a, (INT)p_event->val_b,
                      comm_names_avoid((comm_avoid_action_t)p_event->val_e));
        break;

        default: /* EVT_ERROR */
            tm_printf((UB *)"s1: EVENT error 0x%02x (S%d #%d) detail %d\n",
                      (INT)p_event->val_a, (INT)ERR_SUBSYS(p_event->val_a),
                      (INT)ERR_NUMBER(p_event->val_a), (INT)p_event->val_b);
        break;
    }

    send_frame(comm_pack_event(g_frame, COMM_PAYLOAD_MAX, g_event_seq,
                               p_event),
               COMM_TOPIC_EVENT);
}

/** @brief Publish the 3-byte ack for one command. */
static void
send_ack (const comm_pack_cmd_t * p_cmd, comm_pack_result_t result)
{
    /* Ack results are 0..14 (5 bits): they fit in a byte. */
    send_frame(comm_pack_ack(g_frame, COMM_PAYLOAD_MAX, p_cmd->seq,
                             p_cmd->cmd_id, (uint8_t)result),
               COMM_TOPIC_ACK);
}

/** @brief Hand a validated command to the Vehicle Controller.
 *
 * TMO_POL: if its queue is full we say so in the ack rather than wait.
 * The ack confirms delivery to the controller, not motor execution.
 */
static void
forward_command (const comm_pack_cmd_t * p_cmd)
{
    comm_remote_cmd_type_t type = (comm_remote_cmd_type_t)p_cmd->cmd.type;

    if (E_OK == tk_snd_mbf(mbf_remote_cmd, &p_cmd->cmd,
                           (INT)sizeof(p_cmd->cmd), TMO_POL))
    {
        tm_printf((UB *)"s1: command %s (arg %d, #%d) -> controller\n",
                  comm_names_remote(type), (INT)p_cmd->cmd.arg,
                  (INT)p_cmd->cmd.seq);
        send_ack(p_cmd, CMD_OK);
    }
    else
    {
        g_s1_stats.fwd_failed++;
        tm_printf((UB *)"s1: command %s NOT delivered: controller queue "
                        "full\n",
                  comm_names_remote(type));
        send_ack(p_cmd, CMD_ERR_BUSY);
    }
}

/** @brief Validate one received frame, then act on it and ack it. */
static void
handle_command (const uint8_t * p_data, uint32_t len, uint32_t now_ms)
{
    comm_pack_cmd_t    cmd;
    comm_pack_result_t result = comm_pack_cmd_decode(p_data, len, &cmd);

    if (CMD_OK != result)
    {
        g_s1_stats.rx_rejected++;
        tm_printf((UB *)"s1: REJECTED command (%d bytes, id %d, #%d): %s\n",
                  (INT)len, (INT)cmd.cmd_id, (INT)cmd.seq,
                  comm_pack_result_text(result));
        send_ack(&cmd, result);
    }
    else if (WCMD_PING == cmd.cmd_id)
    {
        g_s1_stats.rx_ok++;
        send_ack(&cmd, CMD_OK_PONG);
    }
    else if (WCMD_STATUS == cmd.cmd_id)
    {
        g_s1_stats.rx_ok++;
        send_ack(&cmd, CMD_OK_STATUS);
        publish_status(now_ms);
    }
    else if (WCMD_TELEM == cmd.cmd_id)
    {
        g_s1_stats.rx_ok++;
        g_telem_period_ds = cmd.arg;
        tm_printf((UB *)"s1: telemetry period now %d ms\n",
                  (INT)g_telem_period_ds * (INT)MS_PER_DS);
        send_ack(&cmd, CMD_OK_TELEM);
    }
    else
    {
        g_s1_stats.rx_ok++;
        forward_command(&cmd);
    }
}

/** @brief Handle every queued command (bounded by the queue length). */
static void
drain_commands (uint32_t now_ms)
{
    uint8_t  buf[RX_RECORD_MAX];
    INT      received;
    uint32_t count;
    bool     b_more = true;

    for (count = 0U; (count < RX_QUEUE_LEN) && b_more; count++)
    {
        received = tk_rcv_mbf(gh_s1_rx_mbf, buf, TMO_POL);
        b_more   = (received > 0);
        if (b_more)
        {
            /* received is 1..RX_RECORD_MAX here. */
            handle_command(buf, (uint32_t)received, now_ms);
        }
    }
}

/** @brief Publish every queued event (bounded by the queue length). */
static void
drain_events (void)
{
    comm_pack_event_t event;
    uint32_t          count;
    bool              b_more = true;

    for (count = 0U; (count < EVENT_QUEUE_LEN) && b_more; count++)
    {
        b_more = ((INT)sizeof(event) ==
                  tk_rcv_mbf(gh_s1_event_mbf, &event, TMO_POL));
        if (b_more)
        {
            publish_event(&event);
        }
    }
}

#if S1_POLL_B2_ENCODERS
/** @brief Read B2's encoders and derive counts/s over the period. */
static void
poll_encoders (uint32_t now_ms)
{
    static INT      prev_l     = 0;
    static INT      prev_r     = 0;
    static uint32_t prev_ms    = 0U;
    static bool     b_have_prev = false;
    INT             left;
    INT             right;
    int32_t         speed_l = 0;
    int32_t         speed_r = 0;
    uint32_t        delta_ms = now_ms - prev_ms;

    encoder_get_counts(&left, &right); /* consistent, ISR-safe pair */
    if (b_have_prev && (delta_ms > 0U))
    {
        /* delta_ms is one telemetry period (<= 10 s): fits int32_t. */
        speed_l = ((int32_t)(left - prev_l) * MS_PER_S) / (int32_t)delta_ms;
        speed_r = ((int32_t)(right - prev_r) * MS_PER_S) / (int32_t)delta_ms;
    }

    prev_l      = left;
    prev_r      = right;
    prev_ms     = now_ms;
    b_have_prev = true;
    comm_set_motion(speed_l, speed_r, left, right);
}
#endif

/*** end of file ***/

