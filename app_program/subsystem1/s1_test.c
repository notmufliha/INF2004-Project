/** @file s1_test.c
 * @brief Subsystem 1 on-target tests.  Built only with make TEST=1
 * (add NET=lwip MQTT=1 to include the network tests S1-U09/U10).
 *
 * Each check prints a PASS/FAIL line on the console.  Save the output
 * under testing/subsystem1/logs/ and record the result in
 * testing/subsystem1/TEST_LOG.md.  Test IDs trace to requirements in
 * testing/subsystem1/REQUIREMENTS_TRACE.md.
 *
 * The bit-exact packing of every frame and the reconnect policy are
 * also checked on the laptop: testing/subsystem1/host/.
 */

#include "s1_test.h"
#include "s1_private.h"
#include "s1_comm.h"
#include "s1_led.h"

#if APP_TEST

#define CONNECT_WAIT_MS  30000U /* S1-U09: time allowed to connect   */
#define BURST_EVENTS     40U    /* S1-U06: events posted in a burst  */
#define BURST_LIMIT_MS   20U    /* S1-U06: must finish within this   */
#define FAULT_ROUNDS     1000U  /* S1-U20: set/clear repetitions     */
#define SLEEP_SAMPLES    10U    /* S1-U02: task state samples        */
#define SLEEP_MIN_WAITS  9U     /* S1-U02: samples that must be WAIT */
#define SAMPLE_GAP_MS    37     /* S1-U02: odd gap, not loop-aligned */
#define DRAIN_WAIT_MS    500    /* let the comm task empty a queue   */
#define RX_WAIT_MS       300
#define POLL_MS          250
#define TELEM_WAIT_MS    3000
#define TELEM_MIN_FRAMES 4U     /* S1-U10: >= 4 frames in 3 s at 2 Hz */
#define GUARD_BYTE       0xA5U
#define SMALL_BUF        10U
#define BITS_VER         3U
#define BITS_ID          4U
#define BITS_ARG         7U
#define BITS_SEQ         8U
#define BITS_RES         2U
#define BAD_CMD_ID       14U
#define TOP_CMD_ID       15U
#define SPEED_TOO_HIGH   101U
#define EXTRA_ARG        5U
#define BLIP_TICKS       8U

static uint32_t g_passed = 0U;
static uint32_t g_failed = 0U;

static void               check (const char * p_id, bool b_ok,
                                 const char * p_desc);
static void               make_cmd (uint8_t * p_out, uint32_t version,
                                    uint32_t cmd_id, uint32_t arg,
                                    uint32_t seq, uint32_t reserved);
static comm_pack_result_t result_of (uint32_t version, uint32_t cmd_id,
                                     uint32_t arg, uint32_t reserved);
static bool               decodes_to (uint32_t cmd_id, uint32_t arg,
                                      uint32_t seq);
static void               test_tasks (void);
static void               test_commands (void);
static void               test_frames (void);
static void               test_events (void);
static void               test_setters (void);
static void               test_rx_path (void);
static void               test_faults (void);
static void               test_led (void);
static void               test_network (void);

#endif /* APP_TEST */

/** @brief Run every subsystem 1 on-target test and print a summary. */
void
s1_test_run (void)
{
#if APP_TEST
    g_passed = 0U;
    g_failed = 0U;

    test_tasks();
    test_commands();
    test_frames();
    test_events();
    test_setters();
    test_rx_path();
    test_faults();
    test_led();
    test_network();

    tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed\n",
              "subsystem1", (INT)g_passed, (INT)g_failed);
#endif
}

#if APP_TEST

/** @brief Print one result line (test_report.h format) and count it. */
static void
check (const char * p_id, bool b_ok, const char * p_desc)
{
    if (b_ok)
    {
        g_passed++;
        tm_printf((UB *)"[TEST] %s PASS  %s\n", p_id, p_desc);
    }
    else
    {
        g_failed++;
        tm_printf((UB *)"[TEST] %s FAIL  %s\n", p_id, p_desc);
    }
}

/** @brief Build a command frame: ver:3 id:4 arg:7 seq:8 reserved:2. */
static void
make_cmd (uint8_t * p_out, uint32_t version, uint32_t cmd_id, uint32_t arg,
          uint32_t seq, uint32_t reserved)
{
    comm_pack_bits_t bits;

    comm_pack_bits_begin(&bits, p_out, WIRE_CMD_LEN);
    comm_pack_bits_put(&bits, version, BITS_VER);
    comm_pack_bits_put(&bits, cmd_id, BITS_ID);
    comm_pack_bits_put(&bits, arg, BITS_ARG);
    comm_pack_bits_put(&bits, seq, BITS_SEQ);
    comm_pack_bits_put(&bits, reserved, BITS_RES);
}

/** @brief Decode result for a frame built from these fields. */
static comm_pack_result_t
result_of (uint32_t version, uint32_t cmd_id, uint32_t arg,
           uint32_t reserved)
{
    uint8_t         frame[WIRE_CMD_LEN];
    comm_pack_cmd_t cmd;

    make_cmd(frame, version, cmd_id, arg, 7U, reserved);

    return (comm_pack_cmd_decode(frame, WIRE_CMD_LEN, &cmd));
}

/** @brief True if a valid frame decodes back to the same fields. */
static bool
decodes_to (uint32_t cmd_id, uint32_t arg, uint32_t seq)
{
    uint8_t         frame[WIRE_CMD_LEN];
    comm_pack_cmd_t cmd;
    bool            b_ok;

    make_cmd(frame, WIRE_VERSION, cmd_id, arg, seq, 0U);
    b_ok = (CMD_OK == comm_pack_cmd_decode(frame, WIRE_CMD_LEN, &cmd)) &&
           (cmd_id == cmd.cmd_id) && (arg == cmd.arg) && (seq == cmd.seq);

    if (b_ok && (cmd_id < (uint32_t)RCMD_COUNT))
    {
        /* Forwarded commands must also fill the VC message. */
        b_ok = (cmd_id == cmd.cmd.type) && (arg == cmd.cmd.arg) &&
               (seq == cmd.cmd.seq);
    }

    return (b_ok);
}

/** @brief S1-U01 LED cyclic running; S1-U02 comm task sleeps. */
static void
test_tasks (void)
{
    T_RCYC   rcyc;
    T_RTSK   rtsk;
    uint32_t sample;
    uint32_t waiting = 0U;

    check("S1-U01",
          (E_OK == tk_ref_cyc(gh_s1_led_cyc, &rcyc)) &&
              (0U != (rcyc.cycstat & TCYC_STA)),
          "LED cyclic handler is running");

    /* NFR-B1-3: the comm task must not busy-wait. */
    for (sample = 0U; sample < SLEEP_SAMPLES; sample++)
    {
        (void)tk_dly_tsk(SAMPLE_GAP_MS);
        if ((E_OK == tk_ref_tsk(gh_s1_comm_task, &rtsk)) &&
            (0U != (rtsk.tskstat & TTS_WAI)))
        {
            waiting++;
        }
    }
    check("S1-U02", waiting >= SLEEP_MIN_WAITS,
          "comm_task sleeps between loops");
}

/** @brief S1-U03 valid commands decode; S1-U04 invalid ones rejected. */
static void
test_commands (void)
{
    uint8_t         frame[WIRE_CMD_LEN + 1U] = {0};
    comm_pack_cmd_t cmd;
    bool            b_ok;

    b_ok = (decodes_to(RCMD_START, 0U, 0U)) &&
           (decodes_to(RCMD_STOP, 0U, 17U)) &&
           (decodes_to(RCMD_SPEED, 40U, 3U)) &&
           (decodes_to(RCMD_SPEED, 100U, 255U)) &&
           (decodes_to(RCMD_MODE, MODE_LINE_ONLY, 1U)) &&
           (decodes_to(RCMD_ESTOP, 0U, 9U)) &&
           (decodes_to(WCMD_PING, 0U, 2U)) &&
           (decodes_to(WCMD_TELEM, COMM_TELEM_MIN_DS, 4U)) &&
           (decodes_to(WCMD_TELEM, COMM_TELEM_MAX_DS, 5U));
    check("S1-U03", b_ok, "valid 3-byte commands accepted and decoded");

    /* Boundaries: just outside every limit (Lecture 6: -1 / max+1). */
    make_cmd(frame, WIRE_VERSION, RCMD_STOP, 0U, 0U, 0U);
    b_ok = (CMD_ERR_LENGTH == comm_pack_cmd_decode(frame, 2U, &cmd)) &&
           (CMD_ERR_LENGTH == comm_pack_cmd_decode(frame, 4U, &cmd)) &&
           (CMD_ERR_VERSION == result_of(2U, RCMD_STOP, 0U, 0U)) &&
           (CMD_ERR_RESERVED == result_of(WIRE_VERSION, RCMD_STOP, 0U, 1U)) &&
           (CMD_ERR_UNKNOWN == result_of(WIRE_VERSION, 0U, 0U, 0U)) &&
           (CMD_ERR_UNKNOWN == result_of(WIRE_VERSION, TOP_CMD_ID, 0U, 0U)) &&
           (CMD_ERR_RANGE ==
            result_of(WIRE_VERSION, RCMD_SPEED, SPEED_TOO_HIGH, 0U)) &&
           (CMD_ERR_RANGE ==
            result_of(WIRE_VERSION, RCMD_MODE, MODE_COUNT, 0U)) &&
           (CMD_ERR_RANGE == result_of(WIRE_VERSION, WCMD_TELEM, 0U, 0U)) &&
           (CMD_ERR_EXTRA_ARG ==
            result_of(WIRE_VERSION, RCMD_STOP, EXTRA_ARG, 0U));
    check("S1-U04", b_ok, "invalid command frames rejected");
}

/** @brief S1-U05 frame sizes, and packing never writes past the end. */
static void
test_frames (void)
{
    uint8_t            buf[COMM_PAYLOAD_MAX + 1U];
    comm_telemetry_t   telem  = {0};
    comm_pack_event_t  event  = {0};
    comm_pack_status_t status = {0};
    int32_t            n_tel;
    int32_t            n_bar;
    int32_t            n_obs;
    int32_t            n_err;
    int32_t            n_sta;
    int32_t            n_ack;
    int32_t            n_small;

    buf[COMM_PAYLOAD_MAX] = GUARD_BYTE;
    n_tel = comm_pack_telemetry(buf, COMM_PAYLOAD_MAX, 1U, 2U, 0U, &telem);
    event.type = EVT_BARCODE;
    n_bar = comm_pack_event(buf, COMM_PAYLOAD_MAX, 1U, &event);
    event.type = EVT_OBSTACLE;
    n_obs = comm_pack_event(buf, COMM_PAYLOAD_MAX, 1U, &event);
    event.type = EVT_ERROR;
    n_err = comm_pack_event(buf, COMM_PAYLOAD_MAX, 1U, &event);
    n_sta = comm_pack_status(buf, COMM_PAYLOAD_MAX, &status);
    n_ack = comm_pack_ack(buf, COMM_PAYLOAD_MAX, 1U, 1U, (uint8_t)CMD_OK);
    n_small = comm_pack_telemetry(buf, SMALL_BUF, 1U, 2U, 0U, &telem);

    tm_printf((UB *)"[TEST] S1-U05 info: telemetry %d B, event %d-%d B, "
                    "status %d B, ack %d B; sizeof telem %d, event %d, "
                    "cmd %d\n",
              (INT)n_tel, (INT)n_bar, (INT)n_obs, (INT)n_sta, (INT)n_ack,
              (INT)sizeof(comm_telemetry_t), (INT)sizeof(comm_pack_event_t),
              (INT)sizeof(comm_remote_cmd_t));
    check("S1-U05",
          (WIRE_TELEM_LEN == n_tel) && (WIRE_EVT_BAR_LEN == n_bar) &&
              (WIRE_EVT_ERR_LEN == n_err) && (WIRE_EVT_MAX_LEN == n_obs) &&
              (WIRE_STATUS_LEN == n_sta) && (WIRE_ACK_LEN == n_ack) &&
              (-1 == n_small) && (GUARD_BYTE == buf[COMM_PAYLOAD_MAX]) &&
              (24U == sizeof(comm_telemetry_t)) &&
              (8U == sizeof(comm_pack_event_t)) &&
              (4U == sizeof(comm_remote_cmd_t)),
          "binary frame sizes and bounded packing");
}

/** @brief S1-U06 reporting events never blocks, even when full. */
static void
test_events (void)
{
    uint32_t start_ms = s1_comm_now_ms();
    uint32_t index;
    uint32_t sent     = 0U;
    uint32_t full     = 0U;
    ER       result;

    for (index = 0U; index < BURST_EVENTS; index++)
    {
        /* index < 40: fits the 8-bit detail. */
        result = comm_report_error(ERR_S1_TEST, (uint8_t)index);
        if (E_OK == result)
        {
            sent++;
        }
        else if (E_TMOUT == result)
        {
            full++;
        }
        else
        {
            /* Any other result is a failure: counted by neither. */
        }
    }

    check("S1-U06",
          ((s1_comm_now_ms() - start_ms) < BURST_LIMIT_MS) &&
              (BURST_EVENTS == (sent + full)),
          "event reporting is non-blocking");
    (void)tk_dly_tsk(DRAIN_WAIT_MS);
}

/** @brief S1-U07 setters saturate and keep 24-bit encoder totals. */
static void
test_setters (void)
{
    comm_telemetry_t telem = {0};

    comm_set_motion(123, -9000, 1000, -1);
    comm_set_line(11U, 5000U, 1U, LINE_F_DETECTED | LINE_F_JUNCTION);
    comm_set_imu(3U, -700);
    comm_set_obstacle_dist(COMM_CM_INVALID);
    comm_get_telemetry(&telem);

    check("S1-U07",
          (123 == comm_pack_sign_extend(telem.speed_left, COMM_SPEED_BITS)) &&
              (-COMM_SPEED_MAX ==
               comm_pack_sign_extend(telem.speed_right, COMM_SPEED_BITS)) &&
              (1000U == telem.enc_left) && (COMM_ENC_MASK == telem.enc_right) &&
              (11U == telem.ir_left) && (COMM_IR_MAX == telem.ir_centre) &&
              (1U == telem.ir_right) &&
              ((LINE_F_DETECTED | LINE_F_JUNCTION) == telem.line_flags) &&
              (3U == telem.motion) &&
              (-COMM_PITCH_MAX ==
               comm_pack_sign_extend(telem.pitch_ddeg, COMM_PITCH_BITS)) &&
              (0x1FFU == telem.obs_dist_cm),
          "packed telemetry setters/getter, saturation");
}

/** @brief S1-U08 received frames are validated: 1 ok, 1 rejected. */
static void
test_rx_path (void)
{
    comm_stats_t before = {0};
    comm_stats_t after  = {0};
    uint8_t      ping[WIRE_CMD_LEN];
    uint8_t      bad[WIRE_CMD_LEN];

    make_cmd(ping, WIRE_VERSION, WCMD_PING, 0U, 1U, 0U);
    make_cmd(bad, WIRE_VERSION, BAD_CMD_ID, 0U, 2U, 0U);
    comm_get_stats(&before);
    comm_bridge_rx_command(ping, WIRE_CMD_LEN);
    comm_bridge_rx_command(bad, WIRE_CMD_LEN);
    (void)tk_dly_tsk(RX_WAIT_MS);
    comm_get_stats(&after);

    check("S1-U08",
          ((uint16_t)(before.rx_ok + 1U) == after.rx_ok) &&
              ((uint16_t)(before.rx_rejected + 1U) == after.rx_rejected),
          "received commands validated (1 ok, 1 rejected)");
}

/** @brief S1-U20 setting/clearing one bit never disturbs another. */
static void
test_faults (void)
{
    uint8_t  saved = fault_get();
    bool     b_ok  = true;
    uint32_t round;

    fault_clear(FAULT_ALL);
    for (round = 0U; round < FAULT_ROUNDS; round++)
    {
        fault_raise(FAULT_IMU);
        fault_raise(FAULT_NO_ECHO);
        fault_clear(FAULT_IMU);
        if (FAULT_NO_ECHO != fault_get())
        {
            b_ok = false;
        }
        fault_clear(FAULT_NO_ECHO);
    }
    fault_raise(saved);

    check("S1-U20", b_ok && (flg_faults > 0),
          "fault bitmask set/clear keeps other bits");
}

/** @brief S1-U23 LED patterns at their boundaries (FR-B1-8). */
static void
test_led (void)
{
    bool b_ok;

    b_ok = (1U == s1_led_level(COMM_CONNECTED, 0U)) &&
           (1U == s1_led_level(COMM_CONNECTED, 7U)) &&
           (0U == s1_led_level(COMM_CONNECTING_WIFI, 0U)) &&
           (1U == s1_led_level(COMM_CONNECTING_WIFI, 1U)) &&
           (0U == s1_led_level(COMM_RECONNECTING, 1U)) &&
           (1U == s1_led_level(COMM_RECONNECTING, 2U)) &&
           (1U == s1_led_level(COMM_OFFLINE, 0U)) &&
           (0U == s1_led_level(COMM_OFFLINE, BLIP_TICKS - 1U)) &&
           (1U == s1_led_level(COMM_OFFLINE, BLIP_TICKS)) &&
           (0U == s1_led_level(COMM_INITIALISING, 1U)) &&
           (0U == s1_led_level(COMM_STATE_COUNT, 1U));
    check("S1-U23", b_ok, "LED pattern per link state");
}

/** @brief S1-U09 connects; S1-U10 telemetry flows (network builds). */
static void
test_network (void)
{
#if TM_MQTT
    comm_stats_t before   = {0};
    comm_stats_t after    = {0};
    uint32_t     start_ms = s1_comm_now_ms();

    while ((!comm_is_connected()) &&
           ((s1_comm_now_ms() - start_ms) < CONNECT_WAIT_MS))
    {
        (void)tk_dly_tsk(POLL_MS);
    }
    tm_printf((UB *)"[TEST] S1-U09 info: connected after %d ms\n",
              (INT)(s1_comm_now_ms() - start_ms));
    check("S1-U09", comm_is_connected(), "WiFi + MQTT connected within 30 s");

    comm_get_stats(&before);
    (void)tk_dly_tsk(TELEM_WAIT_MS);
    comm_get_stats(&after);
    check("S1-U10",
          ((uint16_t)(after.tx_queued - before.tx_queued) >=
           TELEM_MIN_FRAMES) &&
              (after.tx_dropped == before.tx_dropped),
          "telemetry published for 3 s with no drops");
#endif
}

#endif /* APP_TEST */

/*** end of file ***/

