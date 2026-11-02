/** @file testpack_main.c
 * @brief Host-side test of Subsystem 1's binary wire format (comm_pack.c),
 * built with the laptop's gcc against the real kernel headers.  No Pico
 * needed.  Run everything with:  sh run_host_tests.sh
 *
 *   ./test_comm_pack vectors   pack min / middle / max / invalid values of
 *                              every field; print one line per frame:
 *                              kind hex field=value ...  test_wire.py
 *                              decodes the hex with the laptop decoder and
 *                              compares each field.
 *   ./test_comm_pack decode    read hex command frames from stdin (one per
 *                              line), print "result id arg seq".
 *   ./test_comm_pack sizes     message and struct sizes, before/after.
 *
 * Exit status 0 = every C-side check passed.
 */

#include "testpack_main.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/subsystem1/comm_pack.h"
#include "../../../app_program/subsystem1/comm_bridge.h"
#include "../../../app_program/fault.h"
#include "../../../app_program/error_codes.h"

#define HEX_LINE_MAX  64
#define FRAME_MAX     8U
#define ENC_BITS      24U
#define TIME_MASK     0xFFFFFU
#define BITS_STATE    4U
#define BITS_FAULTS   8U
#define BITS_SEQ16    16U
#define BITS_TIME     20U
#define BITS_VER      3U
#define BITS_SPEED    14U
#define STATS_PATTERN 0x5555U

static int32_t g_failures = 0;

/** @brief Telemetry field values, as the setters would store them. */
typedef struct telem_case
{
    int32_t  spd_l;
    int32_t  spd_r;
    int32_t  enc_l;
    int32_t  enc_r;
    uint16_t ir_l;
    uint16_t ir_c;
    uint8_t  ir_r;
    uint8_t  line;
    uint8_t  motion;
    int16_t  pitch;
    uint8_t  hump_valid;
    uint8_t  hump_last;
    uint8_t  hump_peak;
    uint16_t dist;
    uint16_t width;
    uint16_t left;
    uint16_t right;
    uint16_t seq;
    uint32_t tds;
    uint8_t  state;
    uint8_t  faults;
    uint8_t  barcode;
    uint8_t  nav;
} telem_case_t;

static void             check_eq (const char * p_what, int64_t got,
                                  int64_t want);
static void             print_hex (const char * p_kind,
                                   const uint8_t * p_buf, int32_t len);
static void             telemetry_vector (const telem_case_t * p_case);
static void             event_vector (uint32_t type, uint8_t seq, uint32_t tds,
                                      uint32_t val_a, uint32_t val_b,
                                      uint32_t val_c, uint32_t val_d,
                                      uint32_t val_e, uint32_t flag);
static void             status_vector (uint8_t link, bool b_demo,
                                       uint16_t seq, uint32_t tds,
                                       uint32_t ip_addr, uint8_t telem_ds,
                                       uint8_t faults, uint16_t count);
static void             ack_vector (uint8_t seq, uint8_t cmd_id,
                                    uint8_t result);
static void             vectors (void);
static int32_t          hex_to_bytes (const char * p_text, uint8_t * p_out,
                                      uint32_t cap);
static void             decode_stdin (void);
static void             sizes (void);

/** @brief Entry point: vectors (default), decode or sizes. */
int
main (int argc, char ** pp_argv)
{
    const char * p_mode = (argc > 1) ? pp_argv[1] : "vectors";

    if (0 == strcmp(p_mode, "decode"))
    {
        decode_stdin();
    }
    else if (0 == strcmp(p_mode, "sizes"))
    {
        sizes();
    }
    else
    {
        vectors();
    }

    if (0 != g_failures)
    {
        fprintf(stderr, "%d C-side check(s) FAILED\n", (int)g_failures);
    }

    return ((0 != g_failures) ? 1 : 0);
}

/** @brief Count and report a mismatch between two values. */
static void
check_eq (const char * p_what, int64_t got, int64_t want)
{
    if (got != want)
    {
        printf("FAIL %s: got %" PRId64 " want %" PRId64 "\n", p_what, got,
               want);
        g_failures++;
    }
}

/** @brief Print "kind hexbytes" (no newline). */
static void
print_hex (const char * p_kind, const uint8_t * p_buf, int32_t len)
{
    int32_t index;

    printf("%s ", p_kind);
    for (index = 0; index < len; index++)
    {
        printf("%02x", (unsigned)p_buf[index]);
    }
}


/** @brief Store a case exactly as comm.c does (saturate, then fields). */
static comm_telemetry_t
make_telemetry (const telem_case_t * p_case)
{
    comm_telemetry_t telem = {0};

    telem.robot_state  = p_case->state & 0x0FU;
    telem.speed_left   = comm_pack_twos(comm_pack_sat_s(p_case->spd_l,
                                                        COMM_SPEED_MAX),
                                        COMM_SPEED_BITS) & 0x3FFFU;
    telem.speed_right  = comm_pack_twos(comm_pack_sat_s(p_case->spd_r,
                                                        COMM_SPEED_MAX),
                                        COMM_SPEED_BITS) & 0x3FFFU;
    telem.enc_left     = (uint32_t)p_case->enc_l & COMM_ENC_MASK;
    telem.enc_right    = (uint32_t)p_case->enc_r & COMM_ENC_MASK;
    telem.ir_left      = comm_pack_sat_u(p_case->ir_l, COMM_IR_MAX) & 0xFFFU;
    telem.ir_centre    = comm_pack_sat_u(p_case->ir_c, COMM_IR_MAX) & 0xFFFU;
    telem.ir_right     = p_case->ir_r & 1U;
    telem.line_flags   = p_case->line & 7U;
    telem.barcode      = p_case->barcode & 7U;
    telem.nav          = p_case->nav & 7U;
    telem.motion       = p_case->motion & 7U;
    telem.pitch_ddeg   = comm_pack_twos(comm_pack_sat_s(p_case->pitch,
                                                        COMM_PITCH_MAX),
                                        COMM_PITCH_BITS) & 0x3FFU;
    telem.hump_valid   = p_case->hump_valid & 1U;
    telem.hump_last_mm = p_case->hump_last & 0x7FU;
    telem.hump_peak_mm = p_case->hump_peak & 0x7FU;
    telem.obs_dist_cm  = p_case->dist & 0x1FFU;
    telem.obs_width_cm = p_case->width & 0x7FU;
    telem.obs_left_cm  = p_case->left & 0x1FFU;
    telem.obs_right_cm = p_case->right & 0x1FFU;

    return (telem);
}

/** @brief Pack one telemetry case, self-check it and print it. */
static void
telemetry_vector (const telem_case_t * p_case)
{
    comm_telemetry_t   telem = make_telemetry(p_case);
    uint8_t            buf[COMM_PAYLOAD_MAX];
    comm_pack_reader_t reader;
    int32_t            len;
    int32_t            spd_l = comm_pack_sign_extend(telem.speed_left,
                                                     COMM_SPEED_BITS);
    int32_t            spd_r = comm_pack_sign_extend(telem.speed_right,
                                                     COMM_SPEED_BITS);

    len = comm_pack_telemetry(buf, COMM_PAYLOAD_MAX, p_case->seq, p_case->tds,
                              p_case->faults, &telem);
    check_eq("telemetry length", len, WIRE_TELEM_LEN);

    /* C-side round trip of a few fields with the bit reader. */
    comm_pack_read_begin(&reader, buf, (uint32_t)len);
    check_eq("telem ver", comm_pack_read(&reader, BITS_VER), WIRE_VERSION);
    check_eq("telem seq", comm_pack_read(&reader, BITS_SEQ16), p_case->seq);
    check_eq("telem time", comm_pack_read(&reader, BITS_TIME),
             p_case->tds & TIME_MASK);
    (void)comm_pack_read(&reader, BITS_STATE + BITS_FAULTS);
    check_eq("telem spd_l", comm_pack_read_signed(&reader, BITS_SPEED), spd_l);
    check_eq("telem spd_r", comm_pack_read_signed(&reader, BITS_SPEED), spd_r);

    print_hex("telemetry", buf, len);
    printf(" seq=%u t_ds=%u state=%u faults=%u spd_l=%d spd_r=%d "
           "enc_l=%d enc_r=%d ir_l=%u ir_c=%u ir_r=%u line=%u "
           "barcode=%u nav=%u motion=%u pitch_dd=%d hump_valid=%u "
           "hump_last_mm=%u hump_peak_mm=%u obs_d_cm=%u obs_w_cm=%u "
           "obs_lc_cm=%u obs_rc_cm=%u\n",
           (unsigned)p_case->seq, (unsigned)p_case->tds,
           (unsigned)p_case->state, (unsigned)p_case->faults, (int)spd_l,
           (int)spd_r,
           /* sign-extend the 24-bit totals, as the decoder does */
           (int)comm_pack_sign_extend(telem.enc_left, ENC_BITS),
           (int)comm_pack_sign_extend(telem.enc_right, ENC_BITS),
           (unsigned)telem.ir_left, (unsigned)telem.ir_centre,
           (unsigned)telem.ir_right, (unsigned)p_case->line,
           (unsigned)p_case->barcode, (unsigned)p_case->nav,
           (unsigned)p_case->motion,
           (int)comm_pack_sign_extend(telem.pitch_ddeg, COMM_PITCH_BITS),
           (unsigned)p_case->hump_valid, (unsigned)p_case->hump_last,
           (unsigned)p_case->hump_peak, (unsigned)p_case->dist,
           (unsigned)p_case->width, (unsigned)p_case->left,
           (unsigned)p_case->right);
}

/** @brief Pack one event, check its length and print it. */
static void
event_vector (uint32_t type, uint8_t seq, uint32_t tds, uint32_t val_a,
              uint32_t val_b, uint32_t val_c, uint32_t val_d, uint32_t val_e,
              uint32_t flag)
{
    static const int32_t want_len[4] = {5, 6, 9, 7};
    comm_pack_event_t    event       = {0};
    uint8_t              buf[COMM_PAYLOAD_MAX];
    int32_t              len;

    event.type    = type & 3U;
    event.time_ds = tds & TIME_MASK;
    event.val_a   = val_a & 0x1FFU;
    event.val_b   = val_b & 0x1FFU;
    event.val_c   = val_c & 0x1FFU;
    event.val_d   = val_d & 0x1FFU;
    event.val_e   = val_e & 7U;
    event.flag    = flag & 1U;
    len = comm_pack_event(buf, COMM_PAYLOAD_MAX, seq, &event);
    check_eq("event length", len, want_len[type & 3U]);

    print_hex("event", buf, len);
    printf(" type=%u seq=%u t_ds=%u", (unsigned)type, (unsigned)seq,
           (unsigned)tds);
    switch (type)
    {
        case EVT_BARCODE:
            printf(" code=%u nav=%u\n", (unsigned)val_a,
                   (unsigned)val_e);
        break;

        case EVT_HUMP:
            printf(" valid=%u height_mm=%u peak_mm=%u\n",
                   (unsigned)flag, (unsigned)val_a,
                   (unsigned)val_b);
        break;

        case EVT_OBSTACLE:
            printf(" d_cm=%u w_cm=%u lc_cm=%u rc_cm=%u action=%u\n",
                   (unsigned)val_a, (unsigned)val_b,
                   (unsigned)val_c, (unsigned)val_d,
                   (unsigned)val_e);
        break;

        default:
            printf(" err=%u detail=%u\n", (unsigned)val_a,
                   (unsigned)val_b);
        break;
    }
}

/** @brief Pack one status frame, check it and print it. */
static void
status_vector (uint8_t link, bool b_demo, uint16_t seq, uint32_t tds,
               uint32_t ip_addr, uint8_t telem_ds, uint8_t faults,
               uint16_t count)
{
    comm_pack_status_t status = {0};
    uint8_t            buf[COMM_PAYLOAD_MAX];
    int32_t            len;

    status.link              = link;
    status.b_demo            = b_demo;
    status.seq               = seq;
    status.time_ds           = tds;
    status.ip_addr           = ip_addr;
    status.telem_ds          = telem_ds;
    status.faults            = faults;
    status.stats.tx_queued   = count;
    status.stats.tx_dropped  = (uint16_t)(count ^ STATS_PATTERN);
    status.stats.evt_dropped = 1U;
    status.stats.rx_ok       = (uint16_t)(count >> 1U);
    status.stats.rx_rejected = 2U;
    status.stats.reconnects  = (uint16_t)(count & 0xFFU);
    len = comm_pack_status(buf, COMM_PAYLOAD_MAX, &status);
    check_eq("status length", len, WIRE_STATUS_LEN);
    check_eq("status byte 0 = online byte", buf[0] & 0xF0U,
             (uint8_t)COMM_STATUS_ONLINE[0]);

    print_hex("status", buf, len);
    printf(" online=1 link=%u demo=%u seq=%u t_ds=%u ip=%u "
           "telem_ds=%u faults=%u tx=%u drop=%u evdrop=1 rx=%u rej=2 "
           "reconn=%u\n",
           (unsigned)link, b_demo ? 1U : 0U, (unsigned)seq,
           (unsigned)tds, (unsigned)ip_addr, (unsigned)telem_ds,
           (unsigned)faults, (unsigned)count,
           (unsigned)(uint16_t)(count ^ STATS_PATTERN),
           (unsigned)(count >> 1U), (unsigned)(count & 0xFFU));
}

/** @brief Pack one ack, check it and print it. */
static void
ack_vector (uint8_t seq, uint8_t cmd_id, uint8_t result)
{
    uint8_t buf[COMM_PAYLOAD_MAX];
    int32_t len = comm_pack_ack(buf, COMM_PAYLOAD_MAX, seq, cmd_id, result);

    check_eq("ack length", len, WIRE_ACK_LEN);
    print_hex("cmd/ack", buf, len);
    printf(" seq=%u cmd=%u ok=%u result=%u\n", (unsigned)seq,
           (unsigned)cmd_id, (result < (uint8_t)CMD_ERR_FIRST) ? 1U : 0U,
           (unsigned)result);
}

/** @brief Every frame kind at its minimum, maximum and invalid values. */
static void
vectors (void)
{
    static const telem_case_t telem_cases[] =
    {
        /* minimum / zero */
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        /* maximum of every field, B2 max speed 5000 counts/s */
        {5000, -5000, 8388607, -8388608, 4095, 4095, 1, 7, 7, 511, 1, 126,
         126, 510, 126, 510, 510, 65535, 0xFFFFFU, ROBOT_SAFE_STOP, 0xFF, 4,
         NAV_UTURN},
        /* saturation and invalid markers */
        {20000, -20000, 1000, -1, 9999, 4096, 1, 1, 3, -900, 0, 0, 0, 0x1FF,
         0x7F, 0x1FF, 0x1FF, 1234, 98765, ROBOT_LINE_FOLLOW, FAULT_WIFI_LOSS,
         1, NAV_LEFT},
        /* typical, with a wrapped encoder total (> 2^24 counts) */
        {3000, 2987, 16777216 + 42, 123456, 812, 2400, 0, LINE_F_DETECTED, 1,
         -45, 1, 23, 31, 250, 12, 30, 9, 42, 2100, ROBOT_HUMP,
         FAULT_NO_ECHO | FAULT_IMU, 2, NAV_RIGHT},
    };
    uint32_t index;

    for (index = 0U; index < (sizeof(telem_cases) / sizeof(telem_cases[0]));
         index++)
    {
        telemetry_vector(&telem_cases[index]);
    }

    event_vector(EVT_BARCODE, 0U, 0U, 1U, 0U, 0U, 0U, NAV_LEFT, 0U);
    event_vector(EVT_BARCODE, 255U, 0xFFFFFU, 4U, 0U, 0U, 0U, NAV_UTURN, 0U);
    event_vector(EVT_HUMP, 7U, 160U, 23U, 31U, 0U, 0U, 0U, 1U);
    event_vector(EVT_HUMP, 8U, 161U, 0U, 0U, 0U, 0U, 0U, 0U);
    event_vector(EVT_OBSTACLE, 9U, 420U, 25U, 12U, 30U, 9U, AVOID_LEFT, 0U);
    event_vector(EVT_OBSTACLE, 10U, 421U, 0x1FFU, 0x7FU, 510U, 0U,
                 AVOID_REVERSE, 0U);
    event_vector(EVT_ERROR, 11U, 430U, ERR_S2_ENCODER, 17U, 0U, 0U, 0U, 0U);
    event_vector(EVT_ERROR, 12U, 431U, ERR_S5(9), 255U, 0U, 0U, 0U, 0U);

    status_vector(COMM_CONNECTED, false, 12U, 600U, 0x1701A8C0U, 5U, 0U, 151U);
    status_vector(COMM_DISABLED, true, 65535U, 0xFFFFFU, 0xFFFFFFFFU, 100U,
                  0xFFU, 65535U);

    ack_vector(5U, RCMD_SPEED, CMD_OK);
    ack_vector(0U, 0U, CMD_ERR_LENGTH);
    ack_vector(255U, WCMD_PING, CMD_OK_PONG);
    ack_vector(7U, RCMD_STOP, CMD_ERR_BUSY);
}

/** @brief Hex text to bytes; -1 if malformed or longer than @p cap. */
static int32_t
hex_to_bytes (const char * p_text, uint8_t * p_out, uint32_t cap)
{
    uint32_t count = 0U;
    unsigned value;
    int32_t  result = 0;

    while ((0 == result) && ('\0' != p_text[0]) && ('\n' != p_text[0]) &&
           ('\r' != p_text[0]))
    {
        if ((count >= cap) || (1 != sscanf(p_text, "%2x", &value)))
        {
            result = -1;
        }
        else
        {
            p_out[count] = (uint8_t)value;
            count++;
            p_text += 2;
        }
    }

    return ((0 == result) ? (int32_t)count : -1);
}

/** @brief Decode hex command frames from stdin (test_wire.py feeds it). */
static void
decode_stdin (void)
{
    char               line[HEX_LINE_MAX];
    uint8_t            frame[FRAME_MAX];
    comm_pack_cmd_t    cmd;
    comm_pack_result_t result;
    int32_t            len;

    while (NULL != fgets(line, (int)sizeof(line), stdin))
    {
        len    = hex_to_bytes(line, frame, FRAME_MAX);
        result = comm_pack_cmd_decode(frame, (len < 0) ? 0U : (uint32_t)len,
                                      &cmd);
        printf("%d %u %u %u\n", (int)result, (unsigned)cmd.cmd_id,
               (unsigned)cmd.arg, (unsigned)cmd.seq);
    }
}

/** @brief Message and struct sizes, before (JSON) and after (binary). */
static void
sizes (void)
{
    /* "before" = the JSON/text version (commit ec0b859). */
    printf("%-28s %12s %8s\n", "item", "before", "after");
    printf("%-28s %12s %8d\n", "telemetry frame (B)", "290 / 374",
           WIRE_TELEM_LEN);
    printf("%-28s %12s %8s\n", "event frame (B)", "55-86 / 117", "5-9");
    printf("%-28s %12s %8d\n", "status frame (B)", "157 / 233",
           WIRE_STATUS_LEN);
    printf("%-28s %12s %8u\n", "command frame (B)", "11 / 64",
           (unsigned)WIRE_CMD_LEN);
    printf("%-28s %12s %8d\n", "ack frame (B)", "60 / 143", WIRE_ACK_LEN);
    printf("%-28s %8d %8d\n", "COMM_TELEMETRY struct", 40,
           (int)sizeof(comm_telemetry_t));
    printf("%-28s %8d %8d\n", "COMM_EVENT struct", 44,
           (int)sizeof(comm_pack_event_t));
    printf("%-28s %8d %8d\n", "REMOTE_CMD struct", 8,
           (int)sizeof(comm_remote_cmd_t));
    printf("%-28s %8d %8d\n", "COMM_STATS struct", 32,
           (int)sizeof(comm_stats_t));
    check_eq("sizeof comm_telemetry_t", (int64_t)sizeof(comm_telemetry_t), 24);
    check_eq("sizeof comm_pack_event_t", (int64_t)sizeof(comm_pack_event_t),
             8);
    check_eq("sizeof comm_remote_cmd_t", (int64_t)sizeof(comm_remote_cmd_t),
             4);
}

/*** end of file ***/

