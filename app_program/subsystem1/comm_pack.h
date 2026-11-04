/** @file comm_pack.h
 * @brief Subsystem 1 binary wire format (design element D1.3): pure
 * functions with no kernel calls and no globals, unit-tested on the
 * laptop (testing/subsystem1/host/) and on target (S1-U03..U05).
 *
 * Every frame is a sequence of fixed-width fields packed MSB-first with
 * shifts and masks, starting with a 3-bit wire version.  Field order and
 * widths are listed in MQTT.md section 2 and mirrored exactly in the
 * laptop decoder tools/s1_wire.py and tools/dashboard/s1_wire.js;
 * change all of them together.
 *
 *     frame       bits  bytes   was (JSON / text)
 *     telemetry    223     28   250 typical, 374 worst case
 *     event      41-70    6-9   60-110
 *     status       179     23   ~190
 *     ack           24      3   ~60
 *     command       24      3   up to 64
 */

#ifndef COMM_PACK_H
#define COMM_PACK_H

#include <stdint.h>
#include <stdbool.h>
#include "comm.h"

#define WIRE_VERSION      1U  /* 3 bits, first field of every frame */

/* Frame sizes in bytes */
#define WIRE_TELEM_LEN    28
#define WIRE_STATUS_LEN   23
#define WIRE_ACK_LEN      3
#define WIRE_CMD_LEN      3U
#define WIRE_EVT_MAX_LEN  9   /* obstacle; barcode 5, hump 6, error 7 */
#define WIRE_EVT_BAR_LEN  6
#define WIRE_EVT_ERR_LEN  7

/** @brief MSB-first bit writer over a byte array. */
typedef struct
{
    uint8_t * p_buf;
    uint16_t  cap_bits;
    uint16_t  pos;        /* bits written so far             */
    bool      b_overflow; /* true once a field did not fit   */
} comm_pack_bits_t;

/** @brief MSB-first bit reader over a read-only byte array. */
typedef struct
{
    const uint8_t * p_buf;
    uint16_t        cap_bits;
    uint16_t        pos;        /* bits read so far              */
    bool            b_overflow; /* true once a read ran past end */
} comm_pack_reader_t;

/* ---------------------------------------------------------------------
 * One event as queued by comm_report_xxx(): 8 bytes (was 44).  Built
 * in the caller's own stack and copied whole by tk_snd_mbf, so it is
 * never shared while being written.
 *
 *   type      val_a      val_b     val_c    val_d    val_e    flag
 *   BARCODE   1..26      -         -        -        NAV_CMD  -
 *   HUMP      height mm  peak mm   -        -        -        valid
 *   OBSTACLE  dist cm    width cm  left cm  right cm action   -
 *   ERROR     err code   detail    -        -        -        -
 * ------------------------------------------------------------------- */

/** @brief Event kind (2 bits). */
typedef enum
{
    EVT_BARCODE = 0,
    EVT_HUMP,
    EVT_OBSTACLE,
    EVT_ERROR
} comm_pack_evt_type_t;

/** @brief One queued event (unsigned bit-fields, see comm.h). */
typedef struct
{
    UINT time_ds : 20; /* 0.1 s since boot (wraps after 29 h) */
    UINT type    : 2;  /* comm_pack_evt_type_t                */
    UINT val_a   : 9;
    UINT flag    : 1;
    UINT val_b   : 9;
    UINT val_c   : 9;
    UINT val_d   : 9;
    UINT val_e   : 3;
    UINT spare   : 2;
} comm_pack_event_t;

/** @brief Heartbeat (status frame) contents. */
typedef struct
{
    uint8_t      link;     /* comm_link_state_t               */
    bool         b_demo;
    uint16_t     seq;
    uint32_t     time_ds;
    uint32_t     ip_addr;  /* lwIP order: first octet lowest  */
    uint8_t      telem_ds;
    uint8_t      faults;
    comm_stats_t stats;
} comm_pack_status_t;

/* Commands as sent by the laptop (4-bit id): 1..8 are the
 * comm_remote_cmd_type_t values, then subsystem 1 commands and B3 mapping. */
#define WCMD_PING   9U  /* reply "pong"                   */
#define WCMD_STATUS 10U /* publish a status frame now     */
#define WCMD_TELEM  11U /* arg = period in 100 ms, 1..100 */
#define WCMD_BARCODE_MAP 12U /* arg = (A-Z index << 2) | route code */
#define WCMD_BARCODE_MAP_RESET 13U
#define WCMD_COUNT  14U

/* Telemetry period limits for WCMD_TELEM, 100 ms units (7 bits). */
#define COMM_TELEM_MIN_DS 1U
#define COMM_TELEM_MAX_DS 100U

/** @brief Ack result (5 bits).  Below CMD_ERR_FIRST = accepted. */
typedef enum
{
    CMD_OK = 0, /* forwarded to the Vehicle Controller */
    CMD_OK_PONG,
    CMD_OK_STATUS,
    CMD_OK_TELEM,
    CMD_ERR_FIRST = 8,
    CMD_ERR_LENGTH = CMD_ERR_FIRST, /* frame is not 3 bytes          */
    CMD_ERR_VERSION,                /* unknown wire version          */
    CMD_ERR_RESERVED,               /* reserved bits not zero        */
    CMD_ERR_UNKNOWN,                /* no such command id            */
    CMD_ERR_RANGE,                  /* argument out of range         */
    CMD_ERR_EXTRA_ARG,              /* argument to a no-arg command  */
    CMD_ERR_BUSY,                   /* valid, but the VC queue full  */
    CMD_RESULT_COUNT
} comm_pack_result_t;

/** @brief A decoded command frame. */
typedef struct
{
    comm_remote_cmd_t cmd;    /* filled for ids 1..8                */
    uint8_t           cmd_id; /* wire command id, echoed in the ack */
    uint8_t           seq;
    uint8_t           arg;
} comm_pack_cmd_t;

void    comm_pack_bits_begin (comm_pack_bits_t * p_bits, uint8_t * p_buf,
                              uint32_t cap_bytes);
void    comm_pack_bits_put (comm_pack_bits_t * p_bits, uint32_t value,
                            uint32_t nbits);
int32_t comm_pack_bits_bytes (const comm_pack_bits_t * p_bits);

void     comm_pack_read_begin (comm_pack_reader_t * p_reader,
                               const uint8_t * p_buf, uint32_t cap_bytes);
uint32_t comm_pack_read (comm_pack_reader_t * p_reader, uint32_t nbits);
int32_t  comm_pack_read_signed (comm_pack_reader_t * p_reader,
                                uint32_t nbits);

uint32_t comm_pack_sat_u (int32_t value, uint32_t max);
int32_t  comm_pack_sat_s (int32_t value, int32_t max);
uint32_t comm_pack_twos (int32_t value, uint32_t nbits);
int32_t  comm_pack_sign_extend (uint32_t raw, uint32_t nbits);

int32_t comm_pack_telemetry (uint8_t * p_out, uint32_t cap, uint16_t seq,
                             uint32_t time_ds, uint8_t faults,
                             const comm_telemetry_t * p_telem);
int32_t comm_pack_event (uint8_t * p_out, uint32_t cap, uint8_t seq,
                         const comm_pack_event_t * p_event);
int32_t comm_pack_status (uint8_t * p_out, uint32_t cap,
                          const comm_pack_status_t * p_status);
int32_t comm_pack_ack (uint8_t * p_out, uint32_t cap, uint8_t seq,
                       uint8_t cmd_id, uint8_t result);

/* Validates one received frame.  cmd_id/seq are filled whenever the
 * frame had the right length, so a rejection can still be matched. */
comm_pack_result_t comm_pack_cmd_decode (const uint8_t * p_in, uint32_t len,
                                         comm_pack_cmd_t * p_out);
const char *       comm_pack_result_text (comm_pack_result_t result);

#endif /* COMM_PACK_H */

/*** end of file ***/
