/** @file comm_pack.c
 * @brief Subsystem 1 binary wire format (see comm_pack.h and MQTT.md
 * section 2).  Change Requirement 1: "compact binary fields rather than
 * sending individual values as text".
 */

#include "comm_pack.h"
#include "../fault.h"

/* Field widths (bits).  The layout tables in MQTT.md use these names. */
#define BITS_VER       3U
#define BITS_SEQ16     16U
#define BITS_SEQ8      8U
#define BITS_TIME      20U /* 0.1 s, wraps after 29 h            */
#define BITS_STATE     4U
#define BITS_FAULTS    FAULT_BITS
#define BITS_SPEED     COMM_SPEED_BITS /* signed               */
#define BITS_ENC       24U /* two's complement, laptop unwraps    */
#define BITS_IR        12U
#define BITS_FLAG      1U
#define BITS_3         3U
#define BITS_PITCH     COMM_PITCH_BITS /* signed               */
#define BITS_HUMP      7U
#define BITS_DIST      9U
#define BITS_WIDTH     7U
#define BITS_EVT_TYPE  2U
#define BITS_ERR       8U
#define BITS_IP        32U
#define BITS_TELEM_DS  7U
#define BITS_CMD_ID    4U
#define BITS_CMD_ARG   7U
#define BITS_RESULT    5U
#define BITS_CMD_RES   2U  /* reserved, must be 0                 */

#define BITS_PER_BYTE  8U
#define BYTE_MSB       0x80U
#define BIT_IN_BYTE    7U
#define MAX_FIELD_BITS 32U
#define SPEED_PERCENT  100U
#define CMD_ID_MASK    0x0FU
#define CMD_ARG_MASK   0x7FU

/* Largest valid argument per command id; 0 = takes no argument. */
static const uint8_t g_cmd_arg_max[WCMD_COUNT] =
{
    0U,                      /* 0: not a command              */
    0U, 0U, 0U, 0U, 0U,      /* START STOP ESTOP PAUSE RESUME */
    (uint8_t)(MODE_COUNT - 1), /* MODE                        */
    SPEED_PERCENT,           /* SPEED, percent                */
    0U,                      /* RESET_PEAK                    */
    0U, 0U,                  /* PING STATUS                   */
    COMM_TELEM_MAX_DS        /* TELEM, 100 ms units           */
};

/* Console text only: const, so it stays in flash. */
static const char * const g_result_text[CMD_RESULT_COUNT] =
{
    "forwarded", "pong", "status sent", "telemetry period set",
    "?", "?", "?", "?",
    "frame is not 3 bytes", "unknown wire version",
    "reserved bits not zero", "unknown command",
    "argument out of range", "unexpected argument", "controller busy"
};

static uint32_t           field_mask (uint32_t nbits);
static comm_pack_result_t check_command (uint32_t version,
                                         uint32_t reserved,
                                         const comm_pack_cmd_t * p_cmd);

/** @brief Start writing into p_buf (zero-filled first). */
void
comm_pack_bits_begin (comm_pack_bits_t * p_bits, uint8_t * p_buf,
                      uint32_t cap_bytes)
{
    uint32_t index;

    for (index = 0U; index < cap_bytes; index++)
    {
        p_buf[index] = 0U;
    }

    p_bits->p_buf      = p_buf;
    /* Frames are at most COMM_PAYLOAD_MAX bytes: fits in 16 bits. */
    p_bits->cap_bits   = (uint16_t)(cap_bytes * BITS_PER_BYTE);
    p_bits->pos        = 0U;
    p_bits->b_overflow = false;
}

/** @brief Append the low @p nbits (1..32) of @p value, MSB first.
 *
 * A field that does not fit sets b_overflow and is skipped, so the
 * buffer is never written past its end.
 */
void
comm_pack_bits_put (comm_pack_bits_t * p_bits, uint32_t value, uint32_t nbits)
{
    uint32_t bit;
    uint32_t byte;
    uint8_t  mask;

    if ((nbits > MAX_FIELD_BITS) ||
        (((uint32_t)p_bits->pos + nbits) > (uint32_t)p_bits->cap_bits))
    {
        p_bits->b_overflow = true;
    }
    else
    {
        for (bit = nbits; bit > 0U; bit--)
        {
            if (0U != ((value >> (bit - 1U)) & 1U))
            {
                byte = (uint32_t)p_bits->pos / BITS_PER_BYTE;
                /* 0x80 >> 0..7 always fits in a byte. */
                mask = (uint8_t)(BYTE_MSB >> ((uint32_t)p_bits->pos &
                                              BIT_IN_BYTE));
                p_bits->p_buf[byte] = (uint8_t)(p_bits->p_buf[byte] | mask);
            }

            p_bits->pos++;
        }
    }
}

/** @brief Bytes used so far, or -1 if anything overflowed. */
int32_t
comm_pack_bits_bytes (const comm_pack_bits_t * p_bits)
{
    int32_t used = -1;

    if (!p_bits->b_overflow)
    {
        /* pos <= cap_bits <= 256, so the byte count fits easily. */
        used = (int32_t)(((uint32_t)p_bits->pos + BIT_IN_BYTE) /
                         BITS_PER_BYTE);
    }

    return (used);
}

/** @brief Start reading from a read-only frame. */
void
comm_pack_read_begin (comm_pack_reader_t * p_reader, const uint8_t * p_buf,
                      uint32_t cap_bytes)
{
    p_reader->p_buf      = p_buf;
    /* Frames are at most COMM_PAYLOAD_MAX bytes: fits in 16 bits. */
    p_reader->cap_bits   = (uint16_t)(cap_bytes * BITS_PER_BYTE);
    p_reader->pos        = 0U;
    p_reader->b_overflow = false;
}

/** @brief Read the next @p nbits (1..32) as an unsigned value. */
uint32_t
comm_pack_read (comm_pack_reader_t * p_reader, uint32_t nbits)
{
    uint32_t value = 0U;
    uint32_t count;
    uint32_t byte;
    uint32_t shift;

    if ((nbits > MAX_FIELD_BITS) ||
        (((uint32_t)p_reader->pos + nbits) > (uint32_t)p_reader->cap_bits))
    {
        p_reader->b_overflow = true;
    }
    else
    {
        for (count = 0U; count < nbits; count++)
        {
            byte  = (uint32_t)p_reader->pos / BITS_PER_BYTE;
            shift = BIT_IN_BYTE - ((uint32_t)p_reader->pos & BIT_IN_BYTE);
            value = (value << 1U) |
                    (((uint32_t)p_reader->p_buf[byte] >> shift) & 1U);
            p_reader->pos++;
        }
    }

    return (value);
}

/** @brief Read the next @p nbits (1..31) as two's complement. */
int32_t
comm_pack_read_signed (comm_pack_reader_t * p_reader, uint32_t nbits)
{
    return (comm_pack_sign_extend(comm_pack_read(p_reader, nbits), nbits));
}

/** @brief Clamp @p value to 0..max. */
uint32_t
comm_pack_sat_u (int32_t value, uint32_t max)
{
    uint32_t result = 0U;

    if (value > 0)
    {
        /* value is positive here, so the conversion is exact. */
        result = (uint32_t)value;
        if (result > max)
        {
            result = max;
        }
    }

    return (result);
}

/** @brief Clamp @p value to -max..max. */
int32_t
comm_pack_sat_s (int32_t value, int32_t max)
{
    int32_t result = value;

    if (value > max)
    {
        result = max;
    }
    else if (value < -max)
    {
        result = -max;
    }
    else
    {
        result = value;
    }

    return (result);
}

/** @brief @p value as an @p nbits two's complement bit pattern. */
uint32_t
comm_pack_twos (int32_t value, uint32_t nbits)
{
    /* Conversion to unsigned is defined as modulo 2^32 (C99 6.3.1.3),
     * which is the two's complement pattern; the mask keeps nbits. */
    return ((uint32_t)value & field_mask(nbits));
}

/** @brief Recover a signed value from an @p nbits (1..31) pattern. */
int32_t
comm_pack_sign_extend (uint32_t raw, uint32_t nbits)
{
    uint32_t mask  = field_mask(nbits);
    uint32_t bits  = raw & mask;
    int32_t  value = 0;

    if ((nbits > 0U) && (nbits < MAX_FIELD_BITS))
    {
        /* bits <= mask <= 2^31 - 1, so both fit in int32_t; for a set
         * sign bit the result is bits - 2^nbits. */
        value = (int32_t)bits;
        if (0U != (bits & (1UL << (nbits - 1U))))
        {
            value = value - (int32_t)mask - 1;
        }
    }

    return (value);
}

/** @brief Pack a telemetry frame (223 bits, 28 bytes); -1 = too small. */
int32_t
comm_pack_telemetry (uint8_t * p_out, uint32_t cap, uint16_t seq,
                     uint32_t time_ds, uint8_t faults,
                     const comm_telemetry_t * p_telem)
{
    comm_pack_bits_t bits;

    comm_pack_bits_begin(&bits, p_out, cap);
    comm_pack_bits_put(&bits, WIRE_VERSION, BITS_VER);
    comm_pack_bits_put(&bits, seq, BITS_SEQ16);
    comm_pack_bits_put(&bits, time_ds, BITS_TIME);
    comm_pack_bits_put(&bits, p_telem->robot_state, BITS_STATE);
    comm_pack_bits_put(&bits, faults, BITS_FAULTS);

    comm_pack_bits_put(&bits, p_telem->speed_left, BITS_SPEED); /* B2 */
    comm_pack_bits_put(&bits, p_telem->speed_right, BITS_SPEED);
    comm_pack_bits_put(&bits, p_telem->enc_left, BITS_ENC);
    comm_pack_bits_put(&bits, p_telem->enc_right, BITS_ENC);

    comm_pack_bits_put(&bits, p_telem->ir_left, BITS_IR); /* B3 */
    comm_pack_bits_put(&bits, p_telem->ir_centre, BITS_IR);
    comm_pack_bits_put(&bits, p_telem->ir_right, BITS_FLAG);
    comm_pack_bits_put(&bits, p_telem->line_flags, BITS_3);
    comm_pack_bits_put(&bits, p_telem->barcode, BITS_3);
    comm_pack_bits_put(&bits, p_telem->nav, BITS_3);

    comm_pack_bits_put(&bits, p_telem->motion, BITS_3); /* B4 */
    comm_pack_bits_put(&bits, p_telem->pitch_ddeg, BITS_PITCH);
    comm_pack_bits_put(&bits, p_telem->hump_valid, BITS_FLAG);
    comm_pack_bits_put(&bits, p_telem->hump_last_mm, BITS_HUMP);
    comm_pack_bits_put(&bits, p_telem->hump_peak_mm, BITS_HUMP);

    comm_pack_bits_put(&bits, p_telem->obs_dist_cm, BITS_DIST); /* B5 */
    comm_pack_bits_put(&bits, p_telem->obs_width_cm, BITS_WIDTH);
    comm_pack_bits_put(&bits, p_telem->obs_left_cm, BITS_DIST);
    comm_pack_bits_put(&bits, p_telem->obs_right_cm, BITS_DIST);

    return (comm_pack_bits_bytes(&bits));
}

/** @brief Pack an event frame (5..9 bytes); -1 = buffer too small. */
int32_t
comm_pack_event (uint8_t * p_out, uint32_t cap, uint8_t seq,
                 const comm_pack_event_t * p_event)
{
    comm_pack_bits_t bits;

    comm_pack_bits_begin(&bits, p_out, cap);
    comm_pack_bits_put(&bits, WIRE_VERSION, BITS_VER);
    comm_pack_bits_put(&bits, p_event->type, BITS_EVT_TYPE);
    comm_pack_bits_put(&bits, seq, BITS_SEQ8);
    comm_pack_bits_put(&bits, p_event->time_ds, BITS_TIME);

    switch (p_event->type)
    {
        case EVT_BARCODE:
            comm_pack_bits_put(&bits, p_event->val_a, BITS_3); /* 1..5  */
            comm_pack_bits_put(&bits, p_event->val_e, BITS_3); /* nav   */
        break;

        case EVT_HUMP:
            comm_pack_bits_put(&bits, p_event->flag, BITS_FLAG); /* valid */
            comm_pack_bits_put(&bits, p_event->val_a, BITS_HUMP);
            comm_pack_bits_put(&bits, p_event->val_b, BITS_HUMP);
        break;

        case EVT_OBSTACLE:
            comm_pack_bits_put(&bits, p_event->val_a, BITS_DIST);
            comm_pack_bits_put(&bits, p_event->val_b, BITS_WIDTH);
            comm_pack_bits_put(&bits, p_event->val_c, BITS_DIST);
            comm_pack_bits_put(&bits, p_event->val_d, BITS_DIST);
            comm_pack_bits_put(&bits, p_event->val_e, BITS_3); /* action */
        break;

        default: /* EVT_ERROR */
            comm_pack_bits_put(&bits, p_event->val_a, BITS_ERR); /* code */
            comm_pack_bits_put(&bits, p_event->val_b, BITS_ERR);
        break;
    }

    return (comm_pack_bits_bytes(&bits));
}

/** @brief Pack a status/heartbeat frame (179 bits, 23 bytes). */
int32_t
comm_pack_status (uint8_t * p_out, uint32_t cap,
                  const comm_pack_status_t * p_status)
{
    comm_pack_bits_t bits;

    comm_pack_bits_begin(&bits, p_out, cap);
    comm_pack_bits_put(&bits, WIRE_VERSION, BITS_VER); /* first byte  */
    comm_pack_bits_put(&bits, 1U, BITS_FLAG);          /* = online    */
    comm_pack_bits_put(&bits, p_status->link, BITS_3);
    comm_pack_bits_put(&bits, p_status->b_demo ? 1U : 0U, BITS_FLAG);
    comm_pack_bits_put(&bits, p_status->seq, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->time_ds, BITS_TIME);
    comm_pack_bits_put(&bits, p_status->ip_addr, BITS_IP);
    comm_pack_bits_put(&bits, p_status->telem_ds, BITS_TELEM_DS);
    comm_pack_bits_put(&bits, p_status->faults, BITS_FAULTS);
    comm_pack_bits_put(&bits, p_status->stats.tx_queued, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->stats.tx_dropped, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->stats.evt_dropped, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->stats.rx_ok, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->stats.rx_rejected, BITS_SEQ16);
    comm_pack_bits_put(&bits, p_status->stats.reconnects, BITS_SEQ8);

    return (comm_pack_bits_bytes(&bits));
}

/** @brief Pack a command ack (24 bits, 3 bytes). */
int32_t
comm_pack_ack (uint8_t * p_out, uint32_t cap, uint8_t seq, uint8_t cmd_id,
               uint8_t result)
{
    comm_pack_bits_t bits;
    uint32_t         accepted = 0U;

    if (result < (uint8_t)CMD_ERR_FIRST)
    {
        accepted = 1U;
    }

    comm_pack_bits_begin(&bits, p_out, cap);
    comm_pack_bits_put(&bits, WIRE_VERSION, BITS_VER);
    comm_pack_bits_put(&bits, seq, BITS_SEQ8);
    comm_pack_bits_put(&bits, cmd_id, BITS_CMD_ID);
    comm_pack_bits_put(&bits, accepted, BITS_FLAG);
    comm_pack_bits_put(&bits, result, BITS_RESULT);
    comm_pack_bits_put(&bits, 0U, BITS_3); /* pad to 3 bytes */

    return (comm_pack_bits_bytes(&bits));
}

/** @brief Decode one command: ver:3 id:4 arg:7 seq:8 reserved:2. */
comm_pack_result_t
comm_pack_cmd_decode (const uint8_t * p_in, uint32_t len,
                      comm_pack_cmd_t * p_out)
{
    comm_pack_reader_t reader;
    comm_pack_result_t result = CMD_ERR_LENGTH;
    uint32_t           version;
    uint32_t           reserved;

    p_out->cmd    = (comm_remote_cmd_t){0};
    p_out->cmd_id = 0U;
    p_out->seq    = 0U;
    p_out->arg    = 0U;

    if (WIRE_CMD_LEN == len)
    {
        comm_pack_read_begin(&reader, p_in, WIRE_CMD_LEN);
        version = comm_pack_read(&reader, BITS_VER);
        /* Each read returns at most its field width (4/7/8 bits). */
        p_out->cmd_id = (uint8_t)comm_pack_read(&reader, BITS_CMD_ID);
        p_out->arg    = (uint8_t)comm_pack_read(&reader, BITS_CMD_ARG);
        p_out->seq    = (uint8_t)comm_pack_read(&reader, BITS_SEQ8);
        reserved      = comm_pack_read(&reader, BITS_CMD_RES);
        result        = check_command(version, reserved, p_out);

        if ((CMD_OK == result) && (p_out->cmd_id < (uint8_t)RCMD_COUNT))
        {
            p_out->cmd.type = (UINT)p_out->cmd_id & CMD_ID_MASK;
            p_out->cmd.arg  = (UINT)p_out->arg & CMD_ARG_MASK;
            p_out->cmd.seq  = p_out->seq;
        }
    }

    return (result);
}

/** @brief Console text for an ack result; never NULL. */
const char *
comm_pack_result_text (comm_pack_result_t result)
{
    const char * p_text = "?";

    if ((uint32_t)result < (uint32_t)CMD_RESULT_COUNT)
    {
        p_text = g_result_text[result];
    }

    return (p_text);
}

/** @brief Mask with the low @p nbits (0..32) set. */
static uint32_t
field_mask (uint32_t nbits)
{
    uint32_t mask = UINT32_MAX;

    if (nbits < MAX_FIELD_BITS)
    {
        mask = (1UL << nbits) - 1UL;
    }

    return (mask);
}

/** @brief Validate the decoded fields of a 3-byte command frame. */
static comm_pack_result_t
check_command (uint32_t version, uint32_t reserved,
               const comm_pack_cmd_t * p_cmd)
{
    comm_pack_result_t result = CMD_OK;

    if (WIRE_VERSION != version)
    {
        result = CMD_ERR_VERSION;
    }
    else if (0U != reserved)
    {
        result = CMD_ERR_RESERVED;
    }
    else if ((0U == p_cmd->cmd_id) || (p_cmd->cmd_id >= WCMD_COUNT))
    {
        result = CMD_ERR_UNKNOWN;
    }
    else if ((0U == g_cmd_arg_max[p_cmd->cmd_id]) && (0U != p_cmd->arg))
    {
        result = CMD_ERR_EXTRA_ARG;
    }
    else if (p_cmd->arg > g_cmd_arg_max[p_cmd->cmd_id])
    {
        result = CMD_ERR_RANGE;
    }
    else if ((WCMD_TELEM == p_cmd->cmd_id) &&
             (p_cmd->arg < COMM_TELEM_MIN_DS))
    {
        result = CMD_ERR_RANGE; /* period below 100 ms */
    }
    else
    {
        result = CMD_OK;
    }

    return (result);
}

/*** end of file ***/
