/*
 *	comm_pack.c
 *	Subsystem 1 -- the binary MQTT wire format (Change Requirement 1:
 *	"compact binary fields rather than sending individual values as
 *	text").  Replaces the JSON writer and the text command parser.
 *
 *	Every frame is a sequence of fixed-width fields packed MSB-first with
 *	shifts and masks, starting with a 3-bit wire version.  Field order
 *	and widths are listed in MQTT.md section 2 and mirrored exactly in
 *	the laptop decoder tools/s1_wire.py; change all three together.
 *
 *	    frame       bits  bytes   was (JSON / text)
 *	    telemetry    223     28   250 typical, 374 worst case
 *	    event      39-70    5-9   60-110
 *	    status       179     23   ~190
 *	    ack           24      3   ~60
 *	    command       24      3   up to 64
 *
 *	No kernel calls and no globals: unit-tested on the host
 *	(testing/subsystem1/host/) as well as on target (S1-U03..U05).
 */

#include "s1_private.h"

/* ---------------------------------------------------------------------
 *  Field widths (bits).  The layout tables in MQTT.md use these names.
 * --------------------------------------------------------------------- */
#define B_VER		3
#define B_SEQ16		16
#define B_SEQ8		8
#define B_TIME		20	/* 0.1 s, wraps after 29 h */
#define B_STATE		4
#define B_FAULTS	FAULT_BITS
#define B_SPEED		14	/* signed */
#define B_ENC		24	/* two's complement, laptop unwraps */
#define B_IR		12
#define B_BIT		1
#define B_3		3
#define B_PITCH		10	/* signed */
#define B_HUMP		7
#define B_DIST		9
#define B_WIDTH		7
#define B_EVT_TYPE	2
#define B_ERR		8
#define B_IP		32
#define B_TELEM_DS	7
#define B_CMD_ID	4
#define B_CMD_ARG	7
#define B_RESULT	5
#define B_CMD_RES	2	/* reserved, must be 0 */

/* =====================================================================
 *  Bit writer / reader
 * ===================================================================== */

EXPORT void bits_begin(BITBUF *b, UB *buf, INT cap_bytes)
{
	INT i;

	for (i = 0; i < cap_bytes; i++) {
		buf[i] = 0U;
	}
	b->buf      = buf;
	b->cap_bits = (UH)(cap_bytes * 8);
	b->pos      = 0U;
	b->overflow = FALSE;
}

/* Writes the low nbits of value (1..32), most significant bit first.
 * Never writes past the buffer: a field that does not fit sets
 * overflow and is skipped. */
EXPORT void bits_put(BITBUF *b, UW value, INT nbits)
{
	INT i;

	if ((UW)b->pos + (UW)nbits > (UW)b->cap_bits) {
		b->overflow = TRUE;
		return;
	}
	for (i = nbits - 1; i >= 0; i--) {
		if (((value >> i) & 1U) != 0U) {
			b->buf[b->pos >> 3] |= (UB)(0x80U >> (b->pos & 7U));
		}
		b->pos++;
	}
}

EXPORT UW bits_get(BITBUF *b, INT nbits)
{
	UW	v = 0U;
	INT	i;

	if ((UW)b->pos + (UW)nbits > (UW)b->cap_bits) {
		b->overflow = TRUE;
		return 0U;
	}
	for (i = 0; i < nbits; i++) {
		v = (v << 1) |
		    (UW)((b->buf[b->pos >> 3] >> (7U - (b->pos & 7U))) & 1U);
		b->pos++;
	}
	return v;
}

EXPORT W bits_get_signed(BITBUF *b, INT nbits)
{
	UW v    = bits_get(b, nbits);
	UW sign = 1UL << (nbits - 1);

	return (W)((v ^ sign) - sign);		/* sign-extend */
}

EXPORT INT bits_bytes(const BITBUF *b)
{
	return b->overflow ? -1 : (INT)((b->pos + 7U) >> 3);
}

EXPORT UW sat_u(W value, UW max)
{
	if (value < 0) {
		return 0U;
	}
	return ((UW)value > max) ? max : (UW)value;
}

EXPORT W sat_s(W value, W max)
{
	if (value > max) {
		return max;
	}
	return (value < -max) ? -max : value;
}

/* =====================================================================
 *  Car -> laptop frames
 * ===================================================================== */

EXPORT INT pack_telemetry(UB *out, INT cap, UH seq, UW time_ds, UB faults,
			  const COMM_TELEMETRY *t)
{
	BITBUF b;

	bits_begin(&b, out, cap);
	bits_put(&b, WIRE_VERSION, B_VER);
	bits_put(&b, seq, B_SEQ16);
	bits_put(&b, time_ds, B_TIME);
	bits_put(&b, t->robot_state, B_STATE);
	bits_put(&b, faults, B_FAULTS);

	bits_put(&b, (UW)t->speed_l, B_SPEED);		/* B2 */
	bits_put(&b, (UW)t->speed_r, B_SPEED);
	bits_put(&b, t->enc_l, B_ENC);
	bits_put(&b, t->enc_r, B_ENC);

	bits_put(&b, t->ir_left, B_IR);			/* B3 */
	bits_put(&b, t->ir_centre, B_IR);
	bits_put(&b, t->ir_right, B_BIT);
	bits_put(&b, t->line_flags, B_3);
	bits_put(&b, t->barcode, B_3);
	bits_put(&b, t->nav, B_3);

	bits_put(&b, t->motion, B_3);			/* B4 */
	bits_put(&b, (UW)t->pitch_ddeg, B_PITCH);
	bits_put(&b, t->hump_valid, B_BIT);
	bits_put(&b, t->hump_last_mm, B_HUMP);
	bits_put(&b, t->hump_peak_mm, B_HUMP);

	bits_put(&b, t->obs_dist_cm, B_DIST);		/* B5 */
	bits_put(&b, t->obs_width_cm, B_WIDTH);
	bits_put(&b, t->obs_left_cm, B_DIST);
	bits_put(&b, t->obs_right_cm, B_DIST);
	return bits_bytes(&b);
}

EXPORT INT pack_event(UB *out, INT cap, UB seq, const COMM_EVENT *ev)
{
	BITBUF b;

	bits_begin(&b, out, cap);
	bits_put(&b, WIRE_VERSION, B_VER);
	bits_put(&b, ev->type, B_EVT_TYPE);
	bits_put(&b, seq, B_SEQ8);
	bits_put(&b, ev->time_ds, B_TIME);

	switch (ev->type) {
	case EVT_BARCODE:
		bits_put(&b, ev->a, B_3);		/* 1..4 = 'A'..'D' */
		bits_put(&b, ev->e, B_3);		/* NAV_CMD */
		break;
	case EVT_HUMP:
		bits_put(&b, ev->flag, B_BIT);		/* height valid */
		bits_put(&b, ev->a, B_HUMP);
		bits_put(&b, ev->b, B_HUMP);
		break;
	case EVT_OBSTACLE:
		bits_put(&b, ev->a, B_DIST);
		bits_put(&b, ev->b, B_WIDTH);
		bits_put(&b, ev->c, B_DIST);
		bits_put(&b, ev->d, B_DIST);
		bits_put(&b, ev->e, B_3);		/* AVOID_ACTION */
		break;
	default:	/* EVT_ERROR */
		bits_put(&b, ev->a, B_ERR);		/* error_codes.h */
		bits_put(&b, ev->b, B_ERR);		/* detail */
		break;
	}
	return bits_bytes(&b);
}

EXPORT INT pack_status(UB *out, INT cap, const STATUS_INFO *s)
{
	BITBUF b;

	bits_begin(&b, out, cap);
	bits_put(&b, WIRE_VERSION, B_VER);	/* first byte matches the */
	bits_put(&b, 1U, B_BIT);		/* 1-byte online/offline  */
	bits_put(&b, s->link, B_3);
	bits_put(&b, s->demo ? 1U : 0U, B_BIT);
	bits_put(&b, s->seq, B_SEQ16);
	bits_put(&b, s->time_ds, B_TIME);
	bits_put(&b, s->ip, B_IP);
	bits_put(&b, s->telem_ds, B_TELEM_DS);
	bits_put(&b, s->faults, B_FAULTS);
	bits_put(&b, s->st.tx_queued, B_SEQ16);
	bits_put(&b, s->st.tx_dropped, B_SEQ16);
	bits_put(&b, s->st.evt_dropped, B_SEQ16);
	bits_put(&b, s->st.rx_ok, B_SEQ16);
	bits_put(&b, s->st.rx_rejected, B_SEQ16);
	bits_put(&b, s->st.reconnects, B_SEQ8);
	return bits_bytes(&b);
}

EXPORT INT pack_ack(UB *out, INT cap, UB seq, UB cmd_id, UB result)
{
	BITBUF b;

	bits_begin(&b, out, cap);
	bits_put(&b, WIRE_VERSION, B_VER);
	bits_put(&b, seq, B_SEQ8);
	bits_put(&b, cmd_id, B_CMD_ID);
	bits_put(&b, (result < CMD_ERR_FIRST) ? 1U : 0U, B_BIT);
	bits_put(&b, result, B_RESULT);
	bits_put(&b, 0U, B_3);			/* pad to 3 bytes */
	return bits_bytes(&b);
}

/* =====================================================================
 *  Laptop -> car: command  ver:3 id:4 arg:7 seq:8 reserved:2
 * ===================================================================== */

/* Largest valid argument per command id; 0 = takes no argument */
LOCAL const UB cmd_arg_max[WCMD_COUNT] = {
	0,			/* 0: not a command */
	0, 0, 0, 0, 0,		/* START STOP ESTOP PAUSE RESUME */
	MODE_COUNT - 1,		/* MODE  */
	100,			/* SPEED, percent */
	0,			/* RESET_PEAK */
	0, 0,			/* PING STATUS */
	COMM_TELEM_MAX_DS	/* TELEM, 100 ms units */
};

EXPORT CMD_RESULT comm_cmd_decode(const UB *in, INT len, PARSED_CMD *out)
{
	BITBUF	b;
	UW	ver;
	UW	reserved;

	out->cmd.type  = RCMD_NONE;
	out->cmd.arg   = 0U;
	out->cmd.seq   = 0U;
	out->cmd.spare = 0U;
	out->id  = 0U;
	out->seq = 0U;
	out->arg = 0U;

	if (len != WIRE_CMD_LEN) {
		return CMD_ERR_LENGTH;
	}

	/* Read-only use of the bit reader: in[] is not modified */
	b.buf      = (UB *)in;
	b.cap_bits = WIRE_CMD_LEN * 8;
	b.pos      = 0U;
	b.overflow = FALSE;

	ver      = bits_get(&b, B_VER);
	out->id  = (UB)bits_get(&b, B_CMD_ID);
	out->arg = (UB)bits_get(&b, B_CMD_ARG);
	out->seq = (UB)bits_get(&b, B_SEQ8);
	reserved = bits_get(&b, B_CMD_RES);

	if (ver != WIRE_VERSION) {
		return CMD_ERR_VERSION;
	}
	if (reserved != 0U) {
		return CMD_ERR_RESERVED;
	}
	if ((out->id == 0U) || (out->id >= WCMD_COUNT)) {
		return CMD_ERR_UNKNOWN;
	}
	if ((cmd_arg_max[out->id] == 0U) && (out->arg != 0U)) {
		return CMD_ERR_EXTRA_ARG;
	}
	if (out->arg > cmd_arg_max[out->id]) {
		return CMD_ERR_RANGE;
	}
	if ((out->id == WCMD_TELEM) && (out->arg < COMM_TELEM_MIN_DS)) {
		return CMD_ERR_RANGE;
	}

	if (out->id < RCMD_COUNT) {
		out->cmd.type = out->id;
		out->cmd.arg  = out->arg;
		out->cmd.seq  = out->seq;
	}
	return CMD_OK;
}

/* Console text only: const, so it stays in flash */
LOCAL const char *const result_text[CMD_RESULT_COUNT] = {
	"forwarded", "pong", "status sent", "telemetry period set",
	"?", "?", "?", "?",
	"frame is not 3 bytes", "unknown wire version",
	"reserved bits not zero", "unknown command",
	"argument out of range", "unexpected argument", "controller busy"
};

EXPORT const char *cmd_result_text(CMD_RESULT r)
{
	return ((UINT)r < (UINT)CMD_RESULT_COUNT) ? result_text[r] : "?";
}
