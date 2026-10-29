/*
 *	s1_private.h
 *	Subsystem 1 -- definitions shared ONLY between subsystem 1's own .c
 *	files.  Other subsystems must not include this; they use
 *	subsystem1.h.
 */

#ifndef S1_PRIVATE_H
#define S1_PRIVATE_H

#include "subsystem1.h"
#include "comm_bridge.h"

/* ---------------------------------------------------------------------
 *  Kernel objects (created in subsystem1.c)
 * --------------------------------------------------------------------- */
IMPORT ID	cyc_led;	/* LED pattern tick */
IMPORT ID	tid_comm;	/* the WiFi & Telemetry task */
IMPORT ID	mbf_events;	/* app tasks -> comm_task, COMM_EVENT  */
IMPORT ID	mbf_tx;		/* comm_task -> WiFi task, TX record   */
IMPORT ID	mbf_rx;		/* WiFi task -> comm_task, command     */
IMPORT ID	mtx_telem;	/* protects the telemetry snapshot     */
IMPORT ID	flg_comm;	/* wakes comm_task early               */

/* flg_comm bits */
#define FLG_COMM_EVENT		0x01U	/* something in mbf_events */
#define FLG_COMM_RX		0x02U	/* something in mbf_rx     */
#define FLG_COMM_LINK		0x04U	/* MQTT state changed      */
#define FLG_COMM_ALL		(FLG_COMM_EVENT | FLG_COMM_RX | FLG_COMM_LINK)

/* Queue sizes (binary frames: was 3072 + 288 + 832 bytes of buffers) */
#define EVENT_QUEUE_LEN		16		/* events buffered      */
#define TX_RECORD_HDR		1		/* topic (2 bits) + retain (1 bit) */
#define TX_RECORD_MAX		(TX_RECORD_HDR + COMM_PAYLOAD_MAX)
#define TX_BUF_SIZE		640		/* ~17 telemetry or ~40 event frames */
#define RX_QUEUE_LEN		8		/* commands buffered    */
#define MBF_MSG_OVERHEAD	8		/* kernel header/align  */
#define RX_BUF_SIZE		(RX_QUEUE_LEN * (COMM_CMD_MAX + 1 + MBF_MSG_OVERHEAD))

#define TELEM_LOCK_TMO		5		/* ms, mutex wait limit */

/* ---------------------------------------------------------------------
 *  One event, as queued by comm_report_xxx(): 8 bytes (was 44).
 *  Built in the caller's own stack and copied whole by tk_snd_mbf, so it
 *  is never shared while being written.
 *
 *  type      a          b         c        d        e        flag
 *  BARCODE   1..4       -         -        -        NAV_CMD  -
 *  HUMP      height mm  peak mm   -        -        -        valid
 *  OBSTACLE  dist cm    width cm  left cm  right cm action   -
 *  ERROR     err code   detail    -        -        -        -
 * --------------------------------------------------------------------- */
typedef enum {
	EVT_BARCODE = 0,
	EVT_HUMP,
	EVT_OBSTACLE,
	EVT_ERROR		/* 4 types: 2 bits */
} COMM_EVT_TYPE;

typedef struct {
	UINT	time_ds	: 20;	/* 0.1 s since boot (wraps after 29 h) */
	UINT	type	: 2;	/* COMM_EVT_TYPE */
	UINT	a	: 9;
	UINT	flag	: 1;
	UINT	b	: 9;
	UINT	c	: 9;
	UINT	d	: 9;
	UINT	e	: 3;
	UINT	spare	: 2;
} COMM_EVENT;

/* ---------------------------------------------------------------------
 *  Shared state (comm_bridge.c / task_comm.c)
 *
 *  The two link states are 3-bit values but are kept in a whole byte:
 *  each has ONE writer and is read by other tasks without a lock, and a
 *  byte store is atomic on the RP2040 while a bit-field store is a
 *  read-modify-write of the whole word.
 * --------------------------------------------------------------------- */
IMPORT volatile UB	s1_mqtt_state;	/* COMM_MQTT_STATE, from bridge */
IMPORT volatile UB	s1_comm_state;	/* COMM_STATE, from comm_task   */
IMPORT COMM_STATS	s1_stats;
IMPORT COMM_TELEMETRY	s1_telem;	/* guarded by mtx_telem */

IMPORT UW   s1_now_ms(void);
IMPORT void comm_task(INT stacd, void *exinf);
IMPORT void led_task(INT stacd, void *exinf);
IMPORT void led_cyclic_handler(void *exinf);

/* Put one message in mbf_tx (never blocks).  FALSE = dropped. */
IMPORT BOOL s1_tx_enqueue(COMM_TOPIC_ID topic, BOOL retain,
			  const UB *payload, INT len);

#if S1_STANDALONE_DEMO
IMPORT void demo_task(INT stacd, void *exinf);
#endif

/* ---------------------------------------------------------------------
 *  comm_pack.c -- binary wire format (pure functions, no kernel calls,
 *  so they are unit-tested on the host: testing/subsystem1/host/).
 *  The layouts are in MQTT.md section 2.
 * --------------------------------------------------------------------- */
#define WIRE_VERSION		1	/* 3 bits, first field of every frame */

/* Frame sizes in bytes */
#define WIRE_TELEM_LEN		28
#define WIRE_STATUS_LEN		23
#define WIRE_ACK_LEN		3
#define WIRE_CMD_LEN		3
#define WIRE_EVT_MAX_LEN	9	/* obstacle; barcode 5, hump 6, error 7 */

/* MSB-first bit writer / reader over a byte array */
typedef struct {
	UB	*buf;
	UH	cap_bits;
	UH	pos;		/* bits written / read so far */
	BOOL	overflow;	/* TRUE once anything did not fit */
} BITBUF;

IMPORT void bits_begin(BITBUF *b, UB *buf, INT cap_bytes);	/* zero-fills */
IMPORT void bits_put(BITBUF *b, UW value, INT nbits);
IMPORT UW   bits_get(BITBUF *b, INT nbits);
IMPORT W    bits_get_signed(BITBUF *b, INT nbits);
IMPORT INT  bits_bytes(const BITBUF *b);	/* bytes used, -1 if overflow */

/* Clamp to the range of an n-bit field */
IMPORT UW   sat_u(W value, UW max);
IMPORT W    sat_s(W value, W max);

/* Heartbeat contents */
typedef struct {
	UB		link;		/* COMM_STATE */
	BOOL		demo;
	UH		seq;
	UW		time_ds;
	UW		ip;		/* lwIP order: first octet lowest */
	UB		telem_ds;
	UB		faults;
	COMM_STATS	st;
} STATUS_INFO;

IMPORT INT pack_telemetry(UB *out, INT cap, UH seq, UW time_ds, UB faults,
			  const COMM_TELEMETRY *t);
IMPORT INT pack_event(UB *out, INT cap, UB seq, const COMM_EVENT *ev);
IMPORT INT pack_status(UB *out, INT cap, const STATUS_INFO *s);
IMPORT INT pack_ack(UB *out, INT cap, UB seq, UB cmd_id, UB result);

/* Commands as sent by the laptop (4-bit id): 1..8 = REMOTE_CMD_TYPE,
 * then the ones subsystem 1 handles itself. */
#define WCMD_PING		9	/* reply "pong"                    */
#define WCMD_STATUS		10	/* publish a status message now    */
#define WCMD_TELEM		11	/* arg = period in 100 ms, 1..100  */
#define WCMD_COUNT		12

/* Ack result (5 bits).  Below CMD_ERR_FIRST = accepted. */
typedef enum {
	CMD_OK = 0,		/* forwarded to the Vehicle Controller */
	CMD_OK_PONG,
	CMD_OK_STATUS,
	CMD_OK_TELEM,
	CMD_ERR_FIRST = 8,
	CMD_ERR_LENGTH = CMD_ERR_FIRST,	/* frame is not 3 bytes         */
	CMD_ERR_VERSION,	/* unknown wire version                 */
	CMD_ERR_RESERVED,	/* reserved bits not zero               */
	CMD_ERR_UNKNOWN,	/* no such command id                   */
	CMD_ERR_RANGE,		/* argument out of range                */
	CMD_ERR_EXTRA_ARG,	/* argument given to a command without  */
	CMD_ERR_BUSY,		/* valid, but the VC queue was full     */
	CMD_RESULT_COUNT
} CMD_RESULT;

typedef struct {
	REMOTE_CMD	cmd;	/* filled for ids 1..8 */
	UB		id;	/* wire command id, echoed in the ack */
	UB		seq;
	UB		arg;
} PARSED_CMD;

/* Validates one received frame.  id/seq are filled whenever the frame
 * had the right length, so even a rejection can be matched by the
 * sender. */
IMPORT CMD_RESULT comm_cmd_decode(const UB *in, INT len, PARSED_CMD *out);
IMPORT const char *cmd_result_text(CMD_RESULT r);	/* console only */

#endif /* S1_PRIVATE_H */
