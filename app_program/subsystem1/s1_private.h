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
IMPORT ID	mbf_tx;		/* comm_task -> WiFi task, TX_RECORD   */
IMPORT ID	mbf_rx;		/* WiFi task -> comm_task, command text */
IMPORT ID	mtx_telem;	/* protects the telemetry snapshot     */
IMPORT ID	flg_comm;	/* wakes comm_task early               */

/* flg_comm bits */
#define FLG_COMM_EVENT		0x01U	/* something in mbf_events */
#define FLG_COMM_RX		0x02U	/* something in mbf_rx     */
#define FLG_COMM_LINK		0x04U	/* MQTT state changed      */
#define FLG_COMM_ALL		(FLG_COMM_EVENT | FLG_COMM_RX | FLG_COMM_LINK)

/* Queue sizes */
#define EVENT_QUEUE_LEN		16		/* events buffered      */
#define TX_RECORD_HDR		2		/* topic id + retain    */
#define TX_RECORD_MAX		(TX_RECORD_HDR + COMM_PAYLOAD_MAX)
#define TX_BUF_SIZE		3072		/* bytes of mbf_tx, ~8 typical msgs */
#define RX_BUF_SIZE		(4 * (COMM_CMD_MAX + 8))
#define MBF_MSG_OVERHEAD	8		/* kernel header/align  */

#define TELEM_LOCK_TMO		5		/* ms, mutex wait limit */

/* ---------------------------------------------------------------------
 *  One event, as queued by comm_report_xxx()
 * --------------------------------------------------------------------- */
typedef enum {
	EVT_BARCODE = 1,
	EVT_HUMP,
	EVT_OBSTACLE,
	EVT_ERROR
} COMM_EVT_TYPE;

typedef struct {
	UB	type;		/* COMM_EVT_TYPE */
	UB	code;		/* barcode letter / subsystem no. / action */
	UB	nav;		/* NAV_CMD (barcode) */
	UB	reserved;
	H	v[4];		/* values, meaning per type */
	UW	time_ms;	/* when it was reported */
	char	text[COMM_ERR_TEXT_MAX + 1];
} COMM_EVENT;

/* ---------------------------------------------------------------------
 *  Shared state (comm_bridge.c / task_comm.c)
 * --------------------------------------------------------------------- */
IMPORT volatile INT	s1_mqtt_state;	/* COMM_MQTT_STATE, from bridge */
IMPORT volatile INT	s1_comm_state;	/* COMM_STATE, from comm_task   */
IMPORT COMM_STATS	s1_stats;
IMPORT COMM_TELEMETRY	s1_telem;	/* guarded by mtx_telem */

IMPORT UW   s1_now_ms(void);
IMPORT void s1_stat_inc(volatile UW *counter);
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
 *  comm_json.c -- tiny bounded JSON writer (never overruns the buffer)
 * --------------------------------------------------------------------- */
typedef struct {
	UB	*buf;
	INT	cap;		/* total bytes available, incl. NUL */
	INT	len;		/* bytes written so far */
	BOOL	overflow;	/* TRUE once anything did not fit */
} JSON_BUF;

IMPORT void json_begin(JSON_BUF *jb, UB *buf, INT cap);
IMPORT void json_int(JSON_BUF *jb, const char *key, W value);
IMPORT void json_uint(JSON_BUF *jb, const char *key, UW value);
IMPORT void json_str(JSON_BUF *jb, const char *key, const char *value);
IMPORT void json_bool(JSON_BUF *jb, const char *key, BOOL value);
IMPORT void json_pair(JSON_BUF *jb, const char *key, W a, W b);
IMPORT void json_obj_open(JSON_BUF *jb, const char *key);
IMPORT void json_obj_close(JSON_BUF *jb);
IMPORT INT  json_end(JSON_BUF *jb);	/* length, or -1 if it overflowed */

/* ---------------------------------------------------------------------
 *  comm_cmd.c -- command parser / validator (pure function)
 * --------------------------------------------------------------------- */
typedef enum {
	CMD_OK = 0,
	CMD_ERR_EMPTY,
	CMD_ERR_TOO_LONG,
	CMD_ERR_BAD_CHAR,
	CMD_ERR_UNKNOWN,
	CMD_ERR_MISSING_ARG,
	CMD_ERR_EXTRA_ARG,
	CMD_ERR_BAD_NUMBER,
	CMD_ERR_RANGE,
	CMD_ERR_BAD_MODE,
	CMD_ERR_COUNT
} CMD_RESULT;

/* Commands handled by subsystem 1 itself (never forwarded) */
typedef enum {
	LOCAL_NONE = 0,		/* forward to the Vehicle Controller */
	LOCAL_PING,		/* reply "pong"                      */
	LOCAL_STATUS,		/* publish a status message now      */
	LOCAL_TELEM		/* change the telemetry period       */
} LOCAL_CMD;

typedef struct {
	REMOTE_CMD	cmd;	/* filled when local == LOCAL_NONE */
	LOCAL_CMD	local;
	W		local_arg;
} PARSED_CMD;

IMPORT CMD_RESULT comm_cmd_parse(const UB *text, INT len, PARSED_CMD *out);
IMPORT const char *cmd_result_text(CMD_RESULT r);

#endif /* S1_PRIVATE_H */
