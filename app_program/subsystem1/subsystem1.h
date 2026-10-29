/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi Communication, Command & Telemetry (Buddy 1).
 *	Student ID: 2502999.
 *
 *	This header is subsystem 1's PUBLIC interface: the only things other
 *	subsystems may use.  Include it as "../subsystem1/subsystem1.h".
 *
 *	HOW OTHER SUBSYSTEMS USE IT (all calls are non-blocking, NFR-B1-2)
 *	-----------------------------------------------------------------
 *	Every value is stored and sent in the minimum number of bits for its
 *	range (Change Requirement 1).  Out-of-range values are SATURATED to
 *	the nearest limit, never wrapped, except the encoder totals (below).
 *
 *	1. Keep the telemetry snapshot up to date (cheap, call as often as
 *	   you like; subsystem 1 samples it every telemetry period):
 *	       comm_set_motion(spd_l, spd_r, enc_l, enc_r);           B2
 *	       comm_set_line(ir_l, ir_c, ir_r, LINE_F_DETECTED);       B3
 *	       comm_set_imu(IMU_MOTION_CLIMBING_HUMP, pitch_ddeg);     B4
 *	       comm_set_obstacle_dist(front_cm);                       B5
 *	       comm_set_robot_state(ROBOT_LINE_FOLLOW);                VC
 *	   (B2 calls comm_set_motion() from subsystem2/b2_integration.c.)
 *
 *	2. Report one-off events (queued, published once each):
 *	       comm_report_barcode('A', NAV_LEFT);                     B3
 *	       comm_report_hump(TRUE, height_mm, peak_mm);             B4
 *	       comm_report_obstacle(d, w, lc, rc, AVOID_LEFT);  (cm)   B5
 *	       comm_report_error(ERR_S2_ENCODER, detail);              any
 *	   They return E_OK, E_PAR for an invalid argument, or E_TMOUT if the
 *	   queue is full (the event is dropped and counted).  They NEVER wait
 *	   for the network.
 *
 *	3. Faults: set/clear your bit in the shared mask (fault_flags.h).
 *
 *	4. Vehicle Controller: receive validated remote commands:
 *	       REMOTE_CMD cmd;
 *	       if (tk_rcv_mbf(mbf_remote_cmd, &cmd, TMO_POL) > 0) { ... }
 *
 *	Nothing here may be called from an interrupt handler, except
 *	fault_raise() / fault_clear().
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../error_codes.h"
#include "../fault_flags.h"

/* ---------------------------------------------------------------------
 *  Configuration
 * --------------------------------------------------------------------- */
#define S1_PRI_LED		12	/* priority band 12-13 (TEAM.md)      */
#define S1_PRI_DEMO		12	/* only when S1_STANDALONE_DEMO == 1  */
#define S1_PRI_COMM		13	/* lowest in the band (NFR-B1-3)      */

#define S1_LED_PIN		19	/* GP19: link-status LED + 330R to GND */
#define S1_TICK_MS		250	/* LED pattern step                    */

/* Telemetry period, in units of 100 ms (7 bits on the wire: 1..100) */
#define COMM_TELEM_PERIOD_DS	5	/* default 500 ms (2 Hz)               */
#define COMM_TELEM_MIN_DS	1	/* limits for the TELEM command        */
#define COMM_TELEM_MAX_DS	100	/* 10 s                                */
#define COMM_HEARTBEAT_MS	5000	/* status/heartbeat period             */
#define COMM_WIFI_OFFLINE_MS	30000	/* WiFi down this long -> OFFLINE      */

/* 1 = standalone test mode: subsystem 1 also runs a fake "robot" that
 *     feeds telemetry/events and consumes remote commands, so the whole
 *     WiFi/MQTT path can be demonstrated without the other subsystems.
 * 0 = integration: the real subsystems feed it and the Vehicle
 *     Controller consumes mbf_remote_cmd. */
#ifndef S1_STANDALONE_DEMO
#define S1_STANDALONE_DEMO	0
#endif

/* 0 = B2's telemetry task calls comm_set_motion() every 100 ms
 *     (subsystem2/b2_integration.c).  This is the normal setting.
 * 1 = comm_task reads B2's encoder_get_counts() itself and works out
 *     the speeds.  Only for a build without B2's telemetry task: with
 *     both, each would overwrite the other's values. */
#ifndef S1_POLL_B2_ENCODERS
#define S1_POLL_B2_ENCODERS	0
#endif

/* ---------------------------------------------------------------------
 *  Shared enums.  Each one fits the bit width given; the laptop decoder
 *  (tools/s1_wire.py) holds the names.
 * --------------------------------------------------------------------- */

/* Robot operating state = the system-level statechart (4 bits) */
typedef enum {
	ROBOT_INIT = 0,
	ROBOT_IDLE,
	ROBOT_LINE_FOLLOW,
	ROBOT_NAV_COMMAND,
	ROBOT_HUMP,
	ROBOT_OBSTACLE_SCAN,
	ROBOT_OBSTACLE_AVOID,
	ROBOT_LINE_RECOVERY,
	ROBOT_SAFE_STOP,
	ROBOT_STATE_COUNT
} ROBOT_STATE;

/* Navigation command decoded from a barcode (3 bits) */
typedef enum {
	NAV_NONE = 0,
	NAV_LEFT,
	NAV_RIGHT,
	NAV_STRAIGHT,
	NAV_UTURN,
	NAV_COUNT
} NAV_CMD;

/* Barcodes 'A'..'D' are sent as 1..4, 0 = none yet (3 bits) */
#define COMM_BARCODE_FIRST	'A'
#define COMM_BARCODE_LAST	'D'

/* IMU motion state: B4's imu_motion_state_t, 0..7 (3 bits) */
#define COMM_MOTION_COUNT	8

/* Obstacle avoidance decision (B5, 3 bits) */
typedef enum {
	AVOID_NONE = 0,
	AVOID_STOP,
	AVOID_CONTINUE,
	AVOID_LEFT,
	AVOID_RIGHT,
	AVOID_REVERSE,
	AVOID_COUNT
} AVOID_ACTION;

/* Line-sensor flags for comm_set_line() (B3), OR them together (3 bits) */
#define LINE_F_DETECTED		0x01U
#define LINE_F_LOST		0x02U
#define LINE_F_JUNCTION		0x04U

/* Link state = the Buddy 1 subsystem statechart (3 bits) */
typedef enum {
	COMM_INITIALISING = 0,
	COMM_CONNECTING_WIFI,
	COMM_CONNECTING_MQTT,
	COMM_CONNECTED,
	COMM_RECONNECTING,
	COMM_OFFLINE,
	COMM_DISABLED,		/* built without WiFi/MQTT, or radio failed */
	COMM_STATE_COUNT
} COMM_STATE;

/* Field limits (bits on the wire in brackets) */
#define COMM_SPEED_MAX		8191	/* [14 signed] counts/s; B2 max 5000 */
#define COMM_IR_MAX		4095	/* [12] raw ADC                       */
#define COMM_PITCH_MAX		511	/* [10 signed] 0.1 deg (+-51.1 deg)   */
#define COMM_HUMP_MAX_MM	126	/* [7]  0.1 cm = 1 mm resolution      */
#define COMM_DIST_MAX_CM	510	/* [9]  distances / clearances, 1 cm  */
#define COMM_WIDTH_MAX_CM	126	/* [7]  obstacle width, 1 cm          */
#define COMM_CM_INVALID		0xFFFFU	/* pass for "no echo / not measured"  */

/* ---------------------------------------------------------------------
 *  IF-S1-CMD: remote commands, Subsystem 1 -> Vehicle Controller
 *  Message buffer, fixed-size REMOTE_CMD messages, owned by S1.
 * --------------------------------------------------------------------- */
typedef enum {
	RCMD_NONE = 0,
	RCMD_START,		/* begin / continue the mission           */
	RCMD_STOP,		/* controlled stop -> SAFE_STOP            */
	RCMD_ESTOP,		/* emergency stop, motors off now          */
	RCMD_PAUSE,		/* hold position, keep state               */
	RCMD_RESUME,		/* continue after PAUSE                    */
	RCMD_MODE,		/* arg = RUN_MODE                          */
	RCMD_SPEED,		/* arg = target speed, percent 0..100      */
	RCMD_RESET_PEAK,	/* forget the highest hump measured so far */
	RCMD_COUNT
} REMOTE_CMD_TYPE;

/* (Not "ROBOT_MODE": robot_config.h #defines that name.) */
typedef enum {
	MODE_AUTO = 0,		/* full autonomous mission                 */
	MODE_LINE_ONLY,		/* line following only, ignore barcodes    */
	MODE_TEST,		/* bench test, motors disabled             */
	MODE_COUNT
} RUN_MODE;

/* 4 bytes (was 8).  Copied whole through the kernel, never shared. */
typedef struct {
	UINT	type	: 4;	/* REMOTE_CMD_TYPE */
	UINT	arg	: 7;	/* meaning depends on type (see above) */
	UINT	seq	: 8;	/* sender's sequence number, echoed in the ack */
	UINT	spare	: 13;
} REMOTE_CMD;

#define REMOTE_CMD_QUEUE_LEN	8	/* commands buffered for the VC */

IMPORT ID	mbf_remote_cmd;		/* IF-S1-CMD */

/* ---------------------------------------------------------------------
 *  IF-S1-TLM: telemetry snapshot setters (any task, non-blocking)
 *
 *  The snapshot is packed into C bit-fields (24 bytes, was 40).  Several
 *  tasks write different fields of the same 32-bit words, so EVERY
 *  access is a read-modify-write under mtx_telem (see comm_bridge.c).
 * --------------------------------------------------------------------- */
typedef struct {
	UINT	robot_state	: 4;	/* ROBOT_STATE */
	UINT	line_flags	: 3;	/* LINE_F_xxx */
	UINT	ir_right	: 1;	/* digital 0/1 */
	UINT	ir_left		: 12;	/* raw ADC 0..4095 */
	UINT	ir_centre	: 12;

	INT	speed_l		: 14;	/* counts/s, saturated */
	INT	speed_r		: 14;
	UINT	motion		: 3;	/* imu_motion_state_t (B4) */
	UINT	hump_valid	: 1;	/* hump heights are measured */

	UINT	enc_l		: 24;	/* low 24 bits of B2's total count */
	UINT	barcode		: 3;	/* 0 none, 1..4 = 'A'..'D' */
	UINT	nav		: 3;	/* NAV_CMD of last barcode */
	UINT	spare1		: 2;

	UINT	enc_r		: 24;
	UINT	hump_last_mm	: 7;	/* height of the most recent hump */
	UINT	spare2		: 1;

	INT	pitch_ddeg	: 10;	/* pitch, tenths of a degree */
	UINT	hump_peak_mm	: 7;	/* highest hump this run */
	UINT	obs_width_cm	: 7;	/* 127 = not measured */
	UINT	spare3		: 8;

	UINT	obs_dist_cm	: 9;	/* 511 = no echo / not measured */
	UINT	obs_left_cm	: 9;	/* left clearance */
	UINT	obs_right_cm	: 9;	/* right clearance */
	UINT	spare4		: 5;
} COMM_TELEMETRY;

IMPORT void comm_set_robot_state(ROBOT_STATE state);
IMPORT void comm_set_motion(W speed_l_cps, W speed_r_cps, W enc_l, W enc_r);
IMPORT void comm_set_line(UH ir_left, UH ir_centre, UB ir_right, UB flags);
IMPORT void comm_set_imu(UB motion_state, H pitch_ddeg);
IMPORT void comm_set_obstacle_dist(UH dist_cm);
IMPORT void comm_get_telemetry(COMM_TELEMETRY *out);	/* copy of snapshot */

/* ---------------------------------------------------------------------
 *  IF-S1-EVT: one-off events (any task, non-blocking)
 * --------------------------------------------------------------------- */
IMPORT ER comm_report_barcode(UB letter, NAV_CMD nav);
IMPORT ER comm_report_hump(BOOL valid, H height_mm, H peak_mm);
IMPORT ER comm_report_obstacle(UH dist_cm, UH width_cm, UH left_cm,
			       UH right_cm, AVOID_ACTION action);
IMPORT ER comm_report_error(UB err_code, UB detail);	/* error_codes.h */

/* ---------------------------------------------------------------------
 *  Link status, readable by anyone.  16-bit counters that wrap: the
 *  laptop works with differences between heartbeats.
 * --------------------------------------------------------------------- */
typedef struct {
	UH	tx_queued;	/* messages handed to the MQTT client      */
	UH	tx_dropped;	/* lost because a queue was full/too long  */
	UH	evt_dropped;	/* comm_report_xxx() calls that got E_TMOUT */
	UH	rx_ok;		/* valid commands received                 */
	UH	rx_rejected;	/* invalid commands rejected (NFR-B1-5)    */
	UH	fwd_failed;	/* valid but the VC queue was full         */
	UH	rx_dropped;	/* commands lost because mbf_rx was full   */
	UH	reconnects;	/* times the link came back after a loss   */
} COMM_STATS;

IMPORT COMM_STATE comm_get_state(void);
IMPORT BOOL	  comm_is_connected(void);
IMPORT void	  comm_get_stats(COMM_STATS *out);

/* Names for the console only (const tables: flash, no RAM; never NULL) */
IMPORT const char *comm_state_name(COMM_STATE s);
IMPORT const char *nav_cmd_name(NAV_CMD n);
IMPORT const char *avoid_action_name(AVOID_ACTION a);
IMPORT const char *remote_cmd_name(REMOTE_CMD_TYPE c);

/* IF-05 (template example, kept): the LED task's ID, so subsystem 4's
   example monitor can still pause/resume it */
IMPORT ID	tid_led;

#endif /* SUBSYSTEM1_H */
