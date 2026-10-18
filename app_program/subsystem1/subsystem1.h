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
 *	1. Keep the telemetry snapshot up to date (cheap, call as often as
 *	   you like; subsystem 1 samples it every COMM_TELEM_PERIOD_MS):
 *	       comm_set_motion(spd_l, spd_r, enc_l, enc_r, dist_mm);   B2
 *	       comm_set_line(ir_l, ir_c, ir_r, LINE_F_DETECTED);       B3
 *	       comm_set_imu(IMU_EV_CLIMBING, pitch_ddeg, peak_mm);     B4
 *	       comm_set_obstacle(dist, width, left_clr, right_clr);    B5
 *	       comm_set_robot_state(ROBOT_LINE_FOLLOW);                VC
 *
 *	2. Report one-off events (queued, published once each):
 *	       comm_report_barcode('A', NAV_LEFT);                     B3
 *	       comm_report_hump(height_mm, peak_mm);                   B4
 *	       comm_report_obstacle(dist, width, lc, rc, AVOID_LEFT);  B5
 *	       comm_report_error(2, 17, "encoder stalled");            any
 *	   They return E_OK, or E_TMOUT if the queue is full (the event is
 *	   dropped and counted).  They NEVER wait for the network.
 *
 *	3. Vehicle Controller: receive validated remote commands:
 *	       REMOTE_CMD cmd;
 *	       if (tk_rcv_mbf(mbf_remote_cmd, &cmd, TMO_POL) > 0) { ... }
 *
 *	Nothing here may be called from an interrupt handler.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Configuration
 * --------------------------------------------------------------------- */
#define S1_PRI_LED		12	/* priority band 12-13 (TEAM.md)      */
#define S1_PRI_DEMO		12	/* only when S1_STANDALONE_DEMO == 1  */
#define S1_PRI_COMM		13	/* lowest in the band (NFR-B1-3)      */

#define S1_LED_PIN		19	/* GP19: link-status LED + 330R to GND */
#define S1_TICK_MS		250	/* LED pattern step                    */

#define COMM_TELEM_PERIOD_MS	500	/* default telemetry rate (2 Hz)       */
#define COMM_TELEM_MIN_MS	100	/* limits for the TELEM command        */
#define COMM_TELEM_MAX_MS	10000
#define COMM_HEARTBEAT_MS	5000	/* status/heartbeat period             */
#define COMM_WIFI_OFFLINE_MS	30000	/* WiFi down this long -> OFFLINE      */

/* 1 = standalone test mode: subsystem 1 also runs a fake "robot" that
 *     feeds telemetry/events and consumes remote commands, so the whole
 *     WiFi/MQTT path can be demonstrated without the other subsystems.
 * 0 = integration: the real subsystems feed it and the Vehicle
 *     Controller consumes mbf_remote_cmd.  SET TO 0 BEFORE INTEGRATING. */
#ifndef S1_STANDALONE_DEMO
#define S1_STANDALONE_DEMO	1
#endif

/* ---------------------------------------------------------------------
 *  Shared enums (string names are in comm_names.c)
 * --------------------------------------------------------------------- */

/* Robot operating state = the system-level statechart */
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

/* Navigation command decoded from a barcode */
typedef enum {
	NAV_NONE = 0,
	NAV_LEFT,
	NAV_RIGHT,
	NAV_STRAIGHT,
	NAV_UTURN,
	NAV_COUNT
} NAV_CMD;

/* IMU motion events (B4) */
typedef enum {
	IMU_EV_STATIONARY = 0,
	IMU_EV_MOVING,
	IMU_EV_ACCELERATING,
	IMU_EV_TURNING,
	IMU_EV_CLIMBING,
	IMU_EV_DESCENDING,
	IMU_EV_IMPACT,
	IMU_EV_COUNT
} IMU_EVENT;

/* Obstacle avoidance decision (B5) */
typedef enum {
	AVOID_NONE = 0,
	AVOID_STOP,
	AVOID_CONTINUE,
	AVOID_LEFT,
	AVOID_RIGHT,
	AVOID_REVERSE,
	AVOID_COUNT
} AVOID_ACTION;

/* Line-sensor flags for comm_set_line() (B3), OR them together */
#define LINE_F_DETECTED		0x01U
#define LINE_F_LOST		0x02U
#define LINE_F_JUNCTION		0x04U

/* Link state = the Buddy 1 subsystem statechart */
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
	RCMD_MODE,		/* arg = ROBOT_MODE                        */
	RCMD_SPEED,		/* arg = target speed, percent 0..100      */
	RCMD_RESET_PEAK,	/* forget the highest hump measured so far */
	RCMD_COUNT
} REMOTE_CMD_TYPE;

typedef enum {
	MODE_AUTO = 0,		/* full autonomous mission                 */
	MODE_LINE_ONLY,		/* line following only, ignore barcodes    */
	MODE_TEST,		/* bench test, motors disabled             */
	MODE_COUNT
} ROBOT_MODE;

typedef struct {
	UB	type;		/* REMOTE_CMD_TYPE */
	UB	reserved;
	H	arg;		/* meaning depends on type (see above) */
	UW	seq;		/* "#n" sent with the command, else 0 */
} REMOTE_CMD;

#define REMOTE_CMD_QUEUE_LEN	8	/* commands buffered for the VC */

IMPORT ID	mbf_remote_cmd;		/* IF-S1-CMD */

/* ---------------------------------------------------------------------
 *  IF-S1-TLM: telemetry snapshot setters (any task, non-blocking)
 * --------------------------------------------------------------------- */
typedef struct {
	UB	robot_state;	/* ROBOT_STATE */
	UB	line_flags;	/* LINE_F_xxx */
	UB	imu_event;	/* IMU_EVENT */
	UB	last_barcode;	/* ASCII letter, 0 = none yet */
	H	speed_l;	/* mm/s */
	H	speed_r;	/* mm/s */
	W	enc_l;		/* encoder counts */
	W	enc_r;
	W	distance_mm;	/* total distance travelled */
	UH	ir_left;	/* raw ADC 0..4095 */
	UH	ir_centre;
	UB	ir_right;	/* digital 0/1 */
	UB	last_nav;	/* NAV_CMD of last barcode */
	H	pitch_ddeg;	/* pitch, tenths of a degree */
	H	hump_last_mm;	/* height of the most recent hump */
	H	hump_peak_mm;	/* highest hump this run */
	H	obs_dist_mm;	/* 0 = no obstacle */
	H	obs_width_mm;
	H	obs_left_mm;	/* left clearance */
	H	obs_right_mm;	/* right clearance */
} COMM_TELEMETRY;

IMPORT void comm_set_robot_state(ROBOT_STATE state);
IMPORT void comm_set_motion(H speed_l, H speed_r, W enc_l, W enc_r,
			    W distance_mm);
IMPORT void comm_set_line(UH ir_left, UH ir_centre, UB ir_right, UB flags);
IMPORT void comm_set_imu(IMU_EVENT ev, H pitch_ddeg, H hump_peak_mm);
IMPORT void comm_set_obstacle(H dist_mm, H width_mm, H left_mm, H right_mm);
IMPORT void comm_get_telemetry(COMM_TELEMETRY *out);	/* copy of snapshot */

/* ---------------------------------------------------------------------
 *  IF-S1-EVT: one-off events (any task, non-blocking)
 *  Return E_OK, or E_TMOUT when the queue is full (event dropped).
 * --------------------------------------------------------------------- */
#define COMM_ERR_TEXT_MAX	24	/* error text is cut to this */

IMPORT ER comm_report_barcode(UB code, NAV_CMD nav);
IMPORT ER comm_report_hump(H height_mm, H peak_mm);
IMPORT ER comm_report_obstacle(H dist_mm, H width_mm, H left_mm,
			       H right_mm, AVOID_ACTION action);
IMPORT ER comm_report_error(UB subsystem, H code, const char *text);

/* ---------------------------------------------------------------------
 *  Link status, readable by anyone
 * --------------------------------------------------------------------- */
typedef struct {
	UW	tx_queued;	/* messages handed to the MQTT client      */
	UW	tx_dropped;	/* lost because a queue was full/too long  */
	UW	evt_dropped;	/* comm_report_xxx() calls that got E_TMOUT */
	UW	rx_ok;		/* valid commands received                 */
	UW	rx_rejected;	/* invalid commands rejected (NFR-B1-5)    */
	UW	fwd_failed;	/* valid but the VC queue was full         */
	UW	rx_dropped;	/* commands lost because mbf_rx was full   */
	UW	reconnects;	/* times the link came back after a loss   */
} COMM_STATS;

IMPORT COMM_STATE comm_get_state(void);
IMPORT BOOL	  comm_is_connected(void);
IMPORT void	  comm_get_stats(COMM_STATS *out);

/* Names for printing / JSON (never NULL) */
IMPORT const char *comm_state_name(COMM_STATE s);
IMPORT const char *robot_state_name(ROBOT_STATE s);
IMPORT const char *nav_cmd_name(NAV_CMD n);
IMPORT const char *imu_event_name(IMU_EVENT e);
IMPORT const char *avoid_action_name(AVOID_ACTION a);
IMPORT const char *remote_cmd_name(REMOTE_CMD_TYPE c);
IMPORT const char *robot_mode_name(ROBOT_MODE m);

/* IF-05 (template example, kept): the LED task's ID, so subsystem 4's
   example monitor can still pause/resume it */
IMPORT ID	tid_led;

#endif /* SUBSYSTEM1_H */
