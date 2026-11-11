/** @file comm.h
 * @brief Subsystem 1 public API: telemetry setters, one-off events,
 * remote commands and link status (Buddy 1, student 2502999).
 *
 * Other subsystems include "../subsystem1/subsystem1.h", which includes
 * this file and keeps the older type names working (see that header).
 * Every call here is non-blocking (NFR-B1-2) and may be made from any
 * task, never from an interrupt handler.
 *
 * Every value is stored and sent in the minimum number of bits for its
 * range (Change Requirement 1).  Out-of-range values are SATURATED to
 * the nearest limit, never wrapped, except the encoder totals.
 *
 * Interfaces (IDs used in testing/subsystem1/DESIGN_B1.md):
 *   I1.1 IF-S1-TLM  comm_set_xxx()      B2/B3/B4/B5/VC -> D1.1
 *   I1.2 IF-S1-EVT  comm_report_xxx()   B3/B4/B5/any   -> D1.1
 *   I1.3 IF-S1-CMD  mbf_remote_cmd      D1.2 -> Vehicle Controller
 *   I1.4 link       comm_get_state()    D1.2 -> anyone
 *
 * This header needs only <stdint.h>, <stdbool.h> and the kernel's ID/ER
 * types, so the team's host tests can compile it against mock kernels.
 */

#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

/* ---------------------------------------------------------------------
 * Shared enumerations.  Each fits the bit width given; the laptop
 * decoder (tools/s1_wire.py) and dashboard hold the names.
 * ------------------------------------------------------------------- */

/** @brief Robot operating state, the system statechart (4 bits). */
typedef enum
{
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
} comm_robot_state_t;

/** @brief Navigation command decoded from a barcode (3 bits). */
typedef enum
{
    NAV_NONE = 0,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_UTURN,
    NAV_COUNT
} comm_nav_cmd_t;

/** @brief Obstacle avoidance decision from B5 (3 bits). */
typedef enum
{
    AVOID_NONE = 0,
    AVOID_STOP,
    AVOID_CONTINUE,
    AVOID_LEFT,
    AVOID_RIGHT,
    AVOID_REVERSE,
    AVOID_COUNT
} comm_avoid_action_t;

/** @brief Link state, the Buddy 1 subsystem statechart (3 bits). */
typedef enum
{
    COMM_INITIALISING = 0,
    COMM_CONNECTING_WIFI,
    COMM_CONNECTING_MQTT,
    COMM_CONNECTED,
    COMM_RECONNECTING,
    COMM_OFFLINE,
    COMM_DISABLED, /* built without WiFi/MQTT, or the radio failed */
    COMM_STATE_COUNT
} comm_link_state_t;

/** @brief Remote command type, laptop -> Vehicle Controller (4 bits). */
typedef enum
{
    RCMD_NONE = 0,
    RCMD_START,      /* begin / continue the mission           */
    RCMD_STOP,       /* controlled stop -> SAFE_STOP           */
    RCMD_ESTOP,      /* emergency stop, motors off now         */
    RCMD_PAUSE,      /* hold position, keep state              */
    RCMD_RESUME,     /* continue after PAUSE                   */
    RCMD_MODE,       /* arg = comm_run_mode_t                  */
    RCMD_SPEED,      /* arg = target speed, percent 0..100     */
    RCMD_RESET_PEAK, /* forget the highest hump measured so far */
    RCMD_COUNT
} comm_remote_cmd_type_t;

/** @brief Run mode carried by RCMD_MODE (not "ROBOT_MODE": robot_config.h
 * already #defines that name). */
typedef enum
{
    MODE_AUTO = 0,  /* full autonomous mission              */
    MODE_LINE_ONLY, /* line following only, ignore barcodes */
    MODE_TEST,      /* bench test, motors disabled          */
    MODE_COUNT
} comm_run_mode_t;

/* A..D are 1..4; Z is 5; 0 = none yet (3 bits). */
#define COMM_BARCODE_FIRST  'A'
#define COMM_BARCODE_LAST   'D'
#define COMM_BARCODE_Z      'Z'
#define COMM_BARCODE_Z_CODE 5U

/* IMU motion state: B4's imu_motion_state_t, 0..7 (3 bits). */
#define COMM_MOTION_COUNT   8U

/* Line-sensor flags for comm_set_line() (B3); OR them (3 bits). */
#define LINE_F_DETECTED     0x01U
#define LINE_F_LOST         0x02U
#define LINE_F_JUNCTION     0x04U
#define LINE_F_ALL          0x07U

/* Field limits, bits on the wire in brackets.  B2 checks that
 * COMM_SPEED_MAX is defined to detect this binary interface. */
#define COMM_SPEED_MAX      8191   /* [14 signed] counts/s; B2 max 5000 */
#define COMM_IR_MAX         4095U  /* [12] raw ADC                       */
#define COMM_PITCH_MAX      511    /* [10 signed] 0.1 deg (+-51.1 deg)   */
#define COMM_HUMP_MAX_MM    126U   /* [7]  0.1 cm = 1 mm resolution      */
#define COMM_DIST_MAX_CM    510U   /* [9]  distances, clearances, 1 cm   */
#define COMM_WIDTH_MAX_CM   126U   /* [7]  obstacle width, 1 cm          */
#define COMM_CM_INVALID     0xFFFFU /* "no echo / not measured"          */
/* Front telemetry only: B5 ranges end at 400 cm. Keep assumed open space
 * distinct from an actual range without increasing the 9-bit payload. */
#define COMM_FRONT_OPEN_SPACE_CM 510U

/* Widths of the fields that hold signed values (two's complement). */
#define COMM_SPEED_BITS     14U
#define COMM_PITCH_BITS     10U
#define COMM_ENC_MASK       0xFFFFFFU /* encoder totals keep 24 bits     */

/* ---------------------------------------------------------------------
 * I1.3 IF-S1-CMD: remote commands, subsystem 1 -> Vehicle Controller.
 * Message buffer of fixed-size messages, owned by subsystem 1.
 *
 * Bit-fields: C99 6.7.2.1 makes only (unsigned) int and _Bool portable
 * bit-field types, so these use the kernel's UINT (unsigned int).  The
 * struct never leaves the Pico (the wire format is packed by shifts in
 * comm_pack.c), so compiler bit order does not matter (BARR-C 5.5).
 * ------------------------------------------------------------------- */

/** @brief One remote command, 4 bytes, copied whole through the kernel. */
typedef struct
{
    UINT type  : 4;  /* comm_remote_cmd_type_t                  */
    UINT arg   : 7;  /* meaning depends on type (see above)     */
    UINT seq   : 8;  /* sender's sequence number, echoed in ack */
    UINT spare : 13;
} comm_remote_cmd_t;

#define REMOTE_CMD_QUEUE_LEN 8U /* commands buffered for the VC */

extern ID mbf_remote_cmd; /* I1.3; name kept for the team (DEV-1) */

/* ---------------------------------------------------------------------
 * I1.1 IF-S1-TLM: the telemetry snapshot (24 bytes, was 40).
 *
 * Several tasks write different fields of the same 32-bit words, so
 * every access is a read-modify-write under a mutex (comm.c).  Signed
 * quantities (speeds, pitch) are held as two's complement bit patterns
 * in unsigned fields (BARR-C 5.3.a forbids signed bit-fields): exactly
 * the bits that go on the wire.  comm_pack_sign_extend() recovers them.
 * ------------------------------------------------------------------- */

/** @brief Latest value of every telemetry field. */
typedef struct
{
    UINT robot_state  : 4;  /* comm_robot_state_t              */
    UINT line_flags   : 3;  /* LINE_F_xxx                      */
    UINT ir_right     : 1;  /* digital 0/1                     */
    UINT ir_left      : 12; /* raw ADC 0..4095                 */
    UINT ir_centre    : 12;

    UINT speed_left   : 14; /* counts/s, two's complement      */
    UINT speed_right  : 14;
    UINT motion       : 3;  /* imu_motion_state_t (B4)         */
    UINT hump_valid   : 1;  /* hump heights are measured       */

    UINT enc_left     : 24; /* low 24 bits of B2's total count */
    UINT barcode      : 3;  /* 0 none, 1..4 = A..D, 5 = Z     */
    UINT nav          : 3;  /* comm_nav_cmd_t of last barcode  */
    UINT spare1       : 2;

    UINT enc_right    : 24;
    UINT hump_last_mm : 7;  /* height of the most recent hump  */
    UINT spare2       : 1;

    UINT pitch_ddeg   : 10; /* 0.1 deg, two's complement      */
    UINT hump_peak_mm : 7;  /* highest hump this run           */
    UINT obs_width_cm : 7;  /* 127 = not measured              */
    UINT spare3       : 8;

    UINT obs_dist_cm  : 9;  /* 511 = no echo / not measured    */
    UINT obs_left_cm  : 9;  /* left clearance                  */
    UINT obs_right_cm : 9;  /* right clearance                 */
    UINT spare4       : 5;
} comm_telemetry_t;

/** @brief Link counters: 16 bits, wrapping; the laptop uses differences
 * between heartbeats. */
typedef struct
{
    uint16_t tx_queued;   /* frames handed to the MQTT client        */
    uint16_t tx_dropped;  /* lost: a queue was full / frame too long */
    uint16_t evt_dropped; /* comm_report_xxx() calls that got E_TMOUT */
    uint16_t rx_ok;       /* valid commands received                 */
    uint16_t rx_rejected; /* invalid commands rejected (NFR-B1-5)    */
    uint16_t fwd_failed;  /* valid, but the VC queue was full        */
    uint16_t rx_dropped;  /* lost because the receive queue was full */
    uint16_t reconnects;  /* times the link came back after a loss   */
} comm_stats_t;

/* I1.1 setters: any task, non-blocking.  A setter whose mutex wait
 * times out skips that one update (the next call catches up). */
void comm_set_robot_state (comm_robot_state_t state);
void comm_set_motion (int32_t speed_l_cps, int32_t speed_r_cps,
                      int32_t enc_l, int32_t enc_r);
void comm_set_line (uint16_t ir_left, uint16_t ir_centre, uint8_t ir_right,
                    uint8_t flags);
void comm_set_imu (uint8_t motion_state, int16_t pitch_ddeg);
void comm_set_obstacle_dist (uint16_t dist_cm);
void comm_get_telemetry (comm_telemetry_t * p_out);

/* I1.2 events: E_OK, E_PAR (bad argument) or E_TMOUT (queue full: the
 * event is dropped and counted).  They never wait for the network. */
ER   comm_report_barcode (uint8_t letter, comm_nav_cmd_t nav);
ER   comm_report_hump (bool b_valid, int16_t height_mm, int16_t peak_mm);
void comm_reset_hump_peak (void);
ER   comm_report_obstacle (uint16_t dist_cm, uint16_t width_cm,
                           uint16_t left_cm, uint16_t right_cm,
                           comm_avoid_action_t action);
ER   comm_report_error (uint8_t err_code, uint8_t detail);

/* I1.4 link status, readable by anyone. */
comm_link_state_t comm_get_state (void);
bool              comm_is_connected (void);
void              comm_get_stats (comm_stats_t * p_out);

#endif /* COMM_H */

/*** end of file ***/
