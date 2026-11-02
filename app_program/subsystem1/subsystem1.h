/** @file subsystem1.h
 * @brief Subsystem 1, WiFi Communication, Command & Telemetry (Buddy 1,
 * student 2502999): the header other subsystems include, as
 * "../subsystem1/subsystem1.h".
 *
 * HOW OTHER SUBSYSTEMS USE IT (all calls are non-blocking, NFR-B1-2)
 *
 * 1. Keep the telemetry snapshot up to date (cheap, call as often as
 *    you like; subsystem 1 samples it every telemetry period):
 *        comm_set_motion(spd_l, spd_r, enc_l, enc_r);           B2
 *        comm_set_line(ir_l, ir_c, ir_r, LINE_F_DETECTED);       B3
 *        comm_set_imu(IMU_MOTION_CLIMBING_HUMP, pitch_ddeg);     B4
 *        comm_set_obstacle_dist(front_cm);                       B5
 *        comm_set_robot_state(ROBOT_LINE_FOLLOW);                VC
 *
 * 2. Report one-off events (queued, published once each):
 *        comm_report_barcode('A', NAV_LEFT);                     B3
 *        comm_report_hump(true, height_mm, peak_mm);             B4
 *        comm_report_obstacle(d, w, lc, rc, AVOID_LEFT);  (cm)   B5
 *        comm_report_error(ERR_S2_ENCODER, detail);              any
 *
 * 3. Faults: set/clear your bit in the shared mask (fault.h).
 *
 * 4. Vehicle Controller: receive validated remote commands:
 *        REMOTE_CMD cmd;
 *        if (tk_rcv_mbf(mbf_remote_cmd, &cmd, TMO_POL) > 0) { ... }
 *
 * Nothing here may be called from an interrupt handler, except
 * fault_raise() / fault_clear() / fault_get().
 *
 * BARR-C note: the upper-case type names at the end of this file are
 * aliases kept so B2, B3, B4, B5 and the Vehicle Controller compile
 * unchanged (deviation DEV-1, testing/subsystem1/CODING_STANDARD.md).
 * New code should use the comm_xxx_t names from comm.h.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "comm.h"
#include "../fault_flags.h"
#include "../error_codes.h"

/* ---------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------- */
#define S1_PRI_LED   12 /* priority band 12-13 (TEAM.md)       */
#define S1_PRI_DEMO  12 /* only when S1_STANDALONE_DEMO == 1   */
#define S1_PRI_COMM  13 /* lowest in the band (NFR-B1-3)       */

#define S1_LED_PIN   19U  /* GP19: link-status LED + 330R to GND */
#define S1_TICK_MS   250U /* LED pattern step                     */

#define COMM_TELEM_PERIOD_DS 5U     /* telemetry every 500 ms (2 Hz)  */
#define COMM_HEARTBEAT_MS    5000U  /* status/heartbeat period         */
#define COMM_WIFI_OFFLINE_MS 30000U /* WiFi down this long -> OFFLINE  */

/* 1 = standalone test mode: subsystem 1 also runs a fake "robot" that
 *     feeds telemetry/events and consumes remote commands, so the whole
 *     WiFi/MQTT path can be shown without the other subsystems.
 * 0 = integration: the real subsystems feed it and the Vehicle
 *     Controller consumes mbf_remote_cmd. */
#ifndef S1_STANDALONE_DEMO
#define S1_STANDALONE_DEMO 0
#endif

/* 0 = B2's telemetry task calls comm_set_motion() every 100 ms
 *     (subsystem2/b2_integration.c).  This is the normal setting.
 * 1 = the comm task reads B2's encoder_get_counts() itself.  Only for a
 *     build without B2's telemetry task: with both, each would
 *     overwrite the other's values. */
#ifndef S1_POLL_B2_ENCODERS
#define S1_POLL_B2_ENCODERS 0
#endif

/* IF-05 (template example, kept): the LED task's ID, so subsystem 4's
 * example monitor can pause/resume it.  Name kept for the team. */
extern ID tid_led;

ER subsystem1_init (void);
ER subsystem1_start (void);
#if APP_TEST
void subsystem1_test (void);
#endif

/* ---------------------------------------------------------------------
 * Team-facing aliases (DEV-1).  Same types, older names.
 * ------------------------------------------------------------------- */
typedef comm_robot_state_t     ROBOT_STATE;
typedef comm_nav_cmd_t         NAV_CMD;
typedef comm_avoid_action_t    AVOID_ACTION;
typedef comm_link_state_t      COMM_STATE;
typedef comm_remote_cmd_type_t REMOTE_CMD_TYPE;
typedef comm_run_mode_t        RUN_MODE;
typedef comm_remote_cmd_t      REMOTE_CMD;
typedef comm_telemetry_t       COMM_TELEMETRY;
typedef comm_stats_t           COMM_STATS;

#endif /* SUBSYSTEM1_H */

/*** end of file ***/

