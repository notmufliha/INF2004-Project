#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "subsystem1/subsystem1.h"
#include "subsystem2/motion_command.h"
#include <stdbool.h>

/* Temporary steering diagnosis: set to 0 and rebuild to remove logging. */
#ifndef VC_STEERING_DIAGNOSTICS
#define VC_STEERING_DIAGNOSTICS 1
#endif
#define VC_STEERING_LOG_MS 250u

#define VC_PERIOD_MS 20u
#define VC_LINE_MAX_AGE_MS 100u
#define VC_IMU_MAX_AGE_MS 1500u /* B4 publishes a quiet snapshot every second. */
#define VC_MOTION_MAX_AGE_MS 100u
#define VC_RECOVERY_MS 3000u
#define VC_LINE_SEARCH_SWITCH_MS 600u /* first sweep; each later sweep is 600 ms longer */
#define VC_LINE_SEARCH_MS 8000u /* then hold, still armed, until the line returns */
#define VC_LINE_SEARCH_STEER 250
/* Line just lost: first turn hard toward the side it was last seen (what
 * Pololu's saturated position + PD does) for this long, then sweep. */
#define VC_LINE_REACQUIRE_MS 1500u
#define VC_JUNCTION_CONFIRM_SAMPLES 3u
/* Curve speed: at full steer the base speed drops to VC_CURVE_MIN_PERMILLE.
 * The slowdown follows the hardest recent steering and relaxes by
 * VC_CURVE_DECAY_PER_TICK each 20 ms tick, so the car stays slow through
 * the whole curve instead of speeding up between corrections. */
#define VC_CURVE_MIN_PERMILLE 600
#define VC_CURVE_DECAY_PER_TICK 40
/* Smoothness: steering may change by at most this much per 20 ms tick
 * (0 to full 800 in about 0.1 s), and speed may rise by at most
 * VC_SPEED_RISE_PER_TICK counts/s per tick.  Slowing down is immediate. */
#define VC_STEER_SLEW_PER_TICK 150
/* Easing off a correction: full 800 to straight in 3 ticks (60 ms). */
#define VC_STEER_RELEASE_PER_TICK 300
/* Following never pivots: on a hard turn the inner wheel stays at
 * VC_MIN_WHEEL_CPS and the outer runs at least this many times faster. */
#define VC_FOLLOW_MIN_OUTER_RATIO 3
#define VC_SPEED_RISE_PER_TICK 100
/* CALIBRATE: slowest wheel speed (counts/s) the loaded motors hold without
 * stalling; about 25 mm/s.  A curve's inner wheel is never asked to run
 * between 0 and this: a little below it is raised to it, far below it is
 * set to 0 so the car pivots around that wheel (a wheel commanded to 0 is
 * never reported as stalled). */
#define VC_MIN_WHEEL_CPS 300
#define VC_JUNCTION_RELEASE_SAMPLES 3u
/* Clear-path mode, line lost on a curve: keep the last following steer
 * (if at least MIN_STEER) for this long before driving straight. */
#define VC_CURVE_REACQUIRE_MS 500u
#define VC_CURVE_REACQUIRE_MIN_STEER 200
/* Junction = a crossing tape (18 mm wide, perpendicular to the line).
 * Both line sensors sit on the 18 mm line whenever the car is centred, so
 * B3's JUNCTION flag alone is set all the time and a barcode's turn ran at
 * once.  The barcode sensor (GP28) beside the line reads floor except over
 * barcode bars (<= ~9 mm) and a crossing tape (18 mm): a junction is GP28
 * black for at least VC_JUNCTION_MIN_UM of travel with both line sensors
 * black and the car not in a hard curve. */
#define VC_JUNCTION_MIN_UM 13000u
/* GP28 level held for this much travel is the floor (learned at run time:
 * the sensor's polarity is not fixed by the code). */
#define VC_FLOOR_LEARN_UM 80000u
/* Wheel travel per encoder count: pi * 65 mm / ~2420 counts. */
#define VC_UM_PER_COUNT 84u
/* Close layout: a GP28 tape is not a junction while the car is in a hard
 * curve (the line itself can pass under GP28 there).  Was 300, which the
 * car's normal corrections on a straight exceed: barcode turns never ran. */
#define VC_JUNCTION_MAX_CURVE_STEER 600
/* Spread layout junction: BOTH line sensors touched black (>= 201) within
 * the last VC_STRADDLE_JCT_WINDOW_UM of travel, not necessarily at the
 * same moment (a tape crossed at an angle, and GP26 reads tape weakly:
 * "both >= 1400 together for 10 mm" missed real junctions); or the GP28
 * tape with either line sensor black in that window (side branch on
 * GP28's side).  Normal following touches one side, then the other only
 * centimetres later. */
#define VC_STRADDLE_JCT_WINDOW_UM 25000u
/* ...but only while the car is following steadily.  A car crooked to the
 * line (just after a barcode re-read reverse, a turn, a line search or a
 * pivot) drives ACROSS it at a steep angle: left then right sensor touch
 * within 25 mm, exactly like a junction tape.  That made the car turn left
 * near the barcode (2026-11-18).  A junction candidate counts only when it
 * starts at least VC_JCT_SETTLE_UM after the last such event (as
 * BlackberryDealer/INF2004_Project classifies junctions only in steady
 * line tracking). */
#define VC_JCT_SETTLE_UM 80000u
/* While crossing barcode bars (spread layout) steering is held within
 * +-VC_BARCODE_MAX_STEER and never pivots: swinging over the strip
 * changes the barcode sensor's speed and spoils the bar widths. */
#define VC_BARCODE_MAX_STEER 400
/* Motor stall: up to this many automatic retries (stop, wait
 * VC_STALL_RETRY_MS, drive again) before the car stays stopped for START.
 * The count resets after VC_STALL_RETRY_RESET_MS without a stall. */
#define VC_STALL_AUTO_RETRIES 2u
#define VC_STALL_RETRY_MS 1000u
#define VC_STALL_RETRY_RESET_MS 10000u
/* After START or a stall retry, B5 needs a few of its own ticks to re-arm
 * from the acknowledged fault; its mission FAULT is tolerated (motors
 * stopped) for this long instead of safe-stopping again. */
#define VC_FAULT_ACK_GRACE_MS 600u
/* Barcode turn at a junction: the sensors find the junction ahead of the
 * wheels.  Turning there swung the sensors into the floor beside the new
 * line, so the car first keeps following the line for this distance (its
 * wheels then stand on the junction) and only then turns.  MEASURE: the
 * distance from the line sensors to the wheel axle, minus ~10 mm (the
 * junction is confirmed a little after the sensors reach the tape). */
#ifndef VC_JUNCTION_APPROACH_MM
#define VC_JUNCTION_APPROACH_MM 50u
#endif
/* STOP barcode: time standing at the crossing before going straight on. */
#define VC_NAV_STOP_MS 3000u
/* Line-sensor layout.
 * 1 (default): sensors SPREAD OUT, the 18 mm line runs between them.
 *   Centred, both read floor (white): that is "on the line", not lost.
 *   One sensor on black: steer toward it.  Both black: a crossing tape.
 * 0: sensors close together, both on the line when centred.
 * Changeable at run time through VEHICLE_CONTROLLER.line_straddle.
 * Auto-detect: with 1, both sensors on black for VC_LAYOUT_CLOSE_UM of
 * travel (a crossing tape is only 18 mm) means they are close together:
 * the controller switches to the close layout until reboot. */
#ifndef VC_LINE_STRADDLE
#define VC_LINE_STRADDLE 1 /* spread; switches itself if together */
#endif
/* Spread layout: steering reaches B3_LINE_STEER_MAX when one sensor reads
 * this much black (0..1000).  Lower = harder corrections (curves missed?),
 * higher = gentler (weaving on straights?). */
#define VC_STRADDLE_FULL_SCALE B3_LINE_STEER_ERROR_FULL_SCALE
#define VC_LAYOUT_CLOSE_UM 100000u /* 10 cm; a crossing tape is 18 mm */
/* A sensor this black had the line under it.  When it then reads floor
 * again the line is either back between the sensors (usual: the car's
 * correction worked) or went OUT past it (sharp curve).  Turning hard
 * toward that side at once pushed a centred line across to the other
 * sensor, then back: the zig-zag of the 2026-11-18 run logs.  So, as in
 * BlackberryDealer/INF2004_Project: first coast straight for COAST_MS
 * (FLIP_COAST_MS when the exit is on the other side from the last one),
 * then turn toward the exit side with a gentle arc, RECOVER_START permille
 * growing RECOVER_RAMP per 20 ms tick up to OUTSIDE_STEER.  Not found
 * after OUTSIDE_MS: the normal line search. */
/* 450, not 700: the right sensor (GP26) reads the tape at only ~450-750
 * normalised, so its exits never counted and the car drove straight on
 * after right bends (2026-11-18 test). */
#define VC_STRADDLE_DEEP 450
#define VC_STRADDLE_COAST_MS 120u
#define VC_STRADDLE_FLIP_COAST_MS 400u
#define VC_STRADDLE_RECOVER_START 200
#define VC_STRADDLE_RECOVER_RAMP 40
#define VC_STRADDLE_OUTSIDE_STEER B3_LINE_STEER_MAX
#define VC_STRADDLE_OUTSIDE_MS 1500u
/* Curve: the line leaves the SAME sensor again within VC_STRADDLE_CURVE_MS
 * of the last exit.  The car is in a bend, so no coasting: the arc toward
 * that side starts at VC_STRADDLE_CURVE_ARC at once and keeps the car
 * turning with the bend instead of drifting out between touches. */
#define VC_STRADDLE_CURVE_MS 1000u
#define VC_STRADDLE_CURVE_ARC 350
/* Double turn (the hump): the line stayed under one sensor for 1 s while
 * steering was capped at ~500-565 (wheel-speed limits keep the inner wheel
 * at its minimum), i.e. the car could not turn as tight as the bend.  Only
 * then (one sensor black for >= VC_STRADDLE_HELD_PIVOT_MS, steering >= 700)
 * may the inner wheel stop.  Normal touches last 300-450 ms; a 250 ms
 * version of this rule (commit 3e849eb) pivoted on every touch and made
 * the car zig-zag from full left to full right. */
#define VC_STRADDLE_HELD_PIVOT_MS 700u
#define VC_STRADDLE_PIVOT_STEER 700
/* Neither sensor has touched the line for this long: assume it is really
 * lost (end of line, or off the course) and run the normal line search. */
#define VC_STRADDLE_BLIND_MS 3000u
/* Seek: both sensors on floor for VC_STRADDLE_SEEK_MS -> drift gently
 * toward the side that last touched the line (SEEK_START permille, +RAMP
 * per tick, up to SEEK_MAX).  A centred line is then met by the other
 * sensor (a slow, small weave); a line that slipped out past a sensor
 * without a strong reading (car offset after a bend) is found again
 * instead of the car driving straight off the course. */
#define VC_STRADDLE_SEEK_MS 400u
#define VC_STRADDLE_SEEK_START 100
#define VC_STRADDLE_SEEK_RAMP 15
#define VC_STRADDLE_SEEK_MAX 450
/* Barcode re-read: after a rejected scan (B3 beeps twice), stop, reverse
 * past the start of the strip by VC_RESCAN_MARGIN_UM and drive over it
 * again.  At most VC_BARCODE_RESCANS tries per barcode; 0 disables. */
#define VC_BARCODE_RESCANS 1u
#define VC_RESCAN_MARGIN_UM 50000u   /* 5 cm before the first bar */
#define VC_RESCAN_MAX_UM 400000u     /* never reverse more than 40 cm */
/* The strip is over when GP28 has not changed for this much travel; only
 * then does the re-read reverse (a pause on the strip also ends a B3 scan,
 * and reversing then backed up from the middle of the barcode). */
#define VC_STRIP_END_UM 40000u
#define VC_RESCAN_TIMEOUT_MS 8000u
#define VC_RESCAN_FORGET_MS 10000u   /* then a new barcode gets new tries */

typedef struct {
    uint32_t now_ms, safety_generation;
    bool clearance, line_valid, imu_valid, hump, hazard, motion_valid;
    bool barcode_active; /* B3 barcode sensor is crossing bars */
    bool obstacle_active, obstacle_fault;
    bool approach_slow;
    UH left, centre;
    UH barcode_rejects; /* B3 rejected-scan counter (only increases) */
    UB right; /* barcode sensor GP28 level (0/1), beside the line */
    UB line_flags, faults;
    motion_status_snapshot_t motion;
} VC_INPUT;

/* Owned exclusively by the controller task. Exposed for deterministic tests. */
typedef struct {
    ROBOT_STATE state;
    RUN_MODE mode;
    UINT speed_percent;
    bool running, paused, estop, stopped, drive_sent;
    bool obstacle_handoff; /* B5 moved the robot since our last line command. */
    bool line_recovery_active; /* Survives temporary clearance stops. */
    uint32_t last_drive_ms, recovery_ms, navigation_id, navigation_ms;
    INT last_line_search_direction; /* -1 left / +1 right: last line side */
    INT line_pos;   /* last on-line position 0..1000 (Pololu _lastPosition) */
    INT last_error; /* previous PD error, for the derivative term */
    INT filtered_error; /* low-passed sensor difference */
    bool error_filter_valid; /* filtered_error holds a following sample */
    INT curve_steer; /* hardest recent steering, decaying; sets curve speed */
    INT last_steer, last_speed; /* last DRIVE sent, for smoothing */
    INT follow_steer; /* last steering sent while following the line */
    uint32_t line_seen_ms; /* last tick the line was detected */
#if VC_STEERING_DIAGNOSTICS
    uint32_t last_steering_log_ms;
#endif
    UINT junction_enter_samples, junction_clear_samples;
    bool junction_latched;
    /* Crossing detection from the barcode sensor (see VC_JUNCTION_MIN_UM). */
    UB gp28_level, floor_level;
    bool floor_known, travel_time_valid;
    uint32_t gp28_run_um, travel_last_ms;
    /* Fault acknowledgement and stall retries. */
    bool ack_grace_pending, hold_active, tolerate_active;
    uint32_t hold_until_ms, tolerate_until_ms, last_stall_ms;
    UINT stall_retries;
    bool slow_mode; /* SLOW barcode: half following speed until NORMAL */
    /* Spread sensor layout (VC_LINE_STRADDLE). */
    bool line_straddle;
    INT straddle_side;     /* -1/+1: only that sensor is on black, else 0 */
    INT straddle_touch;    /* -1/+1: the sensor that last touched alone */
    uint32_t straddle_side_ms; /* when straddle_side last changed */
    bool straddle_deep;    /* ...and it has read >= VC_STRADDLE_DEEP */
    INT straddle_outside;  /* -1/+1: line left past that sensor (maybe) */
    INT straddle_last_exit;  /* side of the previous exit */
    uint32_t straddle_coast_ms, straddle_exit_ms;
    INT straddle_arc_start;
    /* Junction approach before a barcode turn. */
    bool approach_active;
    uint32_t approach_um, approach_start_um;
    bool straddle_lost;    /* lost until a sensor sees black again */
    bool straddle_both;    /* both sensors on black since straddle_both_um */
    uint32_t straddle_both_um;
    uint32_t straddle_outside_ms, straddle_black_ms;
    /* Barcode re-read after a rejected scan. */
    UB rescan_phase;       /* 0 none, 1 stopping, 2 reversing */
    UINT rescan_tries;
    bool rejects_known, rescan_pending;
    bool strip_active, strip_known; /* GP28 strip episode (rescan) */
    UB strip_level;
    uint32_t strip_start_um, strip_last_um;
    UH seen_rejects;
    uint32_t rescan_id, rescan_ms, rescan_back_um;
    uint32_t travel_um; /* odometer, wraps */
    /* Junction evidence, for the run log and the spread layout. */
    bool tape_now, jct_waiting_logged;
    bool jct_left_seen, jct_right_seen;
    bool jct_candidate_steady, pivoting;
    uint32_t jct_block_um; /* travel at the last unsteady moment */
    uint32_t jct_left_um, jct_right_um;
    UB navigation, pending_navigation;
} VEHICLE_CONTROLLER;

void vehicle_controller_init(VEHICLE_CONTROLLER *controller);
void vehicle_controller_command(VEHICLE_CONTROLLER *controller,
                                const REMOTE_CMD *command);
void vehicle_controller_step(VEHICLE_CONTROLLER *controller,
                             const VC_INPUT *input, UB navigation);
ER vehicle_controller_start(void);

#endif
