#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "subsystem1/subsystem1.h"
#include "subsystem2/motion_command.h"
#include <stdbool.h>
#include "vc_follow.h"

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
#define VC_CURVE_MIN_PERMILLE 500 /* 50 % at full steer, as BlackberryDealer */
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
/* Spread layout, barcode beside the line: the line sensor on the barcode
 * side passes over the bars, and each bar looked like the line (the car
 * steered toward the barcode, and bar + line looked like a junction).
 * While the barcode sensor is on a strip, a line sensor that turns black
 * VC_BAR_EDGES times (the other sensor not at all) is reading bars and is
 * ignored until the strip is passed; no junction is taken on a strip. */
#define VC_BAR_EDGES 2u
#define VC_BAR_STRIP_EDGES 3u /* GP28 changes before a strip counts as bars
                               * (a junction tape gives one black run) */
/* Barcode turns (spread layout) stop this many degrees short, then keep
 * turning the same way until a line sensor finds the line.  B2's turn is
 * not exact (a U-turn stopped far short and the car drove off straight):
 * both sensors on floor after a turn is NOT "centred". */
#define VC_TURN_SHORT_DEG 20
/* Spread layout steering (BlackberryDealer/INF2004_Project's law, own
 * code): target = e * |e| / 1000 scaled to B3_LINE_STEER_MAX, e = right -
 * left (normalised).  Small errors give almost no steering (smooth
 * straights), large ones strong steering; no deadband step. */
#define VC_STRADDLE_QUADRATIC 1
/* Spread layout, line lost: rotate ON THE SPOT (B2 turns of
 * VC_SPIN_STEP_DEG) toward the side the line was last seen, stopping as
 * soon as a sensor sees it; widening sweeps: 90 deg one way, back 180,
 * then 270...  (LordPercivalXII/INF2004_Project searches in place so the
 * car stays over the line; the old pivot around one wheel drove it away
 * from the corner.)  Gives up (stops, stays armed) after
 * VC_LINE_SEARCH_MS. */
#define VC_SPIN_STEP_DEG 15
/* 4 steps = 60 deg each side (BlackberryDealer sweeps +-60): 60 one way,
 * 120 back, 60 back to the start heading, then stop.  Wider sweeps turned
 * the car round and it followed the track backwards. */
#define VC_SPIN_FIRST_STEPS 4u
#define VC_SPIN_SPEED_CPS 400
#define VC_SPIN_STEP_TIMEOUT_MS 1500u
/* Junction, spread layout: both sensors >= this at once for
 * VC_STRADDLE_JCT_BOTH_TICKS ticks is a crossing for certain (the 18 mm
 * line cannot cover both spread sensors), steady or not. */
#define VC_STRADDLE_JCT_BOTH 700
#define VC_STRADDLE_JCT_BOTH_TICKS 2u
/* Spread layout wheel speeds while following (BlackberryDealer's motion
 * split): the inner wheel may slow to 1/VC_STRADDLE_MAX_RATIO of the outer
 * (5:1, turn radius ~10 cm) and to VC_STRADDLE_MIN_INNER_CPS (B2's own
 * move minimum, "move_min_cps=150" in its boot log).  The old floor of
 * 300 cps with outer <= 3x capped turns at ~500-565 permille (radius
 * ~13 cm): in curves the line outran the car ("goes straight a bit"). */
#define VC_STRADDLE_MIN_INNER_CPS 150
#define VC_STRADDLE_MAX_RATIO 5
/* Line leaves a sensor while the car was ALREADY steering at least this
 * hard toward that sensor: the curve is tighter than the turn, so keep
 * turning (no coast, arc from the current steering).  Smaller steering:
 * the usual coast (the line most likely went back between the sensors). */
#define VC_STRADDLE_BEND_STEER 450
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
/* Bend hold, after SITProjects/INF2004 line_track.c (MIT; same layout:
 * two sensors straddling the tape, both white when centred).  On a bend the
 * line keeps moving toward one sensor, so dropping to zero steering as soon
 * as that sensor clears made the car straighten, let the line run out past
 * the sensor and only then correct (the "go out, then come back" turns).
 * A bend is: the same sensor touching again within VC_BEND_TOUCH_UM of the
 * start of its last touch (their LINE_CURVE_TOUCH_MM 80), or one touch held
 * for VC_BEND_HELD_UM.  The other sensor touching ends it.  In a bend, while
 * both sensors read white, keep steering toward the bend: from
 * VC_BEND_HOLD_PERMILLE of the touch's peak steering (at least
 * VC_BEND_SEEK_MIN) growing to VC_BEND_SEEK_MAX over VC_BEND_RAMP_UM of
 * white travel (their LINE_SEEK_RAMP_MM 20), so the turn never pauses.
 * A centred line soon reaches the other sensor and ends the bend; after
 * VC_BEND_LOST_UM of white (their LINE_LOST_MM 250) the normal outside /
 * search logic takes over.  Touch steering itself is unchanged. */
#ifndef VC_BEND_HOLD
#define VC_BEND_HOLD 1
#endif
#define VC_BEND_TOUCH_UM 80000u
#define VC_BEND_HELD_UM 30000u
#define VC_BEND_HOLD_PERMILLE 700
#define VC_BEND_SEEK_MIN 250
#define VC_BEND_SEEK_MAX 600
#define VC_BEND_RAMP_UM 20000u
#define VC_BEND_LOST_UM 250000u
/* The other sensor touching inside a bend steers back at half strength and
 * keeps the bend, unless it stays black for VC_BEND_COUNTER_UM (their
 * LINE_COUNTER_MM 100 / LINE_COUNTER_END_MM 40). */
#define VC_BEND_COUNTER_UM 60000u
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


/* ---- Line following v4 (vc_follow.c) -------------------------------------
 * 1 (default): the v4 controller runs following, line search, junction
 * turns and the barcode slow re-read; the older spread-layout code below
 * is not used.  0: the previous controller.
 *
 * Following: smoothed steering (no deadband, no slew limiter), coast then
 * a growing arc after the line leaves a sensor, curve slow-down to 50 %,
 * inner wheel >= 1/3 of the base speed.  Lost = both sensors white for
 * 200 ms AND 60 mm after the line went black under one of them (centred,
 * both white is never lost).
 * Line search: spin 60 deg toward the side the line was last seen, 120
 * back, 60 again; found = either sensor on tape; 5 s -> the run stops.
 * Junction: nL + nR >= 1400 for >= 10 mm, classified 30 mm later; both
 * sensors saturated = cross-junction.  A barcode turn waits for a
 * cross-junction, then: stop, forward 60 mm (wheels on the crossing),
 * turn 90 / 180 (U-turn to the left), follow.
 * Barcode: on the first bars (>= 3 GP28 edges) the car stops, reverses
 * past the start of the strip and reads it at 60 mm/s; a failed pass is
 * retried twice from its own start.
 */
#ifndef VC_FOLLOW_V4
#define VC_FOLLOW_V4 1
#endif
/* Lost line: first back up over the stretch driven on white (the sensors
 * sit only 6 cm ahead of the axle, so a spin from where the car stopped
 * cannot reach a line it overshot), then sweep 90 to the side the line
 * left on, 180 back, 90 to centre. */
#define VC4_SEARCH_BACK_EXTRA_MM 20
#define VC4_SEARCH_BACK_MAX_MM  150
#define VC4_SEARCH_SWEEP_DEG 90
#define VC4_SEARCH_CONFIRM_MS 60u /* sustained tape during the sweep     */
#define VC4_SEARCH_ARC_MS 800u    /* gentle curve recovery before backup */
#define VC4_SEARCH_ARC_MM_S 70
#define VC4_SEARCH_MS 9000u
#define VC4_TURN_MM_S 150         /* spin turns (search, junction)       */
#define VC4_MOVE_MM_S 150         /* straight moves (junction advance)   */
#define VC4_REVERSE_MM_S 100
#define VC4_BAR_REVERSE_MM_S 60   /* controlled barcode rewind           */
#define VC4_BAR_READ_MM_S 60      /* retry target after settling         */
#define VC4_JCT_ADVANCE_CM 6.0f   /* line sensors to axle                */
/* Floor trial: left junction turns overshoot at 90 degrees / 150 mm/s.
 * Shorten only that command and approach its endpoint more slowly. */
#define VC4_JCT_LEFT_TURN_DEG 80
#define VC4_JCT_LEFT_TURN_MM_S 90
#define VC4_MOTION_TIMEOUT_MS 10000u
/* Barcode slow re-read. */
#define VC4_BAR_STRIP_EDGES 3u    /* GP28 changes: bars, not a tape      */
#define VC4_BAR_LEAD_UM 40000u    /* reverse this far before the 1st bar */
#define VC4_BAR_BACK_EXTRA_UM 70000u
#define VC4_BAR_RETRY_EXTRA_UM 10000u
#define VC4_BAR_REWIND_MAX_UM 600000u
#define VC4_BAR_WAIT_MS 600u      /* stand still: B3 ends a scan after 500 ms quiet */
#define VC4_BAR_READ_UM 650000u   /* give up a pass after this travel    */
#define VC4_BAR_READ_MS 15000u
#define VC4_BAR_RETRIES 2u
#define VC4_BAR_REARM_UM 60000u
/* After an obstacle bypass: slow speed and no junctions for 150 mm. */
#define VC4_POST_AVOID_UM 150000u
/* Lift-stop: car picked up while running -> stop. */
#define VC4_LIFT_IR_RAW 3500u     /* both line sensors (no floor below)  */
#define VC4_LIFT_IR_MS 300u
#define VC4_LIFT_TILT_MS2 4.9f    /* |az| below: tilted > 60 deg         */
#define VC4_LIFT_DG_MS2 2.94f     /* ||a| - 1 g| above: lifted / dropped */
#define VC4_LIFT_IMU_MS 60u
/* Ramp: while the car is pitched (B4 hump) and 480 ms / 150 mm after,
 * the obstacle stop uses the shorter ramp distance (B5). */
#define VC4_RAMP_HOLD_MS 480u
#define VC4_RAMP_CLEAR_UM 150000u
#define VC4_RAMP_PITCH_DEG 2.0f

typedef struct {
    uint32_t now_ms, safety_generation;
    uint32_t line_age_ms;
    bool clearance, line_valid, imu_valid, hump, hazard, motion_valid;
    bool barcode_active; /* B3 barcode sensor is crossing bars */
    bool obstacle_active, obstacle_fault;
    bool approach_slow;
    UH left, centre;
    UH barcode_rejects; /* B3 rejected-scan counter (only increases) */
    UB right; /* barcode sensor GP28 level (0/1), beside the line */
    UH raw26, raw27; /* raw ADC of the line sensors (lift-stop) */
    bool lifted_imu; /* B4 live sample says lifted/tilted (debounced) */
    float pitch_deg; /* B4 live pitch */
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
    /* Bend hold (VC_BEND_HOLD). */
    INT bend_side;         /* -1/+1: in a bend that way, else 0 */
    INT bend_touch_side;   /* side of the last single-sensor touch */
    bool bend_touching;    /* that touch is still on */
    INT bend_peak;         /* hardest steering toward it during the touch */
    uint32_t bend_touch_um, bend_white_um;
    bool bend_counter;     /* the other sensor is touching inside the bend */
    INT bend_touch_now;
    uint32_t bend_counter_um;
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
    UB strip_edges;        /* GP28 changes in this episode (bars: many) */
    uint32_t strip_start_um, strip_last_um;
    UH seen_rejects;
    uint32_t rescan_id, rescan_ms, rescan_back_um;
    uint32_t travel_um; /* odometer, wraps */
    /* Junction evidence, for the run log and the spread layout. */
    bool tape_now, jct_waiting_logged;
    bool jct_left_seen, jct_right_seen;
    bool jct_candidate_steady;
    /* Barcode bars under a line sensor (see VC_BAR_EDGES). */
    bool bars_active, bar_prev_l, bar_prev_r;
    UB bar_edges_l, bar_edges_r;
    INT bar_ignore; /* -1 left / +1 right sensor ignored, 0 none */
    /* In-place line search (spread layout). */
    uint32_t spin_id, spin_ms;
    INT spin_dir;
    UINT spin_steps, spin_target, spin_sweep;
    bool spin_done;
    UINT jct_both_ticks;
    uint32_t jct_block_um; /* travel at the last unsteady moment */
    uint32_t jct_left_um, jct_right_um;
    UB navigation, pending_navigation;
    /* Line following v4 (VC_FOLLOW_V4). */
    vc4_track_t trk;
    UB v4_phase;           /* V4_FOLLOW, V4_SEARCH, V4_CROSS, V4_BARCODE */
    UB v4_sub;
    uint32_t v4_last_ms, v4_phase_ms, v4_motion_id;
    bool v4_time_valid;
    INT v4_search_first;   /* +1 left / -1 right */
    INT v4_search_back_mm; /* back-up before the sweep                */
    float v4_jct_advance_cm;
    uint32_t v4_line_seen_ms;
    UB bar_phase, bar_tries;
    INT bar_speed_cps;     /* fixed target for this pass, across edge gaps */
    uint32_t bar_mark_um, bar_back_um, bar_ms, bar_clear_um;
    bool bar_clear_known;
    bool bar_wait_clear;   /* decoded/skipped strip must fully leave GP28 */
    UH bar_rejects;
    bool post_avoid_slow;
    uint32_t post_avoid_um;
    uint32_t lift_ir_ms;
    bool on_ramp;
    uint32_t ramp_ms, ramp_um;
} VEHICLE_CONTROLLER;

void vehicle_controller_init(VEHICLE_CONTROLLER *controller);
void vehicle_controller_command(VEHICLE_CONTROLLER *controller,
                                const REMOTE_CMD *command);
void vehicle_controller_step(VEHICLE_CONTROLLER *controller,
                             const VC_INPUT *input, UB navigation);
ER vehicle_controller_start(void);

#endif
