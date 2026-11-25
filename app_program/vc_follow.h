/* vc_follow.h - line following v4: steering, loss, junctions, wheel split.
 *
 * Pure logic (no RTOS calls) so it can be unit-tested on the host.
 *
 * Sensor layout: the two line sensors are SPREAD, the 18 mm line runs
 * between them.  Centred, both read floor ("white").  Inputs are B3's
 * normalised readings, 0 = floor .. 1000 = tape.
 *
 * Sign convention inside this module: steering + = line to the LEFT of
 * centre (turn left).  vc4_wheels() converts to B2's convention
 * (+ steering_permille = turn right).
 */
#ifndef VC_FOLLOW_H
#define VC_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

/* ---- thresholds on normalised readings (0..1000) ---------------------- */
/* Both below this = line between the sensors.  B3 now tracks each
 * sensor's floor, so bare mat reads 0 and anything above ~60 is the
 * sensor reaching the tape edge: steer on it at once (was 300, which
 * threw away the early edge signal, so the car ran straight and then
 * corrected late). */
#ifndef VC4_WHITE
#define VC4_WHITE           60
#endif
#define VC4_BLACK           700   /* at/above = line under that sensor     */
/* Line counts as having gone out past a sensor (exit side) only if that
 * sensor was fully on the tape.  A partial touch that fades means the
 * line came back toward the centre: arcing toward it then made the car
 * weave (simulated). */
#ifndef VC4_EXIT_MIN
#define VC4_EXIT_MIN        850
#endif
/* Sustained partial left tape can be a curve even without full coverage.
 * Require 200 ms with the right sensor on floor to reject short grazes. */
#define VC4_LEFT_CURVE_MIN  600u
#define VC4_LEFT_CURVE_MS   200u
/* ...and only if it was that fully on the tape at most this long before
 * both sensors saw floor (else the line faded back to the centre). */
#ifndef VC4_EXIT_FRESH_MS
#define VC4_EXIT_FRESH_MS   200u
#endif
/* The left bend in the latest run faded from full tape to white over more
 * than 200 ms. Keep its exit direction briefly longer; right exits retain
 * the existing straight-graze behavior. */
#ifndef VC4_LEFT_EXIT_FRESH_MS
#define VC4_LEFT_EXIT_FRESH_MS 400u
#endif
/* Junction thresholds, lowered 24 Nov from 600/1300 (keep 450/1100):
 * an off-centre or angled crossing half-covers one sensor (~300..500).
 * Replaying the day's runs: 5 crossings found before, 7 now, the same 7
 * at 300; one line under one sensor (a curve) never reaches both. */
#ifndef VC4_SAT
#define VC4_SAT             400   /* both sensors need usable tape contrast */
#endif
#ifndef VC4_JCT_SUM_MIN
#define VC4_JCT_SUM_MIN     1000  /* nL + nR on a crossing tape            */
#endif
#ifndef VC4_JCT_SENSOR_MIN
#define VC4_JCT_SENSOR_MIN  400   /* each sensor must independently see tape */
#endif
#ifndef VC4_JCT_KEEP_MIN
#define VC4_JCT_KEEP_MIN    300   /* hysteresis through a fading tape edge */
#endif
#ifndef VC4_JCT_KEEP_SUM
#define VC4_JCT_KEEP_SUM    800
#endif

/* ---- steering ---------------------------------------------------------- */
#define VC4_STEER_MAX       1000
#define VC4_FILT_DIV        4     /* per 10 ms: filt += (target-filt)/4    */
#define VC4_SIDE_MIN        150   /* |error| that records the line's side  */
/* Below this |error| the response is squared (calm on the straight),
 * above it linear. Attenuate small edge noise without discarding it;
 * strong curve entries retain the existing gain and wheel split. */
#ifndef VC4_SOFT_BAND
#define VC4_SOFT_BAND       200
#endif
/* Proportional part, percent of the error (the curve memory adds to it). */
/* Damping, percent of the error's change per 20 ms: steers against the
 * line's movement so a correction stops before it overshoots. */
#ifndef VC4_D_PCT
#define VC4_D_PCT           600
#endif
#define VC4_D_MAX           250   /* sensor edges must not reverse a curve */
#ifndef VC4_D_EXIT_PCT
#define VC4_D_EXIT_PCT      50    /* share of it after a possible exit     */
#endif
#ifndef VC4_D_SMOOTH
#define VC4_D_SMOOTH        3     /* damping input: move 1/n per tick      */
#endif
#ifndef VC4_P_PCT
#define VC4_P_PCT           120
#endif
#define VC4_COAST_MS        40u   /* black exit: coast before the arc      */
#define VC4_FLIP_COAST_MS   200u  /* exit on the other side from last one  */
#define VC4_RECOVER_START   500   /* first recovery arc, permille          */
#define VC4_RECOVER_RAMP    40    /* + per 10 ms                           */
#define VC4_RECOVER_MAX     1000

/* ---- curve memory ------------------------------------------------------
 * A steady turn learned from which sensor keeps meeting the line.  On a
 * constant curve it settles at the turn the curve needs, so the car arcs
 * smoothly and the sensors only trim it (instead of turn / straight /
 * turn).  It fades while both sensors see floor, quickly when the line
 * shows up on the other side (S-bend), and is cleared at a crossing. */
#ifndef VC4_BIAS_MAX
#define VC4_BIAS_MAX        800
#endif
#ifndef VC4_BIAS_GAIN_DIV
#define VC4_BIAS_GAIN_DIV   300   /* per 10 ms: bias += error / 300        */
#endif
#ifndef VC4_BIAS_DECAY_DIV
#define VC4_BIAS_DECAY_DIV  200   /* per 10 ms on floor: bias -= bias/200  */
#endif
#ifndef VC4_BIAS_FLIP_DIV
#define VC4_BIAS_FLIP_DIV   12    /* per 10 ms, line on the other side     */
#endif

/* ---- loss ------------------------------------------------------------- */
#define VC4_LOST_MS         200u  /* both white this long after a black exit */
#define VC4_EXIT_LOST_UM    60000u /* ...and this much travel               */

/* ---- junction classifier ---------------------------------------------- */
/* Junction = BOTH IR sensors on the tape (each >= VC4_JCT_SENSOR_MIN,
 * kept while each >= VC4_JCT_KEEP_MIN) together for at least this much
 * travel.  5 mm (was 10 mm, which also needed nL + nR >= 1000): angled or
 * off-centre crossings left the sensors overlapping for less than 10 mm. */
#ifndef VC4_JCT_MIN_UM
#define VC4_JCT_MIN_UM      5000u
#endif
#define VC4_JCT_CONFIRM_UM  12000u /* clear travel after the crossbar     */
/* Crossing tape (both sensors on tape): steer straight and ignore which
 * sensor touched it first/last, until both are back on floor or this far
 * past the tape.  Otherwise an unmarked cross reads as "line left on one
 * side" and the car arcs off and searches. */
#define VC4_CROSS_HOLD_UM   40000u

/* ---- speeds (mm/s) and wheel split ------------------------------------ */
#define VC4_CRUISE_MM_S     120
#define VC4_SLOW_MM_S       100   /* obstacle ahead, after a bypass        */
/* (VC4_BARCODE_MM_S removed: barcodes are read at the following speed.) */
/* Full steering error changes each wheel by 85 mm/s before the inner-wheel
 * limit. Stronger split lets the robot follow tighter curves at low speed. */
#ifndef VC4_SPLIT_FULL_X10
#define VC4_SPLIT_FULL_X10  850
#endif
#ifndef VC4_CURVE_SLOW_PCT
#define VC4_CURVE_SLOW_PCT  50    /* base speed cut at full error          */
#endif
/* Tightest turn: the inner wheel may slow to this percentage of the
 * (curve-reduced) base speed. At full error this cap, not the split, sets
 * the turn radius. */
#ifndef VC4_INNER_MIN_PCT
#define VC4_INNER_MIN_PCT   0     /* was 20: inner >= 1/5 of base     */
#endif
/* Steering above this would ask the inner wheel to crawl under B2's
 * usable 150 cps (and the floor rule below would then speed BOTH wheels up
 * to keep the radius). Stop the inner wheel instead and pivot on it at the
 * outer wheel's computed speed. 800 = the old full-error split, so the
 * pivot only adds steering beyond what the follower could ask for before. */
#ifndef VC4_PIVOT_STEER
#define VC4_PIVOT_STEER     800
#endif
/* Wheel travel per encoder count on this car (pi x 65 mm / ~2420). */
#define VC4_UM_PER_COUNT    84u

typedef enum {
    VC4_JCT_NONE = 0,
    VC4_JCT_CROSS,
    VC4_JCT_SIDE_L,
    VC4_JCT_SIDE_R
} vc4_jct_t;

typedef struct {
    /* steering */
    int32_t  filt;
    int32_t  bias;        /* curve memory, + = left                       */
    int32_t  prev_raw;    /* last tick's error, for damping               */
    int8_t   last_side;   /* -1 right / +1 left: last side with an error  */
    int8_t   exit_side;   /* side the line went black under, 0 = between */
    uint32_t exit_age_ms; /* since a sensor was last fully on the tape    */
    uint32_t left_curve_ms; /* continuous strong left-only tape */
    int8_t   prev_exit;
    bool     was_white;
    int32_t  recover;
    uint32_t coast_ms;
    bool     cross_hold;  /* passing a crossing tape                      */
    uint32_t cross_um;    /* odometer when both sensors last saw tape     */
    int8_t   cross_side;  /* side alone on the line before the overlap:
                             +1 left, -1 right, 0 none (came from white) */
    /* loss */
    uint32_t white_ms;
    uint32_t white_start_um;
    bool     lost;
    /* junction */
    uint8_t  jct_state;   /* 0 idle, 1 candidate, 2 confirm */
    bool     sat_l, sat_r;
    uint32_t jct_mark_um;
    uint32_t jct_width_um;
} vc4_track_t;

void     vc4_reset(vc4_track_t *t);
/* One controller tick.  nl/nr = robot's left/right sensor (0..1000),
 * dt_ms = time since the last tick, travel_um = odometer (wraps).
 * Returns the steering error, -1000..+1000, + = line left. */
int32_t  vc4_steer(vc4_track_t *t, uint32_t nl, uint32_t nr, uint32_t dt_ms,
                   uint32_t travel_um);
/* True while the line counts as found (after vc4_steer this tick). */
bool     vc4_on_line(const vc4_track_t *t);
/* Travel since both sensors went white (to back up before a search). */
uint32_t vc4_white_um(const vc4_track_t *t, uint32_t travel_um);
/* Either sensor on the tape. */
bool     vc4_black(uint32_t nl, uint32_t nr);
/* Junction classifier step; returns a class once per junction. */
vc4_jct_t vc4_junction(vc4_track_t *t, uint32_t nl, uint32_t nr,
                       uint32_t travel_um);
void     vc4_junction_reset(vc4_track_t *t);

/* Wheel speeds for following at base_mm_s with steering error s (+ = line
 * left).  slow_mm_s is the speed below which the split shrinks with the
 * base.  Outputs B2's DRIVE fields: mean speed in counts/s and steering
 * permille (+ = right wheel slower = turn right). */
void     vc4_wheels(int32_t s, int32_t base_mm_s, int32_t slow_mm_s,
                    int32_t *speed_cps, int32_t *steering_permille);
int32_t  vc4_mm_s_to_cps(int32_t mm_s);

#endif
