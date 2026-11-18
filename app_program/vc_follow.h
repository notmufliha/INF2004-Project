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
#define VC4_WHITE           300   /* below on both = both white            */
#define VC4_BLACK           700   /* at/above = line under that sensor     */
#define VC4_SAT             900   /* sensor fully over tape (junction)     */
#define VC4_JCT_SUM_MIN     1400  /* nL + nR on a crossing tape            */

/* ---- steering ---------------------------------------------------------- */
#define VC4_STEER_MAX       1000
#define VC4_FILT_DIV        4     /* per 10 ms: filt += (target-filt)/4    */
#define VC4_SIDE_MIN        150   /* |error| that records the line's side  */
#define VC4_COAST_MS        120u  /* black exit: coast before the arc      */
#define VC4_FLIP_COAST_MS   400u  /* exit on the other side from last one  */
#define VC4_RECOVER_START   200   /* first recovery arc, permille          */
#define VC4_RECOVER_RAMP    20    /* + per 10 ms                           */
#define VC4_RECOVER_MAX     1000

/* ---- loss ------------------------------------------------------------- */
#define VC4_LOST_MS         200u  /* both white this long after a black exit */
#define VC4_EXIT_LOST_UM    60000u /* ...and this much travel               */

/* ---- junction classifier ---------------------------------------------- */
#define VC4_JCT_MIN_UM      10000u /* tape at least this wide             */
#define VC4_JCT_CONFIRM_UM  30000u /* travel past it before classifying   */
/* Crossing tape (both sensors on tape): steer straight and ignore which
 * sensor touched it first/last, until both are back on floor or this far
 * past the tape.  Otherwise an unmarked cross reads as "line left on one
 * side" and the car arcs off and searches. */
#define VC4_CROSS_HOLD_UM   40000u

/* ---- speeds (mm/s) and wheel split ------------------------------------ */
#define VC4_CRUISE_MM_S     120
#define VC4_SLOW_MM_S       100   /* obstacle ahead, after a bypass        */
#define VC4_BARCODE_MM_S    60    /* barcode read pass                     */
/* Full steering error moves each wheel by this much (mm/s x 10): 2/256 of
 * a count per 50 ms window at 324 um/count = 50.6 mm/s. */
#define VC4_SPLIT_FULL_X10  506
#define VC4_CURVE_SLOW_PCT  50    /* base speed cut at full error          */
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
    int8_t   last_side;   /* -1 right / +1 left: last side with an error  */
    int8_t   exit_side;   /* side the line went black under, 0 = between */
    int8_t   prev_exit;
    bool     was_white;
    int32_t  recover;
    uint32_t coast_ms;
    bool     cross_hold;  /* passing a crossing tape                      */
    uint32_t cross_um;    /* odometer when both sensors last saw tape     */
    /* loss */
    uint32_t white_ms;
    uint32_t white_start_um;
    bool     lost;
    /* junction */
    uint8_t  jct_state;   /* 0 idle, 1 candidate, 2 confirm */
    bool     sat_l, sat_r;
    uint32_t jct_mark_um;
} vc4_track_t;

void     vc4_reset(vc4_track_t *t);
/* One controller tick.  nl/nr = robot's left/right sensor (0..1000),
 * dt_ms = time since the last tick, travel_um = odometer (wraps).
 * Returns the steering error, -1000..+1000, + = line left. */
int32_t  vc4_steer(vc4_track_t *t, uint32_t nl, uint32_t nr, uint32_t dt_ms,
                   uint32_t travel_um);
/* True while the line counts as found (after vc4_steer this tick). */
bool     vc4_on_line(const vc4_track_t *t);
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
