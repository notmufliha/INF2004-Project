/* line_follow_ref.h - simple proportional line follower (VC_FOLLOW_REF).
 *
 * Pure logic (no RTOS calls), host-testable.  Per control step:
 *   error  = left - right (normalised 0..1000 each; + = line to the LEFT)
 *   smooth = smooth + 45 % of (error - smooth) per 10 ms
 *   deadband: |smooth| < 100 -> 0, else the band is subtracted
 *   steer  = 0.60 x that, limited to +-900 (+ = turn left)
 * Line between the sensors after it was seen off-centre: keep turning
 * toward that side for up to 400 ms, harder the longer it is missing.
 * Still missing after that: search toward the last side seen.
 *
 * LFR_EDGE_LAYOUT selects the sensor layout:
 *   1: each sensor sits over one EDGE of the line (~18-20 mm apart), so a
 *      centred car reads ~500 on both and "both white" means lost.
 *   0: sensors SPREAD with the line between them (this car as built):
 *      "both white" is centred, unless a sensor had been well onto the
 *      tape just before (the line left past it), which is handled as above.
 */
#ifndef LINE_FOLLOW_REF_H
#define LINE_FOLLOW_REF_H

#include <stdbool.h>
#include <stdint.h>

#ifndef LFR_EDGE_LAYOUT
#define LFR_EDGE_LAYOUT 0
#endif

#define LFR_DETECT 60     /* a sensor at/above this sees the line */
#define LFR_SMOOTH_PCT 45 /* per 10 ms step                       */
#define LFR_DEADBAND 100
#define LFR_KP_PCT 60
#define LFR_STEER_LIMIT 900
#define LFR_SIDE_SET 200         /* |error| that records the line's side */
#define LFR_CENTRE 120           /* |error| below this counts as centred */
#define LFR_CENTRE_MS 100u       /* ...for this long: forget the side     */
#define LFR_HOLD_ERR_MS 80u      /* short dropout: hold the smoothed error */
#define LFR_BETWEEN_MS 400u      /* line between the sensors: keep turning */
#define LFR_BETWEEN_STEER 550    /* ...starting here, growing to the limit */
#define LFR_BETWEEN_GROW_MS 150u /* +100 % of LFR_BETWEEN_STEER per this  */
#define LFR_LOST_SIDE_BAND 50    /* edge layout: lost error sign deadband */
/* Spread layout: both white counts as "the line left past a sensor" only
 * if that sensor was at least this far onto the tape just before. */
#define LFR_SPREAD_EXIT 600

typedef struct
{
    int32_t  smooth;        /* smoothed error                           */
    int32_t  last_seen;     /* last error with the line detected        */
    int32_t  last_error;    /* edge layout: last detected error (sign)  */
    int8_t   last_side;     /* +1 left, -1 right, 0 centred             */
    int8_t   last_side_any; /* last non-zero side, kept for the search  */
    uint32_t lost_ms;       /* time without the line                    */
    uint32_t centred_ms;
    bool     primed;
} lfr_t;

typedef struct
{
    int32_t steer;       /* permille, + = turn left                  */
    bool    search;      /* line lost: start the search              */
    int8_t  search_side; /* +1 left, -1 right                        */
    bool    detected;
} lfr_out_t;

void lfr_reset (lfr_t * f);
/* nl / nr: robot's left / right sensor, normalised 0 (floor) .. 1000. */
lfr_out_t lfr_step (lfr_t * f, uint32_t nl, uint32_t nr, uint32_t dt_ms);

#endif

/*** end of file ***/

