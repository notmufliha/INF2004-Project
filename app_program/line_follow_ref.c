/* line_follow_ref.c - simple proportional line follower (see the header). */
#include "line_follow_ref.h"

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }
static int32_t isign(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

void lfr_reset(lfr_t *f)
{
    f->smooth = 0;
    f->last_seen = 0;
    f->last_error = 0;
    f->last_side = 0;
    f->last_side_any = 0;
    f->lost_ms = 0u;
    f->centred_ms = 0u;
    f->primed = false;
}

lfr_out_t lfr_step(lfr_t *f, uint32_t nl, uint32_t nr, uint32_t dt_ms)
{
    lfr_out_t out = {0, false, 0, false};
    bool det = nl >= LFR_DETECT || nr >= LFR_DETECT;
    int32_t e = (int32_t)nl - (int32_t)nr;
    int32_t raw, mag, eb, pd;
    bool between;
    uint32_t steps = dt_ms / 10u;

    if (steps == 0u) steps = 1u;
    if (steps > 10u) steps = 10u;   /* a stalled loop must not jump */

#if !LFR_EDGE_LAYOUT
    /* Spread layout: both on floor = line between the sensors = centred,
     * unless a sensor was well onto the tape just before (line left past
     * it): then fall through to the lost / between handling below. */
    if (!det && iabs(f->last_seen) < LFR_SPREAD_EXIT) {
        det = true;
        e = 0;
    }
#else
    if (!det) {
        /* Edge layout: both white = lost; report full error toward the
         * side the line was last on. */
        e = f->last_error > LFR_LOST_SIDE_BAND ? 1000 :
            (f->last_error < -LFR_LOST_SIDE_BAND ? -1000 : 0);
    }
#endif
    out.detected = det;

    /* Memory of where the line was. */
    if (det) {
        f->lost_ms = 0u;
        f->last_seen = e;
        f->last_error = e;
        if (iabs(e) >= LFR_SIDE_SET) {
            f->last_side = (int8_t)isign(e);
            f->last_side_any = f->last_side;
            f->centred_ms = 0u;
        } else if (iabs(e) < LFR_CENTRE) {
            f->centred_ms += dt_ms;
            if (f->centred_ms >= LFR_CENTRE_MS) f->last_side = 0;
        }
    } else if (f->lost_ms < 100000u) {
        f->lost_ms += dt_ms;
    }

    between = !det && f->lost_ms < LFR_BETWEEN_MS;

    /* Gone for good: search toward the last side seen. */
    if (!det && f->last_side_any != 0 && !between) {
        out.search = true;
        out.search_side = f->last_side_any;
        f->primed = false;
        return out;
    }

    if (!det && (between || f->lost_ms < LFR_HOLD_ERR_MS)) {
        raw = between ? f->last_seen / 2 : f->smooth;
    } else {
        raw = e;
    }
    if (!f->primed) {
        f->smooth = raw;
    } else {
        for (uint32_t i = 0u; i < steps; ++i) {
            f->smooth += (raw - f->smooth) * LFR_SMOOTH_PCT / 100;
        }
    }
    f->primed = true;

    mag = iabs(f->smooth);
    eb = mag < LFR_DEADBAND ? 0 : isign(f->smooth) * (mag - LFR_DEADBAND);
    pd = eb * LFR_KP_PCT / 100;
    if (pd > LFR_STEER_LIMIT) pd = LFR_STEER_LIMIT;
    if (pd < -LFR_STEER_LIMIT) pd = -LFR_STEER_LIMIT;

    if (between && iabs(f->last_seen) >= LFR_SIDE_SET) {
        int32_t hold = LFR_BETWEEN_STEER +
            LFR_BETWEEN_STEER * (int32_t)f->lost_ms / (int32_t)LFR_BETWEEN_GROW_MS;
        if (hold > LFR_STEER_LIMIT) hold = LFR_STEER_LIMIT;
        out.steer = isign(f->last_seen) * hold;
    } else {
        out.steer = pd;
    }
    return out;
}
