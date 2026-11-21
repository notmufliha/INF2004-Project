/* vc_follow.c - line following v4 (see vc_follow.h). */
#include "vc_follow.h"

static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

void vc4_reset(vc4_track_t *t)
{
    t->filt = 0;
    t->bias = 0;
    t->prev_raw = 0;
    t->last_side = 0;
    t->exit_side = 0;
    t->exit_age_ms = 0u;
    t->prev_exit = 0;
    t->was_white = true;
    t->recover = VC4_RECOVER_START;
    t->coast_ms = 0u;
    t->cross_hold = false;
    t->cross_um = 0u;
    t->white_ms = 0u;
    t->white_start_um = 0u;
    t->lost = false;
    vc4_junction_reset(t);
}

bool vc4_black(uint32_t nl, uint32_t nr)
{
    return nl >= VC4_BLACK || nr >= VC4_BLACK;
}

/* Not both white: a new excursion from "both white" starts with the line
 * between the sensors; once it goes black under exactly one sensor it may
 * leave on that side. */
static void track_exit(vc4_track_t *t, uint32_t nl, uint32_t nr,
                       uint32_t dt_ms)
{
    bool bl = nl >= VC4_EXIT_MIN;
    bool br = nr >= VC4_EXIT_MIN;

    if (t->was_white) {
        if (t->exit_side != 0) t->prev_exit = t->exit_side;
        t->exit_side = 0;
        t->was_white = false;
        t->recover = VC4_RECOVER_START;
        t->coast_ms = 0u;
    }
    if (bl && !br) {
        t->exit_side = 1;
        t->exit_age_ms = 0u;
    } else if (br && !bl) {
        t->exit_side = -1;
        t->exit_age_ms = 0u;
    } else if (t->exit_age_ms < 100000u) {
        t->exit_age_ms += dt_ms;
    }
}

static int32_t clamp_steer(int32_t v, int32_t lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* Fade the curve memory by 1/div (at least 1) per 10 ms step. */
static void bias_fade(vc4_track_t *t, int32_t div, uint32_t steps)
{
    for (uint32_t i = 0u; i < steps && t->bias != 0; ++i) {
        int32_t d = t->bias / div;
        if (d == 0) d = t->bias > 0 ? 1 : -1;
        t->bias -= d;
    }
}

static uint32_t coast_len(const vc4_track_t *t)
{
    return (t->prev_exit != 0 && t->exit_side == -t->prev_exit) ?
        VC4_FLIP_COAST_MS : VC4_COAST_MS;
}

int32_t vc4_steer(vc4_track_t *t, uint32_t nl, uint32_t nr, uint32_t dt_ms,
                  uint32_t travel_um)
{
    int32_t raw = (int32_t)nl - (int32_t)nr;
    bool white = nl < VC4_WHITE && nr < VC4_WHITE;
    int32_t target = 0;
    uint32_t steps = dt_ms / 10u;

    if (steps == 0u) steps = 1u;
    if (steps > 10u) steps = 10u;   /* a stall in the loop: do not jump */

    /* Crossing tape: forget the side the first sensor touched it on and
     * drive straight across; the line carries on between the sensors. */
    if (nl >= VC4_JCT_SENSOR_MIN && nr >= VC4_JCT_SENSOR_MIN &&
        nl + nr >= VC4_JCT_SUM_MIN) {
        t->cross_hold = true;
        t->cross_um = travel_um;
        t->bias = 0;          /* the arm, not a curve */
        t->exit_side = 0;
        t->recover = VC4_RECOVER_START;
        t->coast_ms = 0u;
    }
    if (t->cross_hold) {
        if (white || travel_um - t->cross_um >= VC4_CROSS_HOLD_UM) {
            t->cross_hold = false;
            t->exit_side = 0;
            t->was_white = white;
        }
        if (t->cross_hold || white) {
            t->prev_raw = raw;   /* no damping kick off the arm */
            t->white_ms = 0u;
            for (uint32_t i = 0u; i < steps; ++i) {
                t->filt += (0 - t->filt) / VC4_FILT_DIV;
            }
            return t->filt;
        }
    }

    if (white) {
        /* The sensor was fully on the tape too long ago: the line faded
         * back toward the centre, it did not go out past the sensor. */
        if (!t->was_white && t->exit_side != 0) {
            uint32_t fresh_ms = t->exit_side > 0 ?
                VC4_LEFT_EXIT_FRESH_MS : VC4_EXIT_FRESH_MS;
            if (t->exit_age_ms > fresh_ms) t->exit_side = 0;
        }
        t->was_white = true;
        if (t->exit_side != 0) {
            if (t->coast_ms < coast_len(t)) {
                /* Coast: keep the steering the car already had (a curve
                 * stays a curve); do not let it relax toward straight. */
                t->coast_ms += dt_ms;
                target = t->filt;
            } else {
                int32_t rec = t->recover;
                if (t->exit_side * t->bias > rec) rec = t->exit_side * t->bias;
                target = t->exit_side * rec;
                t->recover += VC4_RECOVER_RAMP * (int32_t)steps;
                if (t->recover > VC4_RECOVER_MAX) t->recover = VC4_RECOVER_MAX;
            }
        } else {
            /* Between the sensors: keep the learned curve turn, fading. */
            target = t->bias;
            bias_fade(t, VC4_BIAS_DECAY_DIV, steps);
        }
        /* Loss: only after a black exit, 200 ms and 60 mm of white. */
        if (t->white_ms == 0u) t->white_start_um = travel_um;
        if (t->white_ms < 100000u) t->white_ms += dt_ms;
        if (!t->lost && t->exit_side != 0 && t->white_ms >= VC4_LOST_MS &&
            travel_um - t->white_start_um >= VC4_EXIT_LOST_UM) {
            t->lost = true;
        }
    } else {
        if (iabs32(raw) < VC4_SOFT_BAND) {
            target = raw * iabs32(raw) / VC4_SOFT_BAND;  /* calm near centre */
        } else {
            target = raw;
        }
        target = target * VC4_P_PCT / 100;
        /* Curve memory: grows toward the side the line keeps meeting. */
        if ((raw > 0 && t->bias < 0) || (raw < 0 && t->bias > 0)) {
            bias_fade(t, VC4_BIAS_FLIP_DIV, steps);
        }
        t->bias = clamp_steer(t->bias + raw / VC4_BIAS_GAIN_DIV *
                              (int32_t)steps, VC4_BIAS_MAX);
        target = clamp_steer(target + t->bias, VC4_STEER_MAX);
        track_exit(t, nl, nr, dt_ms);
        if (raw >= VC4_SIDE_MIN) {
            t->last_side = 1;
        } else if (raw <= -VC4_SIDE_MIN) {
            t->last_side = -1;
        }
        t->white_ms = 0u;
        if (t->lost && vc4_black(nl, nr)) t->lost = false;   /* found */
    }

    /* Damping against the line's movement under the sensors. */
    {
        /* On a smoothed error (sensor noise must not make it twitch). */
        int32_t rs = t->prev_raw + (raw - t->prev_raw) / VC4_D_SMOOTH;
        /* Weaker once the line may have gone out past a sensor: its
         * reading falls then just as when the line comes back, so full
         * damping would steer away from it. */
        if (!t->lost) {
            int32_t d = (rs - t->prev_raw) * VC4_D_PCT / 100;
            if (white && t->exit_side != 0) d = d * VC4_D_EXIT_PCT / 100;
            if (dt_ms > 0u) d = d * 20 / (int32_t)dt_ms;
            d = clamp_steer(d, VC4_D_MAX);
            target = clamp_steer(target + d, VC4_STEER_MAX);
        }
        t->prev_raw = rs;
    }

    /* Low-pass, applied once per 10 ms of elapsed time. */
    for (uint32_t i = 0u; i < steps; ++i) {
        t->filt += (target - t->filt) / VC4_FILT_DIV;
    }
    if (t->filt > VC4_STEER_MAX) t->filt = VC4_STEER_MAX;
    if (t->filt < -VC4_STEER_MAX) t->filt = -VC4_STEER_MAX;
    return t->filt;
}

bool vc4_on_line(const vc4_track_t *t)
{
    return !t->lost;
}

uint32_t vc4_white_um(const vc4_track_t *t, uint32_t travel_um)
{
    return t->white_ms == 0u ? 0u : travel_um - t->white_start_um;
}

void vc4_junction_reset(vc4_track_t *t)
{
    t->jct_state = 0u;
    t->sat_l = t->sat_r = false;
    t->jct_mark_um = 0u;
    t->jct_width_um = 0u;
}

vc4_jct_t vc4_junction(vc4_track_t *t, uint32_t nl, uint32_t nr,
                       uint32_t travel_um)
{
    uint32_t sum = nl + nr;
    bool wide = nl >= VC4_JCT_SENSOR_MIN && nr >= VC4_JCT_SENSOR_MIN &&
        sum >= VC4_JCT_SUM_MIN;
    bool still_tape = nl >= VC4_JCT_KEEP_MIN && nr >= VC4_JCT_KEEP_MIN &&
        sum >= VC4_JCT_KEEP_SUM;
    vc4_jct_t cls = VC4_JCT_NONE;

    switch (t->jct_state) {
    case 0u:
        if (wide) {
            t->jct_mark_um = travel_um;
            t->sat_l = nl >= VC4_SAT;
            t->sat_r = nr >= VC4_SAT;
            t->jct_state = 1u;
        }
        break;
    case 1u:
        t->sat_l = t->sat_l || nl >= VC4_SAT;
        t->sat_r = t->sat_r || nr >= VC4_SAT;
        if (!still_tape) {
            if (travel_um - t->jct_mark_um >= VC4_JCT_MIN_UM) {
                t->jct_width_um = travel_um - t->jct_mark_um;
                t->jct_mark_um = travel_um;
                t->jct_state = 2u;
            } else {
                vc4_junction_reset(t);       /* too narrow: not a tape */
            }
        }
        break;
    default:
        if (wide) {
            /* More tape before the clear gap: do not classify bar stripes
             * or sensor flicker as a completed junction. */
            t->jct_state = 1u;
            t->jct_mark_um = travel_um;
            t->sat_l = nl >= VC4_SAT;
            t->sat_r = nr >= VC4_SAT;
            break;
        }
        if (travel_um - t->jct_mark_um >= VC4_JCT_CONFIRM_UM) {
            if (t->sat_l && t->sat_r) {
                cls = VC4_JCT_CROSS;
            } else if (t->sat_l) {
                cls = VC4_JCT_SIDE_L;
            } else if (t->sat_r) {
                cls = VC4_JCT_SIDE_R;
            }
            vc4_junction_reset(t);
        }
        break;
    }
    return cls;
}

int32_t vc4_mm_s_to_cps(int32_t mm_s)
{
    return (mm_s * 1000 + (int32_t)VC4_UM_PER_COUNT / 2) /
        (int32_t)VC4_UM_PER_COUNT;
}

void vc4_wheels(int32_t s, int32_t base_mm_s, int32_t slow_mm_s,
                int32_t *speed_cps, int32_t *steering_permille)
{
    int32_t mag = iabs32(s);
    /* Curve slow-down: base drops by up to VC4_CURVE_SLOW_PCT at full. */
    int32_t base_x10 = base_mm_s * 10 *
        (1000 - mag * VC4_CURVE_SLOW_PCT / 100) / 1000;
    int32_t diff_x10 = s * VC4_SPLIT_FULL_X10 / 1000;
    int32_t diff_max;
    int32_t left_x10, right_x10;

    if (base_mm_s > 0 && base_mm_s < slow_mm_s) {
        /* Below the slow speed the split shrinks with the base, so an
         * error bends the path as much per mm as at slow speed. */
        diff_x10 = diff_x10 * base_mm_s / slow_mm_s;
    }
    diff_max = 2 * base_x10 / 3;              /* inner >= 1/3 of base */
    if (diff_x10 > diff_max) diff_x10 = diff_max;
    if (diff_x10 < -diff_max) diff_x10 = -diff_max;
    /* + s = line left: slow the left wheel. */
    left_x10 = base_x10 - diff_x10;
    right_x10 = base_x10 + diff_x10;
    *speed_cps = vc4_mm_s_to_cps((left_x10 + right_x10) / 20);
    if (left_x10 + right_x10 <= 0) {
        *steering_permille = 0;
    } else {
        *steering_permille = (left_x10 - right_x10) * 1000 /
            (left_x10 + right_x10);
    }
    /* Do not ask the inner wheel to crawl below its usable 150 cps floor
     * on normal-speed curves. Preserve the radius by scaling both wheels,
     * rather than straightening the requested steering split. */
    if (base_mm_s >= 60 && *speed_cps > 0) {
        int32_t inner_share = 1000 - iabs32(*steering_permille);
        int32_t minimum_mean = (150000 + inner_share - 1) / inner_share;
        if (*speed_cps < minimum_mean) *speed_cps = minimum_mean;
    }
}
