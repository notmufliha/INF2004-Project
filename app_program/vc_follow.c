/* vc_follow.c - line following v4 (see vc_follow.h). */
#include "vc_follow.h"

static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

void vc4_reset(vc4_track_t *t)
{
    t->filt = 0;
    t->last_side = 0;
    t->exit_side = 0;
    t->prev_exit = 0;
    t->was_white = true;
    t->recover = VC4_RECOVER_START;
    t->coast_ms = 0u;
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
static void track_exit(vc4_track_t *t, uint32_t nl, uint32_t nr)
{
    bool bl = nl >= VC4_BLACK;
    bool br = nr >= VC4_BLACK;

    if (t->was_white) {
        if (t->exit_side != 0) t->prev_exit = t->exit_side;
        t->exit_side = 0;
        t->was_white = false;
        t->recover = VC4_RECOVER_START;
        t->coast_ms = 0u;
    }
    if (bl && !br) {
        t->exit_side = 1;
    } else if (br && !bl) {
        t->exit_side = -1;
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

    if (white) {
        t->was_white = true;
        if (t->exit_side != 0) {
            if (t->coast_ms < coast_len(t)) {
                t->coast_ms += dt_ms;
            } else {
                target = t->exit_side * t->recover;
                t->recover += VC4_RECOVER_RAMP * (int32_t)steps;
                if (t->recover > VC4_RECOVER_MAX) t->recover = VC4_RECOVER_MAX;
            }
        }
        /* Loss: only after a black exit, 200 ms and 60 mm of white. */
        if (t->white_ms == 0u) t->white_start_um = travel_um;
        if (t->white_ms < 100000u) t->white_ms += dt_ms;
        if (!t->lost && t->exit_side != 0 && t->white_ms >= VC4_LOST_MS &&
            travel_um - t->white_start_um >= VC4_EXIT_LOST_UM) {
            t->lost = true;
        }
    } else {
        target = raw * iabs32(raw) / 1000;   /* gentle near centre */
        track_exit(t, nl, nr);
        if (raw >= VC4_SIDE_MIN) {
            t->last_side = 1;
        } else if (raw <= -VC4_SIDE_MIN) {
            t->last_side = -1;
        }
        t->white_ms = 0u;
        if (t->lost && vc4_black(nl, nr)) t->lost = false;   /* found */
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

void vc4_junction_reset(vc4_track_t *t)
{
    t->jct_state = 0u;
    t->sat_l = t->sat_r = false;
    t->jct_mark_um = 0u;
}

vc4_jct_t vc4_junction(vc4_track_t *t, uint32_t nl, uint32_t nr,
                       uint32_t travel_um)
{
    uint32_t sum = nl + nr;
    vc4_jct_t cls = VC4_JCT_NONE;

    switch (t->jct_state) {
    case 0u:
        if (sum >= VC4_JCT_SUM_MIN) {
            t->jct_mark_um = travel_um;
            t->sat_l = nl >= VC4_SAT;
            t->sat_r = nr >= VC4_SAT;
            t->jct_state = 1u;
        }
        break;
    case 1u:
        t->sat_l = t->sat_l || nl >= VC4_SAT;
        t->sat_r = t->sat_r || nr >= VC4_SAT;
        if (sum < VC4_JCT_SUM_MIN) {
            if (travel_um - t->jct_mark_um >= VC4_JCT_MIN_UM) {
                t->jct_mark_um = travel_um;
                t->jct_state = 2u;
            } else {
                vc4_junction_reset(t);       /* too narrow: not a tape */
            }
        }
        break;
    default:
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
}
