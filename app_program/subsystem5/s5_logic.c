#include "s5_logic.h"
#include <stddef.h>

static float s5_abs(float x) { return x < 0.0f ? -x : x; }
static float s5_min(float a, float b) { return a < b ? a : b; }
/* cos(theta) = sin(90-theta), with input within [-90,90]. */
static float s5_cos_deg(float degrees);

/* Sine approximation over [-90,90] degrees, max error < 0.001.
   No libm dependency: compatible with the project's bare-metal Makefile. */
static float s5_sin_deg(float degrees)
{
    const float pi_over_180 = 0.017453292519943295f;
    float x = degrees * pi_over_180;
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f + x2*x2 / 120.0f - x2*x2*x2 / 5040.0f);
}

static float s5_cos_deg(float degrees) { return s5_sin_deg(90.0f - s5_abs(degrees)); }

void s5_make_fine_angles(uint16_t center_deg, uint16_t out[S5_SCAN_COUNT])
{
    int center = (int)center_deg;
    int i;
    if (!out)
    {
        return;
    }
    if (center < (int)ROBOT_SCAN_SAFE_MIN_DEG + 10)
    {
        center = (int)ROBOT_SCAN_SAFE_MIN_DEG + 10;
    }
    if (center > (int)ROBOT_SCAN_SAFE_MAX_DEG - 10)
    {
        center = (int)ROBOT_SCAN_SAFE_MAX_DEG - 10;
    }
    for (i = 0; i < S5_SCAN_COUNT; ++i)
        out[i] = (uint16_t)(center + (i - 2) * 5);
}

static S5Command change(S5Controller *c, S5State next, S5Command action);
static S5Command wait_for_route(S5Controller *c, S5Error why)
{
    c->error = why;
    c->clear_route_hits = 0u;
    if (c->route_waits < UINT8_MAX) ++c->route_waits;
    return change(c, S5_WAIT_ROUTE, S5_CMD_STOP);
}
static S5Command fail(S5Controller *c, S5Error why)
{
    c->state = S5_FAULT;
    c->error = why;
    c->ticks_in_state = 0;
    return S5_CMD_HALT;
}
static S5Command change(S5Controller *c, S5State next, S5Command action)
{
    c->state = next;
    c->ticks_in_state = 0;
    c->events++;
    return action;
}
void s5_controller_init(S5Controller *c)
{
    if (!c)
    {
        return;
    }
    *c = (S5Controller){0};
    c->state = S5_FOLLOW;
}
static bool valid_range(const S5ScanPoint *p)
{
    return p->valid && p->distance_cm > 0.0f &&
           p->distance_cm <= S5_MAX_RANGE_CM;
}
static bool safe_point(const S5ScanPoint *p)
{
    return valid_range(p) && p->distance_cm >= S5_CLEAR_CM;
}
static float side_range(const S5Scan *scan, S5Side side)
{
    int first = side == S5_SIDE_LEFT ? 0 : 3;
    float range = 0.0f;
    for (int i = first; i < first + 2; ++i) {
        /* An unobserved ray cannot certify this side, but must not block
         * the opposite side when both of its rays are trustworthy. */
        if (ROBOT_OBSTACLE_TEST && !valid_range(&scan->points[i])) return 0.0f;
        /* Do not select a side with an observed object inside the turn
         * clearance, even if its other ray sees a distant background. */
        if (valid_range(&scan->points[i]) &&
            scan->points[i].distance_cm <= ROBOT_BYPASS_MIN_TURN_CM) return 0.0f;
        if (valid_range(&scan->points[i]) && scan->points[i].distance_cm > range)
            range = scan->points[i].distance_cm;
    }
    return range;
}
static S5Side opening_side(float left, float right)
{
    bool l = left >= ROBOT_BYPASS_SIDE_CLEAR_CM;
    bool r = right >= ROBOT_BYPASS_SIDE_CLEAR_CM;
    if (l && (!r || left + ROBOT_BYPASS_SIDE_TIE_CM >= right)) return S5_SIDE_LEFT;
    return r ? S5_SIDE_RIGHT : S5_SIDE_NONE;
}
S5Side s5_probe_side(const S5Scan *scan)
{
    if (!scan) return S5_SIDE_NONE;
    float room[2] = {0.0f, 0.0f};
    for (int side = 0; side < 2; ++side) {
        int first = side == 0 ? 0 : 3;
        if (valid_range(&scan->points[first]) && valid_range(&scan->points[first+1])) {
            float nearest = s5_min(scan->points[first].distance_cm, scan->points[first+1].distance_cm);
            if (nearest > ROBOT_BYPASS_MIN_TURN_CM) room[side] = nearest;
        }
    }
    if (room[0] > 0.0f && (room[1] == 0.0f || room[0] + ROBOT_BYPASS_SIDE_TIE_CM >= room[1]))
        return S5_SIDE_LEFT;
    return room[1] > 0.0f ? S5_SIDE_RIGHT : S5_SIDE_NONE;
}
S5Side s5_choose_side(const S5Scan *scan)
{
    float left, right;
    bool l, r;
    if (!scan)
    {
        return S5_SIDE_NONE;
    }
    /* Both rays on the chosen side must be measured and clear.
       Invalid readings NEVER count as open space. */
    l = safe_point(&scan->points[0]) && safe_point(&scan->points[1]);
    r = safe_point(&scan->points[3]) && safe_point(&scan->points[4]);
    if (!l && !r)
    {
        return S5_SIDE_NONE;
    }
    if (l && !r)
    {
        return S5_SIDE_LEFT;
    }
    if (r && !l)
    {
        return S5_SIDE_RIGHT;
    }
    left = scan->points[0].distance_cm < scan->points[1].distance_cm ?
           scan->points[0].distance_cm : scan->points[1].distance_cm;
    right = scan->points[3].distance_cm < scan->points[4].distance_cm ?
            scan->points[3].distance_cm : scan->points[4].distance_cm;
    return left >= right ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
}

/* A five-ray fine scan can only give an approximate *visible* width.
   Require clear rays on BOTH sides of one contiguous obstacle segment.
   Place angular boundaries halfway between the last clear ray and first
   obstacle ray. Project each boundary to lateral x = distance*sin(angle-90).
   This estimates a frontal silhouette, NOT the obstacle's full 3-D width. */
S5Profile s5_estimate_profile(const S5Scan *fine, S5Side side)
{
    S5Profile p = {0};
    int i, first = -1, last = -1;
    float left_x, right_x;
    uint16_t left_angle, right_angle;
    p.bypass_side = side;
    if (!fine) return p; /* Width can be profiled even if no bypass is safe. */
    for (i = 0; i < S5_SCAN_COUNT; i++) {
        const S5ScanPoint *v = &fine->points[i];
        if (!valid_range(v) || v->angle_deg > 180u ||
            (i && v->angle_deg <= fine->points[i-1].angle_deg)) return p;
        if (i == 0 || v->distance_cm < p.nearest_cm) {
            p.nearest_cm = v->distance_cm;
            p.nearest_angle_deg = v->angle_deg;
            p.nearest_lateral_cm = v->distance_cm *
                s5_sin_deg((float)v->angle_deg - 90.0f);
            p.nearest_forward_cm = v->distance_cm *
                s5_cos_deg((float)v->angle_deg - 90.0f);
        }
        if (v->distance_cm < S5_CLEAR_CM) {
            if (first < 0)
            {
                first = i;
            }
            last = i;
        }
    }
    /* Reject zero/one hit, unbracketed obstacles, or multiple disconnected
       clusters. All cases need a wider/finer scan, not an invented width. */
    if (first < 1 || last >= S5_SCAN_COUNT-1)
    {
        return p;
    }
    if (first == last && fine->points[last+1].angle_deg - fine->points[first-1].angle_deg < 80)
        return p;
    for (i = first; i <= last; ++i)
        if (fine->points[i].distance_cm >= S5_CLEAR_CM)
        {
            return p;
        }
    left_angle = (uint16_t)((fine->points[first-1].angle_deg +
                             fine->points[first].angle_deg) / 2u);
    right_angle = (uint16_t)((fine->points[last].angle_deg +
                              fine->points[last+1].angle_deg) / 2u);
    /* A single central hit in the chassis-assisted wide view is bracketed
     * by clear rays. Use their full angles as a conservative width bound. */
    if (first == last) {
        left_angle = fine->points[first-1].angle_deg;
        right_angle = fine->points[last+1].angle_deg;
    }
    /* The first/last obstacle distances are used for the two edges. */
    left_x = fine->points[first].distance_cm *
             s5_sin_deg((float)left_angle - 90.0f);
    right_x = fine->points[last].distance_cm *
              s5_sin_deg((float)right_angle - 90.0f);
    p.estimated_width_cm = s5_abs(right_x - left_x);
    if (p.estimated_width_cm < 0.01f)
    {
        return p;
    }
    p.left_boundary_deg = left_angle;
    p.right_boundary_deg = right_angle;
    p.center_x_cm = (left_x + right_x) * 0.5f;
    p.center_y_cm = p.nearest_cm *
        s5_sin_deg(90.0f - s5_abs((float)p.nearest_angle_deg - 90.0f));
    p.required_lateral_clearance_cm = p.estimated_width_cm * 0.5f +
        S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    p.size = p.estimated_width_cm <= ROBOT_OBSTACLE_SMALL_CM ? S5_SIZE_SMALL :
             p.estimated_width_cm <= ROBOT_OBSTACLE_MAX_BYPASS_CM ? S5_SIZE_LARGE :
             S5_SIZE_OVERSIZED;
    p.valid = true;
    return p;
}

S5Profile s5_profile_from_scans(const S5Scan *coarse, const S5Scan *fine, S5Side side)
{
    S5Profile p = s5_estimate_profile(fine, side);
    int i, first = -1, last = -1;
    if (p.valid || !fine) return p;
    for (i = 0; i < S5_SCAN_COUNT; ++i) {
        if (!valid_range(&fine->points[i]) || fine->points[i].angle_deg > 180u ||
            (i && fine->points[i].angle_deg <= fine->points[i - 1].angle_deg))
            return p;
        if (fine->points[i].distance_cm < S5_CLEAR_CM) {
            if (first < 0) first = i;
            last = i;
        }
    }
    if (first < 0 || last - first < 1 || (first != 0 && last != S5_SCAN_COUNT - 1))
        return p;
    for (i = first; i <= last; ++i)
        if (fine->points[i].distance_cm >= S5_CLEAR_CM) return p;
    return s5_estimate_profile(coarse, side);
}

S5Profile s5_profile_from_course_bounds(const S5Scan *coarse, const S5Scan *fine)
{
    S5Profile p = {0};
    float left = 0.0f, right = 0.0f;
    bool clipped = false;
    int first = -1, last = -1;
    if (!coarse || !fine) return p;
    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
        if (!valid_range(&coarse->points[i]) || !valid_range(&fine->points[i]) ||
            coarse->points[i].angle_deg != (uint16_t)(70 + i * 10) ||
            fine->points[i].angle_deg < ROBOT_SCAN_SAFE_MIN_DEG ||
            fine->points[i].angle_deg > ROBOT_SCAN_SAFE_MAX_DEG ||
            (i && fine->points[i].angle_deg <= fine->points[i - 1].angle_deg)) return p;
        if (fine->points[i].distance_cm < S5_CLEAR_CM) {
            if (first < 0) first = i;
            last = i;
        }
    }
    if (first < 0) return p;
    clipped = first == 0 || last == S5_SCAN_COUNT - 1;
    if (!clipped && first != last) return p;
    for (int i = first; i <= last; ++i)
        if (fine->points[i].distance_cm >= S5_CLEAR_CM) return p;
    /* Scanning can measure slightly farther than the triggering sample.
     * Keep the obstacle latch through the stop/release hysteresis band. */
    if (coarse->points[2].distance_cm >= ROBOT_RELEASE_CM ||
        coarse->points[2].distance_cm <= ROBOT_BYPASS_MIN_TURN_CM) return p;
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
    /* The frontal ray can hit an edge rather than the centre. Bound each
     * side by the FULL course width from that ray, not half the width. */
    p.required_lateral_clearance_cm = p.estimated_width_cm +
        S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    p.left_clearance_cm = left = side_range(coarse, S5_SIDE_LEFT);
    p.right_clearance_cm = right = side_range(coarse, S5_SIDE_RIGHT);
    /* Side rays select a candidate turn direction. They do not certify
     * the full sideways corridor: after turning, the executor measures
     * that corridor head-on before permitting any forward translation. */
    p.bypass_side = opening_side(left, right);
    if (p.bypass_side == S5_SIDE_NONE)
        return (S5Profile){0};
    p.center_x_cm = 0.0f; /* Unknown lateral centre; executor uses full width. */
    p.nearest_cm = p.center_y_cm = coarse->points[2].distance_cm;
    p.nearest_forward_cm = p.center_y_cm;
    p.nearest_angle_deg = 90u;
    p.size = S5_SIZE_LARGE;
    p.valid = p.uses_course_bounds = true;
    return p;
}

/* Multiple close surfaces can split the silhouette. A valid inner side
 * ray still supplies a candidate heading; do not require one contiguous
 * object before issuing a monitored turn under explicit course bounds. */
static S5Profile profile_from_openings(const S5Scan *scan)
{
    S5Profile p = {0};
    if (!scan || !valid_range(&scan->points[2]) ||
        scan->points[2].distance_cm <= ROBOT_BYPASS_MIN_TURN_CM ||
        scan->points[2].distance_cm >= ROBOT_RELEASE_CM) return p;
    for (int i = 0; i < S5_SCAN_COUNT; ++i)
        if (scan->points[i].angle_deg != (uint16_t)(70 + i * 10)) return p;
    p.left_clearance_cm = side_range(scan, S5_SIDE_LEFT);
    p.right_clearance_cm = side_range(scan, S5_SIDE_RIGHT);
    p.bypass_side = opening_side(p.left_clearance_cm, p.right_clearance_cm);
    if (p.bypass_side == S5_SIDE_NONE) return p;
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
    p.required_lateral_clearance_cm = p.estimated_width_cm +
        S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    p.nearest_cm = p.nearest_forward_cm = p.center_y_cm = scan->points[2].distance_cm;
    p.nearest_angle_deg = 90u;
    p.size = S5_SIZE_LARGE;
    p.valid = p.uses_course_bounds = true;
    return p;
}

/* Clearance screening is a recommendation. Execution additionally checks
   live sensing, encoder completion and the validated course geometry. */
static float side_clearance(const S5Scan *scan, S5Side side)
{
    int a = side == S5_SIDE_LEFT ? 0 : 3;
    if (!scan || side == S5_SIDE_NONE)
    {
        return 0.0f;
    }
    if (!safe_point(&scan->points[a]) || !safe_point(&scan->points[a+1]))
        return 0.0f;
    return s5_min(scan->points[a].distance_cm, scan->points[a+1].distance_cm);
}
/* Project each side ray onto the lateral axis. Radial distance alone
 * overstates room, especially for rays close to the forward axis. */
static float lateral_room(const S5Scan *scan, S5Side side)
{
    int a = side == S5_SIDE_LEFT ? 0 : 3;
    float room = S5_MAX_RANGE_CM;
    int i;
    if (side_clearance(scan, side) <= 0.0f) return 0.0f;
    for (i = a; i < a + 2; ++i) {
        float angle = (float)scan->points[i].angle_deg - 90.0f;
        if ((side == S5_SIDE_LEFT && (angle >= 0.0f || angle < -90.0f)) ||
            (side == S5_SIDE_RIGHT && (angle <= 0.0f || angle > 90.0f)))
            return 0.0f;
        room = s5_min(room, scan->points[i].distance_cm * s5_abs(s5_sin_deg(angle)));
    }
    return room;
}
S5Side s5_plan_bypass(S5Profile *p, const S5Scan *coarse)
{
    float left, right;
    bool l, r;
    if (!p) return S5_SIDE_NONE;
    p->bypass_side = S5_SIDE_NONE;
    p->left_clearance_cm = side_clearance(coarse, S5_SIDE_LEFT);
    p->right_clearance_cm = side_clearance(coarse, S5_SIDE_RIGHT);
    if (!p->valid || !(p->estimated_width_cm > 0.0f) ||
        p->estimated_width_cm > ROBOT_OBSTACLE_MAX_BYPASS_CM ||
        !(p->center_x_cm >= -S5_MAX_RANGE_CM && p->center_x_cm <= S5_MAX_RANGE_CM))
        return S5_SIDE_NONE;
    p->required_lateral_clearance_cm = p->estimated_width_cm * 0.5f +
                                      S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    left = lateral_room(coarse, S5_SIDE_LEFT);
    right = lateral_room(coarse, S5_SIDE_RIGHT);
    /* Account for an obstacle offset from the robot's forward axis. */
    l = left > 0.0f && left >= p->required_lateral_clearance_cm - p->center_x_cm;
    r = right > 0.0f && right >= p->required_lateral_clearance_cm + p->center_x_cm;
    if (l && (!r || left + p->center_x_cm >= right - p->center_x_cm))
        p->bypass_side = S5_SIDE_LEFT;
    else if (r) p->bypass_side = S5_SIDE_RIGHT;
    return p->bypass_side;
}
static S5Side choose_route(S5Profile *profile, const S5Scan *coarse)
{
    if (!profile->valid || profile->size == S5_SIZE_OVERSIZED) return S5_SIDE_NONE;
    if (profile->uses_course_bounds) return profile->bypass_side;
    /* Compare BOTH candidate openings before committing. The old early
     * return could pick a smaller two-ray right corridor while ignoring
     * a larger left opening whose inner ray still hit the obstacle edge.
     * The guarded turn and live readings validate the selected heading. */
    profile->left_clearance_cm = side_range(coarse, S5_SIDE_LEFT);
    profile->right_clearance_cm = side_range(coarse, S5_SIDE_RIGHT);
    profile->bypass_side = opening_side(profile->left_clearance_cm, profile->right_clearance_cm);
    return profile->bypass_side;
}

S5Command s5_controller_step(S5Controller *c, const S5Input *in)
{
    if (!c || !in)
    {
        return S5_CMD_HALT;
    }
    if (c->ticks_in_state < UINT16_MAX)
    {
        c->ticks_in_state++;
    }
    /* Test mode consumes one complete scan; live echo fluctuations must not
     * send that cycle back through WAIT_STOP and another servo sweep. */
    if (!ROBOT_OBSTACLE_TEST && (c->state == S5_COARSE || c->state == S5_FINE)) {
        if (in->stopped && in->front_valid && in->front_cm >= ROBOT_RELEASE_CM &&
            in->front_cm <= S5_MAX_RANGE_CM) {
            if (++c->clear_route_hits >= ROBOT_CLEAR_CONFIRM) {
                c->error = S5_ERR_NONE;
                return change(c, S5_DONE, S5_CMD_RESUME);
            }
            return S5_CMD_NONE;
        }
        if (c->clear_route_hits != 0u) {
            c->clear_route_hits = 0u;
            return change(c, S5_WAIT_STOP, S5_CMD_STOP);
        }
    }
    switch (c->state) {
    case S5_FOLLOW:
        if (!in->front_valid || !(in->front_cm > 0.0f) ||
            in->front_cm > S5_MAX_RANGE_CM) {
            if (++c->invalid_front_count >= S5_FRONT_INVALID_LIMIT)
                return fail(c, S5_ERR_FRONT_SENSOR);
            return S5_CMD_NONE;
        }
        c->invalid_front_count = 0;
        if (in->front_cm <= S5_STOP_CM)
            return change(c, S5_WAIT_STOP, S5_CMD_STOP);
        return S5_CMD_NONE;
    case S5_WAIT_STOP:
        if (in->stopped)
        {
            return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
        }
        if (c->ticks_in_state >= S5_WAIT_LIMIT)
            return fail(c, S5_ERR_STOP_TIMEOUT);
        return S5_CMD_STOP;
    case S5_COARSE:
        if (in->coarse_ready) {
            c->last_coarse = in->coarse;
            if (ROBOT_OBSTACLE_TEST) {
                c->side = s5_choose_side(&in->coarse);
                int nearest = in->coarse.nearest_index;
                c->fine_center_deg = nearest >= 0 && nearest < S5_SCAN_COUNT ?
                    in->coarse.points[nearest].angle_deg : 90u;
                return change(c, S5_FINE, S5_CMD_SCAN_FINE);
            }
            c->side = s5_choose_side(&in->coarse);
            /* A usable coarse route is enough to start the guarded turn.
             * Do not spend a second complete servo sweep refining a width
             * already bounded by the configured course dimensions. */
            c->profile = s5_estimate_profile(&in->coarse, c->side);
            if (!c->profile.valid)
                c->profile = s5_profile_from_course_bounds(&in->coarse, &in->coarse);
            if (!c->profile.valid)
                c->profile = profile_from_openings(&in->coarse);
            c->side = choose_route(&c->profile, &in->coarse);
            if (c->side != S5_SIDE_NONE) {
                c->error = S5_ERR_NONE;
                return change(c, S5_AVOID,
                    c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
            }
            /* Fine scan focuses on the nearest detected coarse point. */
            if (in->coarse.nearest_index >= 0 &&
                in->coarse.nearest_index < S5_SCAN_COUNT)
                c->fine_center_deg = in->coarse.points[in->coarse.nearest_index].angle_deg;
            else c->fine_center_deg = 90u;
            return change(c, S5_FINE, S5_CMD_SCAN_FINE);
        }
        if (c->ticks_in_state >= S5_WAIT_LIMIT)
            return fail(c, S5_ERR_SCAN_TIMEOUT);
        return S5_CMD_NONE;
    case S5_FINE:
        if (in->fine_ready) {
            if (ROBOT_OBSTACLE_TEST) {
                unsigned fine_hits = 0u;
                for (int i = 0; i < S5_SCAN_COUNT; ++i)
                    if (valid_range(&in->fine.points[i])) ++fine_hits;
                /* Require the obstacle-facing fine centre and a majority of
                 * fine returns. A missing ray on the unused side no longer
                 * vetoes an otherwise measured opening. */
                if (!valid_range(&c->last_coarse.points[2]) ||
                    !valid_range(&in->fine.points[2]) || fine_hits < 3u ||
                    in->fine.points[2].distance_cm <= ROBOT_BYPASS_MIN_TURN_CM)
                    return wait_for_route(c, S5_ERR_NO_CLEAR_ROUTE);
                if (c->last_coarse.points[2].distance_cm >= ROBOT_RELEASE_CM)
                    return wait_for_route(c, S5_ERR_NONE);
            }
            c->profile = s5_profile_from_scans(&c->last_coarse, &in->fine, c->side);
            if (!c->profile.valid)
                c->profile = s5_profile_from_course_bounds(&c->last_coarse, &in->fine);
            if (ROBOT_OBSTACLE_TEST && !c->profile.valid)
                c->profile = profile_from_openings(&c->last_coarse);
            if (!c->profile.valid)
            {
                bool valid_scans = true;
                for (int i = 0; i < S5_SCAN_COUNT; ++i)
                    if (!valid_range(&c->last_coarse.points[i]) ||
                        !valid_range(&in->fine.points[i])) valid_scans = false;
                if (!valid_scans) return fail(c, S5_ERR_FINE_SCAN);
                return wait_for_route(c, S5_ERR_NO_CLEAR_ROUTE);
            }
            c->side = choose_route(&c->profile, &c->last_coarse);
            if (c->side == S5_SIDE_NONE) {
                if (c->profile.size == S5_SIZE_OVERSIZED)
                    return wait_for_route(c, S5_ERR_INSUFFICIENT_CLEARANCE);
                return wait_for_route(c, S5_ERR_NO_CLEAR_ROUTE);
            }
            c->error = S5_ERR_NONE;
            return change(c, S5_AVOID,
                          c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
        }
        if (c->ticks_in_state >= S5_WAIT_LIMIT)
            return fail(c, S5_ERR_SCAN_TIMEOUT);
        return S5_CMD_NONE;
    case S5_WAIT_ROUTE:
        if (!in->front_valid || !(in->front_cm > 0.0f) ||
            in->front_cm > S5_MAX_RANGE_CM) {
            c->clear_route_hits = 0u;
            if (++c->invalid_front_count >= S5_FRONT_INVALID_LIMIT)
                return fail(c, S5_ERR_FRONT_SENSOR);
            return S5_CMD_NONE;
        }
        c->invalid_front_count = 0u;
        if (in->front_cm >= ROBOT_RELEASE_CM && in->stopped) {
            if (++c->clear_route_hits >= ROBOT_CLEAR_CONFIRM) {
                c->error = S5_ERR_NONE;
                return change(c, S5_DONE, S5_CMD_RESUME);
            }
        } else c->clear_route_hits = 0u;
        if (!ROBOT_OBSTACLE_TEST && c->ticks_in_state >= S5_ROUTE_RESCAN_TICKS && in->stopped)
            return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
        return S5_CMD_NONE;
    case S5_REVERSE:
        if (in->motion_complete)
            return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
        if (c->ticks_in_state >= S5_MOTION_LIMIT)
            return fail(c, S5_ERR_REVERSE_TIMEOUT);
        return S5_CMD_NONE;
    case S5_AVOID:
        if (in->motion_complete)
            return change(c, S5_RECOVER, S5_CMD_FIND_LINE);
        if (c->ticks_in_state >= S5_MOTION_LIMIT)
            return fail(c, S5_ERR_MOTION_TIMEOUT);
        return S5_CMD_NONE;
    case S5_RECOVER:
        if (in->line_found)
        {
            return change(c, S5_DONE, S5_CMD_RESUME);
        }
        if (c->ticks_in_state >= S5_RECOVERY_LIMIT)
            return fail(c, S5_ERR_LINE_TIMEOUT);
        return S5_CMD_NONE;
    case S5_DONE: return S5_CMD_NONE;
    case S5_FAULT: return S5_CMD_HALT;
    default: return fail(c, S5_ERR_FRONT_SENSOR);
    }
}
const char *s5_state_name(S5State s)
{
    static const char *names[] = {"FOLLOW", "WAIT_STOP", "COARSE", "FINE", "REVERSE",
                                  "AVOID", "RECOVER", "DONE", "FAULT", "WAIT_ROUTE"};
    return (unsigned)s < sizeof(names)/sizeof(names[0]) ? names[s] : "UNKNOWN";
}
const char *s5_command_name(S5Command a)
{
    static const char *names[] = {"NONE", "STOP", "SCAN_COARSE", "SCAN_FINE", "REVERSE",
        "GO_LEFT", "GO_RIGHT", "FIND_LINE", "RESUME", "HALT"};
    return (unsigned)a < sizeof(names)/sizeof(names[0]) ? names[a] : "UNKNOWN";
}
const char *s5_error_name(S5Error e)
{
    static const char *names[] = {"NONE", "FRONT_SENSOR", "STOP_TIMEOUT",
        "SCAN_TIMEOUT", "NO_CLEAR_ROUTE", "FINE_SCAN", "MOTION_TIMEOUT", "LINE_TIMEOUT",
        "INSUFFICIENT_CLEARANCE", "REVERSE_TIMEOUT"};
    return (unsigned)e < sizeof(names)/sizeof(names[0]) ? names[e] : "UNKNOWN";
}
