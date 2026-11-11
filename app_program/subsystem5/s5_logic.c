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
static S5Command reverse_or_fail(S5Controller *c, S5Error why)
{
    if (c->reverse_attempts == 0u) {
        c->reverse_attempts++;
        return change(c, S5_REVERSE, S5_CMD_REVERSE);
    }
    c->state = S5_FAULT;
    c->error = why;
    c->ticks_in_state = 0;
    return S5_CMD_HALT;
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
    if (first < 1 || last >= S5_SCAN_COUNT-1 || last-first < 1)
    {
        return p;
    }
    for (i = first; i <= last; ++i)
        if (fine->points[i].distance_cm >= S5_CLEAR_CM)
        {
            return p;
        }
    left_angle = (uint16_t)((fine->points[first-1].angle_deg +
                             fine->points[first].angle_deg) / 2u);
    right_angle = (uint16_t)((fine->points[last].angle_deg +
                              fine->points[last+1].angle_deg) / 2u);
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
            c->side = s5_choose_side(&in->coarse);
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
            c->profile = s5_profile_from_scans(&c->last_coarse, &in->fine, c->side);
            if (!c->profile.valid)
            {
                return fail(c, S5_ERR_FINE_SCAN);
            }
            c->side = s5_plan_bypass(&c->profile, &c->last_coarse);
            if (c->side == S5_SIDE_NONE) {
                if (c->profile.size == S5_SIZE_OVERSIZED)
                    return fail(c, S5_ERR_INSUFFICIENT_CLEARANCE);
                return reverse_or_fail(c, s5_choose_side(&c->last_coarse) == S5_SIDE_NONE ?
                                       S5_ERR_NO_CLEAR_ROUTE : S5_ERR_INSUFFICIENT_CLEARANCE);
            }
            return change(c, S5_AVOID,
                          c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
        }
        if (c->ticks_in_state >= S5_WAIT_LIMIT)
            return fail(c, S5_ERR_SCAN_TIMEOUT);
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
                                  "AVOID", "RECOVER", "DONE", "FAULT"};
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
