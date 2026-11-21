#include "s5_logic.h"
#include "s5_geometry.h"
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
    int outer = side == S5_SIDE_LEFT ? 0 : 4;
    int inner = side == S5_SIDE_LEFT ? 1 : 3;
    float range = 0.0f;
    for (int i = first; i < first + 2; ++i) {
        /* An unobserved ray cannot certify this side, but must not block
         * the opposite side when both of its rays are trustworthy. */
        if (ROBOT_OBSTACLE_TEST && !valid_range(&scan->points[i])) {
            /* A missing inner edge echo does not erase a measured outer
             * opening. This nominates a monitored TURN only: forward travel
             * still requires fresh valid head-on clearance after rotation.
             * Missing outer rays and observed close returns still veto. */
            if (i == inner && valid_range(&scan->points[outer]) &&
                scan->points[outer].distance_cm >= ROBOT_BYPASS_MOVE_RELEASE_CM)
                continue;
            return 0.0f;
        }
        /* Do not select a side with an observed object inside the turn
         * clearance, even if its other ray sees a distant background. */
        if (valid_range(&scan->points[i]) &&
            s5_turn_range_cm(scan->points[i].distance_cm, scan->points[i].angle_deg) <= ROBOT_BYPASS_MIN_TURN_CM) return 0.0f;
        float pivot_range = s5_turn_range_cm(scan->points[i].distance_cm, scan->points[i].angle_deg);
        if (valid_range(&scan->points[i]) && pivot_range > range)
            range = pivot_range;
    }
    return range;
}
static float nearest_side_ray(const S5Scan *scan, int first)
{
    if (!valid_range(&scan->points[first]) || !valid_range(&scan->points[first + 1]))
        return 0.0f;
    return s5_min(s5_turn_range_cm(scan->points[first].distance_cm, scan->points[first].angle_deg),
                  s5_turn_range_cm(scan->points[first + 1].distance_cm, scan->points[first + 1].angle_deg));
}

static float width_side_clearance(const S5ScanPoint *points, S5Side side, float clearance)
{
    for (int i = 0; points && i < S5_WIDTH_SCAN_COUNT; ++i) {
        const S5ScanPoint *ray = &points[i];
        bool on_side = side == S5_SIDE_LEFT ? ray->angle_deg < 90u : ray->angle_deg > 90u;
        if (on_side && ray->angle_deg >= ROBOT_WIDTH_ROUTE_LEFT_FALLBACK_DEG &&
            ray->angle_deg <= ROBOT_WIDTH_ROUTE_RIGHT_DEG && valid_range(ray))
            clearance = s5_min(clearance, ray->distance_cm);
    }
    return clearance;
}

static S5Side compare_clearance(const S5Scan *scan, float left_near, float right_near,
                                float left_far, float right_far)
{
    float gap = left_near - right_near;
    if (s5_abs(gap) > ROBOT_BYPASS_SIDE_TIE_CM)
        return gap > 0.0f ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
    gap = left_far - right_far;
    if (s5_abs(gap) > ROBOT_BYPASS_SIDE_TIE_CM)
        return gap > 0.0f ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
    bool left_clear = safe_point(&scan->points[0]) && safe_point(&scan->points[1]);
    bool right_clear = safe_point(&scan->points[3]) && safe_point(&scan->points[4]);
    if (left_clear) return S5_SIDE_LEFT;
    if (right_clear) return S5_SIDE_RIGHT;
    if (left_near >= ROBOT_BYPASS_MOVE_RELEASE_CM) return S5_SIDE_LEFT;
    return right_near >= ROBOT_BYPASS_MOVE_RELEASE_CM ? S5_SIDE_RIGHT : S5_SIDE_NONE;
}

static S5Side opening_side(const S5Scan *scan, float left, float right)
{
    bool l = left >= ROBOT_BYPASS_SIDE_CLEAR_CM;
    bool r = right >= ROBOT_BYPASS_SIDE_CLEAR_CM;
    if (l && r) {
        /* Rank the candidate corridors by their tighter ray. A distant
         * background echo on one ray must not outweigh a nearby obstacle. */
        float left_near = nearest_side_ray(scan, 0);
        float right_near = nearest_side_ray(scan, 3);
        if (ROBOT_OBSTACLE_TEST) {
            if (left_near != right_near)
                return left_near > right_near ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
            return left >= right ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
        }
        return compare_clearance(scan, left_near, right_near, left, right);
    }
    if (l) return S5_SIDE_LEFT;
    return r ? S5_SIDE_RIGHT : S5_SIDE_NONE;
}
/* Predict only a candidate after a completed bounded retreat from a static
 * obstacle. Live front guards still decide whether rotation may execute. */
static S5Scan retreat_candidate(S5Scan scan)
{
    for (int i = 0; i < S5_SCAN_COUNT; ++i)
        if (valid_range(&scan.points[i]))
            scan.points[i].distance_cm += ROBOT_BYPASS_REVERSE_CM *
                s5_cos_deg((float)scan.points[i].angle_deg - 90.0f);
    return scan;
}
S5Side s5_probe_side(const S5Scan *scan)
{
    if (!scan) return S5_SIDE_NONE;
    /* Missing left echoes can nominate an in-place look. The live pivot
     * guard and the post-turn corridor scan still gate motion. */
    if (!valid_range(&scan->points[0]) && !valid_range(&scan->points[1]))
        return S5_SIDE_LEFT;
    bool left_obstacle = false, right_obstacle = false;
    for (int i = 0; i < 2; ++i) {
        const S5ScanPoint *left = &scan->points[i];
        const S5ScanPoint *right = &scan->points[i + 3];
        left_obstacle |= valid_range(left) && left->distance_cm <= ROBOT_SCAN_OBJECT_CM;
        right_obstacle |= valid_range(right) && right->distance_cm <= ROBOT_SCAN_OBJECT_CM;
    }
    if (right_obstacle && !left_obstacle) return S5_SIDE_LEFT;
    float room[2] = {0.0f, 0.0f};
    for (int side = 0; side < 2; ++side) {
        int first = side == 0 ? 0 : 3;
        float nearest = nearest_side_ray(scan, first);
        if (nearest > ROBOT_BYPASS_MIN_TURN_CM) room[side] = nearest;
    }
    if (room[0] > 0.0f && room[1] > 0.0f) {
        float left_far = side_range(scan, S5_SIDE_LEFT);
        float right_far = side_range(scan, S5_SIDE_RIGHT);
        if (s5_abs(room[0] - room[1]) <= ROBOT_BYPASS_SIDE_TIE_CM &&
            s5_abs(left_far - right_far) <= ROBOT_BYPASS_SIDE_TIE_CM)
            return S5_SIDE_LEFT;
        return compare_clearance(scan, room[0], room[1],
                                 left_far, right_far);
    }
    /* With incomplete evidence on the opposite side, a marginal pivot
     * candidate is insufficient. Wait for a rescan unless both rays on
     * the measured side already meet the bypass clearance. */
    for (int side = 0; side < 2; ++side) {
        int first = side == 0 ? 0 : 3;
        int other = side == 0 ? 3 : 0;
        if (room[side] > 0.0f &&
            ((valid_range(&scan->points[other]) && valid_range(&scan->points[other + 1])) ||
             (scan->points[first].distance_cm >= ROBOT_BYPASS_MOVE_RELEASE_CM &&
              scan->points[first + 1].distance_cm >= ROBOT_BYPASS_MOVE_RELEASE_CM)))
            return side == 0 ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
    }
    return S5_SIDE_NONE;
}

bool s5_probe_corridor_blocked(const S5Scan *scan)
{
    if (!scan) return true;
    const S5ScanPoint *front = &scan->points[2];
    if (!valid_range(front) ||
        front->distance_cm < ROBOT_COURSE_CLEAR_STEP_CM + ROBOT_BYPASS_MOVE_RELEASE_CM)
        return true;
    for (int i = 1; i <= 3; ++i) {
        const S5ScanPoint *ray = &scan->points[i];
        if (!valid_range(ray) || ray->distance_cm < ROBOT_BYPASS_MOVE_RELEASE_CM)
            return true;
    }
    return false;
}
S5Side s5_choose_side(const S5Scan *scan)
{
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
    return opening_side(scan, side_range(scan, S5_SIDE_LEFT), side_range(scan, S5_SIDE_RIGHT));
}

/* A five-ray fine scan can only give an approximate *visible* width.
   Require clear rays on BOTH sides of one contiguous obstacle segment.
   Place angular boundaries halfway between the last clear ray and first
   obstacle ray. Project each boundary to lateral x = distance*sin(angle-90).
   This estimates a frontal silhouette, NOT the obstacle's full 3-D width. */
static S5Profile estimate_profile(const S5ScanPoint *points, int count,
                                  S5Side side, bool width_sweep)
{
    S5Profile p = {0};
    int i, first = -1, last = -1;
    float left_x, right_x;
    uint16_t left_angle, right_angle;
    p.bypass_side = side;
    for (i = 0; i < count; i++) {
        const S5ScanPoint *v = &points[i];
        if (!valid_range(v) || v->angle_deg > 180u ||
            (i && v->angle_deg <= points[i-1].angle_deg)) return p;
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
    if (first < 1 || last >= count - 1)
    {
        return p;
    }
    if (first == last && !width_sweep &&
        points[last+1].angle_deg - points[first-1].angle_deg < 80)
        return p;
    if (width_sweep)
        for (i = first; i <= last + 1; ++i)
            if (points[i].angle_deg - points[i - 1].angle_deg > 30) return p;
    for (i = first; i <= last; ++i)
        if (points[i].distance_cm >= S5_CLEAR_CM)
        {
            return p;
        }
    left_angle = (uint16_t)((points[first-1].angle_deg + points[first].angle_deg) / 2u);
    right_angle = (uint16_t)((points[last].angle_deg + points[last+1].angle_deg) / 2u);
    /* A single hit in a wide view is bracketed
     * by clear rays. Use their full angles as a conservative width bound. */
    if (first == last) {
        left_angle = points[first-1].angle_deg;
        right_angle = points[last+1].angle_deg;
    }
    /* The first/last obstacle distances are used for the two edges. */
    left_x = points[first].distance_cm *
             s5_sin_deg((float)left_angle - 90.0f);
    right_x = points[last].distance_cm *
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

S5Profile s5_estimate_profile(const S5Scan *fine, S5Side side)
{
    if (!fine) return (S5Profile){0};
    return estimate_profile(fine->points, S5_SCAN_COUNT, side,
                            fine->points[0].angle_deg == ROBOT_WIDTH_SCAN_MIN_DEG);
}

#define S5_WIDTH_VIEW_RAYS (3 * S5_WIDTH_SCAN_COUNT)

static float face_lateral_distance(float range, float depth)
{
    return range > depth ? sqrtf((range - depth) * (range + depth)) : 0.0f;
}

static S5Profile estimate_width_sweep(const S5ScanPoint *points, int point_count)
{
    S5ScanPoint measured[S5_WIDTH_VIEW_RAYS];
    int count = 0, center = -1;
    for (int i = 0; i < point_count; ++i) {
        if (!valid_range(&points[i])) continue;
        if (points[i].angle_deg == 90u) center = count;
        measured[count++] = points[i];
    }
    if (center < 0 || measured[center].distance_cm >= S5_CLEAR_CM)
        return (S5Profile){0};

    /* Start at the frontal hit and follow continuous surface returns
     * outward. Compare adjacent depths so a slanted face can stay in the
     * cluster even when its edge is closer than the centre. */
    const float tolerance = ROBOT_ECHO_AGREEMENT_MIN_CM;
    int edges[2];
    for (int side = 0; side < 2; ++side) {
        const int step = side == 0 ? -1 : 1;
        float depth = measured[center].distance_cm;
        int i = center + step;
        for (; i >= 0 && i < count; i += step) {
            float offset = (float)measured[i].angle_deg - 90.0f;
            float ray_depth = measured[i].distance_cm * s5_cos_deg(offset);
            float delta = ray_depth - depth;
            if (delta > tolerance) break;
            if (delta < -tolerance || measured[i].distance_cm >= S5_CLEAR_CM)
                return (S5Profile){0};
            depth = ray_depth;
        }
        if (i < 0 || i >= count) return (S5Profile){0};
        edges[side] = i;
    }
    measured[edges[0]].distance_cm = S5_MAX_RANGE_CM;
    measured[edges[1]].distance_cm = S5_MAX_RANGE_CM;
    S5Profile profile = estimate_profile(&measured[edges[0]],
        edges[1] - edges[0] + 1, S5_SIDE_NONE, true);
    if (profile.valid) {
        /* With a roughly flat face at the centre-ray depth, the farther
         * edge echoes constrain lateral extent by range. The servo bearing
         * marks the centre of the sonar beam, not the point that reflected
         * the pulse. Keep the angular envelope for bypass clearance. */
        const float depth = measured[center].distance_cm;
        const float left = measured[edges[0] + 1].distance_cm;
        const float right = measured[edges[1] - 1].distance_cm;
        if (left > depth && right > depth) {
            float left_x = face_lateral_distance(left, depth);
            float right_x = face_lateral_distance(right, depth);
            /* Use the neighbouring inward hit on both flanks. When only
             * one off-centre ray hits, the centre ray is its neighbour. */
            left_x = (left_x + face_lateral_distance(measured[edges[0] + 2].distance_cm, depth)) * 0.5f;
            right_x = (right_x + face_lateral_distance(measured[edges[1] - 2].distance_cm, depth)) * 0.5f;
            profile.frontal_width_cm = s5_min(left_x + right_x, profile.estimated_width_cm);
        }
    }
    return profile;
}

S5Profile s5_estimate_width_from_views(const S5ScanPoint *front,
                                      const S5ScanPoint *opposite,
                                      const S5ScanPoint *turned, int heading)
{
    S5ScanPoint combined[S5_WIDTH_VIEW_RAYS];
    int count = 0;
    for (int i = 0; i < S5_WIDTH_SCAN_COUNT; ++i) combined[count++] = front[i];
    for (int view = 0; view < 2; ++view) {
        const S5ScanPoint *points = view == 0 ? opposite : turned;
        const int yaw = view == 0 ? -heading : heading;
        if (points == NULL) continue;
        for (int i = 0; i < S5_WIDTH_SCAN_COUNT; ++i) {
            if (!valid_range(&points[i])) continue;
            int bearing = (int)points[i].angle_deg + yaw;
            if (bearing <= 0 || bearing >= 180) continue;
            float distance = points[i].distance_cm;
            /* Bring actual echoes into the original sensor frame. The
             * front-mounted sensor moves around the axle during a pivot. */
            if (distance < S5_CLEAR_CM) {
                float x = distance * s5_sin_deg((float)bearing - 90.0f) +
                    ROBOT_SENSOR_FORWARD_OFFSET_CM * s5_sin_deg((float)yaw);
                float y = distance * s5_cos_deg((float)bearing - 90.0f) +
                    ROBOT_SENSOR_FORWARD_OFFSET_CM * (s5_cos_deg((float)yaw) - 1.0f);
                if (y <= 0.0f) continue;
                bearing = 90 + (int)(atan2f(x, y) * 57.2957795f);
                distance = sqrtf(x * x + y * y);
            }
            if (bearing >= (int)ROBOT_WIDTH_SWEEP_MIN_DEG &&
                bearing <= (int)ROBOT_WIDTH_SWEEP_MAX_DEG) continue;
            int pos = 0;
            while (pos < count && combined[pos].angle_deg < bearing) ++pos;
            if (pos < count && combined[pos].angle_deg == bearing) continue;
            for (int j = count; j > pos; --j) combined[j] = combined[j - 1];
            combined[pos] = (S5ScanPoint){(uint16_t)bearing, distance, true};
            ++count;
        }
    }
    return estimate_width_sweep(combined, count);
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
        coarse->points[2].distance_cm <= ROBOT_BYPASS_FRONT_TURN_CM) return p;
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
    /* The frontal ray can hit an edge rather than the centre. Bound each
     * side by the FULL course width from that ray, not half the width. */
    p.required_lateral_clearance_cm = p.estimated_width_cm * (ROBOT_BYPASS_CENTERED_TEST_BOX ? 0.5f : 1.0f) +
        S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    p.left_clearance_cm = left = side_range(coarse, S5_SIDE_LEFT);
    p.right_clearance_cm = right = side_range(coarse, S5_SIDE_RIGHT);
    /* Side rays select a candidate turn direction. They do not certify
     * the full sideways corridor: after turning, the executor measures
     * that corridor head-on before permitting any forward translation. */
    p.bypass_side = opening_side(coarse, left, right);
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
        scan->points[2].distance_cm <= ROBOT_BYPASS_FRONT_TURN_CM ||
        scan->points[2].distance_cm >= ROBOT_RELEASE_CM) return p;
    for (int i = 0; i < S5_SCAN_COUNT; ++i)
        if (scan->points[i].angle_deg != (uint16_t)(70 + i * 10)) return p;
    p.left_clearance_cm = side_range(scan, S5_SIDE_LEFT);
    p.right_clearance_cm = side_range(scan, S5_SIDE_RIGHT);
    if (!ROBOT_OBSTACLE_TEST && scan->points[2].distance_cm >= S5_CLEAR_CM) {
        /* At the earlier stopping distance, compare the side rays to the
         * front face so a wide wall does not look like two open routes. */
        float opening_cm = scan->points[2].distance_cm + S5_EXTRA_MARGIN_CM;
        if (p.left_clearance_cm < opening_cm) p.left_clearance_cm = 0.0f;
        if (p.right_clearance_cm < opening_cm) p.right_clearance_cm = 0.0f;
    }
    p.bypass_side = opening_side(scan, p.left_clearance_cm, p.right_clearance_cm);
    if (p.bypass_side == S5_SIDE_NONE) return p;
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
    p.required_lateral_clearance_cm = p.estimated_width_cm * (ROBOT_BYPASS_CENTERED_TEST_BOX ? 0.5f : 1.0f) +
        S5_CHASSIS_HALF_WIDTH_CM + S5_EXTRA_MARGIN_CM;
    p.nearest_cm = p.nearest_forward_cm = p.center_y_cm = scan->points[2].distance_cm;
    p.nearest_angle_deg = 90u;
    p.size = S5_SIZE_LARGE;
    p.valid = p.uses_course_bounds = true;
    return p;
}

S5Profile s5_profile_from_probe(const S5Profile *profile, const S5Scan *coarse,
                                const S5Scan *view, S5Side side, unsigned turn_deg, float front_cm)
{
    if (side != S5_SIDE_LEFT && side != S5_SIDE_RIGHT) return (S5Profile){0};
    S5Profile p = {0};
    if (profile && profile->valid && !profile->uses_course_bounds) {
        p = *profile;
    } else if (coarse && view) {
        /* Combine measured rays from the stopped, turned viewpoint with
         * rays on the opposite flank from the original heading. */
        S5Scan combined = *coarse;
        if (side == S5_SIDE_LEFT) {
            combined.points[0] = view->points[2];
            combined.points[1] = view->points[4];
            combined.points[0].angle_deg -= turn_deg;
            combined.points[1].angle_deg -= turn_deg;
        } else {
            combined.points[3] = view->points[0];
            combined.points[4] = view->points[2];
            combined.points[3].angle_deg += turn_deg;
            combined.points[4].angle_deg += turn_deg;
        }
        p = s5_estimate_profile(&combined, side);
    }
    if (!p.valid) {
        p = (S5Profile){0};
        p.valid = p.uses_course_bounds = true;
        p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
        p.size = S5_SIZE_LARGE;
        p.nearest_cm = front_cm;
    }
    if (p.center_y_cm < front_cm) p.center_y_cm = front_cm;
    p.bypass_side = side;
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
    /* Compare candidate corridors before the guarded turn validates the
     * selected heading with fresh readings. */
    profile->left_clearance_cm = side_range(coarse, S5_SIDE_LEFT);
    profile->right_clearance_cm = side_range(coarse, S5_SIDE_RIGHT);
    profile->bypass_side = opening_side(coarse, profile->left_clearance_cm, profile->right_clearance_cm);
    return profile->bypass_side;
}

static bool can_probe_partial_scan(const S5Scan *scan, const S5Input *in)
{
    return !ROBOT_OBSTACLE_TEST && in->stopped && in->front_valid &&
           in->front_cm > ROBOT_BYPASS_FRONT_TURN_CM && in->front_cm < ROBOT_RELEASE_CM &&
           scan->points[0].angle_deg == ROBOT_SCAN_SAFE_MIN_DEG &&
           scan->points[4].angle_deg == ROBOT_SCAN_SAFE_MAX_DEG &&
           (!valid_range(&scan->points[0]) || !valid_range(&scan->points[4])) &&
           valid_range(&scan->points[2]);
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
    if (!ROBOT_OBSTACLE_TEST && (c->state == S5_COARSE || c->state == S5_FINE) &&
        !(c->state == S5_COARSE && in->coarse_ready && in->width_points)) {
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
        if (in->front_cm <= S5_STOP_CM) {
            c->encounter_front_cm = in->front_cm;
            return change(c, S5_WAIT_STOP, S5_CMD_STOP);
        }
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
            if (!ROBOT_OBSTACLE_TEST &&
                in->coarse.points[0].angle_deg == ROBOT_WIDTH_SCAN_MIN_DEG) {
                c->profile = in->width_points ? estimate_width_sweep(in->width_points, S5_WIDTH_SCAN_COUNT) :
                             s5_estimate_profile(&in->coarse, S5_SIDE_NONE);
                if (in->width_points) {
                    bool obstacle_seen = false;
                    for (int i = 0; i < S5_WIDTH_SCAN_COUNT; ++i) {
                        const S5ScanPoint *ray = &in->width_points[i];
                        if (valid_range(ray) && ray->distance_cm < S5_CLEAR_CM)
                            obstacle_seen = true;
                    }
                    if (!obstacle_seen)
                        return wait_for_route(c, S5_ERR_INSUFFICIENT_CLEARANCE);
                }
                if (c->profile.valid && c->profile.size == S5_SIZE_OVERSIZED)
                    return wait_for_route(c, S5_ERR_INSUFFICIENT_CLEARANCE);
                const S5ScanPoint *left = &in->coarse.points[0];
                const S5ScanPoint *right = &in->coarse.points[4];
                /* If the 45-degree ray is missing, a measured opening at
                 * 30 degrees can nominate a left look. Forward travel
                 * still requires clearance after the chassis turns. */
                if (!valid_range(left) && in->width_points) {
                    for (int i = 0; i < S5_WIDTH_SCAN_COUNT; ++i) {
                        const S5ScanPoint *ray = &in->width_points[i];
                        if (ray->angle_deg == ROBOT_WIDTH_ROUTE_LEFT_FALLBACK_DEG && safe_point(ray))
                            left = ray;
                    }
                }
                bool left_turn = valid_range(left) &&
                    s5_turn_range_cm(left->distance_cm, left->angle_deg) > ROBOT_BYPASS_MIN_TURN_CM;
                bool right_turn = valid_range(right) &&
                    s5_turn_range_cm(right->distance_cm, right->angle_deg) > ROBOT_BYPASS_MIN_TURN_CM;
                if (!left_turn && !right_turn)
                    return wait_for_route(c, S5_ERR_INSUFFICIENT_CLEARANCE);
                /* Use the tighter reading across each flank when both
                 * outward headings are candidates. */
                float left_room = left_turn ?
                    width_side_clearance(in->width_points, S5_SIDE_LEFT, left->distance_cm) : 0.0f;
                float right_room = right_turn ?
                    width_side_clearance(in->width_points, S5_SIDE_RIGHT, right->distance_cm) : 0.0f;
                c->profile.left_clearance_cm = left_room;
                c->profile.right_clearance_cm = right_room;
                c->side = right_turn && (!left_turn ||
                    right_room - left_room > ROBOT_BYPASS_SIDE_TIE_CM) ?
                    S5_SIDE_RIGHT : S5_SIDE_LEFT;
                c->profile.bypass_side = c->side;
                c->error = S5_ERR_NONE;
                return change(c, S5_AVOID,
                              c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
            }
            if (ROBOT_OBSTACLE_TEST) {
                if (valid_range(&in->coarse.points[2]) &&
                    c->encounter_front_cm > 0.0f &&
                    in->coarse.points[2].distance_cm < c->encounter_front_cm)
                    c->encounter_front_cm = in->coarse.points[2].distance_cm;
                S5Scan side_scan = in->coarse;
                if (ROBOT_TEST_BOUNDED_REVERSE && c->encounter_front_cm > 0.0f &&
                    c->encounter_front_cm <= ROBOT_BYPASS_FRONT_TURN_CM + 3.0f)
                    side_scan = retreat_candidate(side_scan);
                c->side = opening_side(&side_scan, side_range(&side_scan, S5_SIDE_LEFT),
                                       side_range(&side_scan, S5_SIDE_RIGHT));
                int nearest = in->coarse.nearest_index;
                /* Refine the candidate corridor, not the blocked opposite
                 * edge. Each sweep still includes the front at 90 degrees. */
                c->fine_center_deg = c->side == S5_SIDE_RIGHT ? 100u :
                    c->side == S5_SIDE_LEFT ? 80u :
                    nearest >= 0 && nearest < S5_SCAN_COUNT ?
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
            /* A missing outer echo can prevent sizing without preventing
             * a guarded viewpoint turn toward a fully measured side. */
            if (can_probe_partial_scan(&in->coarse, in))
                return wait_for_route(c, S5_ERR_NO_CLEAR_ROUTE);
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
            /* Commit the side BEFORE retreating. Narrow scans may lose the
             * frontal echo; retain the measured encounter gap instead of
             * interpreting a later assumed-open reading as obstacle removal.
             * Only measured side rays nominate a turn; execution still checks
             * live pivot clearance and the post-turn forward corridor. */
            if (ROBOT_OBSTACLE_TEST && c->encounter_front_cm > 0.0f &&
                c->encounter_front_cm < ROBOT_RELEASE_CM &&
                (!valid_range(&c->last_coarse.points[2]) ||
                 c->encounter_front_cm <= ROBOT_BYPASS_FRONT_TURN_CM + 3.0f)) {
                S5Scan candidate = c->last_coarse;
                bool retreat = ROBOT_TEST_BOUNDED_REVERSE &&
                    c->encounter_front_cm <= ROBOT_BYPASS_FRONT_TURN_CM + 3.0f;
                if (retreat) candidate = retreat_candidate(candidate);
                candidate.points[2] = (S5ScanPoint){90u,
                    c->encounter_front_cm + (retreat ? ROBOT_BYPASS_REVERSE_CM : 0.0f), true};
                S5Profile planned = profile_from_openings(&candidate);
                if (planned.valid) {
                    bool confirmed = false;
                    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
                        const S5ScanPoint *ray = &in->fine.points[i];
                        if (valid_range(ray) &&
                            ((planned.bypass_side == S5_SIDE_LEFT && ray->angle_deg < 90u) ||
                             (planned.bypass_side == S5_SIDE_RIGHT && ray->angle_deg > 90u)) &&
                            s5_turn_range_cm(ray->distance_cm + (retreat ?
                                ROBOT_BYPASS_REVERSE_CM * s5_cos_deg((float)ray->angle_deg - 90.0f) : 0.0f),
                                ray->angle_deg) >= ROBOT_BYPASS_SIDE_CLEAR_CM)
                            confirmed = true;
                    }
                    if (confirmed) {
                        c->profile = planned; c->side = planned.bypass_side;
                        c->error = S5_ERR_NONE;
                        if (retreat && c->reverse_attempts == 0u) {
                            ++c->reverse_attempts;
                            return change(c, S5_REVERSE, S5_CMD_REVERSE);
                        }
                        if (!retreat)
                            return change(c, S5_AVOID, c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
                    }
                }
            }
            if (ROBOT_OBSTACLE_TEST) {
                unsigned fine_hits = 0u;
                for (int i = 0; i < S5_SCAN_COUNT; ++i)
                    if (valid_range(&in->fine.points[i])) ++fine_hits;
                /* Require the obstacle-facing fine centre and a majority of
                 * fine returns. A missing ray on the unused side no longer
                 * vetoes an otherwise measured opening. */
                if (!valid_range(&c->last_coarse.points[2]) ||
                    !valid_range(&in->fine.points[2]) || fine_hits < 3u ||
                    s5_turn_range_cm(in->fine.points[2].distance_cm, in->fine.points[2].angle_deg) <= ROBOT_BYPASS_MIN_TURN_CM)
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
                bool coarse_valid = true;
                bool fine_valid = true;
                for (int i = 0; i < S5_SCAN_COUNT; ++i)
                {
                    if (!valid_range(&c->last_coarse.points[i])) coarse_valid = false;
                    if (!valid_range(&in->fine.points[i])) fine_valid = false;
                }
                if (!fine_valid || (!coarse_valid && !can_probe_partial_scan(&c->last_coarse, in)))
                    return fail(c, S5_ERR_FINE_SCAN);
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
        if (ROBOT_TEST_BOUNDED_REVERSE && c->profile.valid && in->stopped &&
            in->front_valid && in->front_cm > 0.0f &&
            in->front_cm <= ROBOT_BYPASS_FRONT_TURN_CM + 3.0f &&
            c->reverse_attempts == 0u) {
            ++c->reverse_attempts;
            return change(c, S5_REVERSE, S5_CMD_REVERSE);
        }
        if (!in->front_valid || !(in->front_cm > 0.0f) ||
            in->front_cm > S5_MAX_RANGE_CM) {
            c->clear_route_hits = 0u;
            if (++c->invalid_front_count >= S5_FRONT_INVALID_LIMIT)
                return fail(c, S5_ERR_FRONT_SENSOR);
            return S5_CMD_NONE;
        }
        c->invalid_front_count = 0u;
        /* A failed scan is not a permanent latch. Repositioning the robot or
         * obstacle by a meaningful amount starts a new scan pair. Never retry
         * repeatedly against the same blocked geometry or reuse stale sides. */
        if (ROBOT_OBSTACLE_TEST && c->error == S5_ERR_NO_CLEAR_ROUTE &&
            in->stopped && in->front_cm < ROBOT_RELEASE_CM &&
            in->front_cm > ROBOT_BYPASS_FRONT_TURN_CM + 3.0f &&
            valid_range(&c->last_coarse.points[2]) &&
            in->front_cm >= c->last_coarse.points[2].distance_cm + 5.0f) {
            if (++c->clear_route_hits >= ROBOT_CLEAR_CONFIRM) {
                c->clear_route_hits = 0u;
                c->error = S5_ERR_NONE;
                return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
            }
            return S5_CMD_NONE;
        }
        if (in->front_cm >= ROBOT_RELEASE_CM && in->stopped &&
            !(ROBOT_OBSTACLE_TEST && c->encounter_front_cm > 0.0f &&
              c->error != S5_ERR_NONE)) {
            if (++c->clear_route_hits >= ROBOT_CLEAR_CONFIRM) {
                c->error = S5_ERR_NONE;
                return change(c, S5_DONE, S5_CMD_RESUME);
            }
        } else c->clear_route_hits = 0u;
        if (!ROBOT_OBSTACLE_TEST && c->last_coarse.points[0].angle_deg != ROBOT_WIDTH_SCAN_MIN_DEG &&
            c->ticks_in_state >= S5_ROUTE_RESCAN_TICKS && in->stopped)
            return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
        return S5_CMD_NONE;
    case S5_REVERSE:
        if (in->motion_complete) {
            if (c->profile.valid && c->side != S5_SIDE_NONE)
                return change(c, S5_AVOID, c->side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT);
            return change(c, S5_COARSE, S5_CMD_SCAN_COARSE);
        }
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
