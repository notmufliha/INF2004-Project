#include "s5_maneuver.h"
#include <stddef.h>

static bool positive(float x)
{
    return x > 0.0f && x <= ROBOT_BYPASS_MAX_LEG_CM;
}
void s5_maneuver_init(S5Maneuver *m)
{
    if (m) *m = (S5Maneuver){0};
}
void s5_maneuver_abort(S5Maneuver *m, S5Error error)
{
    if (!m) return;
    m->state = S5_MAN_FAILED;
    m->error = error;
    m->issuing = false;
    m->pending_id = 0u;
}
bool s5_maneuver_begin(S5Maneuver *m, S5Command command,
                       const S5Profile *p, float depth, uint32_t now)
{
    float lateral;
    if (!m || (m->state != S5_MAN_IDLE && m->state != S5_MAN_COMPLETE))
        return false;
    s5_maneuver_init(m);
    m->start_ms = m->leg_ms = now;
    if (command == S5_CMD_REVERSE) {
        m->state = S5_MAN_BACK;
        return true;
    }
    if (!p || !p->valid ||
        (command != S5_CMD_GO_LEFT && command != S5_CMD_GO_RIGHT) ||
        !(p->estimated_width_cm > 0.0f) ||
        !(p->estimated_width_cm <= ROBOT_OBSTACLE_MAX_BYPASS_CM) ||
        !(p->center_x_cm >= -S5_MAX_RANGE_CM && p->center_x_cm <= S5_MAX_RANGE_CM) ||
        !positive(depth) || !positive(p->center_y_cm)) return false;
    m->side = command == S5_CMD_GO_LEFT ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
    if (p->bypass_side != m->side) return false;
    /* Recompute from width and geometry; do not trust a stale derived field. */
    lateral = p->estimated_width_cm * 0.5f + S5_CHASSIS_HALF_WIDTH_CM +
              S5_EXTRA_MARGIN_CM;
    m->lateral_cm = lateral + (m->side == S5_SIDE_LEFT ? -p->center_x_cm : p->center_x_cm);
    m->pass_cm = p->center_y_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM + depth +
                 ROBOT_CHASSIS_HALF_LENGTH_CM + S5_EXTRA_MARGIN_CM;
    m->search_limit_cm = m->lateral_cm + ROBOT_BYPASS_SEARCH_EXTRA_CM;
    if (!positive(m->lateral_cm) || !positive(m->pass_cm) ||
        !positive(m->search_limit_cm)) return false;
    m->state = S5_MAN_TURN_OUT;
    return true;
}
bool s5_maneuver_submitted(S5Maneuver *m, uint32_t id)
{
    if (!m || !m->issuing || !id) {
        s5_maneuver_abort(m, S5_ERR_MOTION_TIMEOUT);
        return false;
    }
    m->issuing = false;
    m->pending_id = id;
    return true;
}
static S5Action stop(S5Maneuver *m, S5Error e)
{
    s5_maneuver_abort(m, e);
    return (S5Action){S5_ACT_STOP, 0.0f, 0};
}
static void next_leg(S5Maneuver *m, S5ManeuverState state, uint32_t now)
{
    m->state = state;
    m->pending_id = 0u;
    m->leg_ms = now;
}
S5Action s5_maneuver_step(S5Maneuver *m, const S5ManeuverInput *in)
{
    S5Action a = {0};
    bool turning;
    int outward;
    if (!m || !in) return (S5Action){S5_ACT_STOP, 0.0f, 0};
    if (m->state == S5_MAN_FAILED) return (S5Action){S5_ACT_STOP, 0.0f, 0};
    if (m->state == S5_MAN_IDLE || m->state == S5_MAN_COMPLETE) return a;
    if (in->now_ms - m->start_ms >= ROBOT_BYPASS_TOTAL_TIMEOUT_MS ||
        in->now_ms - m->leg_ms >= ROBOT_BYPASS_LEG_TIMEOUT_MS)
        return stop(m, m->state == S5_MAN_SEARCH ? S5_ERR_LINE_TIMEOUT : S5_ERR_MOTION_TIMEOUT);
    /* Sample freshness, not task tick count: duplicate snapshots cannot
     * debounce a line, and stale pre-obstacle detection cannot end recovery. */
    if (!in->line_valid ||
        in->now_ms - in->line_sample_ms > ROBOT_BYPASS_LINE_MAX_AGE_MS)
        return stop(m, S5_ERR_LINE_TIMEOUT);
    if (!in->line_detected) m->left_original_line = true;
    if (m->state == S5_MAN_LINE_STOP) {
        if (in->stopped) next_leg(m, S5_MAN_ALIGN, in->now_ms);
        return (S5Action){S5_ACT_STOP, 0.0f, 0};
    }
    turning = m->state == S5_MAN_TURN_OUT || m->state == S5_MAN_FORWARD ||
              m->state == S5_MAN_TURN_IN || m->state == S5_MAN_ALIGN;
    if (!in->front_valid || !(in->front_cm > 0.0f) || in->front_cm > S5_MAX_RANGE_CM)
        return stop(m, S5_ERR_FRONT_SENSOR);
    if (in->front_cm <= (turning || m->state == S5_MAN_BACK ?
                        ROBOT_BYPASS_MIN_TURN_CM : ROBOT_STOP_CM))
        return stop(m, S5_ERR_INSUFFICIENT_CLEARANCE);
    if (m->state == S5_MAN_BACK && !in->rear_clear)
        return stop(m, S5_ERR_NO_CLEAR_ROUTE);
    if (m->state == S5_MAN_SEARCH && in->line_sample_ms != m->last_line_ms) {
        m->last_line_ms = in->line_sample_ms;
        if (m->left_original_line && in->line_detected && !in->line_junction &&
            m->searched_cm >= m->lateral_cm - ROBOT_BYPASS_LINE_WINDOW_CM &&
            m->searched_cm <= m->lateral_cm + ROBOT_BYPASS_LINE_WINDOW_CM)
            ++m->line_hits;
        else m->line_hits = 0u;
        if (m->line_hits >= ROBOT_BYPASS_LINE_CONFIRM) {
            next_leg(m, S5_MAN_LINE_STOP, in->now_ms);
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
    }
    if (m->issuing) return stop(m, S5_ERR_MOTION_TIMEOUT);
    if (m->pending_id) {
        if (!in->motion_terminal || in->result_id != m->pending_id) return a;
        if (!in->motion_success || !in->stopped)
            return stop(m, S5_ERR_MOTION_TIMEOUT);
        switch (m->state) {
        case S5_MAN_TURN_OUT: next_leg(m, S5_MAN_OFFSET, in->now_ms); break;
        case S5_MAN_OFFSET: next_leg(m, S5_MAN_FORWARD, in->now_ms); break;
        case S5_MAN_FORWARD: next_leg(m, S5_MAN_PASS, in->now_ms); break;
        case S5_MAN_PASS: next_leg(m, S5_MAN_TURN_IN, in->now_ms); break;
        case S5_MAN_TURN_IN: next_leg(m, S5_MAN_SEARCH, in->now_ms); break;
        case S5_MAN_SEARCH: next_leg(m, S5_MAN_SEARCH, in->now_ms); break;
        case S5_MAN_ALIGN:
            if (!in->line_detected || in->line_junction) return stop(m, S5_ERR_LINE_TIMEOUT);
            next_leg(m, S5_MAN_COMPLETE, in->now_ms);
            break;
        case S5_MAN_BACK: next_leg(m, S5_MAN_COMPLETE, in->now_ms); break;
        default: return stop(m, S5_ERR_MOTION_TIMEOUT);
        }
        return a; /* Recheck the sensor in the new heading before next leg. */
    }
    if (!in->stopped) return stop(m, S5_ERR_STOP_TIMEOUT);
    outward = m->side == S5_SIDE_LEFT ? -90 : 90;
    switch (m->state) {
    case S5_MAN_TURN_OUT:
    case S5_MAN_ALIGN: a = (S5Action){S5_ACT_TURN, 0.0f, outward}; break;
    case S5_MAN_FORWARD:
    case S5_MAN_TURN_IN: a = (S5Action){S5_ACT_TURN, 0.0f, -outward}; break;
    case S5_MAN_OFFSET: a = (S5Action){S5_ACT_MOVE, m->lateral_cm, 0}; break;
    case S5_MAN_PASS: a = (S5Action){S5_ACT_MOVE, m->pass_cm, 0}; break;
    case S5_MAN_BACK: a = (S5Action){S5_ACT_MOVE, -ROBOT_BYPASS_REVERSE_CM, 0}; break;
    case S5_MAN_SEARCH:
        if (m->searched_cm >= m->search_limit_cm) return stop(m, S5_ERR_LINE_TIMEOUT);
        a = (S5Action){S5_ACT_MOVE, ROBOT_BYPASS_SEARCH_STEP_CM, 0};
        if (a.distance_cm > m->search_limit_cm - m->searched_cm)
            a.distance_cm = m->search_limit_cm - m->searched_cm;
        m->searched_cm += a.distance_cm;
        break;
    default: return stop(m, S5_ERR_MOTION_TIMEOUT);
    }
    m->issuing = true;
    return a;
}
