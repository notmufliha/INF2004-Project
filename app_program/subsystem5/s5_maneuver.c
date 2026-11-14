#include "s5_maneuver.h"
#include <stddef.h>
#include <math.h>

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
    m->waiting_clear = false;
    m->clear_hits = 0u;
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
    lateral = p->estimated_width_cm * (p->uses_course_bounds ? 1.0f : 0.5f) + S5_CHASSIS_HALF_WIDTH_CM +
              S5_EXTRA_MARGIN_CM;
    m->lateral_cm = lateral + (m->side == S5_SIDE_LEFT ? -p->center_x_cm : p->center_x_cm);
    float angle = ROBOT_BYPASS_TURN_DEG * 0.01745329252f;
    m->offset_cm = m->lateral_cm / sinf(angle);
    m->pass_cm = p->center_y_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM + depth +
                 ROBOT_CHASSIS_HALF_LENGTH_CM + S5_EXTRA_MARGIN_CM - m->offset_cm * cosf(angle);
    /* Count the diagonal's forward projection before the parallel leg;
     * retain a short straight
     * segment when the diagonal already clears the configured depth bound. */
    if (m->pass_cm < 5.0f) m->pass_cm = 5.0f;
    m->search_limit_cm = m->offset_cm + (ROBOT_OBSTACLE_TEST ? 0.0f : ROBOT_BYPASS_SEARCH_EXTRA_CM);
    if (!positive(m->lateral_cm) || !positive(m->offset_cm) || !positive(m->pass_cm) ||
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
    m->waiting_clear = false;
    m->clear_hits = 0u;
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
        return stop(m, m->state == S5_MAN_SEARCH || m->state == S5_MAN_ALIGN_CHECK ?
                    S5_ERR_LINE_TIMEOUT : S5_ERR_MOTION_TIMEOUT);
    /* Sample freshness, not task tick count: duplicate snapshots cannot
     * debounce a line, and stale pre-obstacle detection cannot end recovery. */
    if (!in->line_valid ||
        in->now_ms - in->line_sample_ms > ROBOT_BYPASS_LINE_MAX_AGE_MS)
        return stop(m, S5_ERR_LINE_TIMEOUT);
    if (!in->line_detected) m->left_original_line = true;
    if (m->state == S5_MAN_LINE_STOP) {
        /* Catch the tape on first contact, then confirm while stationary.
         * Waiting for three ultrasonic-paced ticks while still driving can
         * carry the sensors completely across a narrow line. */
        if (!in->stopped) {
            m->line_hits = 0u;
            m->last_line_ms = in->line_sample_ms;
        } else if (in->line_sample_ms != m->last_line_ms) {
            m->last_line_ms = in->line_sample_ms;
            if (!in->line_detected) {
                m->line_hits = 0u;
                next_leg(m, S5_MAN_SEARCH, in->now_ms);
            } else if (++m->line_hits >= ROBOT_BYPASS_LINE_CONFIRM) {
                next_leg(m, S5_MAN_ALIGN, in->now_ms);
            }
        }
        return (S5Action){S5_ACT_STOP, 0.0f, 0};
    }
    turning = m->state == S5_MAN_TURN_OUT || m->state == S5_MAN_FORWARD ||
              m->state == S5_MAN_TURN_IN || m->state == S5_MAN_ALIGN ||
              m->state == S5_MAN_ALIGN_CHECK;
    if (!in->front_valid || !(in->front_cm > 0.0f) || in->front_cm > S5_MAX_RANGE_CM)
        return stop(m, S5_ERR_FRONT_SENSOR);
    /* The first post-turn echo may still see the old obstacle/beam edge.
     * Before starting a straight leg, hold STOP and confirm fresh clearance
     * instead of permanently faulting. Active motion retains immediate STOP. */
    if ((m->state == S5_MAN_OFFSET || m->state == S5_MAN_PASS || m->state == S5_MAN_SEARCH) &&
        !m->pending_id && !m->issuing && in->stopped &&
        (m->waiting_clear || in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM)) {
        m->waiting_clear = true;
        if (in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM) m->clear_hits = 0u;
        else if (in->now_ms != m->last_clear_sample_ms) ++m->clear_hits;
        m->last_clear_sample_ms = in->now_ms;
        if (m->clear_hits < ROBOT_CLEAR_CONFIRM)
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        m->waiting_clear = false;
    }
    /* Reversing increases front clearance; the turn envelope applies to turns.
     * Rear clearance is checked separately throughout the reverse leg. */
    if (m->state != S5_MAN_BACK && in->front_cm <= (turning ?
                        ROBOT_BYPASS_MIN_TURN_CM : ROBOT_BYPASS_MOVE_STOP_CM))
        return stop(m, S5_ERR_INSUFFICIENT_CLEARANCE);
    if (m->state == S5_MAN_BACK && !in->rear_clear)
        return stop(m, S5_ERR_NO_CLEAR_ROUTE);
    if (!ROBOT_OBSTACLE_TEST && (m->state == S5_MAN_SEARCH ||
         (m->state == S5_MAN_ALIGN_CHECK && !m->pending_id && !m->issuing && in->stopped)) &&
        in->line_sample_ms != m->last_line_ms) {
        m->last_line_ms = in->line_sample_ms;
        /* The line can curve away from its predicted crossing. Once we
         * have passed the obstacle and left the old line, accept fresh
         * black detections anywhere within the bounded search corridor.
         * Both sensors on black can look like a junction: that is still
         * a valid reacquisition, not a reason to drive over the tape. */
        if (m->left_original_line && in->line_detected)
            ++m->line_hits;
        else m->line_hits = 0u;
        if (m->line_hits >= (m->state == S5_MAN_SEARCH ? 1u : ROBOT_BYPASS_LINE_CONFIRM)) {
            if (m->state == S5_MAN_ALIGN_CHECK) {
                next_leg(m, S5_MAN_COMPLETE, in->now_ms);
                return a; /* Already physically stopped and facing forward. */
            }
            next_leg(m, S5_MAN_LINE_STOP, in->now_ms);
            m->line_hits = 0u;
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
        case S5_MAN_OFFSET:
            m->offset_done_cm += m->active_move_cm;
            next_leg(m, m->offset_done_cm + 0.01f >= m->offset_cm ? S5_MAN_FORWARD : S5_MAN_OFFSET, in->now_ms);
            break;
        case S5_MAN_FORWARD: next_leg(m, S5_MAN_PASS, in->now_ms); break;
        case S5_MAN_PASS:
            m->pass_done_cm += m->active_move_cm;
            next_leg(m, m->pass_done_cm + 0.01f >= m->pass_cm ? S5_MAN_TURN_IN : S5_MAN_PASS, in->now_ms);
            break;
        case S5_MAN_TURN_IN: next_leg(m, S5_MAN_SEARCH, in->now_ms); break;
        case S5_MAN_SEARCH:
            if (ROBOT_OBSTACLE_TEST) m->searched_cm += m->active_move_cm;
            next_leg(m, S5_MAN_SEARCH, in->now_ms); break;
        case S5_MAN_ALIGN:
        case S5_MAN_ALIGN_CHECK:
            /* A sensor mounted ahead of the axle can rotate off the line.
             * Reconfirm it while stopped; do not fault on one rotated view
             * or hand DRIVE to the VC before line acquisition is stable. */
            next_leg(m, S5_MAN_ALIGN_CHECK, in->now_ms);
            m->line_hits = 0u;
            m->last_line_ms = in->line_sample_ms;
            break;
        case S5_MAN_BACK: next_leg(m, S5_MAN_COMPLETE, in->now_ms); break;
        default: return stop(m, S5_ERR_MOTION_TIMEOUT);
        }
        return a; /* Recheck the sensor in the new heading before next leg. */
    }
    if (!in->stopped) return stop(m, S5_ERR_STOP_TIMEOUT);
    outward = m->side == S5_SIDE_LEFT ? -ROBOT_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
    switch (m->state) {
    case S5_MAN_OFFSET: a = (S5Action){S5_ACT_MOVE, m->offset_cm - m->offset_done_cm, 0}; break;
    case S5_MAN_TURN_OUT:
    case S5_MAN_ALIGN: a = (S5Action){S5_ACT_TURN, 0.0f, outward}; break;
    case S5_MAN_FORWARD:
    case S5_MAN_TURN_IN: a = (S5Action){S5_ACT_TURN, 0.0f, -outward}; break;
    case S5_MAN_PASS: a = (S5Action){S5_ACT_MOVE, m->pass_cm - m->pass_done_cm, 0}; break;
    case S5_MAN_BACK: a = (S5Action){S5_ACT_MOVE, -ROBOT_BYPASS_REVERSE_CM, 0}; break;
    case S5_MAN_ALIGN_CHECK: {
        int heading;
        int toward_line = m->side == S5_SIDE_RIGHT ? -1 : 1;
        if (in->now_ms - m->leg_ms < ROBOT_BYPASS_ALIGN_OBSERVE_MS) return a;
        if (ROBOT_CLEAR_PATH_FORWARD && m->alignment_heading_deg == 0) {
            next_leg(m, S5_MAN_COMPLETE, in->now_ms);
            return a; /* Facing forward: VC continues straight until tape appears. */
        }
        if (m->alignment_step >= ROBOT_BYPASS_ALIGN_SWEEPS)
            return stop(m, S5_ERR_LINE_TIMEOUT);
        heading = (int)(m->alignment_step / 2u + 1u) * ROBOT_BYPASS_ALIGN_STEP_DEG;
        heading *= (m->alignment_step % 2u == 0u ? toward_line : -toward_line);
        a = (S5Action){S5_ACT_TURN, 0.0f, heading - m->alignment_heading_deg};
        m->alignment_heading_deg = heading;
        ++m->alignment_step;
        m->line_hits = 0u;
        m->last_line_ms = in->line_sample_ms;
        break;
    }
    case S5_MAN_SEARCH:
        if (m->searched_cm >= m->search_limit_cm) {
            if (!ROBOT_CLEAR_PATH_FORWARD) return stop(m, S5_ERR_LINE_TIMEOUT);
            next_leg(m, S5_MAN_ALIGN, in->now_ms);
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
        a = (S5Action){S5_ACT_MOVE, ROBOT_OBSTACLE_TEST ?
            m->search_limit_cm - m->searched_cm : ROBOT_BYPASS_SEARCH_STEP_CM, 0};
        if (a.distance_cm > m->search_limit_cm - m->searched_cm)
            a.distance_cm = m->search_limit_cm - m->searched_cm;
        if (!ROBOT_OBSTACLE_TEST) m->searched_cm += a.distance_cm;
        break;
    default: return stop(m, S5_ERR_MOTION_TIMEOUT);
    }
    /* After a turn, a clear short section is enough to begin advancing.
     * If the entire planned leg is not visible, take a bounded encoder
     * step, stop, and remeasure. Only completed travel counts toward the
     * width/depth target; live front sensing still guards each movement. */
    if (a.type == S5_ACT_MOVE && a.distance_cm > 0.0f &&
        in->front_cm <= a.distance_cm + ROBOT_CHASSIS_HALF_LENGTH_CM + S5_EXTRA_MARGIN_CM) {
        float available = in->front_cm - ROBOT_CHASSIS_HALF_LENGTH_CM - S5_EXTRA_MARGIN_CM - 1.0f;
        if (ROBOT_OBSTACLE_TEST) {
            float guarded = in->front_cm - ROBOT_BYPASS_MOVE_STOP_CM - 3.0f;
            if (available > guarded) available = guarded;
        }
        if (!(available > 0.0f)) return stop(m, S5_ERR_INSUFFICIENT_CLEARANCE);
        if (!ROBOT_OBSTACLE_TEST && a.distance_cm > 5.0f) a.distance_cm = 5.0f;
        if (a.distance_cm > available) a.distance_cm = available;
    }
    if (a.type == S5_ACT_MOVE) m->active_move_cm = a.distance_cm;
    m->issuing = true;
    return a;
}
