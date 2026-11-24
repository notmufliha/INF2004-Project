#include "s5_maneuver.h"
#include <stddef.h>
#include <math.h>

static bool positive(float x)
{
    return x > 0.0f && x <= ROBOT_BYPASS_MAX_LEG_CM;
}
static float required_lateral_cm(const S5Profile *p, S5Side side)
{
    float width_scale = p->uses_course_bounds && !ROBOT_BYPASS_CENTERED_TEST_BOX ? 1.0f : 0.5f;
    return p->estimated_width_cm * width_scale + S5_CHASSIS_HALF_WIDTH_CM +
           S5_EXTRA_MARGIN_CM + (side == S5_SIDE_LEFT ? -p->center_x_cm : p->center_x_cm);
}

static bool course_has_room(float lateral_cm, float front_cm)
{
    float angle = ROBOT_COURSE_BYPASS_TURN_DEG * 0.01745329252f;
    float forward_room = front_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM -
                         ROBOT_CHASSIS_HALF_LENGTH_CM - S5_EXTRA_MARGIN_CM;
    return forward_room * sinf(angle) >= lateral_cm * cosf(angle);
}

bool s5_maneuver_course_room(const S5Profile *profile, S5Side side, float obstacle_front_cm)
{
    return course_has_room(required_lateral_cm(profile, side), obstacle_front_cm);
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
    m->lateral_cm = required_lateral_cm(p, m->side);
    float angle = ROBOT_BYPASS_TURN_DEG * 0.01745329252f;
    m->offset_cm = m->lateral_cm / sinf(angle);
    m->pass_cm = p->center_y_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM + depth +
                 ROBOT_CHASSIS_HALF_LENGTH_CM + S5_EXTRA_MARGIN_CM - m->offset_cm * cosf(angle);
    m->reactive_required_cm = p->center_y_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM + depth +
                             ROBOT_CHASSIS_HALF_LENGTH_CM + S5_EXTRA_MARGIN_CM;
    /* Count the diagonal's forward projection before the parallel leg;
     * retain a short straight
     * segment when the diagonal already clears the configured depth bound. */
    if (m->pass_cm < 5.0f) m->pass_cm = 5.0f;
    if (ROBOT_BYPASS_EXACT_SEQUENCE && m->pass_cm > ROBOT_BYPASS_MIDDLE_MAX_CM)
        m->pass_cm = ROBOT_BYPASS_MIDDLE_MAX_CM;
    if (ROBOT_BYPASS_EXACT_SEQUENCE) {
        /* Outward diagonal, parallel passing leg, inward diagonal.
         * Keep forward travel between the two opposite turns. */
        m->minimum_offset_cm = m->offset_cm;
        m->offset_cm += ROBOT_BYPASS_EXTRA_DIAGONAL_CM;
        m->offset_cm *= ROBOT_BYPASS_DIAGONAL_DISTANCE_SCALE;
        m->pass_cm = (m->reactive_required_cm - m->offset_cm * cosf(angle)) * ROBOT_BYPASS_MIDDLE_DISTANCE_SCALE;
        if (m->pass_cm < 5.0f) m->pass_cm = 5.0f;
        if (ROBOT_BYPASS_EXACT_SEQUENCE && m->pass_cm > ROBOT_BYPASS_MIDDLE_MAX_CM)
            m->pass_cm = ROBOT_BYPASS_MIDDLE_MAX_CM;
    }
    m->search_limit_cm = m->offset_cm + (ROBOT_OBSTACLE_TEST ? 0.0f : ROBOT_BYPASS_SEARCH_EXTRA_CM);
    if (!positive(m->lateral_cm) || !positive(m->offset_cm) || !positive(m->pass_cm) ||
        !positive(m->search_limit_cm)) return false;
    m->state = S5_MAN_TURN_OUT;
    return true;
}

bool s5_maneuver_plan_equal_legs(S5Maneuver *m, float obstacle_front_cm)
{
    if (!course_has_room(m->lateral_cm, obstacle_front_cm)) return false;
    m->offset_cm = m->search_limit_cm = ROBOT_COURSE_BYPASS_LEG_CM;
    m->pass_cm = ROBOT_COURSE_PASS_CM;
    m->equal_leg_route = true;
    return true;
}
bool s5_maneuver_plan_rejoin(S5Maneuver *m, const S5Profile *p,
                             float obstacle_front_cm, int turn_hint)
{
    float width, toward, lateral;
    if (!m || !p || m->state != S5_MAN_TURN_OUT ||
        (m->side != S5_SIDE_LEFT && m->side != S5_SIDE_RIGHT) ||
        !(obstacle_front_cm > 0.0f) || obstacle_front_cm > S5_MAX_RANGE_CM)
        return false;
    width = p->estimated_width_cm;
    if (!(width >= ROBOT_REJOIN_MIN_PLAN_WIDTH_CM)) width = ROBOT_REJOIN_MIN_PLAN_WIDTH_CM;
    /* Obstacle centre shifted toward the bypass side needs more offset. */
    toward = m->side == S5_SIDE_LEFT ? -p->center_x_cm : p->center_x_cm;
    if (!(toward > -width) || !(toward < width)) toward = 0.0f;
    lateral = width * 0.5f + toward + ROBOT_CHASSIS_HALF_WIDTH_CM +
              ROBOT_REJOIN_SIDE_MARGIN_CM;
    if (lateral < ROBOT_CHASSIS_HALF_WIDTH_CM + ROBOT_REJOIN_SIDE_MARGIN_CM)
        lateral = ROBOT_CHASSIS_HALF_WIDTH_CM + ROBOT_REJOIN_SIDE_MARGIN_CM;
    if (lateral > ROBOT_REJOIN_MAX_LATERAL_CM) return false;
    m->rejoin_route = true;
    m->equal_leg_route = m->guarded_probe_route = false;
    m->turn_hint = turn_hint < 0 ? -1 : turn_hint > 0 ? 1 : 0;
    m->lateral_cm = m->offset_cm = lateral;
    m->minimum_offset_cm = 0.0f;
    /* Axle travel until the rear clears the far edge of the box. */
    m->pass_needed_cm = obstacle_front_cm + ROBOT_SENSOR_FORWARD_OFFSET_CM +
        ROBOT_COURSE_BOX_DEPTH_CM + ROBOT_CHASSIS_REAR_CM + ROBOT_REJOIN_PAST_MARGIN_CM;
    m->pass_cm = m->pass_needed_cm + ROBOT_REJOIN_LOOKAHEAD_CM;
    m->search_limit_cm = lateral + ROBOT_REJOIN_SEARCH_EXTRA_CM;
    m->offset_done_cm = m->pass_done_cm = m->searched_cm = 0.0f;
    m->rejoin_step = 0;
    m->crossing_seen = m->turn_done = m->blocked = false;
    m->final_turn_deg = 0;
    return positive(m->offset_cm) && positive(m->pass_cm) && positive(m->search_limit_cm);
}
bool s5_maneuver_plan_probe_route(S5Maneuver *m, float obstacle_front_cm)
{
    if (obstacle_front_cm <= ROBOT_BYPASS_FRONT_TURN_CM) return false;
    /* Use the same guarded distance targets as the course route. */
    m->offset_cm = m->search_limit_cm = ROBOT_COURSE_BYPASS_LEG_CM;
    m->pass_cm = ROBOT_COURSE_PASS_CM;
    m->equal_leg_route = true;
    m->guarded_probe_route = true;
    m->state = S5_MAN_OFFSET;
    m->outward_turn_complete = true;
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
    if (state == S5_MAN_TURN_IN && m->equal_leg_route) {
        const float radians_per_degree = 0.01745329252f;
        const int pass_heading = ROBOT_COURSE_BYPASS_TURN_DEG - ROBOT_COURSE_PASS_TURN_DEG;
        const int return_heading = ROBOT_COURSE_RETURN_TURN_DEG - pass_heading;
        const float lateral_cm = m->offset_done_cm *
            sinf(ROBOT_COURSE_BYPASS_TURN_DEG * radians_per_degree) +
            m->pass_done_cm * sinf(pass_heading * radians_per_degree);
        m->search_limit_cm = lateral_cm / sinf(return_heading * radians_per_degree);
    }
}
/* ---- Rejoin route (see ROBOT_REJOIN_ROUTE in robot_config.h) ---------- */
#define REJOIN_LINE_HITS 2u
static int rejoin_outward(const S5Maneuver *m)
{
    return m->side == S5_SIDE_LEFT ? -ROBOT_REJOIN_TURN_DEG : ROBOT_REJOIN_TURN_DEG;
}
static float *rejoin_counter(S5Maneuver *m)
{
    if (m->state == S5_MAN_OFFSET) return &m->offset_done_cm;
    if (m->state == S5_MAN_PASS) return &m->pass_done_cm;
    if (m->state == S5_MAN_SEARCH) return &m->searched_cm;
    return &m->active_move_cm; /* ALIGN advance: unused total */
}
static float rejoin_target(const S5Maneuver *m)
{
    if (m->state == S5_MAN_OFFSET) return m->offset_cm;
    if (m->state == S5_MAN_PASS) return m->pass_cm;
    if (m->state == S5_MAN_SEARCH) return m->search_limit_cm;
    return ROBOT_REJOIN_TAPE_ADVANCE_CM;
}
static void rejoin_leg(S5Maneuver *m, S5ManeuverState state, int step, uint32_t now)
{
    next_leg(m, state, now);
    m->rejoin_step = step;
    m->blocked = m->stopping = m->stop_for_line = false;
    m->line_hits = 0u;
}
/* Tape found on the pass leg (a crossing) or the return leg (the line). */
static void rejoin_line_event(S5Maneuver *m, uint32_t now)
{
    int outward = rejoin_outward(m);
    int side_sign = m->side == S5_SIDE_LEFT ? -1 : 1;
    if (m->state == S5_MAN_SEARCH) {
        m->final_turn_deg = outward; /* back to the original heading */
        rejoin_leg(m, S5_MAN_ALIGN, 0, now);
        return;
    }
    m->crossing_seen = true;
    if (m->turn_hint != 0) {
        /* Turn onto the crossing tape in the barcode's direction. */
        m->final_turn_deg = m->turn_hint == side_sign ? outward : -outward;
        m->turn_done = true;
        rejoin_leg(m, S5_MAN_ALIGN, 0, now);
        return;
    }
    /* No turn armed: cross this tape, then return to the original line. */
    float extended = m->pass_done_cm + ROBOT_REJOIN_CROSS_EXTRA_CM;
    if (extended < m->pass_needed_cm) extended = m->pass_needed_cm;
    m->pass_cm = extended;
}
static S5Action rejoin_step(S5Maneuver *m, const S5ManeuverInput *in)
{
    const S5Action none = {S5_ACT_NONE, 0.0f, 0};
    const S5Action halt = {S5_ACT_STOP, 0.0f, 0};
    bool fresh_line = in->line_sample_ms != m->last_line_ms;
    bool front_ok = in->front_valid && in->front_cm > 0.0f && in->front_cm <= S5_MAX_RANGE_CM;
    bool moving_state = m->state == S5_MAN_OFFSET || m->state == S5_MAN_PASS ||
        m->state == S5_MAN_SEARCH || (m->state == S5_MAN_ALIGN && m->rejoin_step == 0);
    bool watch_line = m->state == S5_MAN_SEARCH ||
        (m->state == S5_MAN_PASS && !m->crossing_seen);
    int outward = rejoin_outward(m);

    /* A command the adapter could not submit (B2 still acknowledging the
     * last STOP) is simply asked for again. */
    m->issuing = false;
    if (fresh_line) m->last_line_ms = in->line_sample_ms;

    if (m->pending_id) {
        if (!in->motion_terminal || in->result_id != m->pending_id || !in->stopped) {
            if (moving_state && !m->stopping) {
                if (watch_line && fresh_line) {
                    if (in->line_detected) ++m->line_hits;
                    else m->line_hits = 0u;
                    if (m->line_hits >= REJOIN_LINE_HITS) m->stopping = m->stop_for_line = true;
                }
                if (!m->stopping && front_ok && in->front_cm <= ROBOT_BYPASS_MOVE_STOP_CM) {
                    m->stopping = m->blocked = true;
                    m->blocked_since_ms = in->now_ms;
                    m->clear_hits = 0u;
                }
            }
            return m->stopping ? halt : none;
        }
        if (!in->motion_success && !in->motion_cancelled)
            return stop(m, S5_ERR_MOTION_TIMEOUT);
        if (moving_state) {
            float done = in->motion_progress_cm;
            if (!(done > 0.0f)) done = in->motion_success ? m->active_move_cm : 0.0f;
            if (done > m->active_move_cm) done = m->active_move_cm;
            *rejoin_counter(m) += done;
        }
        m->pending_id = 0u;
        m->stopping = false;
        if (m->stop_for_line) {
            m->stop_for_line = false;
            m->line_hits = 0u;
            rejoin_line_event(m, in->now_ms);
            return none;
        }
        if (!moving_state) {
            /* A pivot finished. */
            switch (m->state) {
            case S5_MAN_TURN_OUT: m->outward_turn_complete = false;
                                  rejoin_leg(m, S5_MAN_OFFSET, 0, in->now_ms); break;
            case S5_MAN_FORWARD:  rejoin_leg(m, S5_MAN_PASS, 0, in->now_ms); break;
            case S5_MAN_TURN_IN:  rejoin_leg(m, S5_MAN_SEARCH, 0, in->now_ms); break;
            case S5_MAN_ALIGN:    rejoin_leg(m, S5_MAN_COMPLETE, 0, in->now_ms); break;
            default: return stop(m, S5_ERR_MOTION_TIMEOUT);
            }
            return none;
        }
        if (m->state == S5_MAN_ALIGN) { /* advance finished: final pivot */
            m->rejoin_step = 1;
            return none;
        }
    }

    if (!moving_state) {
        int turn;
        if (!in->stopped) return halt;
        switch (m->state) {
        case S5_MAN_TURN_OUT: turn = outward; break;
        case S5_MAN_FORWARD:
        case S5_MAN_TURN_IN:  turn = -outward; break;
        case S5_MAN_ALIGN:    turn = m->final_turn_deg; break;
        default: return stop(m, S5_ERR_MOTION_TIMEOUT);
        }
        m->active_move_cm = 0.0f;
        m->issuing = true;
        return (S5Action){S5_ACT_TURN, 0.0f, turn};
    }

    float remaining = rejoin_target(m) - (m->state == S5_MAN_ALIGN ? 0.0f : *rejoin_counter(m));
    if (m->blocked) {
        if (!front_ok || in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM) {
            m->clear_hits = 0u;
            if (in->now_ms - m->blocked_since_ms >= ROBOT_REJOIN_BLOCKED_MS)
                return stop(m, S5_ERR_NO_CLEAR_ROUTE);
            return halt;
        }
        if (in->now_ms != m->last_clear_sample_ms) ++m->clear_hits;
        m->last_clear_sample_ms = in->now_ms;
        if (m->clear_hits < ROBOT_CLEAR_CONFIRM) return halt;
        m->blocked = false;
        m->clear_hits = 0u;
    }
    if (m->state != S5_MAN_ALIGN && remaining <= 0.05f) {
        switch (m->state) {
        case S5_MAN_OFFSET: rejoin_leg(m, S5_MAN_FORWARD, 0, in->now_ms); return none;
        /* The bridge holds TURN_IN until the controller asks for FIND_LINE. */
        case S5_MAN_PASS:   rejoin_leg(m, S5_MAN_TURN_IN, 0, in->now_ms); return none;
        default:            return stop(m, S5_ERR_LINE_TIMEOUT); /* no line */
        }
    }
    if (!in->stopped) return halt;
    if (front_ok && in->front_cm <= ROBOT_BYPASS_MOVE_STOP_CM) {
        m->blocked = true;
        m->blocked_since_ms = in->now_ms;
        m->clear_hits = 0u;
        return halt;
    }
    m->line_hits = 0u;
    m->active_move_cm = remaining;
    m->issuing = true;
    return (S5Action){S5_ACT_MOVE, remaining, 0};
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
    if (m->rejoin_route) return rejoin_step(m, in);
    if (!in->line_detected) m->left_original_line = true;
    if (ROBOT_BYPASS_LINE_HANDOFF && m->outward_turn_complete &&
        !m->equal_leg_route &&
        m->left_original_line && in->line_sample_ms != m->last_line_ms) {
        m->last_line_ms = in->line_sample_ms;
        if (in->line_detected) {
            if (m->state != S5_MAN_HANDOFF_STOP) {
                next_leg(m, S5_MAN_HANDOFF_STOP, in->now_ms);
                m->issuing = false;
                m->line_hits = 0u;
            } else if (in->stopped && ++m->line_hits >= ROBOT_BYPASS_LINE_CONFIRM) {
                next_leg(m, S5_MAN_COMPLETE, in->now_ms);
            }
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
        if (m->state == S5_MAN_HANDOFF_STOP) {
            m->line_hits = 0u;
            /* Let the bounded search find the tape again after overshoot. */
            if (in->stopped) next_leg(m, S5_MAN_SEARCH, in->now_ms);
        }
    }
    if (m->state == S5_MAN_HANDOFF_STOP)
        return (S5Action){S5_ACT_STOP, 0.0f, 0};

    bool front_ok = in->front_valid && in->front_cm > 0.0f && in->front_cm <= S5_MAX_RANGE_CM;
    bool translating = m->state == S5_MAN_OFFSET || m->state == S5_MAN_PASS || m->state == S5_MAN_SEARCH;
    if (ROBOT_BYPASS_EXACT_SEQUENCE && translating) {
        if (m->pending_id && (!front_ok || in->front_cm <= ROBOT_BYPASS_MOVE_STOP_CM)) {
            float progress = in->motion_progress_cm;
            if (!(progress > 0.0f)) progress = 0.0f;
            if (progress > m->active_move_cm) progress = m->active_move_cm;
            if (m->state == S5_MAN_OFFSET) m->offset_done_cm += progress;
            else if (m->state == S5_MAN_PASS) m->pass_done_cm += progress;
            else if (m->state == S5_MAN_SEARCH) m->searched_cm += progress;
            m->pending_id = 0u; m->issuing = false;
            m->forward_paused = true; m->clear_hits = 0u;
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
        /* Advance the fixed test sequence on a blocked first leg after its
         * configured minimum progress, with fresh rotation clearance. This
         * is not a claim that all lateral box clearance has been measured. */
        if (m->state == S5_MAN_OFFSET && !m->pending_id && in->stopped && front_ok &&
            in->front_cm > ROBOT_BYPASS_FRONT_TURN_CM &&
            (m->forward_paused || in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM) &&
            m->minimum_offset_cm > 0.0f &&
            (m->offset_done_cm + 0.01f >= m->minimum_offset_cm ||
             m->offset_done_cm + 0.01f >= ROBOT_BYPASS_BLOCKED_FIRST_MIN_CM)) {
            if (in->now_ms != m->last_turn_clear_ms) ++m->turn_clear_hits;
            m->last_turn_clear_ms = in->now_ms;
            if (m->turn_clear_hits < ROBOT_CLEAR_CONFIRM) return (S5Action){S5_ACT_STOP, 0.0f, 0};
            float angle = ROBOT_BYPASS_TURN_DEG * 0.01745329252f;
            m->offset_cm = m->offset_done_cm;
            m->search_limit_cm = m->offset_cm;
            m->pass_cm = (m->reactive_required_cm - m->offset_cm * cosf(angle)) * ROBOT_BYPASS_MIDDLE_DISTANCE_SCALE;
            if (m->pass_cm < 5.0f) m->pass_cm = 5.0f;
            if (ROBOT_BYPASS_EXACT_SEQUENCE && m->pass_cm > ROBOT_BYPASS_MIDDLE_MAX_CM)
                m->pass_cm = ROBOT_BYPASS_MIDDLE_MAX_CM;
            m->forward_paused = false; m->waiting_clear = false;
            next_leg(m, S5_MAN_FORWARD, in->now_ms);
            return a;
        }
        else m->turn_clear_hits = 0u;
        if (m->forward_paused) {
            if (!front_ok) {
                if (!m->invalid_since_ms) m->invalid_since_ms = in->now_ms;
                if (in->now_ms - m->invalid_since_ms >= 5000u) return stop(m, S5_ERR_FRONT_SENSOR);
            } else m->invalid_since_ms = 0u;
            if (!in->stopped || !front_ok || in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM) {
                m->clear_hits = 0u;
                return (S5Action){S5_ACT_STOP, 0.0f, 0};
            }
            if (in->now_ms != m->last_clear_sample_ms) ++m->clear_hits;
            m->last_clear_sample_ms = in->now_ms;
            if (m->clear_hits < ROBOT_CLEAR_CONFIRM) return a;
            m->forward_paused = false; m->clear_hits = 0u;
            if (m->state == S5_MAN_OFFSET && m->offset_done_cm + 0.01f >= m->offset_cm) {
                next_leg(m, S5_MAN_FORWARD, in->now_ms); return a;
            }
            if (m->state == S5_MAN_PASS && m->pass_done_cm + 0.01f >= m->pass_cm) {
                next_leg(m, S5_MAN_TURN_IN, in->now_ms); return a;
            }
            if (m->state == S5_MAN_SEARCH && m->searched_cm + 0.01f >= m->search_limit_cm) {
                next_leg(m, S5_MAN_ALIGN, in->now_ms); return a;
            }
        }
    }
    if (ROBOT_OBSTACLE_TEST && !ROBOT_BYPASS_EXACT_SEQUENCE && !m->reactive && translating &&
        (!front_ok || in->front_cm <= ROBOT_BYPASS_MOVE_STOP_CM ||
         (in->stopped && !m->pending_id && in->front_cm < ROBOT_BYPASS_MOVE_RELEASE_CM))) {
        /* Cancel the interrupted leg before any new turn. Its old terminal
         * result cannot complete a recovery command with a different ID. */
        m->reactive = true;
        int outward_heading = m->side == S5_SIDE_LEFT ? -ROBOT_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
        m->reactive_entry_heading_deg = m->state == S5_MAN_OFFSET ? outward_heading :
                                       m->state == S5_MAN_SEARCH ? -outward_heading : 0;
        m->reactive_align_parallel = m->reactive_parallel_lock = m->state == S5_MAN_OFFSET;
        next_leg(m, S5_MAN_REPLAN_STOP, in->now_ms);
        m->issuing = false;
        return (S5Action){S5_ACT_STOP, 0.0f, 0};
    }
    if (ROBOT_OBSTACLE_TEST && m->reactive) {
        if (!front_ok) {
            if (!m->invalid_since_ms) m->invalid_since_ms = in->now_ms;
            if (in->now_ms - m->invalid_since_ms >= 5000u)
                return stop(m, S5_ERR_FRONT_SENSOR);
            /* STOP cancels any in-flight turn. Re-enter from confirmed rest. */
            next_leg(m, S5_MAN_REPLAN_STOP, in->now_ms);
            m->issuing = false;
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
        m->invalid_since_ms = 0u;
        if (m->pending_id) {
            if (in->front_cm <= (m->state == S5_MAN_REPLAN_ADVANCE ?
                ROBOT_BYPASS_MOVE_STOP_CM : ROBOT_BYPASS_FRONT_TURN_CM)) {
                next_leg(m, S5_MAN_REPLAN_STOP, in->now_ms);
                return (S5Action){S5_ACT_STOP, 0.0f, 0};
            }
            if (!in->motion_terminal || in->result_id != m->pending_id) return a;
            if (!in->motion_success || !in->stopped) return stop(m, S5_ERR_MOTION_TIMEOUT);
            if (m->state == S5_MAN_REPLAN_ADVANCE) {
                float completed = in->motion_progress_cm > 0.0f ? in->motion_progress_cm : m->active_move_cm;
                m->reactive_progress_cm += completed;
                float projection = cosf((m->reactive_entry_heading_deg + m->reactive_heading_deg) * 0.01745329252f);
                if (projection > 0.0f) m->reactive_forward_cm += completed * projection;
            }
            else m->reactive_heading_deg = m->reactive_target_deg;
            next_leg(m, S5_MAN_REPLAN_CHECK, in->now_ms);
            return a;
        }
        if (!in->stopped) return (S5Action){S5_ACT_STOP, 0.0f, 0};
        bool passed_box = m->reactive_forward_cm + 0.01f >= m->reactive_required_cm;
        int heading_error = m->reactive_entry_heading_deg + m->reactive_heading_deg;
        /* An interrupted initial diagonal must turn OPPOSITE to parallel.
         * Never add a second outward turn toward unscanned side objects. */
        if (m->reactive_align_parallel) {
            int delta = -heading_error;
            if (delta != 0) {
                if (in->front_cm <= ROBOT_BYPASS_FRONT_TURN_CM)
                    return (S5Action){S5_ACT_STOP, 0.0f, 0};
                if (delta > ROBOT_BYPASS_TURN_DEG) delta = ROBOT_BYPASS_TURN_DEG;
                if (delta < -ROBOT_BYPASS_TURN_DEG) delta = -ROBOT_BYPASS_TURN_DEG;
                m->reactive_target_deg = m->reactive_heading_deg + delta;
                m->reactive_turns = 1u;
                m->state = S5_MAN_REPLAN_TURN; m->issuing = true;
                m->leg_ms = in->now_ms;
                return (S5Action){S5_ACT_TURN, 0.0f, delta};
            }
            m->reactive_align_parallel = false;
        }
        if (passed_box && heading_error != 0) {
            if (in->front_cm <= ROBOT_BYPASS_FRONT_TURN_CM)
                return (S5Action){S5_ACT_STOP, 0.0f, 0};
            int restore = -heading_error;
            if (restore > ROBOT_BYPASS_TURN_DEG) restore = ROBOT_BYPASS_TURN_DEG;
            if (restore < -ROBOT_BYPASS_TURN_DEG) restore = -ROBOT_BYPASS_TURN_DEG;
            m->reactive_target_deg = m->reactive_heading_deg + restore;
            m->state = S5_MAN_REPLAN_TURN;
            m->leg_ms = in->now_ms;
            m->issuing = true;
            return (S5Action){S5_ACT_TURN, 0.0f, restore};
        }
        if (in->front_cm >= ROBOT_RELEASE_CM) ++m->reactive_exit_hits;
        else m->reactive_exit_hits = 0u;
        if (passed_box && m->reactive_exit_hits >= ROBOT_CLEAR_CONFIRM) {
            next_leg(m, S5_MAN_COMPLETE, in->now_ms);
            return a;
        }
        if (in->front_cm >= ROBOT_BYPASS_MOVE_RELEASE_CM) {
            if (in->now_ms != m->last_clear_sample_ms) ++m->clear_hits;
            m->last_clear_sample_ms = in->now_ms;
            if (m->clear_hits < ROBOT_CLEAR_CONFIRM) return a;
            if (passed_box && in->front_cm >= ROBOT_RELEASE_CM) return a;
            /* A measured 38 cm corridor is traversable; do not keep rotating
             * just because normal DRIVE needs a larger release distance. */
            float travel_limit = m->reactive_required_cm + 50.0f;
            if (travel_limit > ROBOT_BYPASS_MAX_LEG_CM) travel_limit = ROBOT_BYPASS_MAX_LEG_CM;
            if (m->reactive_progress_cm >= travel_limit) return stop(m, S5_ERR_NO_CLEAR_ROUTE);
            float step = in->front_cm - ROBOT_BYPASS_MOVE_STOP_CM - 3.0f;
            if (step > 10.0f) step = 10.0f;
            if (step > travel_limit - m->reactive_progress_cm) step = travel_limit - m->reactive_progress_cm;
            next_leg(m, S5_MAN_REPLAN_ADVANCE, in->now_ms);
            m->active_move_cm = step;
            m->issuing = true;
            return (S5Action){S5_ACT_MOVE, step, 0};
        }
        m->clear_hits = 0u;
        if (in->front_cm <= ROBOT_BYPASS_FRONT_TURN_CM)
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        if (m->reactive_parallel_lock) return (S5Action){S5_ACT_STOP, 0.0f, 0};
        if (m->reactive_turns >= 2u) return stop(m, S5_ERR_NO_CLEAR_ROUTE);
        int direction = m->side == S5_SIDE_LEFT ? 1 : -1;
        int target = (int)(m->reactive_turns / 2u + 1u) * ROBOT_BYPASS_RECOVERY_TURN_DEG;
        if (m->reactive_turns % 2u) target = -target;
        target *= direction; /* Match the original turn, then test the other heading once. */
        int delta = target - m->reactive_heading_deg;
        m->reactive_target_deg = target;
        ++m->reactive_turns;
        m->state = S5_MAN_REPLAN_TURN;
        m->leg_ms = in->now_ms;
        m->issuing = true;
        return (S5Action){S5_ACT_TURN, 0.0f, delta};
    }
    if (m->state == S5_MAN_LINE_STOP) {
        /* Catch the tape on first contact, then confirm while stationary.
         * Waiting for three ultrasonic-paced ticks while still driving can
         * carry the sensors completely across a narrow line. */
        if (m->pending_id) {
            if (!in->motion_terminal || in->result_id != m->pending_id ||
                !in->stopped) return (S5Action){S5_ACT_STOP, 0.0f, 0};
            if (!in->motion_success && !in->motion_cancelled)
                return stop(m, S5_ERR_MOTION_TIMEOUT);
            m->searched_cm += in->motion_progress_cm > 0.0f ?
                in->motion_progress_cm : (in->motion_success ? m->active_move_cm : 0.0f);
            m->pending_id = 0u;
        }
        if (!in->stopped) {
            m->line_hits = 0u;
            m->last_line_ms = in->line_sample_ms;
        } else if (in->line_sample_ms != m->last_line_ms) {
            m->last_line_ms = in->line_sample_ms;
            if (!in->line_detected) {
                m->line_hits = 0u;
                next_leg(m, S5_MAN_SEARCH, in->now_ms);
            } else if (++m->line_hits >= ROBOT_BYPASS_LINE_CONFIRM) {
                next_leg(m, m->equal_leg_route ? S5_MAN_COMPLETE : S5_MAN_ALIGN, in->now_ms);
            }
        }
        return (S5Action){S5_ACT_STOP, 0.0f, 0};
    }
    turning = m->state == S5_MAN_TURN_OUT || m->state == S5_MAN_FORWARD ||
              m->state == S5_MAN_TURN_IN || m->state == S5_MAN_ALIGN ||
              m->state == S5_MAN_ALIGN_CHECK;
    if (!in->front_valid || !(in->front_cm > 0.0f) || in->front_cm > S5_MAX_RANGE_CM)
        return stop(m, S5_ERR_FRONT_SENSOR);
    if (m->guarded_probe_route && translating && m->pending_id &&
        (m->forward_paused || in->front_cm <= ROBOT_BYPASS_MOVE_STOP_CM)) {
        /* Brake on a close echo. Reuse only encoder travel from the
         * matching terminal result, after both wheels have stopped. */
        m->forward_paused = true;
        if (in->motion_terminal && in->result_id == m->pending_id && in->stopped) {
            if (!in->motion_success && !in->motion_cancelled)
                return stop(m, S5_ERR_MOTION_TIMEOUT);
            float progress = in->motion_progress_cm;
            if (progress <= 0.0f)
                progress = in->motion_success ? m->active_move_cm : 0.0f;
            if (m->state == S5_MAN_OFFSET) m->offset_done_cm += progress;
            else if (m->state == S5_MAN_PASS) m->pass_done_cm += progress;
            else m->searched_cm += progress;
            m->pending_id = 0u;
            m->forward_paused = false;
            m->waiting_clear = true;
            m->clear_hits = 0u;
        }
        return (S5Action){S5_ACT_STOP, 0.0f, 0};
    }
    /* The first post-turn echo may still see the old obstacle/beam edge.
     * Before starting a straight leg, hold STOP and confirm fresh clearance
     * instead of permanently faulting. Active motion retains immediate STOP. */
    if (translating &&
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
    if (m->equal_leg_route && !m->pending_id && in->stopped) {
        if (m->state == S5_MAN_OFFSET && m->offset_done_cm + 0.01f >= m->offset_cm) {
            next_leg(m, S5_MAN_FORWARD, in->now_ms);
            return a;
        }
        if (m->state == S5_MAN_PASS && m->pass_done_cm + 0.01f >= m->pass_cm) {
            next_leg(m, S5_MAN_TURN_IN, in->now_ms);
            return a;
        }
    }
    /* Reversing increases front clearance; the turn envelope applies to turns.
     * Rear clearance is checked separately throughout the reverse leg. */
    if (m->state != S5_MAN_BACK && in->front_cm <= (turning ?
                        ROBOT_BYPASS_FRONT_TURN_CM : ROBOT_BYPASS_MOVE_STOP_CM))
        return stop(m, S5_ERR_INSUFFICIENT_CLEARANCE);
    if (m->state == S5_MAN_BACK && !in->rear_clear)
        return stop(m, S5_ERR_NO_CLEAR_ROUTE);
    if (((m->state == S5_MAN_SEARCH && (!ROBOT_OBSTACLE_TEST || m->equal_leg_route)) ||
         (m->state == S5_MAN_ALIGN_CHECK && !m->pending_id && !m->issuing && in->stopped)) &&
        in->line_sample_ms != m->last_line_ms) {
        m->last_line_ms = in->line_sample_ms;
        /* Accept fresh tape detections during return travel after the
         * robot has left the original line. Both sensors may see tape. */
        if (m->left_original_line && in->line_detected)
            ++m->line_hits;
        else m->line_hits = 0u;
        if (m->line_hits >= (m->state == S5_MAN_SEARCH ? 1u : ROBOT_BYPASS_LINE_CONFIRM)) {
            if (m->state == S5_MAN_ALIGN_CHECK) {
                next_leg(m, S5_MAN_COMPLETE, in->now_ms);
                return a; /* Already physically stopped and facing forward. */
            }
            uint32_t interrupted_id = m->equal_leg_route ? m->pending_id : 0u;
            next_leg(m, S5_MAN_LINE_STOP, in->now_ms);
            m->pending_id = interrupted_id;
            m->line_hits = 0u;
            return (S5Action){S5_ACT_STOP, 0.0f, 0};
        }
    }
    if (m->issuing) return stop(m, S5_ERR_MOTION_TIMEOUT);
    if (m->pending_id) {
        if (!in->motion_terminal || in->result_id != m->pending_id) return a;
        if (!in->motion_success)
            return stop(m, S5_ERR_MOTION_TIMEOUT);
        /* Result history and live status are sampled separately. A completed
         * result can arrive one poll before the live stopped snapshot. Keep
         * its command ID and wait; never treat this handoff race as failure
         * or issue the next move before physical stop is confirmed. */
        if (!in->stopped) return a;
        float completed = in->motion_progress_cm > 0.0f ? in->motion_progress_cm : m->active_move_cm;
        switch (m->state) {
        case S5_MAN_TURN_OUT: m->outward_turn_complete = true; next_leg(m, S5_MAN_OFFSET, in->now_ms); break;
        case S5_MAN_OFFSET:
            m->offset_done_cm += completed;
            if (ROBOT_BYPASS_EXACT_SEQUENCE) {
                m->offset_cm = m->offset_done_cm;
                m->search_limit_cm = m->offset_cm;
                m->pass_cm = (m->reactive_required_cm - m->offset_cm * cosf(ROBOT_BYPASS_TURN_DEG * 0.01745329252f)) * ROBOT_BYPASS_MIDDLE_DISTANCE_SCALE;
                if (m->pass_cm < 5.0f) m->pass_cm = 5.0f;
                if (ROBOT_BYPASS_EXACT_SEQUENCE && m->pass_cm > ROBOT_BYPASS_MIDDLE_MAX_CM)
                    m->pass_cm = ROBOT_BYPASS_MIDDLE_MAX_CM;
            }
            if (m->equal_leg_route) {
                next_leg(m, m->offset_done_cm + 0.01f >= m->offset_cm ?
                         S5_MAN_FORWARD : S5_MAN_OFFSET, in->now_ms);
            } else {
                next_leg(m, ROBOT_BYPASS_EXACT_SEQUENCE || (ROBOT_OBSTACLE_TEST && !ROBOT_BYPASS_DISTANCE_MODE) || m->offset_done_cm + 0.01f >= m->offset_cm ? S5_MAN_FORWARD : S5_MAN_OFFSET, in->now_ms);
            }
            break;
        case S5_MAN_FORWARD: next_leg(m, S5_MAN_PASS, in->now_ms); break;
        case S5_MAN_PASS:
            m->pass_done_cm += completed;
            if (ROBOT_BYPASS_EXACT_SEQUENCE) m->pass_cm = m->pass_done_cm;
            next_leg(m, ROBOT_BYPASS_EXACT_SEQUENCE || (ROBOT_OBSTACLE_TEST && !ROBOT_BYPASS_DISTANCE_MODE) || m->pass_done_cm + 0.01f >= m->pass_cm ?
                S5_MAN_TURN_IN : S5_MAN_PASS, in->now_ms);
            break;
        case S5_MAN_TURN_IN:
            next_leg(m, S5_MAN_SEARCH, in->now_ms);
            if (m->equal_leg_route) m->last_line_ms = in->line_sample_ms;
            if (m->guarded_probe_route) m->waiting_clear = true;
            break;
        case S5_MAN_SEARCH:
            if (ROBOT_OBSTACLE_TEST || m->equal_leg_route) m->searched_cm += completed;
            if (m->equal_leg_route) {
                if (m->searched_cm + 0.01f >= m->search_limit_cm)
                    return stop(m, S5_ERR_LINE_TIMEOUT);
                next_leg(m, S5_MAN_SEARCH, in->now_ms);
                break;
            }
            if (ROBOT_BYPASS_EXACT_SEQUENCE) m->search_limit_cm = m->searched_cm;
            next_leg(m, ROBOT_BYPASS_EXACT_SEQUENCE || (ROBOT_OBSTACLE_TEST && !ROBOT_BYPASS_DISTANCE_MODE) ? S5_MAN_ALIGN : S5_MAN_SEARCH, in->now_ms); break;
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
    outward = m->equal_leg_route ? ROBOT_COURSE_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
    if (m->side == S5_SIDE_LEFT) outward = -outward;
    switch (m->state) {
    case S5_MAN_OFFSET: a = (S5Action){S5_ACT_MOVE, m->offset_cm - m->offset_done_cm, 0}; break;
    case S5_MAN_TURN_OUT:
    case S5_MAN_ALIGN: a = (S5Action){S5_ACT_TURN, 0.0f, outward}; break;
    case S5_MAN_FORWARD:
        a = (S5Action){S5_ACT_TURN, 0.0f, m->equal_leg_route ?
            (m->side == S5_SIDE_LEFT ? ROBOT_COURSE_PASS_TURN_DEG : -ROBOT_COURSE_PASS_TURN_DEG) :
            -outward}; break;
    case S5_MAN_TURN_IN:
        a = (S5Action){S5_ACT_TURN, 0.0f, m->equal_leg_route ?
            (m->side == S5_SIDE_LEFT ? ROBOT_COURSE_RETURN_TURN_DEG : -ROBOT_COURSE_RETURN_TURN_DEG) : -outward};
        break;
    case S5_MAN_PASS: a = (S5Action){S5_ACT_MOVE, m->pass_cm - m->pass_done_cm, 0}; break;
    case S5_MAN_BACK: a = (S5Action){S5_ACT_MOVE, -ROBOT_BYPASS_REVERSE_CM, 0}; break;
    case S5_MAN_ALIGN_CHECK: {
        int heading;
        int toward_line = m->side == S5_SIDE_RIGHT ? -1 : 1;
        if (in->now_ms - m->leg_ms < ROBOT_BYPASS_ALIGN_OBSERVE_MS) return a;
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
            return stop(m, S5_ERR_LINE_TIMEOUT);
        }
        a = (S5Action){S5_ACT_MOVE, (ROBOT_OBSTACLE_TEST || m->equal_leg_route) ?
            m->search_limit_cm - m->searched_cm : ROBOT_BYPASS_SEARCH_STEP_CM, 0};
        if (a.distance_cm > m->search_limit_cm - m->searched_cm)
            a.distance_cm = m->search_limit_cm - m->searched_cm;
        if (!ROBOT_OBSTACLE_TEST && !m->equal_leg_route) m->searched_cm += a.distance_cm;
        break;
    default: return stop(m, S5_ERR_MOTION_TIMEOUT);
    }
    /* Timed test legs must not end at an old short geometric target (e.g.
     * the 5 cm parallel leg). The live measured corridor still caps travel. */
    if (ROBOT_OBSTACLE_TEST && !ROBOT_BYPASS_DISTANCE_MODE && a.type == S5_ACT_MOVE && a.distance_cm > 0.0f)
        a.distance_cm = ROBOT_BYPASS_MAX_LEG_CM;
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
        if (!ROBOT_OBSTACLE_TEST && !m->guarded_probe_route &&
            a.distance_cm > ROBOT_COURSE_CLEAR_STEP_CM)
            a.distance_cm = ROBOT_COURSE_CLEAR_STEP_CM;
        if (a.distance_cm > available) a.distance_cm = available;
    }
    if (a.type == S5_ACT_MOVE) m->active_move_cm = a.distance_cm;
    m->issuing = true;
    return a;
}
