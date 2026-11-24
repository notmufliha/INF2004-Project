#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../../app_program/subsystem5/s5_maneuver.h"
int main(void)
{
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        S5Maneuver m;
        S5Profile p = {0};
        S5ManeuverInput in = {0};
        p.valid = p.uses_course_bounds = true;
        p.estimated_width_cm = 20.0f; p.center_y_cm = 22.38f;
        p.bypass_side = (S5Side)side;
        s5_maneuver_init(&m);
        assert(s5_maneuver_begin(&m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, ROBOT_BYPASS_MAX_DEPTH_CM, 0u));
        assert(ROBOT_BYPASS_TURN_DEG == 40);
        float swept = sqrtf(ROBOT_CHASSIS_HALF_LENGTH_CM * ROBOT_CHASSIS_HALF_LENGTH_CM +
                            ROBOT_CHASSIS_HALF_WIDTH_CM * ROBOT_CHASSIS_HALF_WIDTH_CM);
        assert(ROBOT_BYPASS_MIN_TURN_CM > swept + 0.4f);
        /* Centred test box: half width + half chassis + margin, along the
         * 40-degree diagonal, plus the extra diagonal, scaled. (40.36 cm
         * with the earlier 16 cm chassis estimate.) */
        float expected_offset = ((20.0f * 0.5f + ROBOT_CHASSIS_HALF_WIDTH_CM +
            ROBOT_OBSTACLE_MARGIN_CM) / sinf(40.0f * 0.01745329252f) +
            ROBOT_BYPASS_EXTRA_DIAGONAL_CM) * ROBOT_BYPASS_DIAGONAL_DISTANCE_SCALE;
        assert(fabsf(m.offset_cm - expected_offset) < 0.01f);
        assert(fabsf(m.search_limit_cm - m.offset_cm) < 0.01f);
        assert(fabsf(m.pass_cm - 22.0f) < 0.01f);
        unsigned turns = 0u, moves = 0u;
        int heading = 0;
        int outward = side == S5_SIDE_LEFT ? -ROBOT_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
        const int expected[] = {outward, -outward, -outward, outward};
        bool advanced[4] = {false};
        in.front_valid = in.line_valid = in.stopped = true;
        in.front_cm = 38.39f; /* Actual stationary reading after COM6 turn. */
        /* Supply tape only after the measured route restores its heading. */
        in.line_detected = false;
        for (unsigned tick = 0; tick < 1000u && m.state != S5_MAN_COMPLETE; ++tick) {
            in.line_detected = m.state == S5_MAN_ALIGN_CHECK;
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            S5Action a = s5_maneuver_step(&m, &in);
            assert(m.state != S5_MAN_FAILED);
            in.motion_terminal = false;
            if (a.type == S5_ACT_TURN || a.type == S5_ACT_MOVE) {
                if (a.type == S5_ACT_TURN) {
                    if (turns > 0u) assert(advanced[turns - 1u]);
                    assert(turns < 4u && a.turn_deg == expected[turns++]);
                    heading += a.turn_deg;
                } else { assert(a.distance_cm > 0.0f); ++moves; advanced[turns - 1u] = true; }
                assert(s5_maneuver_submitted(&m, tick + 1u));
                in.result_id = tick + 1u;
                in.motion_terminal = in.motion_success = true;
            }
        }
        assert(m.state == S5_MAN_COMPLETE && turns == 4u && moves == 3u && heading == 0);
        assert(m.offset_done_cm + 0.01f >= m.offset_cm);
        assert(m.pass_done_cm + 0.01f >= m.pass_cm);
        assert(m.searched_cm + 0.01f >= m.search_limit_cm);
    }
    /* Latest capture: first segment 30.14 cm, old second request 30.76 cm
     * stalled before the next turn. New remaining segment is only 16.63 cm. */
    S5Maneuver route; S5Profile box = {0};
    box.valid = box.uses_course_bounds = true; box.estimated_width_cm = 20.0f;
    box.center_y_cm = 15.94f; box.bypass_side = S5_SIDE_LEFT;
    s5_maneuver_init(&route);
    assert(s5_maneuver_begin(&route, S5_CMD_GO_LEFT, &box, 20.0f, 0u));
    route.state = S5_MAN_OFFSET; route.pending_id = 1u; route.active_move_cm = 30.14f;
    S5ManeuverInput completed = {0};
    completed.line_valid = completed.front_valid = completed.stopped = true;
    completed.motion_terminal = completed.motion_success = true;
    completed.result_id = 1u; completed.front_cm = 71.77f;
    assert(s5_maneuver_step(&route, &completed).type == S5_ACT_NONE);
    completed.motion_terminal = false;
    S5Action remaining = s5_maneuver_step(&route, &completed);
    assert(remaining.type == S5_ACT_TURN && remaining.turn_deg == ROBOT_BYPASS_TURN_DEG);
    S5Maneuver paused = {0}; S5ManeuverInput in = {0};
    paused.state = S5_MAN_OFFSET; paused.pending_id = 77u;
    paused.active_move_cm = 10.0f; paused.offset_cm = paused.pass_cm = 50.0f;
    in.line_valid = in.front_valid = true; in.front_cm = 19.58f;
    in.motion_progress_cm = 5.0f;
    assert(s5_maneuver_step(&paused, &in).type == S5_ACT_STOP);
    assert(paused.forward_paused && paused.offset_done_cm == 5.0f && !paused.reactive);
    in.front_cm = 70.0f; in.stopped = true;
    S5Action resumed = {0};
    for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
        in.now_ms += 65u; in.line_sample_ms = in.now_ms;
        resumed = s5_maneuver_step(&paused, &in);
    }
    assert(resumed.type == S5_ACT_MOVE && paused.state == S5_MAN_OFFSET);
    assert(s5_maneuver_submitted(&paused, 88u));
    in.motion_terminal = in.motion_success = true; in.result_id = 77u;
    assert(s5_maneuver_step(&paused, &in).type == S5_ACT_NONE);
    assert(paused.pending_id == 88u);
    /* Encoder coast is real travel: an 8 cm result for a 5 cm request must
     * count as 8, so later segments do not repeat the extra 3 cm. */
    S5Maneuver coast = {0};
    coast.side = S5_SIDE_LEFT;
    coast.state = S5_MAN_OFFSET; coast.pending_id = 10u;
    coast.offset_cm = 10.0f; coast.active_move_cm = 5.0f;
    in = (S5ManeuverInput){0};
    in.line_valid = in.front_valid = in.stopped = in.motion_terminal = in.motion_success = true;
    in.front_cm = 70.0f; in.result_id = 10u; in.motion_progress_cm = 8.0f;
    assert(s5_maneuver_step(&coast, &in).type == S5_ACT_NONE);
    assert(coast.offset_done_cm == 8.0f && coast.state == S5_MAN_FORWARD);
    in.motion_terminal = false;
    S5Action final_piece = s5_maneuver_step(&coast, &in);
    assert(final_piece.type == S5_ACT_TURN && final_piece.turn_deg == ROBOT_BYPASS_TURN_DEG);
    /* Exact current capture: 27.09 cm used to hold STOP forever. */
    S5Maneuver captured = {0};
    captured.state = S5_MAN_OFFSET; captured.forward_paused = true;
    captured.offset_cm = captured.pass_cm = 60.9117f;
    captured.offset_done_cm = 17.2f;
    in = (S5ManeuverInput){0};
    in.line_valid = in.front_valid = in.stopped = true; in.front_cm = 27.09f;
    S5Action advance = {0};
    for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
        in.now_ms += 65u; in.line_sample_ms = in.now_ms;
        advance = s5_maneuver_step(&captured, &in);
    }
    assert(advance.type == S5_ACT_MOVE && advance.distance_cm > 4.0f && advance.distance_cm < 4.2f);
    assert(captured.state == S5_MAN_OFFSET && !captured.reactive);
    assert(ROBOT_BYPASS_MOVE_STOP_CM >= ROBOT_CHASSIS_HALF_LENGTH_CM + ROBOT_OBSTACLE_MARGIN_CM + 2.0f);
    assert(s5_maneuver_submitted(&captured, 99u));
    in.front_cm = 20.0f;
    assert(s5_maneuver_step(&captured, &in).type == S5_ACT_STOP);
    assert(captured.forward_paused);
    /* Latest log: 53.03 cm completed, 21.18 cm ahead; enough lateral
     * clearance exists, so omit only the optional reserve and turn right. */
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        S5Maneuver ready;
        S5Profile p = {0};
        p.valid = p.uses_course_bounds = true; p.estimated_width_cm = 20.0f;
        p.center_y_cm = 17.34f; p.bypass_side = (S5Side)side;
        s5_maneuver_init(&ready);
        assert(s5_maneuver_begin(&ready, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, 20.0f, 0u));
        ready.state = S5_MAN_OFFSET; ready.offset_done_cm = 18.43f;
        in = (S5ManeuverInput){0};
        in.line_valid = in.front_valid = in.stopped = true; in.front_cm = 21.18f;
        assert(s5_maneuver_step(&ready, &in).type == S5_ACT_STOP);
        assert(ready.state == S5_MAN_OFFSET); /* Never turn too early. */
        ready.offset_done_cm = 53.03f;
        for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            S5Action a = s5_maneuver_step(&ready, &in);
            assert(a.type == (i + 1u == ROBOT_CLEAR_CONFIRM ? S5_ACT_NONE : S5_ACT_STOP));
        }
        assert(ready.state == S5_MAN_FORWARD);
        assert(ready.offset_cm == 53.03f && ready.search_limit_cm == 53.03f);
        S5Action turn = s5_maneuver_step(&ready, &in);
        assert(turn.type == S5_ACT_TURN && turn.turn_deg == (side == S5_SIDE_LEFT ? ROBOT_BYPASS_TURN_DEG : -ROBOT_BYPASS_TURN_DEG));
        float middle_cm = (ready.reactive_required_cm - ready.offset_cm * cosf(ROBOT_BYPASS_TURN_DEG * 0.01745329252f)) * ROBOT_BYPASS_MIDDLE_DISTANCE_SCALE;
        if (middle_cm < 5.0f) middle_cm = 5.0f;
        if (middle_cm > ROBOT_BYPASS_MIDDLE_MAX_CM) middle_cm = ROBOT_BYPASS_MIDDLE_MAX_CM;
        assert(fabsf(ready.pass_cm - middle_cm) < 0.01f);
        /* Latest middle-leg capture: one 21.94 cm completion must be
         * followed by TURN_IN, never the old 7.18 cm top-up MOVE. */
        S5Maneuver middle = {0};
        middle.side = (S5Side)side; middle.state = S5_MAN_PASS;
        middle.pending_id = 9u; middle.active_move_cm = 21.91f; middle.pass_cm = 29.12f;
        S5ManeuverInput result = {0};
        result.line_valid = result.front_valid = result.stopped = true;
        result.motion_terminal = result.motion_success = true;
        result.result_id = 9u; result.motion_progress_cm = 21.94f; result.front_cm = 30.01f;
        assert(s5_maneuver_step(&middle, &result).type == S5_ACT_NONE);
        assert(middle.state == S5_MAN_TURN_IN && middle.pass_cm == 21.94f);
        /* Completed opposite turn can be visible before live stop status.
         * Hold the ID, then start the shortened middle leg once stopped. */
        S5Maneuver handoff = {0};
        handoff.side = (S5Side)side; handoff.state = S5_MAN_FORWARD;
        handoff.pending_id = 11u; handoff.pass_cm = 15.0f;
        result.result_id = 11u; result.stopped = false; result.front_cm = 70.0f;
        assert(s5_maneuver_step(&handoff, &result).type == S5_ACT_NONE);
        assert(handoff.state == S5_MAN_FORWARD && handoff.pending_id == 11u);
        result.stopped = true;
        assert(s5_maneuver_step(&handoff, &result).type == S5_ACT_NONE);
        assert(handoff.state == S5_MAN_PASS);
        S5Action second_forward = s5_maneuver_step(&handoff, &result);
        assert(second_forward.type == S5_ACT_MOVE && fabsf(second_forward.distance_cm - 15.0f) < 0.01f);
        S5Action last_right = s5_maneuver_step(&middle, &result);
        assert(last_right.type == S5_ACT_TURN && last_right.turn_deg == (side == S5_SIDE_LEFT ? ROBOT_BYPASS_TURN_DEG : -ROBOT_BYPASS_TURN_DEG));
        /* Current capture: 22 cm progress, then 18.69 cm front reading. */
        s5_maneuver_init(&ready);
        assert(s5_maneuver_begin(&ready, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, 20.0f, 0u));
        ready.state = S5_MAN_OFFSET; ready.forward_paused = true; ready.offset_done_cm = 22.0f;
        in.front_cm = 18.69f;
        for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            s5_maneuver_step(&ready, &in);
        }
        assert(ready.state == S5_MAN_FORWARD && !ready.forward_paused);
        turn = s5_maneuver_step(&ready, &in);
        assert(turn.type == S5_ACT_TURN && turn.turn_deg == (side == S5_SIDE_LEFT ? ROBOT_BYPASS_TURN_DEG : -ROBOT_BYPASS_TURN_DEG));
    }
    puts("PASS left/right/right/left and mirror: forward leg between every turn, zero final heading, interrupted move resumes");
    return 0;
}
