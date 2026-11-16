#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_maneuver.h"
int main(void)
{
    /* Red-bag video: a 19.71 cm diagonal was abandoned, followed by only
     * 10 cm before a 163 cm background echo prematurely ended avoidance. */
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        S5Maneuver m;
        S5Profile p = {0};
        p.valid = p.uses_course_bounds = true;
        p.estimated_width_cm = 20.0f; p.center_y_cm = 17.86f; p.bypass_side = (S5Side)side;
        s5_maneuver_init(&m);
        assert(s5_maneuver_begin(&m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, 20.0f, 0u));
        m.state = S5_MAN_OFFSET; m.offset_done_cm = 19.71f;
        S5ManeuverInput in = {0};
        in.line_valid = in.front_valid = in.stopped = true;
        in.now_ms = in.line_sample_ms = 1000u; in.front_cm = 33.35f;
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
        assert(m.reactive_align_parallel && m.reactive_parallel_lock);
        in.front_cm = 163.16f;
        for (unsigned i = 0; i < 300u && m.state != S5_MAN_COMPLETE; ++i) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            S5Action a = s5_maneuver_step(&m, &in);
            assert(m.state != S5_MAN_FAILED);
            in.motion_terminal = false;
            if (a.type == S5_ACT_TURN || a.type == S5_ACT_MOVE) {
                if (a.type == S5_ACT_TURN)
                    assert(a.turn_deg == (side == S5_SIDE_LEFT ? 45 : -45));
                if (a.type == S5_ACT_MOVE)
                    assert(m.reactive_entry_heading_deg + m.reactive_heading_deg == 0);
                assert(s5_maneuver_submitted(&m, 500u + i));
                in.result_id = 500u + i; in.motion_terminal = in.motion_success = true;
            }
        }
        assert(m.state == S5_MAN_COMPLETE);
        assert(m.reactive_forward_cm >= 55.86f);
        assert(m.reactive_entry_heading_deg + m.reactive_heading_deg == 0);
    }
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        S5Maneuver m = {0};
        S5ManeuverInput in = {0};
        m.reactive_required_cm = 40.0f; m.side = (S5Side)side; m.state = S5_MAN_OFFSET; m.pending_id = 77u;
        in.now_ms = in.line_sample_ms = 1000u; in.line_valid = true;
        /* The actual log lost its echo during an active forward MOVE. */
        S5Action a = s5_maneuver_step(&m, &in);
        assert(a.type == S5_ACT_STOP && m.reactive && !m.pending_id);
        in.now_ms += 65u; in.line_sample_ms = in.now_ms;
        a = s5_maneuver_step(&m, &in);
        assert(a.type == S5_ACT_STOP); /* No blind turn on invalid echo. */
        in.front_valid = in.stopped = true; in.front_cm = 18.0f;
        for (unsigned turn = 0; turn < 1u; ++turn) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            a = s5_maneuver_step(&m, &in);
            assert(a.type == S5_ACT_TURN);
            int expected = turn == 0u ? ROBOT_BYPASS_TURN_DEG : -2 * ROBOT_BYPASS_TURN_DEG;
            assert(a.turn_deg == (side == S5_SIDE_LEFT ? expected : -expected));
            assert(s5_maneuver_submitted(&m, 100u + turn));
            in.motion_terminal = in.motion_success = true; in.result_id = 77u;
            assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
            assert(m.pending_id == 100u + turn); /* Ignore old cancelled leg. */
            in.result_id = 100u + turn;
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
            in.motion_terminal = false;
            if (turn == 0u) {
                /* Exact failure: first correction saw 38.07 cm. Advance
                 * instead of sending two more same-direction turns. */
                in.front_cm = 38.07f;
                for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
                    in.now_ms += 65u; in.line_sample_ms = in.now_ms;
                    a = s5_maneuver_step(&m, &in);
                }
                assert(a.type == S5_ACT_MOVE && a.distance_cm > 5.0f && a.distance_cm < 5.1f);
                assert(m.reactive_turns == 1u);
                assert(s5_maneuver_submitted(&m, 200u));
                in.result_id = 200u; in.motion_terminal = true;
                in.now_ms += 65u; in.line_sample_ms = in.now_ms;
                assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
                assert(m.reactive_progress_cm > 5.0f && m.reactive_progress_cm < 5.1f);
                in.motion_terminal = false; in.front_cm = 18.0f;
                in.now_ms += 65u; in.line_sample_ms = in.now_ms;
                assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
                assert(m.reactive_turns == 1u); /* No second left/right turn into a keyboard. */
            }
        }
        in.front_cm = 163.16f; /* Logged background echo must not end bypass early. */
        for (unsigned tick = 0; tick < 200u && m.state != S5_MAN_COMPLETE; ++tick) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            S5Action a = s5_maneuver_step(&m, &in);
            assert(m.state != S5_MAN_FAILED);
            in.motion_terminal = false;
            if (a.type == S5_ACT_MOVE) {
                assert(m.reactive_forward_cm < m.reactive_required_cm);
                assert(s5_maneuver_submitted(&m, 300u + tick));
                in.result_id = 300u + tick;
                in.motion_terminal = in.motion_success = true;
            }
        }
        assert(m.state == S5_MAN_COMPLETE && m.reactive_turns == 1u);
        assert(m.reactive_forward_cm >= 40.0f);
        assert(m.reactive_entry_heading_deg + m.reactive_heading_deg == 0);
    }
    /* Latest capture continued to 23.32 cm and hit the IMU impact stop.
     * Interrupt the diagonal at 30 cm and undo the initial 45-degree turn. */
    S5Maneuver m = {0}; S5ManeuverInput in = {0};
    m.reactive_required_cm = 40.0f; m.side = S5_SIDE_LEFT; m.state = S5_MAN_OFFSET; m.pending_id = 77u;
    in.line_valid = in.front_valid = true; in.front_cm = 31.38f;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
    in.front_cm = 30.0f;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP && m.reactive);
    in.stopped = true;
    S5Action opposite = s5_maneuver_step(&m, &in);
    assert(opposite.type == S5_ACT_TURN && opposite.turn_deg == 45);
    m = (S5Maneuver){0}; in = (S5ManeuverInput){0};
    m.reactive = true; m.state = S5_MAN_REPLAN_CHECK; m.reactive_turns = 6u;
    in.line_valid = in.front_valid = in.stopped = true; in.front_cm = 18.0f;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
    assert(m.state == S5_MAN_FAILED && m.error == S5_ERR_NO_CLEAR_ROUTE);
    m = (S5Maneuver){0}; m.reactive = true; m.state = S5_MAN_REPLAN_STOP;
    in.front_valid = false; in.now_ms = in.line_sample_ms = 100u;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
    in.now_ms = in.line_sample_ms = 5100u;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
    assert(m.state == S5_MAN_FAILED && m.error == S5_ERR_FRONT_SENSOR);
    puts("PASS recovery: logged 38.07 cm advances without extra turn, alternate headings, command IDs, sensor pause, bounded failures");
    return 0;
}
