/* Exercise bypass and line recovery through the real navigation adapter. */
#define main legacy_navigation_main
#include "test_navigation.c"
#undef main

static void recover_line(S5Side side, bool tape_present)
{
    unsigned old_turns = turn_calls;
    assert(!s5_bridge_line_found());
    assert(!s5_bridge_request(S5_CMD_RESUME, NULL));
    tick(120.0f);
    assert(turn_calls == old_turns); /* Await recovery ownership, even on tape. */
    assert(s5_bridge_request(S5_CMD_FIND_LINE, NULL));
    tick(120.0f);
    assert(maneuver.state == S5_MAN_TURN_IN && maneuver.pending_id);
    assert(last_angle == 90);
    assert(last_turn == (side == S5_SIDE_LEFT ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT));
    tick(120.0f);
    assert(maneuver.state == S5_MAN_TURN_IN && !s5_bridge_line_found());
    finish_leg();
    line_state.line_flags = 0u;
    tick(120.0f);
    assert(maneuver.state == S5_MAN_SEARCH && maneuver.pending_id);
    assert(last_move == 20.0f && last_direction == MOTION_DIR_FORWARD);
    if (!tape_present) {
        finish_leg();
        tick(120.0f);
        assert(!allowed && !s5_bridge_line_found());
        assert(s5_navigation_error() == S5_ERR_LINE_TIMEOUT);
        return;
    }
    status.left_distance_mm = status.right_distance_mm = 20.0f;
    line_state.line_flags = B3_LINE_F_DETECTED;
    tick(120.0f);
    assert(!allowed && maneuver.state == S5_MAN_LINE_STOP);
    for (unsigned n = 0; n <= ROBOT_BYPASS_LINE_CONFIRM; ++n) tick(120.0f);
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, NULL));
    assert(mode == NAV_FOLLOW && turn_calls == old_turns + 1u);
}

int main(void)
{
    assert(s5_navigation_init());
    line_state.line_flags = 0u;
    for (unsigned n = 0; n < ROBOT_CLEAR_CONFIRM; ++n) {
        clock_ms += 65u;
        s5_navigation_tick(true, 400.0f, true);
    }
    assert(allowed && !s5_navigation_has_line_reference());
    assert(drive_calls == 0u); /* VC can now search rather than deadlock. */
    for (int direction = 0; direction < 2; ++direction) {
        S5Profile p = {0};
        p.valid = true;
        p.estimated_width_cm = 10.0f;
        p.center_y_cm = 55.0f;
        p.bypass_side = direction ? S5_SIDE_RIGHT : S5_SIDE_LEFT;
        assert(!s5_maneuver_course_room(&p, p.bypass_side, 16.0f));
        assert(s5_maneuver_course_room(&p, p.bypass_side, p.center_y_cm));
        assert(s5_bridge_request(S5_CMD_STOP, NULL));
        S5Profile too_close = p;
        too_close.center_y_cm = 16.0f;
        unsigned turns_before = turn_calls;
        assert(!s5_bridge_request(direction ? S5_CMD_GO_RIGHT : S5_CMD_GO_LEFT, &too_close));
        assert(turn_calls == turns_before);
        assert(s5_bridge_request(direction ? S5_CMD_GO_RIGHT : S5_CMD_GO_LEFT, &p));
        line_state.line_flags = 0u;
        tick(120.0f); /* Submit outward turn. */
        assert(last_angle == ROBOT_COURSE_BYPASS_TURN_DEG);
        assert(last_turn == (direction ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT));
        finish_course_turn();
        assert(maneuver.outward_turn_complete);
        finish_offset();
        assert(last_move == 20.0f);
        assert(maneuver.state == S5_MAN_TURN_IN && s5_bridge_motion_complete());
        recover_line(p.bypass_side, true);
        for (unsigned n = 0; n < ROBOT_CLEAR_CONFIRM; ++n) {
            clock_ms += 65u;
            s5_navigation_tick(true, 120.0f, true);
        }
        assert(allowed && drive_calls == 0u); /* VC regains DRIVE ownership. */
    }
    {
        S5Profile p = {0};
        p.valid = p.uses_course_bounds = true;
        p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
        p.center_y_cm = 64.0f;
        p.bypass_side = S5_SIDE_LEFT;
        assert(s5_bridge_request(S5_CMD_STOP, NULL));
        assert(s5_navigation_probe_begin(-ROBOT_COURSE_BYPASS_TURN_DEG));
        finish_leg();
        assert(s5_navigation_probe_bypass(&p));
        assert(maneuver.state == S5_MAN_OFFSET);
        line_state.line_flags = 0u;
        finish_offset();
        recover_line(S5_SIDE_LEFT, false);
    }
    puts("PASS real navigation: mirrored 45/90-degree turns, 20 cm legs, early tape handoff, and bounded stop without tape");
    return 0;
}
