#define main legacy_navigation_main
#include "test_navigation.c"
#undef main
int main(void)
{
    assert(ROBOT_CLEAR_PATH_FORWARD == 1);
    assert(s5_navigation_init());
    line_state.line_flags = 0u;
    for (unsigned i = 0u; i < ROBOT_CLEAR_CONFIRM; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 100.0f, true);
    }
    assert(allowed && !s5_navigation_has_line_reference());
    assert(drive_calls == 0u); /* VC owns forward DRIVE. */
    clock_ms += 65u;
    s5_navigation_tick(true, ROBOT_STOP_CM, true);
    assert(!allowed);
    for (unsigned i = 0u; i < ROBOT_CLEAR_CONFIRM; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 100.0f, true);
    }
    assert(allowed);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    assert(s5_navigation_probe_begin(-45));
    tick(73.04f);
    assert(!s5_navigation_probe_complete());
    finish_leg();
    assert(s5_navigation_probe_complete());
    S5Profile p = {0};
    p.valid = p.uses_course_bounds = true;
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM;
    p.center_y_cm = 18.36f;
    p.bypass_side = S5_SIDE_RIGHT;
    assert(!s5_navigation_probe_bypass(&p)); /* Cannot reuse a left turn for right. */
    p.bypass_side = S5_SIDE_LEFT;
    unsigned before_turns = turn_calls;
    assert(s5_navigation_probe_bypass(&p));
    assert(maneuver.state == S5_MAN_OFFSET);
    assert(!s5_navigation_probe_bypass(&p));
    tick(73.04f); /* Actual clear range after the logged left viewpoint. */
    assert(allowed && move_calls > 0u && last_move > 0.0f);
    assert(turn_calls == before_turns && turn_calls == 1u);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    assert(s5_navigation_probe_begin(45));
    finish_leg();
    p.bypass_side = S5_SIDE_RIGHT;
    before_turns = turn_calls;
    assert(s5_navigation_probe_bypass(&p));
    tick(73.04f);
    assert(allowed && last_move > 0.0f && turn_calls == before_turns);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    p.center_y_cm = 15.36f;
    before_turns = turn_calls;
    assert(s5_bridge_request(S5_CMD_GO_RIGHT, &p));
    tick(14.50f); /* COM6: profile 15.36 cm, live front 14.50 cm. */
    assert(turn_calls == before_turns + 1u && last_angle == 45);
    assert(last_turn == MOTION_TURN_RIGHT && allowed);
    finish_leg();
    tick(78.21f);
    assert(allowed && maneuver.state == S5_MAN_OFFSET && last_move > 0.0f);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    p.bypass_side = S5_SIDE_LEFT;
    assert(s5_bridge_request(S5_CMD_GO_LEFT, &p));
    tick(17.16f);
    finish_leg();
    before_turns = turn_calls;
    unsigned before_moves = move_calls;
    tick(17.16f); /* Latest COM6: close echo just after turn completion. */
    assert(maneuver.waiting_clear && s5_navigation_error() == S5_ERR_NONE && !allowed);
    tick(78.21f); /* An isolated clear reading cannot restart movement. */
    tick(17.16f);
    assert(maneuver.clear_hits == 0u && move_calls == before_moves);
    for (unsigned i = 1u; i < ROBOT_CLEAR_CONFIRM; ++i) {
        tick(400.0f);
        assert(!allowed && move_calls == before_moves);
    }
    tick(400.0f); /* Stable no-target/open-space releases forward movement. */
    assert(allowed && move_calls == before_moves + 1u && last_move > 0.0f);
    assert(turn_calls == before_turns);
    tick(ROBOT_STOP_CM);
    assert(!allowed && s5_navigation_error() == S5_ERR_INSUFFICIENT_CLEARANCE);
    puts("PASS captured left viewpoint commits to forward MOVE with no additional turn; close reading stops");
    puts("PASS default navigation permits clear forward off tape and stops on an obstacle");
    return 0;
}
