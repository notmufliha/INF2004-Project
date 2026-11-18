/* Exercise the real adapter through the early completion bridge events. */
#define main legacy_navigation_main
#include "test_navigation.c"
#undef main
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
        p.center_y_cm = 25.0f;
        p.bypass_side = direction ? S5_SIDE_RIGHT : S5_SIDE_LEFT;
        assert(s5_bridge_request(S5_CMD_STOP, NULL));
        assert(s5_bridge_request(direction ? S5_CMD_GO_RIGHT : S5_CMD_GO_LEFT, &p));
        line_state.line_flags = 0u;
        tick(120.0f); /* Submit outward turn. */
        finish_leg();
        assert(maneuver.outward_turn_complete);
        tick(120.0f); /* Submit offset move. */
        if (direction) {
            /* A black line must also preempt the paused return-turn gate. */
            maneuver.state = S5_MAN_TURN_IN;
            maneuver.pending_id = 0u;
            status.physically_stopped = true;
        }
        unsigned old_turns = turn_calls;
        line_state.line_flags = B3_LINE_F_DETECTED;
        tick(120.0f);
        assert(!allowed && maneuver.state == S5_MAN_HANDOFF_STOP);
        for (unsigned n = 0; n < ROBOT_BYPASS_LINE_CONFIRM; ++n) tick(120.0f);
        assert(maneuver.state == S5_MAN_COMPLETE);
        assert(s5_bridge_motion_complete());
        assert(s5_bridge_request(S5_CMD_FIND_LINE, NULL));
        assert(s5_bridge_line_found());
        assert(s5_bridge_request(S5_CMD_RESUME, NULL));
        assert(mode == NAV_FOLLOW && turn_calls == old_turns);
        for (unsigned n = 0; n < ROBOT_CLEAR_CONFIRM; ++n) {
            clock_ms += 65u;
            s5_navigation_tick(true, 120.0f, true);
        }
        assert(allowed && drive_calls == 0u); /* VC regains DRIVE ownership. */
    }
    puts("PASS real navigation: left/right early line detection, STOP, bridge handoff and resume");
    return 0;
}
