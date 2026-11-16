#include "s5_maneuver.h"
#include <assert.h>
#include <stdio.h>

static S5ManeuverInput input(unsigned time, bool black, bool stopped)
{
    S5ManeuverInput in = {0};
    in.now_ms = in.line_sample_ms = time;
    in.line_valid = true;
    in.line_detected = black;
    in.front_valid = true;
    in.front_cm = 100.0f;
    in.stopped = stopped;
    return in;
}
int main(void)
{
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        S5Maneuver m = {0};
        S5Profile p = {0};
        p.valid = true;
        p.estimated_width_cm = 10.0f;
        p.center_y_cm = 25.0f;
        p.bypass_side = (S5Side)side;
        assert(s5_maneuver_begin(&m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, 20.0f, 100u));
        S5ManeuverInput in = input(110u, true, true);
        S5Action a = s5_maneuver_step(&m, &in);
        assert(a.type == S5_ACT_TURN); /* Original line cannot end bypass. */
        assert(s5_maneuver_submitted(&m, 1u));
        in = input(120u, false, false);
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
        in = input(130u, false, true);
        in.motion_terminal = in.motion_success = true;
        in.result_id = 1u;
        (void)s5_maneuver_step(&m, &in);
        assert(m.outward_turn_complete && m.state == S5_MAN_OFFSET);
        in = input(140u, false, true);
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_MOVE);
        assert(s5_maneuver_submitted(&m, 2u));
        in = input(150u, true, false);
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
        assert(m.state == S5_MAN_HANDOFF_STOP && !m.pending_id);
        /* Duplicate samples and moving samples cannot confirm the handoff. */
        for (int n = 0; n < 5; ++n) (void)s5_maneuver_step(&m, &in);
        assert(!m.line_hits);
        for (unsigned n = 1; n <= ROBOT_BYPASS_LINE_CONFIRM; ++n) {
            in = input(150u + 10u * n, true, true);
            assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
        }
        assert(m.state == S5_MAN_COMPLETE);
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
        m.state = S5_MAN_OFFSET;
        in = input(300u, true, false);
        in.line_sample_ms = 100u;
        assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
        assert(m.state == S5_MAN_FAILED && m.error == S5_ERR_LINE_TIMEOUT);
    }
    const S5ManeuverState states[] = {
        S5_MAN_OFFSET, S5_MAN_FORWARD, S5_MAN_PASS, S5_MAN_TURN_IN,
        S5_MAN_SEARCH, S5_MAN_LINE_STOP, S5_MAN_ALIGN, S5_MAN_ALIGN_CHECK,
        S5_MAN_REPLAN_STOP, S5_MAN_REPLAN_TURN, S5_MAN_REPLAN_CHECK,
        S5_MAN_REPLAN_ADVANCE
    };
    for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); ++i) {
        for (unsigned active = 0; active < 2; ++active) {
            S5Maneuver m = {0};
            m.state = states[i];
            m.outward_turn_complete = m.left_original_line = true;
            m.pending_id = active ? 42u : 0u;
            S5ManeuverInput in = input(100u, true, !active);
            assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
            assert(m.state == S5_MAN_HANDOFF_STOP && !m.pending_id);
            for (unsigned n = 1; n <= ROBOT_BYPASS_LINE_CONFIRM; ++n) {
                in = input(100u + 10u * n, true, true);
                assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP);
            }
            assert(m.state == S5_MAN_COMPLETE);
        }
    }
    puts("PASS black-line handoff throughout bypass moves, later turns and recovery; stale/duplicate rejection");
    return 0;
}
