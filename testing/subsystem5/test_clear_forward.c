#define main legacy_maneuver_main
#include "test_maneuver.c"
#undef main
int main(void)
{
    assert(ROBOT_CLEAR_PATH_FORWARD == 1 && ROBOT_BYPASS_TURN_DEG == 45);
    recovery(S5_SIDE_LEFT, false);
    recovery(S5_SIDE_RIGHT, false);
    for (unsigned side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; ++side) {
        reset((S5Side)side);
        assert(fabsf(m.offset_cm * 0.70710678f - m.lateral_cm) < 0.01f);
        assert(fabsf(m.offset_cm * 0.70710678f + m.pass_cm - 83.0f) < 0.01f);
        assert(m.pass_cm == 63.0f); /* Count the diagonal's 20-cm forward progress. */
        int outward = side == S5_SIDE_LEFT ? -45 : 45;
        leg(S5_ACT_TURN, 0.0f, outward);
        timed_offset();
        leg(S5_ACT_TURN, 0.0f, -outward);
        leg(S5_ACT_MOVE, m.pass_cm, 0);
        leg(S5_ACT_TURN, 0.0f, -outward);
        in.line_detected = false;
        m.searched_cm = m.search_limit_cm;
        assert(tick().type == S5_ACT_STOP && m.state == S5_MAN_ALIGN);
        leg(S5_ACT_TURN, 0.0f, outward);
        in.now_ms += ROBOT_BYPASS_ALIGN_OBSERVE_MS;
        assert(tick().type == S5_ACT_NONE && m.state == S5_MAN_COMPLETE);
    }
    puts("PASS default 45-degree geometry, both bypass directions, off-line forward handoff");
    return 0;
}
