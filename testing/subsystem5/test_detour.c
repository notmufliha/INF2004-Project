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
        p.estimated_width_cm = 30.0f; p.center_y_cm = 19.03f;
        p.bypass_side = (S5Side)side;
        s5_maneuver_init(&m);
        assert(s5_maneuver_begin(&m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                                 &p, 40.0f, 0u));
        assert(ROBOT_BYPASS_TURN_DEG == 30);
        assert(fabsf(m.offset_cm * sinf(30.0f * 0.01745329252f) - m.lateral_cm) < 0.01f);
        unsigned turns = 0u, moves = 0u;
        int heading = 0;
        int outward = side == S5_SIDE_LEFT ? -ROBOT_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
        const int expected[] = {outward, -outward, -outward, outward};
        in.front_valid = in.line_valid = in.stopped = true;
        in.front_cm = 38.39f; /* Actual stationary reading after COM6 turn. */
        /* Tape must not truncate the encoder-measured test detour. */
        in.line_detected = true;
        for (unsigned tick = 0; tick < 1000u && m.state != S5_MAN_COMPLETE; ++tick) {
            in.now_ms += 65u; in.line_sample_ms = in.now_ms;
            S5Action a = s5_maneuver_step(&m, &in);
            assert(m.state != S5_MAN_FAILED);
            in.motion_terminal = false;
            if (a.type == S5_ACT_TURN || a.type == S5_ACT_MOVE) {
                if (a.type == S5_ACT_TURN) {
                    assert(turns < 4u && a.turn_deg == expected[turns++]);
                    heading += a.turn_deg;
                } else { assert(a.distance_cm > 0.0f); ++moves; }
                assert(s5_maneuver_submitted(&m, tick + 1u));
                in.result_id = tick + 1u;
                in.motion_terminal = in.motion_success = true;
            }
        }
        assert(m.state == S5_MAN_COMPLETE && turns == 4u && moves > 3u && heading == 0);
        assert(m.offset_done_cm + 0.01f >= m.offset_cm);
        assert(m.pass_done_cm + 0.01f >= m.pass_cm);
        assert(m.searched_cm + 0.01f >= m.offset_cm);
    }
    puts("PASS complete left/right detours at logged 38.39 cm: forward legs, four turns, original heading, no tape dependency");
    return 0;
}
