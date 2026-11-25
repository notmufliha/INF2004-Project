/* Exercise bypass and line recovery through the real navigation adapter. */
#define main legacy_navigation_main
#include "test_navigation.c"
#undef main
#include <math.h>

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
    assert(last_angle == ROBOT_COURSE_RETURN_TURN_DEG);
    assert(last_turn == (side == S5_SIDE_LEFT ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT));
    tick(120.0f);
    assert(maneuver.state == S5_MAN_TURN_IN && !s5_bridge_line_found());
    finish_leg();
    line_state.line_flags = 0u;
    if (maneuver.guarded_probe_route) {
        tick(14.8f);
        assert(!allowed && !maneuver.pending_id);
        for (unsigned n = 1u; n < ROBOT_CLEAR_CONFIRM; ++n) {
            tick(120.0f);
            assert(!maneuver.pending_id);
        }
    }
    tick(120.0f);
    assert(maneuver.state == S5_MAN_SEARCH && maneuver.pending_id);
    assert(last_move == maneuver.search_limit_cm);
    const float radians = 0.01745329252f;
    const int pass_heading = ROBOT_COURSE_BYPASS_TURN_DEG - ROBOT_COURSE_PASS_TURN_DEG;
    const float lateral_return = last_move *
        sinf((ROBOT_COURSE_RETURN_TURN_DEG - pass_heading) * radians);
    const float lateral_offset = maneuver.offset_done_cm *
        sinf(ROBOT_COURSE_BYPASS_TURN_DEG * radians) + maneuver.pass_done_cm *
        sinf(pass_heading * radians);
    assert(fabsf(lateral_return - lateral_offset) < 0.01f);
    assert(last_direction == MOTION_DIR_FORWARD);
    line_state.line_flags = tape_present ? B3_LINE_F_DETECTED : 0u;
    tick(120.0f);
    assert(!s5_bridge_line_found());
    if (!tape_present) {
        finish_leg();
        assert(maneuver.state == S5_MAN_FAILED && maneuver.error == S5_ERR_LINE_TIMEOUT);
        assert(!s5_bridge_request(S5_CMD_RESUME, NULL));
        return;
    }
    assert(maneuver.state == S5_MAN_LINE_STOP && maneuver.pending_id && !allowed);
    result = status;
    result.physically_stopped = false; /* The cancelled result predates live STOP. */
    for (unsigned n = 0u; n < ROBOT_BYPASS_LINE_CONFIRM; ++n) tick(120.0f);
    unsigned align_turns = maneuver.guarded_probe_route ? 0u :
        finish_rejoin_align(side);
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, NULL));
    assert(mode == NAV_FOLLOW && turn_calls == old_turns + 1u + align_turns);
}

static void test_probe_route(S5Side side, bool tape_present)
{
    /* Replay the COM6 obstacle profile, an initial measured corridor,
     * and a close echo during the continuous outward leg. */
    S5Profile p = {.valid = true, .uses_course_bounds = true,
                   .estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM,
                   .center_y_cm = 29.18f, .bypass_side = side};
    int outward = side == S5_SIDE_LEFT ? -ROBOT_COURSE_BYPASS_TURN_DEG : ROBOT_COURSE_BYPASS_TURN_DEG;
    unsigned moves = move_calls, turns = turn_calls;
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    assert(s5_navigation_probe_begin(outward));
    assert(last_angle == ROBOT_COURSE_BYPASS_TURN_DEG);
    assert(last_turn == (side == S5_SIDE_LEFT ? MOTION_TURN_LEFT : MOTION_TURN_RIGHT));
    finish_leg();
    assert(s5_navigation_probe_bypass(&p));
    assert(maneuver.offset_cm == ROBOT_COURSE_BYPASS_LEG_CM);
    line_state.line_flags = 0u;
    tick(ROBOT_BYPASS_MOVE_RELEASE_CM + ROBOT_COURSE_BYPASS_LEG_CM + 5.0f);
    assert(move_calls == moves + 1u && turn_calls == turns + 1u);
    assert(last_direction == MOTION_DIR_FORWARD && last_move == ROBOT_COURSE_BYPASS_LEG_CM);
    unsigned interrupted_id = status.command_id;
    tick(15.13f);
    assert(allowed && maneuver.pending_id == interrupted_id);
    tick(15.0f);
    assert(!allowed && maneuver.pending_id == interrupted_id && turn_calls == turns + 1u);
    result = status;
    result.left_distance_mm = result.right_distance_mm = 80.0f;
    tick(400.0f);
    assert(!allowed && maneuver.pending_id == 0u && maneuver.offset_done_cm == 8.0f);
    for (unsigned n = 1u; n < ROBOT_CLEAR_CONFIRM; ++n) {
        tick(400.0f);
        assert(!allowed && move_calls == moves + 1u);
    }
    tick(400.0f);
    assert(move_calls == moves + 2u && turn_calls == turns + 1u);
    assert(last_move == ROBOT_COURSE_BYPASS_LEG_CM - 8.0f);
    status.left_distance_mm = status.right_distance_mm = last_move * 10.0f + 5.0f;
    finish_leg();
    assert(maneuver.offset_done_cm == ROBOT_COURSE_BYPASS_LEG_CM + 0.5f);
    finish_middle(side);
    assert(turn_calls == turns + 2u && move_calls == moves + 3u);
    recover_line(side, tape_present);

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
    S5Controller obstacle;
    s5_controller_init(&obstacle);
    S5Input input = {.front_valid = true, .front_cm = 14.47f};
    clock_ms += 65u;
    s5_navigation_tick(input.front_valid, input.front_cm, true);
    assert(!allowed && !s5_navigation_has_line_reference());
    assert(s5_controller_step(&obstacle, &input) == S5_CMD_STOP);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    input.stopped = s5_navigation_stopped();
    assert(s5_controller_step(&obstacle, &input) == S5_CMD_SCAN_COARSE);
    const S5ScanPoint width_points[S5_WIDTH_SCAN_COUNT] = {
        {30u, 95.0f, true}, {45u, 95.0f, true}, {60u, 90.0f, true},
        {75u, 14.47f, true}, {90u, 14.47f, true}, {105u, 14.9f, true},
        {120u, 55.0f, true}
    };
    input.coarse_ready = true;
    input.width_points = width_points;
    input.coarse.points[0] = width_points[1];
    input.coarse.points[4] = width_points[6];
    assert(s5_controller_step(&obstacle, &input) == S5_CMD_GO_LEFT);
    assert(obstacle.profile.valid && obstacle.profile.estimated_width_cm > 0.0f);
    /* COM6 sweep: equal outer readings, close obstacle on the left flank. */
    const S5ScanPoint logged_sweep[S5_WIDTH_SCAN_COUNT] = {
        {30u, 400.0f, true}, {45u, 400.0f, true}, {60u, 13.10f, true},
        {75u, 11.90f, true}, {90u, 12.75f, true}, {105u, 400.0f, true},
        {120u, 400.0f, true}
    };
    s5_controller_init(&obstacle);
    obstacle.state = S5_COARSE;
    input.front_cm = 13.18f;
    input.width_points = logged_sweep;
    input.coarse = (S5Scan){.points = {logged_sweep[1], logged_sweep[2],
                                     logged_sweep[4], logged_sweep[5], logged_sweep[6]}};
    assert(s5_controller_step(&obstacle, &input) == S5_CMD_GO_RIGHT);
    assert(obstacle.profile.left_clearance_cm == 11.90f);
    assert(obstacle.profile.right_clearance_cm == 400.0f);
    assert(s5_navigation_probe_begin(-ROBOT_COURSE_BYPASS_TURN_DEG));
    assert(last_turn == MOTION_TURN_LEFT && !s5_navigation_has_line_reference());
    s5_navigation_pause();
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
        assert(last_move == ROBOT_COURSE_BYPASS_LEG_CM);
        finish_middle(p.bypass_side);
        recover_line(p.bypass_side, true);
        for (unsigned n = 0; n < ROBOT_CLEAR_CONFIRM; ++n) {
            clock_ms += 65u;
            s5_navigation_tick(true, 120.0f, true);
        }
        assert(allowed && drive_calls == 0u); /* VC regains DRIVE ownership. */
    }
    test_probe_route(S5_SIDE_LEFT, true);
    test_probe_route(S5_SIDE_RIGHT, true);
    test_probe_route(S5_SIDE_LEFT, false);
    puts("PASS real navigation: mirrored bypasses, guarded drive, and tape-confirmed return");
    return 0;
}
