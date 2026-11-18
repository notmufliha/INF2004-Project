#define main legacy_navigation_main
#include "test_navigation.c"
#undef main
#include "../../app_program/subsystem5/servo_mapping.h"
#include "../../app_program/subsystem5/s5_geometry.h"
#include "../../app_program/subsystem5/s5_startup.h"
static void single_scan_regression(void)
{
    for (unsigned scenario = 0; scenario < 9u; ++scenario) {
        S5Controller c;
        S5Input in = {0};
        s5_controller_init(&c);
        in.front_valid = true; in.front_cm = 45.0f;
        assert(s5_controller_step(&c, &in) == S5_CMD_STOP);
        in.stopped = true;
        assert(s5_controller_step(&c, &in) == S5_CMD_SCAN_COARSE);
        /* Fluctuating live echoes must not restart the sweep. */
        in.front_cm = 70.0f;
        assert(s5_controller_step(&c, &in) == S5_CMD_NONE);
        in.front_cm = 45.0f;
        in.coarse_ready = true; in.coarse.nearest_index = 2;
        for (int i = 0; i < S5_SCAN_COUNT; ++i) {
            in.coarse.points[i].angle_deg = (uint16_t)(70 + i * 10);
            in.coarse.points[i].valid = true;
            in.coarse.points[i].distance_cm = i == 2 ? 45.0f : scenario == 1u ? 6.0f : 90.0f;
        }
        if (scenario == 2u) in.coarse.points[2].distance_cm = 70.0f;
        if (scenario == 3u) in.coarse.points[0].valid = false;
        /* A 5 cm advantage must choose the clearer side, rather than the
         * former 10 cm left preference. Mirror both measured scenes. */
        if (scenario == 6u || scenario == 7u) {
            for (int i = 0; i < S5_SCAN_COUNT; ++i) {
                if (i == 2) continue;
                bool clearer = scenario == 6u ? i > 2 : i < 2;
                in.coarse.points[i].distance_cm = clearer ? 95.0f : 90.0f;
            }
        }
        if (scenario == 8u) {
            const float captured[] = {54.11f, 13.04f, 13.97f, 0.0f, 99.12f};
            for (int i = 0; i < S5_SCAN_COUNT; ++i)
                in.coarse.points[i].distance_cm = captured[i];
            in.coarse.points[3].valid = false;
        }
        S5Command decision = s5_controller_step(&c, &in);
        assert(decision == S5_CMD_SCAN_FINE && c.state == S5_FINE);
        in.coarse_ready = false;
        for (int i = 0; i < S5_SCAN_COUNT; ++i) {
            in.fine.points[i].angle_deg = (uint16_t)(80 + i * 5);
            in.fine.points[i].valid = true;
            in.fine.points[i].distance_cm = 45.0f;
        }
        in.fine_ready = true;
        if (scenario == 4u) {
            in.fine.points[0].valid = in.fine.points[1].valid = in.fine.points[3].valid = false;
        }
        if (scenario == 5u) in.fine.points[2].distance_cm = 5.0f;
        decision = s5_controller_step(&c, &in);
        in.fine_ready = false;
        if (scenario == 0u) assert(decision == S5_CMD_GO_LEFT || decision == S5_CMD_GO_RIGHT);
        else if (scenario == 3u) assert(decision == S5_CMD_GO_RIGHT);
        else if (scenario == 6u) assert(decision == S5_CMD_GO_RIGHT);
        else if (scenario == 7u) assert(decision == S5_CMD_GO_LEFT);
        else if (scenario == 8u) assert(decision == S5_CMD_GO_LEFT); /* Prefer measured clearance. */
        else assert(c.state == S5_WAIT_ROUTE);
        in.coarse_ready = false;
        for (unsigned tick = 0; tick < 500u; ++tick) {
            S5Command next = s5_controller_step(&c, &in);
            assert(next != S5_CMD_SCAN_COARSE && next != S5_CMD_SCAN_FINE);
        }
        if (scenario != 0u && scenario != 3u && scenario < 6u) {
            in.front_cm = 70.0f;
            for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
                S5Command next = s5_controller_step(&c, &in);
                assert(next == (scenario == 2u && i + 1u == ROBOT_CLEAR_CONFIRM ?
                    S5_CMD_RESUME : S5_CMD_NONE)); /* Failed plans remain latched. */
            }
        }
    }
}
int main(void)
{
    {
        S5Controller close;
        S5Input sample = {0};
        s5_controller_init(&close);
        sample.front_valid = true; sample.front_cm = 7.20f;
        assert(s5_controller_step(&close, &sample) == S5_CMD_STOP);
        sample.stopped = true;
        assert(s5_controller_step(&close, &sample) == S5_CMD_SCAN_COARSE);
        const float ranges[] = {7.54f, 7.63f, 7.20f, 7.20f, 7.11f};
        sample.coarse_ready = true;
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            sample.coarse.points[i] = (S5ScanPoint){70 + i * 10, ranges[i], true};
        assert(s5_controller_step(&close, &sample) == S5_CMD_SCAN_FINE);
        assert(close.side == S5_SIDE_LEFT && close.fine_center_deg == 80u);
        sample.coarse_ready = false; sample.fine_ready = true;
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            sample.fine.points[i] = (S5ScanPoint){70 + i * 5, 7.20f, true};
        assert(s5_controller_step(&close, &sample) == S5_CMD_REVERSE);
        assert(close.reverse_attempts == 1u);
        sample.fine_ready = false; sample.motion_complete = true;
        sample.front_cm = 15.20f;
        assert(s5_controller_step(&close, &sample) == S5_CMD_GO_LEFT);
    }
    {
        /* Captured clear scan: front 99.18 cm, error NONE, prior encounter.
         * Confirmed clear-scan cancellation must release the encounter latch. */
        S5Controller clear_scan;
        S5Input sample = {0};
        s5_controller_init(&clear_scan);
        clear_scan.state = S5_WAIT_ROUTE; clear_scan.encounter_front_cm = 40.0f;
        sample.front_valid = sample.stopped = true; sample.front_cm = 99.18f;
        for (unsigned i = 1; i < ROBOT_CLEAR_CONFIRM; ++i)
            assert(s5_controller_step(&clear_scan, &sample) == S5_CMD_NONE);
        assert(s5_controller_step(&clear_scan, &sample) == S5_CMD_RESUME);
        clear_scan.state = S5_WAIT_ROUTE; clear_scan.error = S5_ERR_NO_CLEAR_ROUTE;
        sample.front_cm = 400.0f;
        for (unsigned i = 0; i < 20u; ++i)
            assert(s5_controller_step(&clear_scan, &sample) != S5_CMD_RESUME);
    }
    {
        S5Controller captured;
        S5Input sample = {0};
        s5_controller_init(&captured);
        sample.front_valid = true; sample.front_cm = 40.0f;
        assert(s5_controller_step(&captured, &sample) == S5_CMD_STOP);
        sample.stopped = true;
        assert(s5_controller_step(&captured, &sample) == S5_CMD_SCAN_COARSE);
        const float ranges[] = {8.44f, 7.82f, 8.67f, 96.97f, 97.74f};
        sample.coarse_ready = true; sample.coarse.nearest_index = 1;
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            sample.coarse.points[i] = (S5ScanPoint){70 + i * 10, ranges[i], true};
        assert(s5_controller_step(&captured, &sample) == S5_CMD_SCAN_FINE);
        assert(captured.fine_center_deg == 100u && captured.side == S5_SIDE_RIGHT);
        assert(fabsf(captured.encounter_front_cm - 8.67f) < 0.01f);
        sample.coarse_ready = false; sample.fine_ready = true;
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            sample.fine.points[i] = (S5ScanPoint){90 + i * 5, i ? 97.0f : 8.67f, true};
        assert(s5_controller_step(&captured, &sample) == S5_CMD_REVERSE);
        sample.fine_ready = false; sample.motion_complete = true;
        assert(s5_controller_step(&captured, &sample) == S5_CMD_GO_RIGHT);
    }
    {
        S5Controller retreat;
        S5Input sample = {0};
        s5_controller_init(&retreat);
        sample.front_valid = true; sample.front_cm = 8.0f;
        assert(s5_controller_step(&retreat, &sample) == S5_CMD_STOP);
        sample.stopped = true;
        assert(s5_controller_step(&retreat, &sample) == S5_CMD_SCAN_COARSE);
        sample.coarse_ready = true;
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            sample.coarse.points[i].angle_deg = 70 + i * 10;
        sample.coarse.points[4].valid = true;
        sample.coarse.points[4].distance_cm = 99.39f;
        assert(s5_controller_step(&retreat, &sample) == S5_CMD_SCAN_FINE);
        sample.coarse_ready = false; sample.fine_ready = true;
        sample.fine.points[4] = (S5ScanPoint){110u, 98.85f, true};
        assert(s5_controller_step(&retreat, &sample) == S5_CMD_REVERSE);
        assert(retreat.side == S5_SIDE_RIGHT && retreat.profile.valid);
        sample.fine_ready = false; sample.motion_complete = true;
        sample.front_cm = 400.0f; /* Far assumption cannot resume approach. */
        assert(s5_controller_step(&retreat, &sample) == S5_CMD_GO_RIGHT);
        assert(retreat.state == S5_AVOID && retreat.reverse_attempts == 1u);
    }
    unsigned startup = 0u;
    /* Captured long OUT_OF_RANGE startup pulses never arm movement. */
    for (unsigned i = 0; i < 100u; ++i)
        assert(!s5_startup_ready(&startup, false));
    assert(!s5_startup_ready(&startup, true));
    assert(!s5_startup_ready(&startup, false)); /* Interrupted consensus. */
    for (unsigned i = 1; i < ROBOT_CLEAR_CONFIRM; ++i)
        assert(!s5_startup_ready(&startup, true));
    assert(s5_startup_ready(&startup, true));
    /* The helper must not suppress operational sensor-fault handling. */
    assert(s5_startup_ready(&startup, false));
    assert(ROBOT_SENSOR_FORWARD_OFFSET_CM == 5.0f);
    assert(s5_turn_range_cm(11.42f, 90u) > ROBOT_BYPASS_MIN_TURN_CM);
    assert(s5_turn_range_cm(11.42f, 70u) < s5_turn_range_cm(11.42f, 90u));
    S5Maneuver blocked_turn = {0}; S5ManeuverInput near = {0};
    blocked_turn.state = S5_MAN_TURN_OUT; blocked_turn.side = S5_SIDE_LEFT;
    near.line_valid = near.front_valid = near.stopped = true;
    near.front_cm = ROBOT_BYPASS_FRONT_TURN_CM;
    assert(s5_maneuver_step(&blocked_turn, &near).type == S5_ACT_STOP);
    assert(blocked_turn.state == S5_MAN_FAILED); /* Physical pivot guard remains. */
    single_scan_regression();
    S5Controller c;
    S5Input in = {0};
    assert(ROBOT_OBSTACLE_TEST == 1);
    assert(ROBOT_NO_TARGET_FORWARD == 1 && ROBOT_NO_TARGET_SCAN == 0);
    s5_controller_init(&c);
    in.front_valid = true; in.front_cm = 50.1f;
    assert(s5_controller_step(&c, &in) == S5_CMD_NONE);
    in.front_cm = 50.0f;
    assert(s5_controller_step(&c, &in) == S5_CMD_STOP);
    assert(s5_controller_step(&c, &in) == S5_CMD_STOP);
    in.stopped = true;
    assert(s5_controller_step(&c, &in) == S5_CMD_SCAN_COARSE);
    assert(s5_navigation_init());
    /* Boot between STOP and RELEASE used to wait indefinitely. */
    for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 50.1f, true);
    }
    assert(allowed);
    /* Missing echo in the crash log must not authorize further driving. */
    clock_ms += 65u;
    s5_navigation_tick(false, 400.0f, true);
    assert(!allowed);
    clock_ms += 65u;
    s5_navigation_tick(true, 50.0f, true);
    assert(!allowed && move_calls == 0u);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    assert(reverse_path_clear()); /* Explicit bounded floor-test retreat enabled. */
    assert(c.state == S5_COARSE && move_calls == 0u);
    /* Replay an earlier COM6 scan in logical bearing order. */
    const float coarse_ranges[5] = {12.96f, 11.85f, 11.56f, 11.42f, 11.42f};
    assert(s5_servo_mount_angle(90u) == 90u);
    in.coarse_ready = true;
    in.coarse.nearest_index = 3;
    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
        in.coarse.points[i].angle_deg = (uint16_t)(70 + i * 10);
        in.coarse.points[i].valid = true;
        in.coarse.points[i].distance_cm = coarse_ranges[i];
    }
    S5Command bypass = s5_controller_step(&c, &in);
    assert(bypass == S5_CMD_SCAN_FINE);
    in.coarse_ready = false;
    in.fine_ready = true;
    const float fine_ranges[5] = {11.85f, 11.42f, 11.42f, 11.42f, 11.42f};
    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
        in.fine.points[i].angle_deg = (uint16_t)(90 + i * 5);
        in.fine.points[i].valid = true;
        in.fine.points[i].distance_cm = fine_ranges[i];
    }
    bypass = s5_controller_step(&c, &in);
    assert(bypass == S5_CMD_GO_LEFT);
    assert(s5_bridge_request(bypass, &c.profile));
    tick(11.42f);
    assert(turn_calls == 1u && last_angle == ROBOT_BYPASS_TURN_DEG);
    assert(last_turn == MOTION_TURN_LEFT);
    assert(s5_navigation_error() == S5_ERR_NONE);
    finish_leg(); tick(52.82f);
    /* Rotation must reveal an actually clear corridor before translation. */
    tick(52.82f);
    assert(move_calls == 1u && last_direction == MOTION_DIR_FORWARD);
    assert(last_run_ms == 0u); /* Encoder distance, never truncate at 5 s. */
    assert(s5_navigation_error() == S5_ERR_NONE);
    status.left_distance_mm = status.right_distance_mm = 220.0f;
    tick(ROBOT_BYPASS_MIN_TURN_CM);
    assert(maneuver.forward_paused && !maneuver.reactive && s5_navigation_error() == S5_ERR_NONE && !allowed);
    tick(18.0f);
    assert(turn_calls == 1u); /* Clearance cannot substitute an unplanned turn. */
    tick(18.0f); tick(18.0f); tick(18.0f);
    assert(turn_calls == 2u && last_turn == MOTION_TURN_RIGHT && last_angle == ROBOT_BYPASS_TURN_DEG);
    for (unsigned i = 0; i < 200u && !s5_bridge_line_found(); ++i) {
        if (mode == NAV_AVOID && s5_bridge_motion_complete()) {
            assert(maneuver.state == S5_MAN_COMPLETE && turn_calls == 4u);
            assert(s5_bridge_request(S5_CMD_FIND_LINE, NULL));
        }
        if (maneuver.pending_id) finish_leg();
        if (maneuver.state == S5_MAN_ALIGN_CHECK) {
            maneuver.left_original_line = true;
            line_state.line_flags = B3_LINE_F_DETECTED;
        }
        tick(70.0f);
        if (maneuver.state == S5_MAN_TURN_IN && !maneuver.pending_id) {
            assert(!s5_bridge_motion_complete());
            assert(!s5_bridge_request(S5_CMD_FIND_LINE, NULL));
        }
        if (turn_calls >= 3u && maneuver.state != S5_MAN_COMPLETE)
            assert(mode == NAV_AVOID); /* No mid-route FIND_LINE gate before right turn #2. */
    }
    assert(turn_calls == 4u && last_turn == MOTION_TURN_LEFT && last_angle == ROBOT_BYPASS_TURN_DEG);
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, NULL));
    /* Replay the valid ranges from the crash log: stop at 47.55 cm,
     * before the last 34.96 cm reading and the motor stall. */
    s5_controller_init(&c);
    in = (S5Input){0}; in.front_valid = true;
    const float approach[] = {70.33f, 65.71f, 62.71f, 47.55f};
    for (unsigned i = 0; i < 4u; ++i) {
        in.front_cm = approach[i];
        assert(s5_controller_step(&c, &in) == (i == 3u ? S5_CMD_STOP : S5_CMD_NONE));
    }
    /* Captured 8.59 cm startup: remain stopped, but recover after external
     * repositioning without needing a reboot or 55 cm of front clearance. */
    s5_controller_init(&c);
    c.state = S5_WAIT_ROUTE; c.error = S5_ERR_NO_CLEAR_ROUTE;
    c.last_coarse.points[2].valid = true;
    c.last_coarse.points[2].distance_cm = 8.59f;
    in = (S5Input){0}; in.front_valid = true; in.stopped = true;
    in.front_cm = 8.59f;
    assert(s5_controller_step(&c, &in) == S5_CMD_NONE);
    /* No blind retreat without a chosen route. */
    c.reverse_attempts = 1u;
    /* A failed retreat cannot trigger an unlimited series of reversals. */
    c.state = S5_WAIT_ROUTE; c.error = S5_ERR_NO_CLEAR_ROUTE;
    for (unsigned i = 0; i < 100u; ++i)
        assert(s5_controller_step(&c, &in) == S5_CMD_NONE);
    assert(c.state == S5_WAIT_ROUTE);
    in.front_cm = 15.0f;
    for (unsigned i = 1; i < ROBOT_CLEAR_CONFIRM; ++i)
        assert(s5_controller_step(&c, &in) == S5_CMD_NONE);
    assert(s5_controller_step(&c, &in) == S5_CMD_SCAN_COARSE);
    assert(c.state == S5_COARSE);
    assert(s5_bridge_request(S5_CMD_STOP, NULL));
    assert(s5_bridge_request(S5_CMD_REVERSE, NULL));
    unsigned before_reverse = move_calls;
    tick(8.59f);
    assert(move_calls == before_reverse + 1u);
    assert(last_direction == MOTION_DIR_REVERSE);
    finish_leg(); tick(16.59f);
    assert(s5_bridge_motion_complete());
    puts("PASS startup close guard and reposition recovery, exact bypass sequence");
    return 0;
}
