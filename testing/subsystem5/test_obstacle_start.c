#define main legacy_navigation_main
#include "test_navigation.c"
#undef main
#include "../../app_program/subsystem5/servo_mapping.h"
static void single_scan_regression(void)
{
    for (unsigned scenario = 0; scenario < 6u; ++scenario) {
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
            in.coarse.points[i].distance_cm = i == 2 ? 45.0f : scenario == 1u ? 13.0f : 90.0f;
        }
        if (scenario == 2u) in.coarse.points[2].distance_cm = 70.0f;
        if (scenario == 3u) in.coarse.points[0].valid = false;
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
        if (scenario == 5u) in.fine.points[2].distance_cm = 10.0f;
        decision = s5_controller_step(&c, &in);
        in.fine_ready = false;
        if (scenario == 0u) assert(decision == S5_CMD_GO_LEFT || decision == S5_CMD_GO_RIGHT);
        else if (scenario == 3u) assert(decision == S5_CMD_GO_RIGHT);
        else assert(c.state == S5_WAIT_ROUTE);
        in.coarse_ready = false;
        for (unsigned tick = 0; tick < 500u; ++tick) {
            S5Command next = s5_controller_step(&c, &in);
            assert(next != S5_CMD_SCAN_COARSE && next != S5_CMD_SCAN_FINE);
        }
        if (scenario != 0u && scenario != 3u) {
            in.front_cm = 70.0f;
            for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) {
                S5Command next = s5_controller_step(&c, &in);
                assert(next == (i + 1u == ROBOT_CLEAR_CONFIRM ? S5_CMD_RESUME : S5_CMD_NONE));
            }
        }
    }
}
int main(void)
{
    single_scan_regression();
    S5Controller c;
    S5Input in = {0};
    assert(ROBOT_OBSTACLE_TEST == 1);
    assert(ROBOT_NO_TARGET_FORWARD == 0 && ROBOT_NO_TARGET_SCAN == 0);
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
    assert(!s5_bridge_request(S5_CMD_REVERSE, NULL));
    assert(c.state == S5_COARSE && move_calls == 0u);
    /* Replay the latest COM6 scan through the mirrored mount mapping. */
    const float shaft_ranges[5] = {19.32f, 18.26f, 18.26f, 17.75f, 17.66f};
    assert(s5_servo_mount_angle(90u) == 90u);
    assert(s5_servo_mount_angle(70u) == 110u);
    assert(s5_servo_mount_angle(110u) == 70u);
    in.coarse_ready = true;
    in.coarse.nearest_index = 0;
    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
        in.coarse.points[i].angle_deg = (uint16_t)(70 + i * 10);
        in.coarse.points[i].valid = true;
        unsigned physical = s5_servo_mount_angle(in.coarse.points[i].angle_deg);
        in.coarse.points[i].distance_cm = shaft_ranges[(physical - 70u) / 10u];
    }
    S5Command bypass = s5_controller_step(&c, &in);
    assert(bypass == S5_CMD_SCAN_FINE);
    in.coarse_ready = false;
    in.fine_ready = true;
    const float fine_ranges[5] = {17.80f, 17.66f, 17.75f, 17.40f, 18.26f};
    for (int i = 0; i < S5_SCAN_COUNT; ++i) {
        in.fine.points[i].angle_deg = (uint16_t)(70 + i * 5);
        in.fine.points[i].valid = true;
        in.fine.points[i].distance_cm = fine_ranges[i];
    }
    bypass = s5_controller_step(&c, &in);
    assert(bypass == S5_CMD_GO_LEFT);
    assert(s5_bridge_request(bypass, &c.profile));
    tick(18.26f);
    assert(turn_calls == 1u && last_angle == ROBOT_BYPASS_TURN_DEG);
    assert(last_turn == MOTION_TURN_LEFT);
    assert(s5_navigation_error() == S5_ERR_NONE);
    finish_leg(); tick(18.26f);
    tick(18.26f);
    assert(move_calls == 0u && maneuver.waiting_clear);
    /* Rotation must reveal an actually clear corridor before translation. */
    for (unsigned i = 0; i < ROBOT_CLEAR_CONFIRM; ++i) tick(38.39f);
    assert(move_calls == 1u && last_direction == MOTION_DIR_FORWARD);
    assert(s5_navigation_error() == S5_ERR_NONE);
    tick(ROBOT_BYPASS_MIN_TURN_CM);
    assert(s5_navigation_error() == S5_ERR_INSUFFICIENT_CLEARANCE && !allowed);
    /* Replay the valid ranges from the crash log: stop at 47.55 cm,
     * before the last 34.96 cm reading and the motor stall. */
    s5_controller_init(&c);
    in = (S5Input){0}; in.front_valid = true;
    const float approach[] = {70.33f, 65.71f, 62.71f, 47.55f};
    for (unsigned i = 0; i < 4u; ++i) {
        in.front_cm = approach[i];
        assert(s5_controller_step(&c, &in) == (i == 3u ? S5_CMD_STOP : S5_CMD_NONE));
    }
    puts("PASS obstacle startup: 50 cm stop, mirrored scanner chooses LEFT, forward continuation, 14 cm guard, no reverse");
    return 0;
}
