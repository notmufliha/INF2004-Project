#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#define SUBSYSTEM5_H
#define S5_SERVO_ENABLED 1
#define S5_HARDWARE_ENABLED 1
#define S5_SIMULATION 0
typedef unsigned char UB;
static unsigned move_calls, read_calls;
static unsigned fail_move, invalid_read;
static unsigned delay_total_ms;
static uint16_t moved[6];
static void tm_printf(UB *format, ...) { (void)format; }
static void tk_dly_tsk(unsigned ms) { assert(ms > 0u); delay_total_ms += ms; }
bool s5_servo_angle(uint16_t angle)
{
    assert(angle >= 70u && angle <= 110u);
    moved[move_calls++] = angle;
    return move_calls != fail_move;
}
bool s5_measure_scan_cm(float *cm)
{
    static const float distances[5] = {100.0f, 25.0f, 20.0f, 26.0f, 100.0f};
    *cm = distances[read_calls++];
    return read_calls != invalid_read;
}
#include "../../app_program/subsystem5/obstacle_scan.c"
static void reset(void)
{
    move_calls = read_calls = fail_move = invalid_read = 0u;
    delay_total_ms = 0u;
}
int main(void)
{
    S5Scan s = {0};
    S5Profile p;
    uint16_t angles[5];
    reset();
    s5_coarse_scan(&s);
    assert(move_calls == 6u && read_calls == 5u && moved[5] == 90u);
    assert(delay_total_ms == 6u * 200u); /* Gentle settling, no redundant inter-ray delay. */
    assert(moved[0] == 70u && moved[4] == 110u && s.nearest_index == 2);
    p = s5_estimate_profile(&s, S5_SIDE_NONE);
    assert(p.valid && p.size == S5_SIZE_LARGE && p.nearest_cm == 20.0f);
    assert(p.nearest_forward_cm > 19.9f && p.nearest_forward_cm <= 20.0f);
    reset();
    s5_fine_scan(&s, 90u);
    assert(moved[0] == 80u && moved[1] == 85u && moved[4] == 100u);
    p = s5_estimate_profile(&s, S5_SIDE_NONE);
    assert(p.valid && p.size == S5_SIZE_SMALL);
    {
        S5Scan coarse = s;
        for (int i = 0; i < 5; ++i) {
            coarse.points[i].angle_deg = (uint16_t)(70 + 10 * i);
            coarse.points[i].distance_cm = i == 0 || i == 4 ? 100.0f : 30.0f;
            s.points[i].distance_cm = 30.0f; /* Fine window clips a wider object. */
        }
        p = s5_profile_from_scans(&coarse, &s, S5_SIDE_NONE);
        assert(p.valid && p.size == S5_SIZE_LARGE);
        s.points[2].valid = false;
        assert(!s5_profile_from_scans(&coarse, &s, S5_SIDE_NONE).valid);
        s.points[2].valid = true;
        for (int i = 0; i < 5; ++i) s.points[i].distance_cm = 100.0f;
        assert(!s5_profile_from_scans(&coarse, &s, S5_SIDE_NONE).valid); /* Obstacle moved away. */
        s.points[0].distance_cm = s.points[4].distance_cm = 20.0f;
        assert(!s5_profile_from_scans(&coarse, &s, S5_SIDE_NONE).valid); /* Disconnected hits. */
    }
    s5_make_fine_angles(0u, angles);
    assert(angles[0] == 70u && angles[4] == 90u);
    s5_make_fine_angles(180u, angles);
    assert(angles[0] == 90u && angles[4] == 110u);
    {
        S5Scan wide = {0}, clipped = {0};
        const uint16_t broad_angles[] = {70u, 80u, 90u, 100u, 110u};
        const float logged[] = {24.30f, 23.52f, 24.81f, 23.52f, 160.29f};
        S5Controller controller;
        S5Input input = {0};
        for (int i = 0; i < 5; ++i) {
            wide.points[i] = (S5ScanPoint){broad_angles[i], logged[i], true};
            clipped.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 5), i == 0 ? 160.30f : 21.90f, true};
        }
        wide.nearest_index = clipped.nearest_index = 2;
        p = s5_profile_from_scans(&wide, &clipped, S5_SIDE_NONE);
        assert(!p.valid); /* Actual clipped scans from the user's capture. */
        /* The historical 24.81 cm centre is now above the release range. */
        assert(!s5_profile_from_course_bounds(&wide, &clipped).valid);
        wide.points[2].distance_cm = ROBOT_RELEASE_CM - 1.0f;
        p = s5_profile_from_course_bounds(&wide, &clipped);
        assert(p.valid && p.uses_course_bounds);
        assert(p.estimated_width_cm == ROBOT_OBSTACLE_MAX_BYPASS_CM);
        assert(p.bypass_side == S5_SIDE_RIGHT);
        assert(p.required_lateral_clearance_cm == 46.0f);
        s5_controller_init(&controller);
        input.front_valid = true; input.front_cm = ROBOT_STOP_CM - 1.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_STOP);
        input.stopped = true;
        assert(s5_controller_step(&controller, &input) == S5_CMD_SCAN_COARSE);
        input.coarse_ready = true; input.coarse = wide;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        assert(controller.state == S5_AVOID); /* No fine scan required. */
        assert(controller.profile.uses_course_bounds);
        input.motion_complete = true;
        assert(s5_controller_step(&controller, &input) == S5_CMD_FIND_LINE);
        input.line_found = true;
        assert(s5_controller_step(&controller, &input) == S5_CMD_RESUME);
        /* Mirror room preference; execution tests cover both turn signs. */
        wide.points[0].distance_cm = 170.0f;
        p = s5_profile_from_course_bounds(&wide, &clipped);
        assert(p.valid && p.bypass_side == S5_SIDE_LEFT);
        wide.points[0].valid = false;
        assert(!s5_profile_from_course_bounds(&wide, &clipped).valid);
        wide.points[0].valid = true;
        wide.points[0].distance_cm = wide.points[4].distance_cm = 22.0f;
        assert(!s5_profile_from_course_bounds(&wide, &clipped).valid);
        wide.points[4].distance_cm = 160.0f;
        wide.points[2].distance_cm = ROBOT_BYPASS_MIN_TURN_CM; /* Too close to turn. */
        assert(!s5_profile_from_course_bounds(&wide, &clipped).valid);
        wide.points[2].distance_cm = 24.0f;
        clipped.points[0].valid = false;
        assert(!s5_profile_from_course_bounds(&wide, &clipped).valid);
        clipped.points[0].valid = true;
        /* Previously this repeated scans: the centre moved above STOP
         * during scanning and an open side was below the old 40 cm gate. */
        wide.points[2].distance_cm = ROBOT_STOP_CM + 1.0f;
        wide.points[0].distance_cm = 28.0f;
        wide.points[4].distance_cm = 24.0f;
        p = s5_profile_from_course_bounds(&wide, &clipped);
        assert(p.valid && p.bypass_side == S5_SIDE_LEFT);
        s5_controller_init(&controller);
        controller.state = S5_FINE;
        controller.last_coarse = wide;
        input.fine = clipped; input.fine_ready = true;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.state == S5_AVOID);
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse = wide;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.state == S5_AVOID);
        wide.points[0].distance_cm = 24.0f;
        wide.points[4].distance_cm = 28.0f;
        p = s5_profile_from_course_bounds(&wide, &clipped);
        assert(p.valid && p.bypass_side == S5_SIDE_RIGHT);
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse = wide;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
    }
    {
        /* Measured width is valid, but the old two-ray corridor test
         * rejects it. An observed outer opening must produce a TURN,
         * never the old REVERSE fallback. */
        S5Input input = {0};
        S5Controller controller;
        const float fine_distances[] = {100.0f, 22.0f, 22.0f, 22.0f, 100.0f};
        input.front_valid = input.stopped = input.fine_ready = true;
        input.front_cm = ROBOT_RELEASE_CM - 1.0f;
        for (unsigned mirror = 0u; mirror < 2u; ++mirror) {
            s5_controller_init(&controller);
            controller.state = S5_FINE;
            for (int i = 0; i < 5; ++i) {
                controller.last_coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), 22.0f, true};
                input.fine.points[i] = (S5ScanPoint){(uint16_t)(80 + 5 * i), fine_distances[i], true};
            }
            controller.last_coarse.points[0].distance_cm = mirror ? 60.0f : 80.0f;
            controller.last_coarse.points[4].distance_cm = mirror ? 80.0f : 60.0f;
            assert(s5_controller_step(&controller, &input) ==
                   (mirror ? S5_CMD_GO_RIGHT : S5_CMD_GO_LEFT));
            assert(controller.state == S5_AVOID && controller.reverse_attempts == 0u);
        }
        /* Reproduce wrong-right choice: left outer ray has more room,
         * left inner ray clips the obstacle, right has two clear rays. */
        s5_controller_init(&controller);
        controller.state = S5_FINE;
        for (int i = 0; i < 5; ++i)
            controller.last_coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), 22.0f, true};
        controller.last_coarse.points[0].distance_cm = 200.0f;
        controller.last_coarse.points[3].distance_cm = 150.0f;
        controller.last_coarse.points[4].distance_cm = 150.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.side == S5_SIDE_LEFT);
        s5_controller_init(&controller);
        controller.state = S5_FINE;
        for (int i = 0; i < 5; ++i)
            controller.last_coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), 22.0f, true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_STOP);
        assert(controller.state == S5_WAIT_ROUTE && controller.reverse_attempts == 0u);
        for (unsigned i = 1u; i < S5_ROUTE_RESCAN_TICKS; ++i)
            assert(s5_controller_step(&controller, &input) == S5_CMD_NONE);
        assert(s5_controller_step(&controller, &input) == S5_CMD_SCAN_COARSE);
        assert(controller.state == S5_COARSE);
        /* Removing the obstacle while paused requires confirmed clearance. */
        controller.state = S5_WAIT_ROUTE;
        controller.ticks_in_state = 0u;
        input.front_cm = ROBOT_RELEASE_CM + 5.0f;
        for (unsigned i = 1u; i < ROBOT_CLEAR_CONFIRM; ++i)
            assert(s5_controller_step(&controller, &input) == S5_CMD_NONE);
        assert(s5_controller_step(&controller, &input) == S5_CMD_RESUME);
        assert(controller.state == S5_DONE);
    }
    {
        S5Controller controller;
        S5Input input = {0};
        const float captured[] = {16.94f, 152.39f, 20.46f, 152.79f, 18.86f};
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse_ready = input.front_valid = input.stopped = true;
        input.front_cm = 17.23f;
        for (int i = 0; i < 5; ++i)
            input.coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), captured[i], true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.state == S5_AVOID && controller.profile.uses_course_bounds);
        assert(fabsf(controller.profile.left_clearance_cm - 152.39f) < 0.01f);
        assert(fabsf(controller.profile.right_clearance_cm - 152.79f) < 0.01f);
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse.points[0].distance_cm = 8.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        input.coarse.points[0].distance_cm = captured[0];
        /* The blocked outer rays cannot erase the valid inner openings. */
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse.points[1].valid = false;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        /* The later 5.33 cm frame must never authorize a turn. */
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        for (int i = 0; i < 5; ++i) {
            input.coarse.points[i].distance_cm = 5.33f;
            input.coarse.points[i].valid = true;
        }
        S5Command command = s5_controller_step(&controller, &input);
        assert(command != S5_CMD_GO_LEFT && command != S5_CMD_GO_RIGHT);
    }
    reset();
    fail_move = 2u;
    s5_coarse_scan(&s);
    assert(s.nearest_index == -1 && read_calls == 1u);
    for (int i = 0; i < 5; ++i) assert(!s.points[i].valid);
    reset();
    fail_move = 6u; /* Failed recenter makes the entire scan unusable. */
    s5_coarse_scan(&s);
    assert(s.nearest_index == -1);
    reset();
    invalid_read = 3u;
    s5_coarse_scan(&s);
    assert(!s.points[2].valid && s.nearest_index == 1);
    assert(!s5_estimate_profile(&s, S5_SIDE_LEFT).valid);
    s.points[0].distance_cm = NAN;
    s.points[1].distance_cm = INFINITY;
    s.points[3].distance_cm = -1.0f;
    s.points[4].distance_cm = 0.0f;
    assert(s5_find_nearest(s.points, 5) == -1);
    assert(s5_find_nearest(NULL, 5) == -1);
    assert(s5_find_nearest(s.points, 0) == -1);
    {
        S5Scan captured = {0};
        const float cm[5] = {13.70f, 14.05f, 14.19f, 13.82f, 14.59f};
        for (int i = 0; i < 5; ++i)
            captured.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 10), cm[i], true};
        assert(s5_probe_side(&captured) == S5_SIDE_NONE); /* Both sides below clearance. */
        captured.points[3].distance_cm = 25.0f;
        assert(s5_probe_side(&captured) == S5_SIDE_RIGHT); /* Do not default left. */
        captured.points[0].distance_cm = captured.points[1].distance_cm = 60.0f;
        assert(s5_probe_side(&captured) == S5_SIDE_LEFT);
        captured.points[0].valid = false;
        assert(s5_probe_side(&captured) == S5_SIDE_RIGHT);
    }
    for (unsigned stage = 0u; stage < 2u; ++stage) {
        S5Controller decision;
        S5Input observed = {0};
        s5_controller_init(&decision);
        decision.state = stage == 0u ? S5_COARSE : S5_FINE;
        observed.front_valid = observed.stopped = true;
        observed.front_cm = 400.0f; /* Logged OPEN_SPACE_ASSUMED front. */
        observed.coarse_ready = observed.fine_ready = true;
        /* Side scan contains invalid/close rays, but front is now clear. */
        assert(s5_controller_step(&decision, &observed) == S5_CMD_NONE);
        observed.coarse_ready = observed.fine_ready = false;
        assert(s5_controller_step(&decision, &observed) == S5_CMD_NONE);
        assert(s5_controller_step(&decision, &observed) == S5_CMD_RESUME);
        assert(decision.state == S5_DONE && decision.error == S5_ERR_NONE);
    }
    {
        S5Controller controller;
        S5Input input = {0};
        const float captured[5] = {18.05f, 16.56f, 16.99f, 17.33f, 17.53f};
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.front_valid = input.stopped = input.coarse_ready = true;
        input.front_cm = 16.99f;
        for (int i = 0; i < 5; ++i)
            input.coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 10), captured[i], true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_SCAN_FINE);
        input.coarse_ready = false; input.fine_ready = true;
        for (int i = 0; i < 5; ++i)
            input.fine.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 5), captured[i], true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_STOP);
        assert(controller.state == S5_WAIT_ROUTE); /* Reproduces the COM6 capture. */
        const uint16_t bearings[5] = {25u, 45u, 90u, 135u, 155u};
        controller.state = S5_COARSE;
        input.coarse_ready = true; input.fine_ready = false;
        for (int i = 0; i < 5; ++i)
            input.coarse.points[i] = (S5ScanPoint){bearings[i], i == 2 ? 16.99f : 120.0f, true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.profile.valid && !controller.profile.uses_course_bounds);
        assert(controller.profile.estimated_width_cm > 20.0f);
        controller.state = S5_COARSE;
        input.coarse.points[0].distance_cm = input.coarse.points[1].distance_cm = 14.0f;
        assert(s5_controller_step(&controller, &input) != S5_CMD_GO_LEFT);
        controller.state = S5_COARSE;
        for (int i = 0; i < 5; ++i) input.coarse.points[i].distance_cm = 17.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_SCAN_FINE); /* Wall stays blocked. */
        puts("PASS COM6 blocked narrow view, chassis-assisted width, side choice and wall rejection");
    }
    puts("PASS B5 scans: coarse/fine refinement, size classes, safe angles, servo failure, recenter failure, invalid echoes, nearest-point validation");
    return 0;
}
