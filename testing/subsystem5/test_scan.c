#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "../../app_program/robot_config.h"
#include "../../app_program/subsystem5/obstacle_scan.h"
#define SUBSYSTEM5_H
#define S5_SERVO_ENABLED 1
#define S5_HARDWARE_ENABLED 1
#define S5_SIMULATION 0
typedef unsigned char UB;
static unsigned move_calls, read_calls;
static unsigned fail_move, invalid_read;
static unsigned delay_total_ms;
static uint16_t moved[S5_WIDTH_SCAN_COUNT + 1];
static void tm_printf(UB *format, ...) { (void)format; }
static void tk_dly_tsk(unsigned ms) { assert(ms > 0u); delay_total_ms += ms; }
bool s5_servo_angle(uint16_t angle)
{
    assert(angle >= ROBOT_WIDTH_SWEEP_MIN_DEG && angle <= ROBOT_WIDTH_SWEEP_MAX_DEG);
    assert(move_calls < S5_WIDTH_SCAN_COUNT + 1u);
    moved[move_calls++] = angle;
    return move_calls != fail_move;
}
bool s5_measure_scan_cm(float *cm)
{
    static const float distances[5] = {100.0f, 25.0f, 20.0f, 26.0f, 100.0f};
    *cm = distances[read_calls++];
    return read_calls != invalid_read;
}
bool s5_measure_probe_cm(float *cm)
{
    static const float distances[S5_WIDTH_SCAN_COUNT] = {
        15.07f, 14.38f, 14.27f, 14.13f, 14.13f, 14.21f, 15.16f
    };
    assert(read_calls < S5_WIDTH_SCAN_COUNT);
    *cm = distances[read_calls++];
    return true;
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
    assert(delay_total_ms == 6u * ROBOT_SCAN_SERVO_SETTLE_MS);
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
        /* A centre reading beyond the encounter release range is clear. */
        wide.points[2].distance_cm = ROBOT_RELEASE_CM;
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
        /* A close side opening can nominate a route for geometry checks. */
        wide.points[2].distance_cm = ROBOT_BYPASS_MIN_TURN_CM + 3.0f;
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
        /* The left inner ray clips the obstacle even though its outer
         * ray sees a farther background. Prefer the two clear right rays. */
        s5_controller_init(&controller);
        controller.state = S5_FINE;
        for (int i = 0; i < 5; ++i)
            controller.last_coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), 22.0f, true};
        controller.last_coarse.points[0].distance_cm = 200.0f;
        controller.last_coarse.points[3].distance_cm = 150.0f;
        controller.last_coarse.points[4].distance_cm = 150.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        assert(controller.side == S5_SIDE_RIGHT);
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
        const float captured[] = {16.94f, 152.39f, ROBOT_STOP_CM, 152.79f, 18.86f};
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse_ready = input.front_valid = input.stopped = true;
        input.front_cm = 17.23f;
        for (int i = 0; i < 5; ++i)
            input.coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), captured[i], true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
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
    {
        S5Input input = {.front_valid = true, .front_cm = 14.64f,
                         .stopped = true, .coarse_ready = true};
        const uint16_t bearings[] = {ROBOT_WIDTH_SCAN_MIN_DEG, 65u, 90u, 115u,
                                     ROBOT_WIDTH_ROUTE_RIGHT_DEG};
        const float distances[] = {95.0f, 14.04f, 14.64f, 15.07f, 55.0f};
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            input.coarse.points[i] = (S5ScanPoint){bearings[i], distances[i], true};
        S5Controller controller;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.profile.valid && !controller.profile.uses_course_bounds);
        assert(controller.profile.estimated_width_cm > 14.0f &&
               controller.profile.estimated_width_cm < 16.0f);
        input.coarse.points[4].distance_cm = 110.0f;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        input.coarse.points[4].distance_cm = 14.56f;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse.points[0].distance_cm = 17.13f;
        input.coarse.points[4].distance_cm = 14.13f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(!controller.profile.valid); /* Width unresolved, heading measured. */
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        input.coarse.points[0].distance_cm = input.coarse.points[4].distance_cm = 5.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_STOP);
        assert(controller.error == S5_ERR_INSUFFICIENT_CLEARANCE);
        for (unsigned tick = 0; tick <= S5_ROUTE_RESCAN_TICKS; ++tick)
            assert(s5_controller_step(&controller, &input) == S5_CMD_NONE);

        /* Measured edges remain usable with a missing inner ray. */
        S5ScanPoint sweep[S5_WIDTH_SCAN_COUNT] = {
            {30u, 100.0f, true}, {45u, 97.40f, true}, {60u, 98.94f, true},
            {75u, 0.0f, false}, {90u, 13.21f, true}, {105u, 12.07f, true},
            {120u, 55.0f, true}
        };
        input.width_points = sweep;
        input.coarse.points[0] = sweep[1];
        input.coarse.points[4] = sweep[6];
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.profile.valid && !controller.profile.uses_course_bounds);
        assert(controller.profile.left_boundary_deg == 75u);
        assert(controller.profile.right_boundary_deg == 112u);
        assert(controller.profile.estimated_width_cm > 7.9f &&
               controller.profile.estimated_width_cm < 8.0f);

        /* A live open-space echo must not discard the completed scan. */
        sweep[6].distance_cm = 12.07f;
        input.coarse.points[4] = sweep[6];
        input.front_cm = 400.0f;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        controller.clear_route_hits = ROBOT_CLEAR_CONFIRM - 1u;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(!controller.profile.valid);

        sweep[0].distance_cm = 74.23f;
        sweep[1].valid = sweep[2].valid = false;
        sweep[3] = (S5ScanPoint){75u, 13.73f, true};
        sweep[4].distance_cm = 12.03f;
        sweep[5].distance_cm = 12.31f;
        sweep[6].distance_cm = 14.07f;
        input.coarse.points[0] = sweep[1];
        input.coarse.points[4] = sweep[6];
        input.front_cm = 13.18f;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(controller.profile.left_clearance_cm == 13.73f);

        /* COM6 replay: the right edge extends beyond the 120-degree sweep. */
        const S5ScanPoint clipped[S5_WIDTH_SCAN_COUNT] = {
            {30u, 400.0f, true}, {45u, 15.24f, true}, {60u, 13.10f, true},
            {75u, 13.70f, true}, {90u, 13.01f, true}, {105u, 13.44f, true},
            {120u, 16.44f, true}
        };
        input.width_points = clipped;
        input.coarse.points[0] = clipped[1];
        input.coarse.points[4] = clipped[6];
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_LEFT);
        assert(!controller.profile.valid);
        S5Profile candidate = s5_profile_from_probe(&controller.profile, &input.coarse,
            NULL, S5_SIDE_LEFT, ROBOT_COURSE_BYPASS_TURN_DEG, input.front_cm);
        assert(candidate.valid && candidate.uses_course_bounds);

        /* A side object beyond a depth break must not widen the target. */
        S5ScanPoint separate[S5_WIDTH_SCAN_COUNT];
        for (int i = 0; i < S5_WIDTH_SCAN_COUNT; ++i) separate[i] = clipped[i];
        separate[2].distance_cm = 30.0f;
        separate[5].distance_cm = 29.0f;
        input.width_points = separate;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        S5Command route = s5_controller_step(&controller, &input);
        assert(route == S5_CMD_GO_LEFT || route == S5_CMD_GO_RIGHT);
        assert(controller.profile.valid);
        assert(controller.profile.left_boundary_deg == 67u);
        assert(controller.profile.right_boundary_deg == 97u);
        assert(controller.profile.estimated_width_cm > 6.9f &&
               controller.profile.estimated_width_cm < 7.0f);

        /* Replay the clipped COM6 sweep, then add clear flank echoes from
         * stopped chassis viewpoints within the same servo limits. */
        reset();
        S5ScanPoint wide[S5_WIDTH_SCAN_COUNT];
        s5_width_scan(&input.coarse, wide);
        assert(read_calls == S5_WIDTH_SCAN_COUNT && move_calls == S5_WIDTH_SCAN_COUNT + 1u);
        assert(moved[0] == 30u && moved[6] == 120u && moved[7] == 90u);
        assert(input.coarse.points[4].angle_deg == ROBOT_WIDTH_ROUTE_RIGHT_DEG);
        input.width_points = wide;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        route = s5_controller_step(&controller, &input);
        assert(route == S5_CMD_GO_LEFT || route == S5_CMD_GO_RIGHT);
        assert(!controller.profile.valid);
        const S5ScanPoint opposite_view[S5_WIDTH_SCAN_COUNT] = {{90u, 400.0f, true}};
        const S5ScanPoint bypass_view[S5_WIDTH_SCAN_COUNT] = {{60u, 400.0f, true}};
        assert(!s5_estimate_width_from_views(wide, NULL, bypass_view, -45).valid);
        S5Profile width = s5_estimate_width_from_views(wide, opposite_view, bypass_view, -45);
        assert(width.valid && !width.uses_course_bounds);
        assert(width.left_boundary_deg == 22u && width.right_boundary_deg == 127u);
        assert(width.estimated_width_cm > 23.0f && width.estimated_width_cm < 23.2f);

        const S5ScanPoint latest[S5_WIDTH_SCAN_COUNT] = {
            {30u, 16.18f, true}, {45u, 14.56f, true}, {60u, 14.13f, true},
            {75u, 14.04f, true}, {90u, 14.04f, true}, {105u, 14.04f, true},
            {120u, 14.90f, true}
        };
        const S5ScanPoint latest_bypass[S5_WIDTH_SCAN_COUNT] = {
            {45u, 400.0f, true}, {60u, 103.66f, true}, {70u, 103.23f, true}
        };
        width = s5_estimate_width_from_views(latest, opposite_view, latest_bypass, -45);
        assert(width.valid && width.estimated_width_cm > 23.3f && width.estimated_width_cm < 23.5f);
        assert(width.frontal_width_cm > 8.4f && width.frontal_width_cm < 8.5f);

        /* This scan has only one hit to the right of the centre ray. */
        const S5ScanPoint single_flank[S5_WIDTH_SCAN_COUNT] = {
            {30u, 400.0f, true}, {45u, 16.64f, true}, {60u, 12.93f, true},
            {75u, 13.27f, true}, {90u, 12.84f, true}, {105u, 14.56f, true},
            {120u, 400.0f, true}
        };
        width = s5_estimate_width_from_views(single_flank, NULL, NULL, 0);
        assert(width.valid && width.frontal_width_cm > 9.47f && width.frontal_width_cm < 9.50f);

        const S5ScanPoint flat_returns[S5_WIDTH_SCAN_COUNT] = {
            {30u, 400.0f, true}, {45u, 400.0f, true}, {60u, 14.0f, true},
            {75u, 14.0f, true}, {90u, 14.0f, true}, {105u, 14.0f, true},
            {120u, 400.0f, true}
        };
        width = s5_estimate_width_from_views(flat_returns, NULL, NULL, 0);
        assert(width.valid && width.frontal_width_cm > 10.6f && width.frontal_width_cm < 10.7f);

        const S5ScanPoint mixed_flanks[S5_WIDTH_SCAN_COUNT] = {
            {30u, 400.0f, true}, {45u, 15.50f, true}, {60u, 15.44f, true},
            {75u, 14.56f, true}, {90u, 14.98f, true}, {105u, 14.90f, true},
            {120u, 31.26f, true}
        };
        width = s5_estimate_width_from_views(mixed_flanks, NULL, NULL, 0);
        assert(width.valid && width.frontal_width_cm > 7.7f && width.frontal_width_cm < 7.8f);

        /* Crash replay: 90 degrees reports open while the leftmost ray
         * still sees the obstacle. Keep the motors stopped and rescan. */
        S5ScanPoint crash[S5_WIDTH_SCAN_COUNT] = {
            {30u, 13.89f, true}, {45u, 12.94f, true}, {60u, 12.94f, true},
            {75u, 13.37f, true}, {90u, 400.0f, true}, {105u, 400.0f, true},
            {120u, 354.28f, true}
        };
        input.width_points = crash;
        input.coarse.points[0] = crash[1];
        input.coarse.points[2] = crash[4];
        input.coarse.points[4] = crash[6];
        input.front_cm = 400.0f;
        s5_controller_init(&controller);
        controller.state = S5_COARSE;
        assert(s5_controller_step(&controller, &input) == S5_CMD_STOP);
        assert(controller.unresolved_flank && controller.state == S5_WAIT_ROUTE);
        for (unsigned i = 1u; i < S5_ROUTE_RESCAN_TICKS; ++i)
            assert(s5_controller_step(&controller, &input) == S5_CMD_NONE);
        assert(s5_controller_step(&controller, &input) == S5_CMD_SCAN_COARSE);
        crash[0].distance_cm = 400.0f;
        assert(s5_controller_step(&controller, &input) == S5_CMD_GO_RIGHT);
        assert(!controller.unresolved_flank && controller.profile.valid &&
               controller.profile.frontal_width_cm > 5.6f &&
               controller.profile.frontal_width_cm < 5.8f);

        reset();
        s5_width_flank_scan(-45, wide);
        assert(read_calls == 3u && move_calls == 4u);
        assert(moved[0] == 45u && moved[1] == 60u && moved[2] == 70u && moved[3] == 90u);
        assert(!wide[3].valid);
        reset();
        s5_width_flank_scan(45, wide);
        assert(read_calls == 3u && move_calls == 4u);
        assert(moved[0] == 90u && moved[1] == 105u && moved[2] == 120u && moved[3] == 90u);
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
        assert(s5_probe_side(&captured) == S5_SIDE_LEFT);
        captured.points[3].distance_cm = captured.points[4].distance_cm = 65.0f;
        assert(s5_probe_side(&captured) == S5_SIDE_RIGHT);
    }
    {
        S5Scan captured = {0};
        const float cm[5] = {14.90f, 15.93f, 15.93f, 17.54f, 16.27f};
        for (int i = 0; i < 5; ++i)
            captured.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 10), cm[i], true};
        assert(s5_probe_side(&captured) == S5_SIDE_RIGHT);
        for (int i = 0; i < 5; ++i)
            captured.points[i].distance_cm = cm[4 - i];
        assert(s5_probe_side(&captured) == S5_SIDE_LEFT);
        const float ambiguous[5] = {15.50f, 16.01f, 16.44f, 15.93f, 15.41f};
        for (int i = 0; i < 5; ++i)
            captured.points[i].distance_cm = ambiguous[i];
        assert(s5_probe_side(&captured) == S5_SIDE_LEFT);
        const float com6_tie[5] = {17.30f, 16.96f, 17.39f, 17.39f, 17.73f};
        for (int i = 0; i < 5; ++i)
            captured.points[i].distance_cm = com6_tie[i];
        assert(s5_probe_side(&captured) == S5_SIDE_LEFT);
        captured.points[3].distance_cm = 30.0f;
        assert(s5_probe_side(&captured) == S5_SIDE_RIGHT);
        captured.points[2].distance_cm = 100.0f;
        assert(s5_probe_corridor_blocked(&captured)); /* Clear centre, close flank. */
        captured.points[1].distance_cm = 100.0f;
        assert(!s5_probe_corridor_blocked(&captured));
        captured.points[2].distance_cm = ROBOT_BYPASS_MOVE_RELEASE_CM;
        assert(s5_probe_corridor_blocked(&captured)); /* Too short to start a step. */
        captured.points[2].distance_cm = 35.0f;
        assert(!s5_probe_corridor_blocked(&captured)); /* Enough to start advancing. */
        captured.points[1].valid = false;
        assert(s5_probe_corridor_blocked(&captured)); /* An unknown flank blocks the route. */
        captured.points[1].valid = true;
        captured.points[2].distance_cm = 100.0f;
        captured.points[2].valid = false;
        assert(s5_probe_corridor_blocked(&captured));
    }
    {
        S5Scan scan = {0};
        const float cm[5] = {60.0f, 60.0f, 15.0f, 60.25f, 60.25f};
        for (int i = 0; i < S5_SCAN_COUNT; ++i)
            scan.points[i] = (S5ScanPoint){(uint16_t)(70 + i * 10), cm[i], true};
        assert(s5_choose_side(&scan) == S5_SIDE_LEFT);
        assert(s5_probe_side(&scan) == S5_SIDE_LEFT);
        S5Controller decision;
        s5_controller_init(&decision);
        decision.state = S5_COARSE;
        S5Input observed = {.front_valid = true, .front_cm = 15.0f,
                            .stopped = true, .coarse_ready = true, .coarse = scan};
        assert(s5_controller_step(&decision, &observed) == S5_CMD_GO_LEFT);

        scan.points[3].distance_cm = scan.points[4].distance_cm = 65.0f;
        assert(s5_choose_side(&scan) == S5_SIDE_RIGHT);
        assert(s5_probe_side(&scan) == S5_SIDE_RIGHT);
        scan.points[0].valid = false;
        assert(s5_choose_side(&scan) == S5_SIDE_RIGHT);
        scan.points[3].distance_cm = 16.0f;
        assert(s5_probe_side(&scan) == S5_SIDE_LEFT); /* Only the right sees the obstacle. */
    }
    {
        S5Scan coarse = {0}, view = {0};
        for (int i = 0; i < S5_SCAN_COUNT; ++i) {
            uint16_t bearing = (uint16_t)(70 + i * 10);
            coarse.points[i] = (S5ScanPoint){bearing, i == 2 || i == 3 ? 16.0f : 100.0f, true};
            view.points[i] = (S5ScanPoint){bearing, 100.0f, true};
        }
        S5Profile measured = s5_profile_from_probe(NULL, &coarse, &view, S5_SIDE_LEFT, ROBOT_COURSE_BYPASS_TURN_DEG, 16.0f);
        assert(measured.valid && !measured.uses_course_bounds);
        assert(measured.estimated_width_cm > 0.0f && measured.estimated_width_cm < ROBOT_OBSTACLE_MAX_BYPASS_CM);
        S5Profile preserved = s5_profile_from_probe(&measured, &coarse, &view, S5_SIDE_RIGHT, ROBOT_COURSE_BYPASS_TURN_DEG, 17.0f);
        assert(preserved.bypass_side == S5_SIDE_RIGHT && preserved.center_y_cm == 17.0f);
        assert(preserved.estimated_width_cm == measured.estimated_width_cm);
        view.points[2].valid = false;
        S5Profile bounded = s5_profile_from_probe(NULL, &coarse, &view, S5_SIDE_LEFT, ROBOT_COURSE_BYPASS_TURN_DEG, 16.0f);
        assert(bounded.valid && bounded.uses_course_bounds);
        assert(bounded.estimated_width_cm == ROBOT_OBSTACLE_MAX_BYPASS_CM);
    }
    for (unsigned stage = 0u; stage < 2u; ++stage) {
        S5Controller decision;
        S5Input observed = {0};
        const float cm[5] = {14.98f, 15.41f, 15.41f, 16.27f, 0.0f};
        s5_controller_init(&decision);
        decision.state = stage == 0u ? S5_COARSE : S5_FINE;
        observed.stopped = observed.front_valid = true;
        observed.front_cm = 14.98f;
        observed.coarse_ready = stage == 0u;
        observed.fine_ready = stage == 1u;
        for (int i = 0; i < S5_SCAN_COUNT; ++i) {
            observed.coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), cm[i], i != 4};
            observed.fine.points[i] = (S5ScanPoint){(uint16_t)(70 + 5 * i), 14.98f, true};
        }
        decision.last_coarse = observed.coarse;
        assert(s5_controller_step(&decision, &observed) == S5_CMD_STOP);
        assert(decision.state == S5_WAIT_ROUTE && decision.error == S5_ERR_NO_CLEAR_ROUTE);
        assert(s5_probe_side(&decision.last_coarse) == S5_SIDE_NONE);
        /* COM6: two invalid left rays and only 16 cm on the right.
         * Favor the left for the guarded in-place look. */
        decision.last_coarse.points[0].valid = decision.last_coarse.points[1].valid = false;
        decision.last_coarse.points[2].distance_cm = 16.87f;
        decision.last_coarse.points[3].distance_cm = 16.87f;
        decision.last_coarse.points[4] = (S5ScanPoint){110u, 15.93f, true};
        assert(s5_probe_side(&decision.last_coarse) == S5_SIDE_LEFT);
        decision.last_coarse.points[0] = (S5ScanPoint){70u, 35.67f, true};
        decision.last_coarse.points[1] = (S5ScanPoint){80u, 35.75f, true};
        assert(s5_probe_side(&decision.last_coarse) == S5_SIDE_LEFT);
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
