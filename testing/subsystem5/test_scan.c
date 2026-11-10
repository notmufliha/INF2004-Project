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
static uint16_t moved[6];
static void tm_printf(UB *format, ...) { (void)format; }
static void tk_dly_tsk(unsigned ms) { assert(ms > 0u); }
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
}
int main(void)
{
    S5Scan s = {0};
    S5Profile p;
    uint16_t angles[5];
    reset();
    s5_coarse_scan(&s);
    assert(move_calls == 6u && read_calls == 5u && moved[5] == 90u);
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
    puts("PASS B5 scans: coarse/fine refinement, size classes, safe angles, servo failure, recenter failure, invalid echoes, nearest-point validation");
    return 0;
}
