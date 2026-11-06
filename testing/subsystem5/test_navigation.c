/* Compile the actual B2/B3 adapter with deterministic host fakes. */
#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#define SUBSYSTEM5_H
#define S5_MOTION_BRIDGE_ENABLED 1
#define MOTION_COMMAND_H
#define B2_MOTOR_H
#define B3_LINE_H
#define SUBSYSTEM1_H
typedef int ER;
typedef int INT;
typedef unsigned char UB;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK 0
#define E_OBJ (-1)
#define B3_LINE_THRESHOLD 2048u
#define B3_LINE_F_DETECTED 1u
#define B3_LINE_F_JUNCTION 4u
#define FAULT_MOTOR_STALL 1u
#define FAULT_ENCODER 2u
enum { ROBOT_SAFE_STOP, ROBOT_LINE_RECOVERY, ROBOT_LINE_FOLLOW, ROBOT_OBSTACLE_AVOID, ROBOT_OBSTACLE_SCAN };
enum { MOTION_CMD_STOP, MOTION_CMD_DRIVE, MOTION_CMD_MOVE, MOTION_CMD_TURN };
enum { MOTION_DIR_FORWARD, MOTION_DIR_REVERSE, MOTION_TURN_LEFT, MOTION_TURN_RIGHT };
enum { MOTION_STATUS_IDLE, MOTION_STATUS_RUNNING, MOTION_STATUS_COMPLETED,
       MOTION_STATUS_CANCELLED, MOTION_STATUS_TIMEOUT, MOTION_STATUS_FAULT };
typedef struct { uint16_t ir_left, ir_centre; uint8_t line_flags; } B3_STATE;
typedef struct {
    int type, direction, target_speed_cps, steering_permille;
    uint32_t safety_generation;
} motion_command_t;
typedef struct {
    int status;
    uint32_t sample_time_ms, command_id;
    bool physically_stopped;
} motion_status_snapshot_t;
static uint32_t clock_ms = 100u;
static uint32_t next_id = 10u;
static uint32_t generation = 1u;
static unsigned faults, stop_calls, drive_calls, move_calls, turn_calls;
static bool allowed, line_valid = true, jam_queue;
static B3_STATE line_state = {3000u, 1000u, B3_LINE_F_DETECTED};
static motion_status_snapshot_t status = {MOTION_STATUS_IDLE, 100u, 0u, true};
static motion_status_snapshot_t result;
static int last_turn, last_direction;
static float last_move;
static ER tk_get_tim(SYSTIM *t) { t->lo = clock_ms; return E_OK; }
void b2_request_stop(void)
{
    ++stop_calls;
    if (allowed) ++generation;
    allowed = false;
    status.physically_stopped = true;
    status.status = MOTION_STATUS_CANCELLED;
}
bool b2_motor_is_stopped(void) { return status.physically_stopped; }
static void b2_request_forward(void) { allowed = true; }
static bool b2_safety_motion_allowed(void) { return allowed; }
static bool b2_safety_stop_requested(void) { return false; }
static uint32_t b2_safety_stop_generation(void) { return generation; }
static unsigned fault_get(void) { return faults; }
static void comm_set_robot_state(int state) { (void)state; }
static void tm_printf(UB *fmt, ...) { (void)fmt; }
static ER b3_get_timed_state(B3_STATE *out, uint32_t *sample)
{
    *out = line_state;
    *sample = clock_ms;
    return line_valid ? E_OK : E_OBJ;
}
static ER motion_get_status(motion_status_snapshot_t *out)
{
    status.sample_time_ms = clock_ms;
    *out = status;
    return E_OK;
}
static ER motion_get_result(uint32_t id, motion_status_snapshot_t *out)
{
    if (result.command_id != id) return E_OBJ;
    *out = result;
    return E_OK;
}
static ER motion_send_command(const motion_command_t *cmd)
{
    if (jam_queue) return E_OBJ;
    if (cmd->type == MOTION_CMD_DRIVE) {
        assert(allowed && cmd->safety_generation == generation);
        ++drive_calls;
        status.status = MOTION_STATUS_RUNNING;
        status.physically_stopped = false;
    }
    return E_OK;
}
static ER issue(uint32_t *id)
{
    if (jam_queue) return E_OBJ;
    assert(allowed);
    *id = next_id++;
    status.command_id = *id;
    status.status = MOTION_STATUS_RUNNING;
    status.physically_stopped = false;
    return E_OK;
}
static ER motion_turn_degrees(INT angle, INT speed, int direction, uint32_t *id)
{
    assert(angle == 90 && speed > 0);
    last_turn = direction;
    ++turn_calls;
    return issue(id);
}
static ER motion_move_cm(float cm, INT speed, int direction, uint32_t *id)
{
    assert(cm > 0.0f && speed > 0);
    last_move = cm;
    last_direction = direction;
    ++move_calls;
    return issue(id);
}
#include "../../app_program/subsystem5/s5_bridge.c"
#include "../../app_program/subsystem5/s5_navigation.c"
static void tick(float cm)
{
    clock_ms += 65u;
    s5_navigation_tick(true, cm, false);
}
static void finish_leg(void)
{
    status.status = MOTION_STATUS_COMPLETED;
    status.physically_stopped = true;
    result = status;
    tick(120.0f);
}
int main(void)
{
    S5Profile p = {0};
    assert(s5_navigation_init());
    assert(!s5_navigation_init());
    assert(s5_bridge_ready());
    assert(s5_navigation_stopped());
    assert(!s5_navigation_has_line_reference());
    line_state.line_flags = 0u;
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 25.0f, true);
    }
    assert(drive_calls == 0u && !allowed); /* Wait on boot until a line is visible. */
    assert(!s5_navigation_has_line_reference());
    line_state.line_flags = B3_LINE_F_DETECTED;
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 120.0f, true);
    }
    assert(drive_calls == 1u);
    assert(s5_navigation_has_line_reference());
    assert(!s5_navigation_stopped());
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    assert(!allowed && stop_calls > 0u);
    assert(s5_navigation_stopped());
    p.valid = true;
    p.bypass_side = S5_SIDE_LEFT;
    p.estimated_width_cm = 8.0f;
    p.center_y_cm = 25.0f;
    assert(!s5_bridge_request(S5_CMD_REVERSE, &p)); /* No rear sensor. */
    assert(s5_bridge_request(S5_CMD_GO_LEFT, &p));
    tick(25.0f); /* Initial turn permitted while front obstacle is near. */
    assert(turn_calls == 1u && last_turn == MOTION_TURN_LEFT);
    assert(!s5_bridge_motion_complete());
    line_state.line_flags = 0u; /* Leave the original line. */
    finish_leg();
    tick(120.0f);
    assert(move_calls == 1u && last_move == 20.0f && last_direction == MOTION_DIR_FORWARD);
    finish_leg();
    tick(120.0f);
    assert(turn_calls == 2u && last_turn == MOTION_TURN_RIGHT);
    finish_leg();
    tick(120.0f);
    assert(move_calls == 2u && last_move == 83.0f);
    finish_leg();
    assert(s5_bridge_motion_complete());
    tick(120.0f);
    assert(turn_calls == 2u); /* Wait explicitly for recovery ownership. */
    assert(s5_bridge_request(S5_CMD_FIND_LINE, &p));
    tick(120.0f);
    assert(turn_calls == 3u);
    finish_leg();
    tick(120.0f);
    assert(move_calls == 3u && last_move == 2.0f);
    for (unsigned i = 0u; i < 7u; ++i) { finish_leg(); tick(120.0f); }
    line_state.line_flags = B3_LINE_F_DETECTED;
    tick(120.0f);
    tick(120.0f);
    tick(120.0f);
    assert(!allowed && !s5_bridge_line_found());
    tick(120.0f); /* STOP confirmed */
    tick(120.0f); /* Restore original heading */
    assert(turn_calls == 4u && last_turn == MOTION_TURN_LEFT);
    finish_leg();
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, &p));
    assert(!s5_bridge_line_found());
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 120.0f, true);
    }
    assert(drive_calls == 2u && allowed); /* Following resumes after recovery. */
    clock_ms += 65u;
    s5_navigation_tick(true, 25.0f, true);
    assert(!allowed); /* A second obstacle interrupts following immediately. */
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    assert(s5_bridge_request(S5_CMD_GO_LEFT, &p));
    tick(25.0f);
    assert(turn_calls == 5u); /* A new avoidance cycle is armed. */
    line_valid = false;
    tick(120.0f);
    assert(!s5_bridge_ready() && !allowed);
    assert(s5_navigation_error() == S5_ERR_LINE_TIMEOUT);
    assert(!s5_bridge_request(S5_CMD_GO_LEFT, &p));
    puts("PASS B5 real navigation adapter: B2 IDs, safety gate, B3 recovery, heading restoration, handoff, stale-line STOP, rear rejection");
    return 0;
}
