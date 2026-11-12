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
#define B3_LINE_BARCODE_H
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
static unsigned permission_calls;
static bool allowed, line_valid = true, jam_queue;
static uint32_t snapshot_delay;
static B3_STATE line_state = {3000u, 1000u, B3_LINE_F_DETECTED};
static motion_status_snapshot_t status = {MOTION_STATUS_IDLE, 100u, 0u, true};
static motion_status_snapshot_t result;
static int last_turn, last_direction, last_angle;
static int last_drive_direction, last_drive_steer;
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
static void b2_request_forward(void) { ++permission_calls; allowed = true; }
static bool b2_safety_motion_allowed(void) { return allowed; }
static uint32_t b2_safety_stop_generation(void) { return generation; }
static bool b2_safety_stop_requested(void) { return false; }

static unsigned fault_get(void) { return faults; }
static void comm_set_robot_state(int state) { (void)state; }
static void tm_printf(UB *fmt, ...) { (void)fmt; }
static ER b3_get_timed_state(B3_STATE *out, uint32_t *sample)
{
    clock_ms += snapshot_delay;
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
        assert(allowed && cmd->safety_generation == b2_safety_stop_generation());
        ++drive_calls;
        last_drive_direction = cmd->direction;
        last_drive_steer = cmd->steering_permille;
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
    assert(angle > 0 && angle <= 120 && speed > 0);
    last_angle = angle;
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
static void finish_offset(void)
{
    tick(120.0f);
    assert(allowed && last_move == maneuver.offset_cm);
    assert(last_direction == MOTION_DIR_FORWARD);
    finish_leg();
    assert(maneuver.state == S5_MAN_FORWARD);
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
        s5_navigation_tick(true, ROBOT_STOP_CM, true);
    }
    assert(drive_calls == 0u && !allowed); /* Wait on boot until a line is visible. */
    assert(!s5_navigation_has_line_reference());
    line_state.line_flags = B3_LINE_F_DETECTED;
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 31.0f, true);
    }
    assert(drive_calls == 0u);
    assert(s5_navigation_has_line_reference());
    assert(s5_navigation_stopped() && allowed); /* VC owns DRIVE. */
    snapshot_delay = 5u; /* Producer ran while snapshot mutex was acquired. */
    clock_ms += 65u;
    s5_navigation_tick(true, 120.0f, true);
    assert(s5_navigation_error() == S5_ERR_NONE && allowed);
    snapshot_delay = 0u;
    clock_ms += 65u;
    s5_navigation_tick(true, (ROBOT_STOP_CM + ROBOT_RELEASE_CM) * 0.5f, true);
    assert(allowed && drive_calls == 0u); /* Continue through hysteresis band. */
    {
        unsigned prior_stops = stop_calls;
        /* Good clear readings from COM6 interleaved with one/two short
         * invalid pulses must not alternate FOLLOW and STOP. */
        for (unsigned i = 0u; i < 30u; ++i) {
            unsigned prior_permissions = permission_calls;
            clock_ms += 65u;
            s5_navigation_tick(false, 0.0f, true);
            assert(allowed && stop_calls == prior_stops);
            assert(permission_calls == prior_permissions);
            if (i % 2u) {
                clock_ms += 65u;
                s5_navigation_tick(false, 0.0f, true);
                assert(allowed && stop_calls == prior_stops);
                assert(permission_calls == prior_permissions);
            }
            clock_ms += 65u;
            s5_navigation_tick(true, 150.0f, true);
            assert(allowed && stop_calls == prior_stops);
        }
        /* An actual close reading still stops on the very first sample. */
        clock_ms += 65u;
        s5_navigation_tick(true, ROBOT_STOP_CM, true);
        assert(!allowed && stop_calls > prior_stops);
        for (unsigned i = 0u; i < ROBOT_CLEAR_CONFIRM; ++i) {
            clock_ms += 65u; s5_navigation_tick(true, 150.0f, true);
        }
        assert(allowed);
        for (unsigned i = 0u; i < S5_FRONT_INVALID_LIMIT; ++i) {
            clock_ms += 30u;
            s5_navigation_tick(false, 0.0f, true);
            if (i + 1u < S5_FRONT_INVALID_LIMIT) assert(allowed);
        }
        assert(!allowed); /* Persistent errors stop within the grace window. */
        for (unsigned i = 0u; i < ROBOT_CLEAR_CONFIRM; ++i) {
            clock_ms += 65u; s5_navigation_tick(true, 150.0f, true);
        }
        assert(allowed);
        /* Missing clearance beyond the grace deadline stops even before
         * the three-invalid-sample count is exhausted. */
        clock_ms += ROBOT_FRONT_GLITCH_GRACE_MS + 1u;
        s5_navigation_tick(false, 0.0f, true);
        assert(!allowed);
        for (unsigned i = 0u; i < ROBOT_CLEAR_CONFIRM; ++i) {
            clock_ms += 65u; s5_navigation_tick(true, 150.0f, true);
        }
        assert(allowed);
    }
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    assert(!allowed && stop_calls > 0u);
    assert(s5_navigation_stopped());
    /* A removed obstacle resumes from a stopped scan without pretending
     * that a bypass/recovery maneuver has completed. */
    assert(s5_bridge_request(S5_CMD_RESUME, &p));
    assert(!allowed);
    assert(s5_bridge_request(S5_CMD_STOP, &p));
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
    finish_offset();
    tick(120.0f);
    assert(turn_calls == 2u && last_turn == MOTION_TURN_RIGHT);
    finish_leg();
    tick(120.0f);
    assert(move_calls == 2u && last_move == 63.0f);
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
    assert(!allowed && maneuver.state == S5_MAN_LINE_STOP);
    assert(!s5_bridge_line_found()); /* First contact brakes, never hands off. */
    tick(120.0f);
    tick(120.0f);
    assert(!allowed && !s5_bridge_line_found());
    tick(120.0f); /* STOP confirmed */
    tick(120.0f); /* Restore original heading */
    assert(turn_calls == 4u && last_turn == MOTION_TURN_LEFT);
    line_state.line_flags = 0u; /* Sensors rotate off the crossing. */
    finish_leg();
    assert(!s5_bridge_line_found());
    for (unsigned i = 0u; i < 6u; ++i) tick(120.0f);
    assert(turn_calls == 5u && last_angle == 15 && last_turn == MOTION_TURN_RIGHT);
    finish_leg();
    line_state.line_flags = B3_LINE_F_DETECTED | B3_LINE_F_JUNCTION;
    for (unsigned i = 0u; i < ROBOT_BYPASS_LINE_CONFIRM; ++i) tick(120.0f);
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, &p));
    assert(!s5_bridge_line_found());
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 120.0f, true);
    }
    assert(allowed); /* VC can resume after width-sized bypass. */
    clock_ms += 65u;
    s5_navigation_tick(true, ROBOT_STOP_CM, true);
    assert(!allowed); /* A second obstacle interrupts following immediately. */
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    {
        S5Controller decision;
        S5Input observation = {0};
        const float captured[] = {16.94f, 152.39f, 20.46f, 152.79f, 18.86f};
        s5_controller_init(&decision);
        decision.state = S5_COARSE;
        for (int i = 0; i < 5; ++i) {
            observation.coarse.points[i] = (S5ScanPoint){(uint16_t)(70 + 10 * i), captured[i], true};
        }
        observation.front_valid = observation.stopped = observation.coarse_ready = true;
        observation.front_cm = 17.23f;
        S5Command chosen = s5_controller_step(&decision, &observation);
        assert(chosen == S5_CMD_GO_LEFT); /* Actual capture: inner left ray is usable. */
        assert(s5_bridge_request(chosen, &decision.profile));
    }
    tick(25.0f);
    assert(turn_calls == 6u && last_turn == MOTION_TURN_LEFT);
    finish_leg();
    finish_offset();
    tick(120.0f);
    assert(last_turn == MOTION_TURN_RIGHT && turn_calls == 7u);
    finish_leg();
    {
        unsigned prior_moves = move_calls;
        tick(120.0f); /* Head-on verification after the turn enables forward motion. */
        assert(move_calls == prior_moves + 1u);
        assert(last_direction == MOTION_DIR_FORWARD && last_move > 0.0f);
    }
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    {
        const int looks[] = {-45, 45, 45, -45};
        int heading = 0;
        for (unsigned i = 0u; i < 4u; ++i) {
            assert(s5_navigation_probe_begin(looks[i]));
            assert(!s5_navigation_probe_complete());
            assert(last_angle == (looks[i] < 0 ? -looks[i] : looks[i]));
            tick(17.0f); /* Logged frontal distance still permits rotation. */
            assert(!s5_navigation_probe_complete());
            assert(!s5_navigation_probe_begin(60)); /* No overlapping turns. */
            finish_leg();
            assert(s5_navigation_probe_complete() && s5_navigation_stopped());
            heading += looks[i];
        }
        assert(heading == 0);
        assert(s5_navigation_probe_begin(-45));
        s5_navigation_pause();
        assert(!allowed && !s5_navigation_probe_complete());
        puts("PASS chassis viewpoints wait for B2 completion, restore heading and cancel on pause");
    }
    line_valid = false;
    tick(120.0f);
    assert(!s5_bridge_ready() && !allowed);
    assert(s5_navigation_error() == S5_ERR_LINE_TIMEOUT);
    assert(!s5_bridge_request(S5_CMD_GO_LEFT, &p));
    puts("PASS B5 real navigation adapter: B2 IDs, safety gate, B3 recovery, heading restoration, handoff, stale-line STOP, rear rejection");
    return 0;
}
