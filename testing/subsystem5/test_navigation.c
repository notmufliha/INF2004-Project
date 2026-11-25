/* Compile the actual B2/B3 adapter with deterministic host fakes. */
#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include "../../app_program/subsystem5/servo_mapping.h"
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
    float left_distance_mm, right_distance_mm;
    int fault;
} motion_status_snapshot_t;
static uint32_t clock_ms = 100u;
static uint32_t next_id = 10u;
static uint32_t generation = 1u;
static unsigned faults, stop_calls, drive_calls, move_calls, turn_calls;
static unsigned permission_calls;
static bool allowed, line_valid = true, jam_queue;
static uint32_t snapshot_delay;
static B3_STATE line_state = {3000u, 1000u, B3_LINE_F_DETECTED};
static motion_status_snapshot_t status = {MOTION_STATUS_IDLE, 100u, 0u, true, 0.0f, 0.0f, 0};
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
/* B2 fault acknowledgement (START), as in motion_command.c. */
static uint32_t acked_fault_id;
static bool acked_fault_valid;
static void motion_acknowledge_fault(void)
{
    if (status.status == MOTION_STATUS_FAULT || status.status == MOTION_STATUS_TIMEOUT) {
        acked_fault_id = status.command_id;
        acked_fault_valid = true;
    }
}
static bool motion_fault_active(const motion_status_snapshot_t *s)
{
    return (s->status == MOTION_STATUS_FAULT || s->status == MOTION_STATUS_TIMEOUT) &&
        !(acked_fault_valid && acked_fault_id == s->command_id);
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
    status.left_distance_mm = status.right_distance_mm = 0.0f;
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
static uint32_t last_run_ms;
static ER motion_move_limited_cm(float cm, INT speed, int direction, uint32_t run_ms, uint32_t *id)
{
    last_run_ms = run_ms;
    return motion_move_cm(cm, speed, direction, id);
}
static int mission_turn_hint;
static unsigned mission_turn_done_marks;
int s5_mission_turn_hint(void) { return mission_turn_hint; }
void s5_mission_mark_turn_done(void) { ++mission_turn_done_marks; }
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
static void finish_course_turn(void)
{
    assert(last_angle == ROBOT_COURSE_BYPASS_TURN_DEG);
    finish_leg();
}
static void finish_offset(void)
{
    tick(120.0f);
    assert(allowed && last_move == maneuver.offset_cm);
    assert(last_direction == MOTION_DIR_FORWARD);
    finish_leg();
    assert(maneuver.state == S5_MAN_FORWARD);
}
static void finish_middle(S5Side side)
{
    int inward = side == S5_SIDE_LEFT ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT;
    assert(!s5_bridge_motion_complete());
    tick(120.0f);
    assert(maneuver.state == S5_MAN_FORWARD && maneuver.pending_id);
    assert(last_turn == inward && last_angle == ROBOT_COURSE_PASS_TURN_DEG);
    finish_leg();
    tick(120.0f);
    assert(maneuver.state == S5_MAN_PASS && maneuver.pending_id);
    assert(last_move == 15.0f && last_direction == MOTION_DIR_FORWARD);
    finish_leg();
    assert(maneuver.state == S5_MAN_TURN_IN);
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
    S5Controller controller;
    s5_controller_init(&controller);
    /* Replay the clear-course front range from the COM6 startup log. */
    const float startup_ranges[] = {34.67f, 35.51f, 35.94f};
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        S5Input input = {.front_valid = true, .front_cm = startup_ranges[i], .stopped = true};
        assert(s5_controller_step(&controller, &input) == S5_CMD_NONE);
        assert(controller.state == S5_FOLLOW);
        s5_navigation_tick(true, input.front_cm, true);
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
    p.center_y_cm = 55.0f;
    assert(s5_bridge_request(S5_CMD_REVERSE, &p)); /* Bounded course retreat. */
    tick(5.0f);
    assert(move_calls == 1u && last_direction == MOTION_DIR_REVERSE);
    assert(last_move == ROBOT_BYPASS_REVERSE_CM && !s5_bridge_motion_complete());
    finish_leg();
    assert(s5_bridge_motion_complete() && s5_navigation_stopped());
    move_calls = 0u;
    assert(s5_bridge_request(S5_CMD_GO_LEFT, &p));
    tick(25.0f); /* Initial turn permitted while front obstacle is near. */
    assert(turn_calls == 1u && last_turn == MOTION_TURN_LEFT);
    assert(!s5_bridge_motion_complete());
    line_state.line_flags = 0u; /* Leave the original line. */
    finish_course_turn();
    finish_offset();
    finish_middle(p.bypass_side);
    assert(s5_bridge_motion_complete());
    tick(120.0f);
    assert(turn_calls == 2u); /* Wait explicitly for recovery ownership. */
    assert(s5_bridge_request(S5_CMD_FIND_LINE, &p));
    tick(120.0f);
    assert(turn_calls == 3u && last_turn == MOTION_TURN_RIGHT && last_angle == ROBOT_COURSE_RETURN_TURN_DEG);
    finish_leg();
    tick(120.0f);
    assert(move_calls == 3u && last_move == maneuver.search_limit_cm);
    line_state.line_flags = B3_LINE_F_DETECTED;
    status.left_distance_mm = status.right_distance_mm = (last_move - 2.0f) * 10.0f;
    tick(120.0f);
    assert(maneuver.state == S5_MAN_LINE_STOP && maneuver.pending_id && !allowed);
    assert(!s5_bridge_line_found());
    result = status;
    for (unsigned n = 0u; n < ROBOT_BYPASS_LINE_CONFIRM; ++n) tick(120.0f);
    assert(s5_bridge_line_found());
    assert(s5_bridge_request(S5_CMD_RESUME, &p));
    assert(!s5_bridge_line_found());
    for (int i = 0; i < 3; ++i) {
        clock_ms += 65u;
        s5_navigation_tick(true, 120.0f, true);
    }
    assert(allowed); /* VC can resume after the bypass. */
    clock_ms += 65u;
    s5_navigation_tick(true, ROBOT_STOP_CM, true);
    assert(!allowed); /* A second obstacle interrupts following immediately. */
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    {
        S5Controller decision;
        S5Input observation = {0};
        /* A close, wide obstacle with an opening at the lower servo pulses. */
        const float captured[] = {130.10f, 129.33f, ROBOT_STOP_CM,
                                  ROBOT_STOP_CM - 0.43f, ROBOT_STOP_CM + 0.35f};
        s5_controller_init(&decision);
        decision.state = S5_COARSE;
        for (int i = 0; i < 5; ++i) {
            uint16_t bearing = (uint16_t)(70 + 10 * i);
            unsigned pulse_index = (s5_servo_mount_angle(bearing) - 70u) / 10u;
            observation.coarse.points[i] = (S5ScanPoint){bearing, captured[pulse_index], true};
        }
        observation.front_valid = observation.stopped = observation.coarse_ready = true;
        observation.front_cm = captured[2];
        S5Command chosen = s5_controller_step(&decision, &observation);
        assert(chosen == (ROBOT_SCAN_SERVO_REVERSED ? S5_CMD_GO_RIGHT : S5_CMD_GO_LEFT));
        assert(!s5_bridge_request(chosen, &decision.profile)); /* Too little room for the route. */
    }
    assert(s5_bridge_request(S5_CMD_GO_LEFT, &p));
    tick(25.0f);
    assert(turn_calls == 4u && last_turn == MOTION_TURN_LEFT);
    finish_course_turn();
    finish_offset();
    finish_middle(p.bypass_side);
    assert(s5_bridge_request(S5_CMD_FIND_LINE, &p));
    tick(120.0f);
    assert(last_turn == MOTION_TURN_RIGHT && turn_calls == 6u && last_angle == ROBOT_COURSE_RETURN_TURN_DEG);
    finish_leg();
    {
        unsigned prior_moves = move_calls;
        tick(120.0f); /* Head-on verification after the turn enables forward motion. */
        assert(move_calls == prior_moves + 1u);
        assert(last_direction == MOTION_DIR_FORWARD && last_move > 0.0f);
    }
    assert(s5_bridge_request(S5_CMD_STOP, &p));
    {
        const int looks[] = {-ROBOT_COURSE_BYPASS_TURN_DEG, ROBOT_COURSE_BYPASS_TURN_DEG,
                             ROBOT_COURSE_BYPASS_TURN_DEG, -ROBOT_COURSE_BYPASS_TURN_DEG};
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
        assert(s5_navigation_probe_begin(-ROBOT_COURSE_BYPASS_TURN_DEG));
        s5_navigation_pause();
        assert(!allowed && !s5_navigation_probe_complete());
        puts("PASS chassis viewpoints wait for B2 completion, restore heading and cancel on pause");
    }
    line_valid = false;
    tick(120.0f);
    assert(!s5_bridge_ready() && !allowed);
    assert(s5_navigation_error() == S5_ERR_LINE_TIMEOUT);
    assert(!s5_bridge_request(S5_CMD_GO_LEFT, &p));
    /* A line fault is not re-armed by a motor-fault acknowledgement. */
    line_valid = true;
    tick(120.0f);
    assert(!s5_bridge_ready() && s5_navigation_error() == S5_ERR_LINE_TIMEOUT);
    /* Motor stall: NAV_FAULT used to be permanent (reflash).  It stays while
     * the stall is unacknowledged, and re-arms once START acknowledged it
     * in B2 and cleared the fault flags. */
    navigation_error = S5_ERR_MOTION_TIMEOUT; /* as fault_stop() on a stall */
    status.status = MOTION_STATUS_FAULT;
    status.command_id = 77u;
    faults = FAULT_MOTOR_STALL;
    tick(120.0f);
    assert(!s5_bridge_ready());
    motion_acknowledge_fault();
    tick(120.0f);
    assert(!s5_bridge_ready()); /* stall flag still raised */
    faults = 0u;
    tick(120.0f);
    assert(s5_bridge_ready() && s5_navigation_error() == S5_ERR_NONE);
    /* A new stall (another command) faults again. */
    status.status = MOTION_STATUS_FAULT; /* the mock STOP set CANCELLED */
    status.command_id = 78u;
    tick(120.0f);
    assert(!s5_bridge_ready() && s5_navigation_error() == S5_ERR_MOTION_TIMEOUT);
    puts("PASS motor stall: B5 navigation re-arms after START acknowledges it, not before");
    puts("PASS B5 real navigation adapter: B2 IDs, safety gate, B3 recovery, heading restoration, handoff, stale-line STOP, rear rejection");
    return 0;
}
