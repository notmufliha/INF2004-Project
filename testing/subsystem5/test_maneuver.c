#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_maneuver.h"

static S5Maneuver m;
static S5ManeuverInput in;
static uint32_t next_id;
static S5Profile profile(S5Side side, float width)
{
    S5Profile p = {0};
    p.valid = true;
    p.bypass_side = side;
    p.estimated_width_cm = width;
    p.center_y_cm = 25.0f;
    return p;
}
static void reset(S5Side side)
{
    S5Profile p = profile(side, 8.0f);
    s5_maneuver_init(&m);
    in = (S5ManeuverInput){0};
    in.front_valid = in.stopped = in.line_valid = in.line_detected = true;
    in.front_cm = 120.0f;
    in.now_ms = in.line_sample_ms = 100u;
    next_id = 1u;
    assert(s5_maneuver_begin(&m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                              &p, 40.0f, in.now_ms));
}
static S5Action tick(void)
{
    in.now_ms += 20u;
    in.line_sample_ms = in.now_ms;
    return s5_maneuver_step(&m, &in);
}
static void leg(S5ActionType type, float distance, int angle)
{
    S5Action a = tick();
    assert(a.type == type);
    assert(fabsf(a.distance_cm - distance) < 0.01f && a.turn_deg == angle);
    assert(s5_maneuver_submitted(&m, next_id++));
    in.stopped = false;
    assert(tick().type == S5_ACT_NONE); /* Acceptance is not completion. */
    in.motion_terminal = true;
    in.motion_success = true;
    in.result_id = m.pending_id + 100u;
    assert(tick().type == S5_ACT_NONE); /* Ignore another command's result. */
    in.result_id = m.pending_id;
    in.stopped = true;
    assert(tick().type == S5_ACT_NONE);
    in.motion_terminal = false;
}
static void recovery(S5Side side)
{
    int out = side == S5_SIDE_LEFT ? -90 : 90;
    reset(side);
    leg(S5_ACT_TURN, 0.0f, out);
    in.line_detected = false;
    leg(S5_ACT_MOVE, 20.0f, 0);
    leg(S5_ACT_TURN, 0.0f, -out);
    leg(S5_ACT_MOVE, 83.0f, 0);
    leg(S5_ACT_TURN, 0.0f, -out);
    assert(m.state == S5_MAN_SEARCH);
    for (unsigned i = 0u; i < 7u; ++i) leg(S5_ACT_MOVE, 2.0f, 0);
    assert(tick().type == S5_ACT_MOVE);
    assert(s5_maneuver_submitted(&m, next_id++));
    in.stopped = false;
    in.line_detected = true;
    assert(tick().type == S5_ACT_NONE);
    /* Same timestamp cannot count twice. */
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_NONE);
    assert(m.line_hits == 1u);
    assert(tick().type == S5_ACT_NONE);
    assert(tick().type == S5_ACT_STOP);
    assert(m.state == S5_MAN_LINE_STOP);
    assert(tick().type == S5_ACT_STOP); /* Must wait for physical STOP. */
    in.stopped = true;
    assert(tick().type == S5_ACT_STOP);
    assert(m.state == S5_MAN_ALIGN);
    leg(S5_ACT_TURN, 0.0f, out);
    assert(m.state == S5_MAN_COMPLETE);
}
int main(void)
{
    S5Profile p;
    S5Action a;
    unsigned i;
    recovery(S5_SIDE_LEFT);
    recovery(S5_SIDE_RIGHT);
    reset(S5_SIDE_LEFT);
    p = profile(S5_SIDE_LEFT, 24.0f);
    s5_maneuver_init(&m);
    assert(s5_maneuver_begin(&m, S5_CMD_GO_LEFT, &p, 40.0f, 100u));
    assert(m.lateral_cm == 28.0f); /* Larger obstacle, wider bypass. */
    s5_maneuver_init(&m);
    p.estimated_width_cm = ROBOT_OBSTACLE_MAX_BYPASS_CM + 1.0f;
    assert(!s5_maneuver_begin(&m, S5_CMD_GO_LEFT, &p, 40.0f, 100u));
    p.estimated_width_cm = NAN;
    assert(!s5_maneuver_begin(&m, S5_CMD_GO_LEFT, &p, 40.0f, 100u));
    p = profile(S5_SIDE_LEFT, 8.0f);
    assert(!s5_maneuver_begin(&m, S5_CMD_GO_RIGHT, &p, 40.0f, 100u));
    assert(!s5_maneuver_begin(&m, S5_CMD_GO_LEFT, &p, 0.0f, 100u));
    assert(!s5_maneuver_begin(&m, S5_CMD_GO_LEFT, &p, INFINITY, 100u));
    reset(S5_SIDE_LEFT);
    in.front_valid = false;
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_FRONT_SENSOR);
    reset(S5_SIDE_LEFT);
    in.front_cm = NAN;
    assert(tick().type == S5_ACT_STOP);
    reset(S5_SIDE_LEFT);
    in.front_cm = ROBOT_BYPASS_MIN_TURN_CM;
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_INSUFFICIENT_CLEARANCE);
    reset(S5_SIDE_LEFT);
    in.line_valid = false;
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_LINE_TIMEOUT);
    reset(S5_SIDE_LEFT);
    in.now_ms = 1000u;
    assert(s5_maneuver_step(&m, &in).type == S5_ACT_STOP); /* stale line */
    reset(S5_SIDE_LEFT);
    in.now_ms += ROBOT_BYPASS_LEG_TIMEOUT_MS;
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_MOTION_TIMEOUT);
    reset(S5_SIDE_LEFT);
    assert(tick().type == S5_ACT_TURN);
    assert(!s5_maneuver_submitted(&m, 0u));
    assert(m.state == S5_MAN_FAILED);
    reset(S5_SIDE_LEFT);
    assert(tick().type == S5_ACT_TURN);
    assert(s5_maneuver_submitted(&m, 42u));
    in.motion_terminal = true;
    in.result_id = 42u;
    in.motion_success = false; /* CANCELLED/FAULT/TIMEOUT is never success. */
    assert(tick().type == S5_ACT_STOP);
    reset(S5_SIDE_LEFT);
    m.state = S5_MAN_SEARCH;
    m.left_original_line = true;
    in.line_detected = false;
    for (i = 0u; i < 15u; ++i) leg(S5_ACT_MOVE, 2.0f, 0);
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_LINE_TIMEOUT);
    reset(S5_SIDE_LEFT);
    m.state = S5_MAN_SEARCH;
    in.line_detected = true;
    assert(tick().type == S5_ACT_MOVE);
    assert(s5_maneuver_submitted(&m, 7u));
    for (i = 0u; i < 5u; ++i) assert(tick().type == S5_ACT_NONE);
    assert(m.line_hits == 0u); /* Original line never departed. */
    m.left_original_line = true;
    in.line_junction = true;
    for (i = 0u; i < 5u; ++i) assert(tick().type == S5_ACT_NONE);
    assert(m.line_hits == 0u); /* Reject junctions as a recovery target. */
    s5_maneuver_init(&m);
    assert(s5_maneuver_begin(&m, S5_CMD_REVERSE, NULL, 0.0f, in.now_ms));
    in.rear_clear = false;
    assert(tick().type == S5_ACT_STOP && m.error == S5_ERR_NO_CLEAR_ROUTE);
    s5_maneuver_init(&m);
    assert(s5_maneuver_begin(&m, S5_CMD_REVERSE, NULL, 0.0f, in.now_ms));
    in.front_cm = 25.0f; /* Reverse can escape a close front obstacle. */
    in.rear_clear = true;
    leg(S5_ACT_MOVE, -ROBOT_BYPASS_REVERSE_CM, 0);
    assert(m.state == S5_MAN_COMPLETE);
    reset(S5_SIDE_LEFT);
    m.start_ms = m.leg_ms = in.now_ms = 0xfffffff0u;
    a = tick(); /* uint32 millisecond rollover */
    assert(a.type == S5_ACT_TURN);
    puts("PASS B5 maneuver: left/right recovery, size, IDs, cancellation, sensor faults, deadlines, bounded search, reverse, timer rollover");
    return 0;
}
