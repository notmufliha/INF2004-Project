/* Host test: rejoin route (ROBOT_REJOIN_ROUTE) of the B5 bypass executor.
 * A tiny simulated B2 runs each MOVE/TURN; the line sensor sees tape at a
 * chosen leg/distance. Final course: 14 x 10 cm box, crossing ~15 cm past
 * its far edge, robot stopped with the sonar 15 cm from the box. */
#include "../../app_program/subsystem5/s5_maneuver.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* B5 ticks every ~65 ms; 680 counts/s at 0.085 mm/count = 58 mm/s. */
#define STEP_CM 0.38f

typedef struct {
    S5Maneuver m;
    uint32_t now, id, result_id;
    bool running, terminal, success, cancelled;
    S5Action act;
    float travelled, result_progress;
    /* Line scenario: tape is under the sensors while the given state's
     * leg counter + current progress is inside [at, at + width). */
    S5ManeuverState tape_state;
    float tape_at, tape_width;
    float front_cm;
    float close_from;   /* PASS distance where a close echo appears */
    int close_ticks;    /* ...and how many ticks it lasts           */
    bool close_used;
    int turns[16], n_turns;
    float moves[16];
    int n_moves;
    bool holds_turn_in;
} Sim;

static float leg_done(const S5Maneuver *m)
{
    return m->state == S5_MAN_PASS ? m->pass_done_cm :
           m->state == S5_MAN_SEARCH ? m->searched_cm :
           m->state == S5_MAN_OFFSET ? m->offset_done_cm : 0.0f;
}

static void sim_init(Sim *s, S5Side side, int hint, S5ManeuverState tape_state, float tape_at)
{
    S5Profile p = {0};
    memset(s, 0, sizeof *s);
    s->now = 1000u;
    s->id = 100u;
    s->front_cm = 80.0f;
    s->tape_state = tape_state;
    s->tape_at = tape_at;
    s->tape_width = 1.8f;
    p.valid = true;
    p.estimated_width_cm = 12.0f; /* sweep under-reads: plan uses 14 */
    p.center_y_cm = 15.0f;
    p.bypass_side = side;
    assert(s5_maneuver_begin(&s->m, side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
                             &p, ROBOT_COURSE_BOX_DEPTH_CM, s->now));
    assert(s5_maneuver_plan_rejoin(&s->m, &p, 15.0f, hint));
}

static void sim_tick(Sim *s)
{
    S5ManeuverInput in = {0};
    float on_leg;
    s->now += 65u;
    if (s->running) {
        if (s->act.type == S5_ACT_TURN) {
            s->running = false; s->terminal = true; s->success = true;
            s->result_id = s->id; s->result_progress = 0.0f;
        } else {
            s->travelled += STEP_CM;
            if (s->travelled >= s->act.distance_cm) {
                s->travelled = s->act.distance_cm;
                s->running = false; s->terminal = true; s->success = true;
                s->result_id = s->id; s->result_progress = s->travelled;
            }
        }
    }
    on_leg = leg_done(&s->m) + (s->running || s->terminal ? s->travelled : 0.0f);
    if (s->terminal && s->act.type != S5_ACT_MOVE) on_leg = leg_done(&s->m);
    in.now_ms = s->now;
    in.front_valid = true;
    if (s->close_from > 0.0f && !s->close_used && s->m.state == S5_MAN_PASS &&
        on_leg >= s->close_from) {
        s->close_used = true;
        s->close_ticks = 8;
    }
    in.front_cm = s->close_ticks > 0 ? 12.0f : s->front_cm;
    if (s->close_ticks > 0) --s->close_ticks;
    in.line_valid = true;
    in.line_sample_ms = s->now;
    in.line_detected = s->m.state == s->tape_state && s->act.type == S5_ACT_MOVE &&
        on_leg >= s->tape_at && on_leg < s->tape_at + s->tape_width;
    in.stopped = !s->running;
    in.motion_terminal = s->terminal;
    in.result_id = s->result_id;
    in.motion_success = s->success;
    in.motion_cancelled = s->cancelled;
    in.motion_progress_cm = s->result_progress;
    S5Action a = s5_maneuver_step(&s->m, &in);
    if (a.type == S5_ACT_STOP && s->running) {
        s->running = false; s->terminal = true; s->success = false; s->cancelled = true;
        s->result_id = s->id; s->result_progress = s->travelled;
        return;
    }
    if (a.type == S5_ACT_MOVE || a.type == S5_ACT_TURN) {
        assert(!s->running);
        s->act = a;
        s->travelled = 0.0f;
        s->running = true;
        s->terminal = s->success = s->cancelled = false;
        ++s->id;
        assert(s5_maneuver_submitted(&s->m, s->id));
        if (a.type == S5_ACT_TURN) s->turns[s->n_turns++] = a.turn_deg;
        else s->moves[s->n_moves++] = a.distance_cm;
    } else if (!s->running && s->terminal && s->m.pending_id == 0u) {
        s->terminal = false;
    }
}

static void sim_run(Sim *s)
{
    for (int i = 0; i < 20000 && s->m.state != S5_MAN_COMPLETE && s->m.state != S5_MAN_FAILED; ++i) {
        /* The bridge holds TURN_IN until the controller sends FIND_LINE;
         * the executor itself never issues the turn before being stepped. */
        if (s->m.state == S5_MAN_TURN_IN && s->m.pending_id == 0u) s->holds_turn_in = true;
        sim_tick(s);
    }
}

static bool near(float a, float b) { return fabsf(a - b) < 0.6f; }

int main(void)
{
    Sim s;
    /* Plan: 14 cm planned width, 6.25 half chassis, 2.5 margin. */
    sim_init(&s, S5_SIDE_RIGHT, +1, S5_MAN_PASS, 39.0f);
    assert(near(s.m.offset_cm, 7.0f + 6.25f + 2.5f));
    assert(near(s.m.pass_needed_cm, 15.0f + 0.0f + 10.0f + 8.0f + 2.0f) ||
           near(s.m.pass_needed_cm, 15.0f + 5.0f + 10.0f + 8.0f + 2.0f));

    /* 1. Barcode RIGHT, bypass right: crossing on the pass leg, turn right
     *    onto that branch. No return to the original line. */
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE);
    assert(s.n_turns == 3 && s.turns[0] == 90 && s.turns[1] == -90 && s.turns[2] == 90);
    assert(s.n_moves == 3 && near(s.moves[0], s.m.offset_cm) &&
           near(s.moves[2], ROBOT_REJOIN_TAPE_ADVANCE_CM));
    assert(s.m.crossing_seen && s.m.turn_done && !s.holds_turn_in);
    assert(s.m.pass_done_cm < 39.0f + 2.5f); /* stopped on the tape */

    /* 2. Barcode LEFT, bypass right: pivot left back across the junction. */
    sim_init(&s, S5_SIDE_RIGHT, -1, S5_MAN_PASS, 39.0f);
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE && s.m.turn_done);
    assert(s.n_turns == 3 && s.turns[2] == -90);

    /* 3. Mirror: barcode LEFT, bypass left -> final left turn. */
    sim_init(&s, S5_SIDE_LEFT, -1, S5_MAN_PASS, 39.0f);
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE && s.m.turn_done);
    assert(s.turns[0] == -90 && s.turns[1] == 90 && s.turns[2] == -90);

    /* 4. No barcode turn: cross the tape, then return to the original line
     *    below the crossing and face the original heading again. */
    sim_init(&s, S5_SIDE_RIGHT, 0, S5_MAN_PASS, 39.0f);
    /* Run until the pass leg is over, then put the line on the return leg. */
    for (int i = 0; i < 20000 && s.m.state != S5_MAN_SEARCH; ++i) sim_tick(&s);
    assert(s.m.state == S5_MAN_SEARCH && s.m.crossing_seen && !s.m.turn_done);
    assert(s.m.pass_done_cm >= 39.0f + ROBOT_REJOIN_CROSS_EXTRA_CM - 1.0f);
    s.tape_state = S5_MAN_SEARCH;
    s.tape_at = s.m.offset_cm - 1.0f;
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE && !s.m.turn_done);
    assert(s.n_turns == 4 && s.turns[2] == -90 && s.turns[3] == 90);

    /* 5. No crossing at all: pass the box, turn in, find the line, align. */
    sim_init(&s, S5_SIDE_RIGHT, +1, S5_MAN_SEARCH, 14.0f);
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE && s.holds_turn_in && !s.m.turn_done);
    assert(near(s.m.pass_done_cm, s.m.pass_cm));
    assert(s.n_turns == 4 && s.turns[2] == -90 && s.turns[3] == 90);

    /* 6. No line on the return leg: bounded, then a line timeout. */
    sim_init(&s, S5_SIDE_RIGHT, 0, S5_MAN_IDLE, 0.0f);
    sim_run(&s);
    assert(s.m.state == S5_MAN_FAILED && s.m.error == S5_ERR_LINE_TIMEOUT);
    assert(s.m.searched_cm <= s.m.search_limit_cm + 0.5f);

    /* 7. A close echo on the pass leg pauses it; the rest is driven after. */
    sim_init(&s, S5_SIDE_RIGHT, +1, S5_MAN_PASS, 39.0f);
    s.close_from = 20.0f;
    for (int i = 0; i < 20000 && !s.m.blocked; ++i) sim_tick(&s);
    assert(s.m.blocked);
    sim_run(&s);
    assert(s.m.state == S5_MAN_COMPLETE && s.m.turn_done);

    puts("PASS B5 rejoin route: crossing turn both ways and mirrored, no-turn crossing, plain rejoin, line timeout, close-echo pause");
    return 0;
}
