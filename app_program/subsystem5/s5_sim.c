#include "s5_sim.h"
#include <stddef.h>
static S5Scan scan_make(bool left, bool right, bool invalid)
{
    S5Scan s = {0};
    static const uint16_t angles[5] = {30, 60, 90, 120, 150};
    int i;
    for (i = 0; i < 5; i++) {
        s.points[i].angle_deg = angles[i];
        s.points[i].valid = true;
        s.points[i].distance_cm = i < 2 ? (left ? 90.0f : 15.0f) :
                                  i > 2 ? (right ? 65.0f : 12.0f) : 16.0f;
    }
    if (invalid) s.points[0].valid = false;
    s.nearest_index = 2;
    return s;
}
static S5Scan fine_make(bool invalid)
{
    S5Scan s = {0};
    static const uint16_t angles[5] = {70, 80, 90, 100, 110};
    static const float distances[5] = {48, 20, 15, 22, 50};
    int i;
    for (i = 0; i < 5; i++) {
        s.points[i].angle_deg = angles[i];
        s.points[i].distance_cm = distances[i];
        s.points[i].valid = true;
    }
    if (invalid) s.points[2].valid = false;
    s.nearest_index = 2;
    return s;
}
const char *s5_scenario_name(S5Scenario s)
{
    static const char *names[] = {"left-clear", "right-clear", "both-blocked",
        "route-clears", "invalid-coarse", "stop-timeout", "motion-timeout",
        "route-blocked", "line-timeout", "clear-path", "front-timeout",
        "fine-invalid"};
    return (unsigned)s < S5_SCENARIO_COUNT ? names[s] : "unknown";
}
S5SimResult s5_sim_run(S5Scenario scenario, S5Trace trace, void *user)
{
    S5SimResult r = {0};
    unsigned i;
    s5_controller_init(&r.controller);
    for (i = 0; i < S5_MOTION_LIMIT + S5_RECOVERY_LIMIT + 4u * S5_WAIT_LIMIT + 100u; i++) {
        S5Input in = {0};
        S5State previous = r.controller.state;
        bool route_clear = scenario == S5_SCENARIO_ROUTE_CLEARS &&
                           r.controller.route_waits > 0u;
        bool left = scenario != S5_SCENARIO_RIGHT_CLEAR &&
                    scenario != S5_SCENARIO_BOTH_BLOCKED &&
                    scenario != S5_SCENARIO_ROUTE_BLOCKED &&
                    (scenario != S5_SCENARIO_ROUTE_CLEARS || route_clear);
        bool right = scenario != S5_SCENARIO_BOTH_BLOCKED &&
                     scenario != S5_SCENARIO_ROUTE_BLOCKED &&
                     (scenario != S5_SCENARIO_ROUTE_CLEARS || route_clear);
        in.front_valid = scenario != S5_SCENARIO_FRONT_TIMEOUT;
        in.front_cm = scenario == S5_SCENARIO_CLEAR_PATH ? 90.0f : S5_STOP_CM;
        in.stopped = scenario != S5_SCENARIO_STOP_TIMEOUT &&
                     r.controller.ticks_in_state >= 2;
        in.coarse_ready = r.controller.ticks_in_state >= 2;
        in.coarse = scan_make(left, right, scenario == S5_SCENARIO_INVALID_COARSE);
        in.fine_ready = r.controller.ticks_in_state >= 2;
        in.fine = fine_make(scenario == S5_SCENARIO_FINE_INVALID);
        in.motion_complete = scenario != S5_SCENARIO_MOTION_TIMEOUT &&
                             r.controller.ticks_in_state >= 2;
        in.line_found = scenario != S5_SCENARIO_LINE_TIMEOUT &&
                        r.controller.ticks_in_state >= 3;
        r.last_command = s5_controller_step(&r.controller, &in);
        if (r.last_command == S5_CMD_REVERSE) return r; /* Never autonomous. */
        r.steps = i + 1;
        if (trace && (r.last_command != S5_CMD_NONE || previous != r.controller.state))
            trace(scenario, r.steps, r.controller.state, r.last_command, user);
        if (r.controller.state == S5_DONE || r.controller.state == S5_FAULT) {
            r.terminated = true;
            break;
        }
        if ((scenario == S5_SCENARIO_BOTH_BLOCKED || scenario == S5_SCENARIO_ROUTE_BLOCKED) &&
            r.controller.state == S5_WAIT_ROUTE && r.controller.route_waits >= 2u) {
            r.terminated = true; /* Verified repeated pause/rescan, no movement. */
            break;
        }
        if (scenario == S5_SCENARIO_CLEAR_PATH && i >= 9) {
            r.terminated = true;
            break;
        }
    }
    return r;
}
