#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/subsystem5/s5_sim.h"
static void trace(S5Scenario scenario, unsigned tick, S5State state,
                  S5Command cmd, void *ctx)
{
    (void)ctx;
    printf("%-17s tick=%02u state=%-10s action=%s\n",
           s5_scenario_name(scenario), tick, s5_state_name(state),
           s5_command_name(cmd));
}
int main(void)
{
    unsigned passed = 0, failed = 0, i;
    const S5State expected_state[S5_SCENARIO_COUNT] = {
        S5_DONE, S5_DONE, S5_FAULT, S5_DONE, S5_DONE, S5_FAULT,
        S5_FAULT, S5_FAULT, S5_FAULT, S5_FOLLOW, S5_FAULT, S5_FAULT
    };
    const S5Error expected_error[S5_SCENARIO_COUNT] = {
        S5_ERR_NONE, S5_ERR_NONE, S5_ERR_NO_CLEAR_ROUTE, S5_ERR_NONE,
        S5_ERR_NONE, S5_ERR_STOP_TIMEOUT, S5_ERR_MOTION_TIMEOUT,
        S5_ERR_REVERSE_TIMEOUT, S5_ERR_LINE_TIMEOUT, S5_ERR_NONE,
        S5_ERR_FRONT_SENSOR, S5_ERR_FINE_SCAN
    };
    for (i = 0; i < S5_SCENARIO_COUNT; i++) {
        S5SimResult r = s5_sim_run((S5Scenario)i, trace, NULL);
        const S5Profile *p = &r.controller.profile;
        int ok = r.terminated && r.controller.state == expected_state[i] &&
                 r.controller.error == expected_error[i];
        if (i == S5_SCENARIO_LEFT_CLEAR)
            ok = ok && r.controller.side == S5_SIDE_LEFT && p->valid &&
                 p->estimated_width_cm > 10.0f && p->estimated_width_cm < 12.0f &&
                 p->nearest_cm == 15.0f && p->nearest_angle_deg == 90u &&
                 p->left_clearance_cm == 90.0f && p->right_clearance_cm == 65.0f;
        if (i == S5_SCENARIO_RIGHT_CLEAR || i == S5_SCENARIO_INVALID_COARSE)
            ok = ok && r.controller.side == S5_SIDE_RIGHT;
        if (i == S5_SCENARIO_REVERSE_CLEARS || i == S5_SCENARIO_BOTH_BLOCKED)
            ok = ok && r.controller.reverse_attempts == 1u;
        printf("[PROFILE] %-17s width=%.2fcm nearest=%.2fcm angle=%u left=%.2fcm right=%.2fcm margin=%.2fcm\n",
               s5_scenario_name((S5Scenario)i), p->estimated_width_cm,
               p->nearest_cm, (unsigned)p->nearest_angle_deg,
               p->left_clearance_cm, p->right_clearance_cm,
               p->required_lateral_clearance_cm);
        printf("[HOST TEST] %-17s %s (state=%s error=%s)\n\n",
               s5_scenario_name((S5Scenario)i), ok ? "PASS" : "FAIL",
               s5_state_name(r.controller.state), s5_error_name(r.controller.error));
        if (ok) passed++; else failed++;
    }
    {
        uint16_t angles[S5_SCAN_COUNT];
        s5_make_fine_angles(90u, angles);
        int ok = angles[0] == 70u && angles[1] == 80u &&
                 angles[2] == 90u && angles[3] == 100u && angles[4] == 110u;
        printf("[HOST TEST] adaptive-angles   %s\n", ok ? "PASS" : "FAIL");
        if (ok) passed++; else failed++;
    }
    printf("[HOST TEST] TOTAL %u passed, %u failed\n", passed, failed);
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
