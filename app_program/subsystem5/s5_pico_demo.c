#include "subsystem5.h"
#include "s5_pico_demo.h"
#include "s5_sim.h"

/* Expected results are mirrored from the host test harness.
 * These assertions run ON the Pico WH using simulated inputs. */
static const S5State expected_states[S5_SCENARIO_COUNT] = {
    S5_DONE, S5_DONE, S5_FAULT, S5_DONE, S5_DONE, S5_FAULT,
    S5_FAULT, S5_FAULT, S5_FAULT, S5_FOLLOW, S5_FAULT, S5_FAULT
};
static const S5Error expected_errors[S5_SCENARIO_COUNT] = {
    S5_ERR_NONE, S5_ERR_NONE, S5_ERR_NO_CLEAR_ROUTE, S5_ERR_NONE,
    S5_ERR_NONE, S5_ERR_STOP_TIMEOUT, S5_ERR_MOTION_TIMEOUT,
    S5_ERR_REVERSE_TIMEOUT, S5_ERR_LINE_TIMEOUT, S5_ERR_NONE,
    S5_ERR_FRONT_SENSOR, S5_ERR_FINE_SCAN
};

static void print_transition(S5Scenario scenario, unsigned step,
                             S5State state, S5Command cmd, void *ctx)
{
    (void)ctx;
    tm_printf((UB*)"[S5-PICO] %-17s tick=%d state=%s action=%s\n",
              s5_scenario_name(scenario), (int)step,
              s5_state_name(state), s5_command_name(cmd));
}

static void print_profile(const S5Profile *p)
{
    if (!p->valid) {
        tm_printf((UB*)"[S5-PICO] PROFILE unavailable (invalid/unbracketed scan)\n");
        return;
    }
    tm_printf((UB*)"[S5-PICO] PROFILE width_cm_x100=%d nearest_cm_x100=%d bearing_deg=%d\n",
              (int)(p->estimated_width_cm * 100.0f),
              (int)(p->nearest_cm * 100.0f),
              (int)p->nearest_angle_deg);
    tm_printf((UB*)"[S5-PICO] PROFILE left_cm_x100=%d right_cm_x100=%d required_cm_x100=%d side=%d\n",
              (int)(p->left_clearance_cm * 100.0f),
              (int)(p->right_clearance_cm * 100.0f),
              (int)(p->required_lateral_clearance_cm * 100.0f),
              (int)p->bypass_side);
}

void s5_pico_demo_run_forever(void)
{
    unsigned cycle = 0u;
    tm_printf((UB*)"\n[S5-PICO] ===== BUDDY 5 PICO-ONLY DEMONSTRATION =====\n");
    tm_printf((UB*)"[S5-PICO] REAL Pico WH CPU / micro T-Kernel; SIMULATED sensor data\n");
    tm_printf((UB*)"[S5-PICO] No HC-SR04, servo, buttons, motor, or wiring required\n");
    tm_printf((UB*)"[S5-PICO] No physical obstacle width or movement measured\n");
    tm_printf((UB*)"[S5-PICO] COARSE angles: 30, 60, 90, 120, 150 deg\n");
    tm_printf((UB*)"[S5-PICO] Example FINE angles: 70, 80, 90, 100, 110 deg\n");
    tm_printf((UB*)"[S5-PICO] Width and clearance units: cm_x100 (divide by 100)\n");
    for (;;) {
        unsigned passed = 0u, failed = 0u, i;
        cycle++;
        tm_printf((UB*)"\n[S5-PICO] === AUTOMATIC TEST CYCLE %d ===\n", (int)cycle);
        for (i = 0; i < (unsigned)S5_SCENARIO_COUNT; i++) {
            S5Scenario scenario = (S5Scenario)i;
            S5SimResult result = s5_sim_run(scenario, print_transition, 0);
            int ok = result.terminated &&
                     result.controller.state == expected_states[i] &&
                     result.controller.error == expected_errors[i];
            /* Additional meaningful profiling and routing checks. */
            if (scenario == S5_SCENARIO_LEFT_CLEAR) {
                ok = ok && result.controller.side == S5_SIDE_LEFT &&
                     result.controller.profile.valid &&
                     result.controller.profile.estimated_width_cm > 10.0f &&
                     result.controller.profile.estimated_width_cm < 12.0f &&
                     result.controller.profile.nearest_cm == 15.0f;
            }
            if (scenario == S5_SCENARIO_RIGHT_CLEAR)
                ok = ok && result.controller.side == S5_SIDE_RIGHT;
            if (scenario == S5_SCENARIO_BOTH_BLOCKED)
                ok = ok && result.controller.reverse_attempts == 1u;
            if (scenario == S5_SCENARIO_LEFT_CLEAR ||
                scenario == S5_SCENARIO_RIGHT_CLEAR ||
                scenario == S5_SCENARIO_FINE_INVALID)
                print_profile(&result.controller.profile);
            tm_printf((UB*)"[S5-PICO] TEST %-17s %s state=%s error=%s\n",
                      s5_scenario_name(scenario), ok ? "PASS" : "FAIL",
                      s5_state_name(result.controller.state),
                      s5_error_name(result.controller.error));
            if (ok) passed++; else failed++;
            tk_dly_tsk(100);
        }
        tm_printf((UB*)"[S5-PICO] SUMMARY cycle=%d passed=%d failed=%d\n",
                  (int)cycle, (int)passed, (int)failed);
        tm_printf((UB*)"[S5-PICO] Waiting 15 seconds before next cycle...\n");
        tk_dly_tsk(S5_PICO_DEMO_PAUSE_MS);
    }
}
