#ifndef S5_SIM_H
#define S5_SIM_H
#include "s5_logic.h"
typedef enum {
    S5_SCENARIO_LEFT_CLEAR, S5_SCENARIO_RIGHT_CLEAR,
    S5_SCENARIO_BOTH_BLOCKED, S5_SCENARIO_REVERSE_CLEARS,
    S5_SCENARIO_INVALID_COARSE, S5_SCENARIO_STOP_TIMEOUT,
    S5_SCENARIO_MOTION_TIMEOUT, S5_SCENARIO_REVERSE_TIMEOUT,
    S5_SCENARIO_LINE_TIMEOUT, S5_SCENARIO_CLEAR_PATH,
    S5_SCENARIO_FRONT_TIMEOUT, S5_SCENARIO_FINE_INVALID,
    S5_SCENARIO_COUNT
} S5Scenario;
typedef struct {
    S5Controller controller;
    S5Command last_command;
    unsigned steps;
    bool terminated;
} S5SimResult;
typedef void (*S5Trace)(S5Scenario scenario, unsigned step,
                        S5State state, S5Command command, void *user);
S5SimResult s5_sim_run(S5Scenario scenario, S5Trace trace, void *user);
const char *s5_scenario_name(S5Scenario scenario);
#endif
