#!/usr/bin/env python3
"""B5-only host test of the exact wire-adapter functions in task_obstacle.c.

Run from repository root: python3 testing/subsystem5/test_b1_contract.py
Requires a C11 host compiler (cc). Does not access motors or Wi-Fi.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
start = source.index('#define S5_WIRE_DISTANCE_MAX_CM')
end = source.index('#endif /* !S5_SIMULATION && S5_HARDWARE_ENABLED */', start)
functions = source[start:end]
functions = functions.replace('#if !S5_SIMULATION && S5_HARDWARE_ENABLED', '')

header = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
typedef uint16_t UH;
#define COMM_CM_INVALID 0xffffU
typedef enum { S5_SIDE_NONE, S5_SIDE_LEFT, S5_SIDE_RIGHT } S5Side;
typedef enum { AVOID_NONE, AVOID_STOP, AVOID_CONTINUE, AVOID_LEFT,
               AVOID_RIGHT, AVOID_REVERSE } AVOID_ACTION;
typedef struct {
    bool valid;
    float nearest_cm;
    float estimated_width_cm;
    float left_clearance_cm;
    float right_clearance_cm;
} S5Profile;
static UH reported[4];
static AVOID_ACTION reported_action;
static unsigned report_count;
static int comm_report_obstacle(UH d, UH w, UH l, UH r, AVOID_ACTION a)
{
    reported[0] = d; reported[1] = w; reported[2] = l; reported[3] = r;
    reported_action = a; ++report_count; return 0;
}
'''
main = r'''
int main(void)
{
    S5Profile profile = {true, 35.4f, 50.5f, 400.0f, 0.0f};
    assert(s5_wire_cm(0.0f, S5_WIRE_DISTANCE_MAX_CM) == 0u);
    assert(s5_wire_cm(35.4f, S5_WIRE_DISTANCE_MAX_CM) == 35u);
    assert(s5_wire_cm(400.0f, S5_WIRE_DISTANCE_MAX_CM) == 400u);
    assert(s5_wire_cm(500.0f, S5_WIRE_DISTANCE_MAX_CM) == 400u);
    assert(s5_wire_cm(-1.0f, S5_WIRE_DISTANCE_MAX_CM) == COMM_CM_INVALID);
    assert(s5_wire_cm(NAN, S5_WIRE_DISTANCE_MAX_CM) == COMM_CM_INVALID);
    assert(s5_wire_cm(101.0f, S5_WIRE_WIDTH_MAX_CM) == 100u);
    s5_report_profile(&profile, S5_SIDE_LEFT);
    assert(report_count == 1u && reported_action == AVOID_LEFT);
    assert(reported[0] == 35u && reported[1] == 51u);
    assert(reported[2] == 400u && reported[3] == 0u);
    s5_report_profile(&profile, S5_SIDE_NONE);
    assert(report_count == 2u && reported_action == AVOID_STOP);
    profile.valid = false;
    s5_report_profile(&profile, S5_SIDE_RIGHT);
    assert(report_count == 3u && reported_action == AVOID_STOP);
    assert(reported[0] == COMM_CM_INVALID && reported[1] == COMM_CM_INVALID);
    profile.valid = true;
    profile.nearest_cm = -1.0f;
    s5_report_profile(&profile, S5_SIDE_LEFT);
    assert(report_count == 4u && reported_action == AVOID_STOP);
    s5_report_profile(NULL, S5_SIDE_RIGHT);
    assert(report_count == 5u && reported_action == AVOID_STOP);
    puts("B5/B1 wire adapter: 14 assertions passed");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='s5_contract_') as folder:
    code = Path(folder) / 'contract.c'
    binary = Path(folder) / 'contract'
    code.write_text(header + '\n' + functions + '\n' + main)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    str(code), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
