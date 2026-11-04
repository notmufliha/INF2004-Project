"""Test real motor setters and the moving obstacle threshold without hardware."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'app_program/subsystem2/motor_control.c').read_text()
setters = source[source.index('void motor_left_set('):
                 source.index('/** @brief Disable motor PWM after')]
task = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
condition = re.search(r'if \((guard_valid && \(guard.cm [^\n]+)\) \{', task).group(1)
scan = task[task.index('        if (scan_needed ||'):task.index('#endif /* !S5_MOTION_BRIDGE_ENABLED */')]
assert scan.index('s5_demo_clear_stop();') < scan.index('if (!b2_motor_is_stopped())') < scan.index('s5_coarse_scan(&coarse)')
assert scan.index('clear_hits = 0u;') < scan.index('s5_coarse_scan(&coarse)')
assert 's5_fine_scan(&fine, center)' in scan
code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "CONFIG"
typedef unsigned UW;
#define MOTOR_L_A_PIN ROBOT_MOTOR_L_REV_PIN
#define MOTOR_L_B_PIN ROBOT_MOTOR_L_FWD_PIN
#define MOTOR_R_A_PIN ROBOT_MOTOR_R_FWD_PIN
#define MOTOR_R_B_PIN ROBOT_MOTOR_R_REV_PIN
#define MOTOR_PWM_WRAP ROBOT_PWM_TOP
static bool gb_motor_ready = true;
static unsigned pins[16], values[16], writes, pwm[32];
static void pwm_set_cc(unsigned pin, unsigned value) {
    pins[writes] = pin; values[writes++] = value; pwm[pin] = value;
}
SETTERS
static bool keep_moving(bool guard_valid, float cm) {
    struct { float cm; } guard = { cm };
    return CONDITION;
}
static void test_motor(void (*set)(int), unsigned fwd, unsigned rev) {
    writes = 0; set(50);
    assert(writes == 2 && pins[0] == rev && values[0] == 0);
    assert(pins[1] == fwd && values[1] == 499);
    writes = 0; set(50); /* Repeated refresh must never clear active input. */
    assert(writes == 2 && pins[0] == rev && pins[1] == fwd && values[1] == 499);
    writes = 0; set(-30); /* Opposite input clears BEFORE reverse starts. */
    assert(writes == 2 && pins[0] == fwd && values[0] == 0);
    assert(pins[1] == rev && values[1] == 299);
    writes = 0; set(0); assert(pwm[fwd] == 0 && pwm[rev] == 0);
    writes = 0; set(150); assert(pwm[fwd] == ROBOT_PWM_TOP);
    writes = 0; gb_motor_ready = false; set(50); assert(writes == 0);
    gb_motor_ready = true;
}
int main(void) {
    test_motor(motor_left_set, ROBOT_MOTOR_L_FWD_PIN, ROBOT_MOTOR_L_REV_PIN);
    test_motor(motor_right_set, ROBOT_MOTOR_R_FWD_PIN, ROBOT_MOTOR_R_REV_PIN);
    assert(keep_moving(true, 100));
    assert(keep_moving(true, 44) && keep_moving(true, 36));
    assert(!keep_moving(true, 35) && !keep_moving(true, 20));
    assert(!keep_moving(false, 100));
    puts("PASS: continuous PWM, direction interlock, STOP, duty limits, moving hysteresis, invalid echo STOP");
}
'''.replace('CONFIG', (root / 'app_program/robot_config.h').as_posix()).replace('SETTERS', setters).replace('CONDITION', condition)
with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'test.c'
    exe = Path(tmp) / 'test'
    src.write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: obstacle scanning reachable in forward/STOP mode after STOP acknowledgement')
