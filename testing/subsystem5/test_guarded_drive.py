#!/usr/bin/env python3
"""B5 forward-demo policy host tests; never sends hardware motor commands."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
task = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
header = (root / 'app_program/subsystem5/subsystem5.h').read_text()
config = (root / 'app_program/robot_config.h').read_text()

assert '#include <math.h>' not in task
assert '#include <stddef.h>' not in task
assert '__builtin_isfinite' in task
assert '#define ROBOT_MOTION_TEST 0' in config
assert '#define ROBOT_ENCODER_DIAGNOSTICS 0' in config
assert '#define S5_DEMO_B2_FORWARD_COMMANDS (!ROBOT_FORWARD_STOP_MODE)' in header
assert '#define S5_DEMO_SENSOR_ONLY 0' in header
assert '#define S5_MOTION_BRIDGE_ENABLED ROBOT_ENABLE_BYPASS' in header
assert '#define ROBOT_FORWARD_STOP_MODE (!ROBOT_ENABLE_BYPASS)' in config
assert 'motion_send_command(&command)' in task
assert 'command.type = MOTION_CMD_DRIVE;' in task
assert 'command.direction = MOTION_DIR_FORWARD;' in task
assert 'command.safety_generation = b2_safety_stop_generation();' in task
assert 's5_demo_drive_fault_stop("B2 controller/encoder fault or timeout")' in task
assert 's5_demo_clear_stop();' in task
assert 'front clearance expired; STOP and recheck' in task
assert 's5_demo_drive_fault_stop("front sensor clearance expired")' not in task
assert 's5_demo_clear_stop();' in task
assert 'b2_request_stop();' in task
assert 'S5_DEMO_DRIVE_REFRESH_MS 35u' in header
assert 'S5_DEMO_CLEAR_MAX_AGE_MS 350u' in header
assert 's5_demo_lock_enter' in task and 's5_demo_lock_leave' in task

# Test the real source predicate (not a rewritten Python equivalent).
match = re.search(r'(static bool s5_demo_can_refresh\([\s\S]*?\n\})', task)
assert match, 'guarded DRIVE predicate missing'
code = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define S5_DEMO_CLEAR_MAX_AGE_MS 350u
""" + match.group(1) + r"""
int main(void)
{
    assert(s5_demo_can_refresh(1000u, 800u, true, true));
    assert(s5_demo_can_refresh(1350u, 1000u, true, true));
    assert(!s5_demo_can_refresh(1351u, 1000u, true, true));
    assert(!s5_demo_can_refresh(1000u, 900u, false, true));
    assert(!s5_demo_can_refresh(1000u, 900u, true, false));
    assert(!s5_demo_can_refresh(1000u, 900u, false, false));
    /* Millisecond timer rollover must not invalidate a fresh echo. */
    assert(s5_demo_can_refresh(10u, 0xfffffff0u, true, true));
    assert(!s5_demo_can_refresh(500u, 0xfffffff0u, true, true));
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix='b5_demo_drive_') as tmp:
    source = Path(tmp) / 'drive_policy.c'
    exe = Path(tmp) / 'drive_policy'
    source.write_text(code)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('B5 guarded DRIVE source/policy tests: PASS')
