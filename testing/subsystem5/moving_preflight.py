#!/usr/bin/env python3
"""Read-only preflight for B5 guarded forward/STOP test. No hardware access."""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[2]
b5 = (root / 'app_program/subsystem5/subsystem5.h').read_text()
task = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
config = (root / 'app_program/robot_config.h').read_text()

checks = {
    'B5 guarded forward/STOP enabled': bool(re.search(r'^#define S5_DEMO_SENSOR_ONLY\s+0\s*$', b5, re.M)),
    'B5 physical bypass remains disabled': (bool(re.search(r'^#define S5_MOTION_BRIDGE_ENABLED\s+0\s*$', b5, re.M)) or ('#define S5_MOTION_BRIDGE_ENABLED ROBOT_ENABLE_BYPASS' in b5 and bool(re.search(r'^#define ROBOT_ENABLE_BYPASS\s+0\s*$', config, re.M)))),
    'B5 single-ping moving monitor present': 'B5-GUARDED-DRIVE-2026-11-04-V3' in task and 's5_guard_stop_history_record' in task,
    'B5 motor STOP on invalid/close echo': 's5_demo_clear_stop();\n            driving = false;' in task,
    'B5 B2 heartbeat refresh while moving': 'b2_request_forward();\n                if (b2_safety_motion_allowed()) s5_demo_clear_publish();' in task,
    'B5 timed STOP hold and recovery configured': all(x in b5 for x in ('S5_GUARD_STOP_LIMIT 3u', 'S5_GUARD_COOLDOWN_MS 8000u', 'S5_GUARD_REARM_CLEAR_BATCHES 5u')) and 's5_guard_stop_history_clear' in task,
    'B5 sends B2 DRIVE via public API': 'motion_send_command(&command)' in task and 'command.type = MOTION_CMD_DRIVE' in task,
    'B5 B2 command watchdog refreshed': 'S5_DEMO_DRIVE_REFRESH_MS 35u' in b5 and 's5_demo_drive_task' in task,
    'B5 no conflicting libc size_t includes': '#include <math.h>' not in task and '#include <stddef.h>' not in task,
    'B2 one-shot test disabled': bool(re.search(r'^#define ROBOT_MOTION_TEST\s+0\s*$', config, re.M)),
    'B2 diagnostics quiet for real driving': bool(re.search(r'^#define ROBOT_ENCODER_DIAGNOSTICS\s+0\s*$', config, re.M)),
    'Robot physical bypass disabled': bool(re.search(r'^#define ROBOT_ENABLE_BYPASS\s+0\s*$', config, re.M)),
    'Expected B5 pins': all(x in b5 for x in ('S5_TRIG_PIN 16u', 'S5_ECHO_PIN 17u', 'S5_SERVO_PIN 12u')),
}
for label, passed in checks.items():
    print(('PASS' if passed else 'FAIL') + ': ' + label)

motion_test = re.search(r'^#define ROBOT_MOTION_TEST\s+(\d+)', config, re.M)
if motion_test and motion_test.group(1) != '0':
    print('WARNING: ROBOT_MOTION_TEST=' + motion_test.group(1) + ' (B2 one-shot test may move the robot independently).')
    print('Coordinate with B2/VC to set ROBOT_MOTION_TEST=0 for continuous guarded driving.')
print('NOTE: These static checks do NOT verify hardware safety, sensor reliability or a Pico build.')
sys.exit(0 if all(checks.values()) else 1)
