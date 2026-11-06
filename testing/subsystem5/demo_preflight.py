#!/usr/bin/env python3
"""Read-only preflight: verifies the B5 demonstration is not armed to drive."""
from pathlib import Path
import re
import sys
root = Path(__file__).resolve().parents[2]
b5 = (root / 'app_program/subsystem5/subsystem5.h').read_text()
task = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
config = (root / 'app_program/robot_config.h').read_text()
checks = {
    'B5 sensor-only demo is enabled': bool(re.search(r'^#define S5_DEMO_SENSOR_ONLY\s+1\s*$', b5, re.M)),
    'B5 full-motion bridge stays disabled': (bool(re.search(r'^#define S5_MOTION_BRIDGE_ENABLED\s+0\s*$', b5, re.M)) or ('#define S5_MOTION_BRIDGE_ENABLED ROBOT_ENABLE_BYPASS' in b5 and bool(re.search(r'^#define ROBOT_ENABLE_BYPASS\s+0\s*$', config, re.M)))),
    'B5 task never releases clearance in demo branch':
        '#if S5_DEMO_SENSOR_ONLY' in task and 'b2_request_stop();\n                    driving = false;\n#else' in task,
    'B5 emergency bridge calls real B2 STOP':
        'b2_request_stop();' in (root / 'app_program/subsystem5/s5_bridge.c').read_text(),
    'Project physical bypass disabled': bool(re.search(r'^#define ROBOT_ENABLE_BYPASS\s+0\s*$', config, re.M)),
    'B5 GPIO pins 16, 17, 12': all(x in b5 for x in ('S5_TRIG_PIN 16u','S5_ECHO_PIN 17u','S5_SERVO_PIN 12u')),
}
for label, passed in checks.items():
    print(('PASS' if passed else 'FAIL'), label)
print('NOTE: ROBOT_MOTION_TEST is a team-owned option; ask B2 to set it to 0 for a quiet demo.')
print('NOTE: Static checks do not replace physical testing or a Pico firmware build.')
sys.exit(0 if all(checks.values()) else 1)
