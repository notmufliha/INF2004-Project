#!/usr/bin/env python3
"""Check B5-owned fault flag, error constants and B1 laptop names."""
from pathlib import Path
import ast
import re

ROOT = Path(__file__).resolve().parents[2]
header = (ROOT / 'app_program/error_codes.h').read_text()
source = (ROOT / 'app_program/subsystem5/task_obstacle.c').read_text()
faults = (ROOT / 'app_program/fault_flags.h').read_text()
wire = (ROOT / 'tools/s1_wire.py').read_text()

expected = {
    'ERR_S5_NO_ECHO': 1,
    'ERR_S5_STOP_ACK': 2,
    'ERR_S5_SCAN_TIMEOUT': 3,
    'ERR_S5_NO_CLEAR_ROUTE': 4,
    'ERR_S5_FINE_SCAN': 5,
    'ERR_S5_MOTION_TIMEOUT': 6,
    'ERR_S5_LINE_TIMEOUT': 7,
    'ERR_S5_CLEARANCE': 8,
    'ERR_S5_REVERSE_TIMEOUT': 9,
}
for name, number in expected.items():
    pattern = rf'^#define\s+{name}\s+ERR_CODE\(5,\s*{number}\)'
    assert re.search(pattern, header, re.M), (name, number)

assert re.search(r'#define\s+FAULT_NO_ECHO\s+0x08U', faults)
for op in ('fault_raise', 'fault_clear'):
    calls = re.findall(rf'\b{op}\s*\(\s*([^)]*)\)', source)
    assert calls, op
    assert all(arg.strip() == 'FAULT_NO_ECHO' for arg in calls), calls

# Parse the Python literal without importing any network-related code.
module = ast.parse(wire)
values = {}
for node in module.body:
    if isinstance(node, ast.Assign):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('ERROR_NAMES', 'FAULT_BITS'):
                values[target.id] = ast.literal_eval(node.value)
assert len([k for k in values['ERROR_NAMES'] if k[0] == 5]) == 9
for number in range(1, 10):
    assert values['ERROR_NAMES'][(5, number)].startswith('B5 ')
assert values['FAULT_BITS'][0x08] == 'NO_ECHO'
# Exactly one B1 obstacle event per scan; the one adapter must validate values.
assert len(re.findall(r'\bcomm_report_obstacle\s*\(', source)) == 1
assert 's5_report_profile(&profile, side);' in source
assert 's5_wire_cm(cm, S5_WIRE_DISTANCE_MAX_CM)' in source
assert source.count('(UH)(cm + 0.5f)') == 1  # rounding occurs only in validated helper
# The header and B5 enum must keep their existing wire numbers.
logic = (ROOT / 'app_program/subsystem5/s5_logic.h').read_text()
enum = re.search(r'typedef enum \{\s*(S5_ERR_NONE.*?)\} S5Error;', logic, re.S)
assert enum is not None
names = [x.strip() for x in enum.group(1).split(',')]
assert len(names) == 10 and names[0] == 'S5_ERR_NONE'

print('[HOST TEST] B5 shared fault/error-code and decoder mapping: PASS')
