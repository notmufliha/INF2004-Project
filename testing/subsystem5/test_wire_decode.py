#!/usr/bin/env python3
"""B5-only end-to-end laptop wire decoding checks (no Wi-Fi or Pico required)."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import s1_wire as wire  # noqa: E402


def event(kind, **fields):
    body = {'ver': wire.WIRE_VERSION, 'type': kind, 'seq': 23, 't_ds': 50}
    body.update(fields)
    layout = wire.EVENT_HEADER + wire.EVENT_BODY[kind]
    return wire.decode_event(wire.pack(layout, body))


# The B5 scan uses 9/7/9/9/3 bits. Each event decodes to exactly one
# semantic obstacle profile, and STOP must not be interpreted as a route.
for action, name in ((1, 'STOP'), (3, 'LEFT'), (4, 'RIGHT')):
    result = event(2, d_cm=400, w_cm=100, lc_cm=400,
                   rc_cm=0, action=action)
    assert result['type_name'] == 'obstacle'
    assert (result['d_cm'], result['w_cm']) == (400, 100)
    assert (result['lc_cm'], result['rc_cm']) == (400, 0)
    assert result['action_name'] == name

# A malformed scan must be transmitted as all-invalid fields and STOP.
result = event(2, d_cm=wire.DIST_INVALID, w_cm=wire.WIDTH_INVALID,
               lc_cm=wire.DIST_INVALID, rc_cm=wire.DIST_INVALID, action=1)
assert result['action_name'] == 'STOP'
assert all(result[field] is None for field in
           ('d_cm', 'w_cm', 'lc_cm', 'rc_cm'))

# Each of B5's 9 error numbers must decode to a meaningful B5 name.
for number in range(1, 10):
    code = (5 << 5) | number
    result = event(3, err=code, detail=45 if number == 1 else 0)
    assert result['type_name'] == 'error'
    assert result['err'] == code
    assert result['err_text'].startswith(f'S5#{number} B5 ')
    assert 'unknown' not in result['err_text']

# The shared B5 bit must be visible in a status frame; unrelated bits
# are decoded independently and are not controlled by B5.
assert wire.fault_names(0x08) == ['NO_ECHO']
assert wire.fault_names(0x00) == ['none']
print('[HOST TEST] B5 laptop wire decode: PASS (3 actions, invalid, 9 errors, faults)')
