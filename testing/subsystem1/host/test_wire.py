#!/usr/bin/env python3
"""
test_wire.py -- checks that the Pico packer (comm_pack.c) and the laptop
decoder (tools/s1_wire.py) agree bit for bit.  Run via run_host_tests.sh.

  1. Every frame printed by "test_comm_pack vectors" is decoded with
     s1_wire and each field compared with the value the C side packed.
  2. Commands made by s1_wire.encode_command() are fed to the C decoder
     ("test_comm_pack decode") and must be accepted with the same
     id/arg/seq; deliberately broken frames must be rejected.
  3. The 24-bit encoder unwrap rebuilds totals across the wrap.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "tools"))
import s1_wire  # noqa: E402

EXE = os.path.join(HERE, "test_comm_pack")
fails = 0


def check(what, got, want):
    global fails
    if got != want:
        print(f"FAIL {what}: got {got!r} want {want!r}")
        fails += 1


def run(mode, stdin=None):
    r = subprocess.run([EXE, mode], input=stdin, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        sys.exit(f"test_comm_pack {mode} failed")
    return r.stdout


# Decoder fields whose raw value is replaced by None for "not measured"
INVALID = {"obs_d_cm": 0x1FF, "obs_lc_cm": 0x1FF, "obs_rc_cm": 0x1FF,
           "obs_w_cm": 0x7F, "d_cm": 0x1FF, "lc_cm": 0x1FF, "rc_cm": 0x1FF,
           "w_cm": 0x7F}

# 1. C pack -> Python decode
n_frames = 0
for line in run("vectors").splitlines():
    kind, hexstr, *pairs = line.split()
    data = bytes.fromhex(hexstr)
    d = s1_wire.decode(kind, data)
    s1_wire.describe(kind, d)                       # must not raise
    check(f"{kind} version", d["ver"], s1_wire.WIRE_VERSION)
    for p in pairs:
        key, val = p.split("=")
        want = int(val)
        if key in INVALID and want == INVALID[key]:
            want = None
        check(f"{kind} {key} in {hexstr}", d[key], want)
    n_frames += 1
print(f"decoded {n_frames} C-packed frames")

# 2. Python encode -> C decode
good = [("START", 0, 1), ("STOP", 0, 2), ("ESTOP", 0, 3), ("PAUSE", 0, 4),
        ("RESUME", 0, 5), ("MODE AUTO", 0, 6), ("MODE TEST", 2, 7),
        ("SPEED 0", 0, 8), ("SPEED 100", 100, 255), ("RESET_PEAK", 0, 0),
        ("PING", 0, 9), ("STATUS", 0, 10), ("TELEM 100", 1, 11),
        ("TELEM 10000", 100, 12)]
frames = [s1_wire.encode_command(t, seq).hex() for t, _, seq in good]
for f in frames:
    check("command frame length", len(f), 6)
out = run("decode", "\n".join(frames) + "\n").splitlines()
for (text, arg, seq), res in zip(good, out):
    r, cid, carg, cseq = (int(x) for x in res.split())
    check(f"C accepts {text}", r, 0)
    check(f"{text} id", cid, s1_wire.COMMANDS[text.split()[0]])
    check(f"{text} arg", carg, arg)
    check(f"{text} seq", cseq, seq)

C = s1_wire.COMMAND
bad = [  # (frame hex, expected result code)
    ("2e00", 8),                                     # 2 bytes
    ("2e002c00", 8),                                 # 4 bytes
    (s1_wire.pack(C, {"ver": 2, "cmd": 2}).hex(), 9),
    (s1_wire.pack(C, {"ver": 1, "cmd": 2, "reserved": 1}).hex(), 10),
    (s1_wire.pack(C, {"ver": 1, "cmd": 0}).hex(), 11),
    (s1_wire.pack(C, {"ver": 1, "cmd": 12}).hex(), 11),
    (s1_wire.pack(C, {"ver": 1, "cmd": 7, "arg": 101}).hex(), 12),
    (s1_wire.pack(C, {"ver": 1, "cmd": 6, "arg": 3}).hex(), 12),
    (s1_wire.pack(C, {"ver": 1, "cmd": 11, "arg": 0}).hex(), 12),
    (s1_wire.pack(C, {"ver": 1, "cmd": 2, "arg": 5}).hex(), 13),
]
out = run("decode", "\n".join(f for f, _ in bad) + "\n").splitlines()
for (f, want), res in zip(bad, out):
    check(f"C rejects {f}", int(res.split()[0]), want)
print(f"checked {len(good)} valid and {len(bad)} invalid command frames")

# Laptop-side validation refuses what the car would refuse
for text in ["FLY", "SPEED 101", "SPEED -5", "STOP now", "MODE FLY",
             "TELEM 50", "TELEM 250", ""]:
    try:
        s1_wire.encode_command(text)
        check(f"encoder refuses {text!r}", "accepted", "ValueError")
    except ValueError:
        pass

# 1-byte online / Last Will messages
check("online byte", s1_wire.decode_status(b"\x30"), {"ver": 1, "online": 1})
check("offline byte", s1_wire.decode_status(b"\x20"), {"ver": 1, "online": 0})

# 3. 24-bit encoder unwrap, forwards and backwards across the wrap.
#    Valid while consecutive frames differ by < 2^23 counts (real worst
#    case: 10 s x 5000 counts/s = 50,000).
u = s1_wire.Unwrap24()
for total in [0, 2500, 8388000, 8389000, 16777000, 16778000, 24000000,
              30000000, 33554500, 33550000]:
    check(f"unwrap {total}", u.update(total & 0xFFFFFF), total)
u = s1_wire.Unwrap24()
for total in [-10, -8388000, -8389000, -12000000, -17000000]:
    check(f"unwrap {total}", u.update(total & 0xFFFFFF), total)

print(run("sizes"), end="")
print("ALL HOST TESTS PASSED" if fails == 0 else f"{fails} HOST TEST(S) FAILED")
sys.exit(1 if fails else 0)
