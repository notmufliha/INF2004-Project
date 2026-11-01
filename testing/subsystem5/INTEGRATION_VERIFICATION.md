# B5 verification record — host checks

Base: team `origin/main` at commit `0850acf` (B1 merged).

Host checks executed successfully on Linux with GCC and Python:

| Check | Result |
|---|---|
| B5 logic simulation `run_host_tests.sh` | PASS — 14 scenarios |
| B5 packed-format `run_packed_tests.sh` | PASS — roundtrips and invalid inputs |
| B5/B1 adapter `test_b1_contract.py` | PASS — 14 assertions |
| Shared B5 fault/error/decoder mapping `test_b5_error_contract.py` | PASS |
| B1 communication host tests | PASS — C and Python decoder tests |
| `python3 -m py_compile tools/s1_wire.py` | PASS |

**Not executed:** full ARM Pico firmware build, robot hardware run, Wi-Fi
priority reordering, before/after RAM measurements, physical obstacle
avoidance, line recovery and timing. These require the team's build
environment, calibration and robot hardware.

The B5 error mapping intentionally preserves the team's existing values:
A1 no echo, A2 stop ack, A3 scan timeout, A4 no safe route, A8 clearance.
The Tribunal's A2/A3 meanings need a team decision before renumbering.
