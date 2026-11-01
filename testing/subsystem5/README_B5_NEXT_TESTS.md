# Buddy 5 — Integration completion and physical verification

**Source baseline:** the user's `INF2004-Project(4).zip`, 1 Nov 2026.

## Implemented in this B5-only update

1. **One obstacle event per scan:** `task_obstacle.c` now calls only `s5_report_profile()` for the stationary-mode scan result. The redundant direct `comm_report_obstacle()` call was removed.
2. **Safe wire values:** front distance and scan profile use the same B5 range adapter. NaN, infinity, negative values and incomplete profiles produce B1's invalid sentinel; an invalid profile always requests `AVOID_STOP`. Finite readings beyond 400 cm (distance/clearance) or 100 cm (width) are saturated.
3. **Fault ownership:** B5 raises and clears only `FAULT_NO_ECHO`. An invalid measurement cannot clear the fault or authorize forward movement.
4. **B5 decoder names:** all nine existing error numbers `0xA1..0xA9` have clear `B5`-prefixed descriptions in `tools/s1_wire.py`; no code numbers were changed.
5. **Tests:** added malformed-profile and numeric-boundary cases, an explicit single-event regression assertion, and a laptop wire decode test. B5 shell scripts use LF line endings so they run in Git Bash/Linux.

## Run the host tests

From the repository root (Git Bash or a Linux terminal):

```bash
bash testing/subsystem5/run_host_tests.sh
bash testing/subsystem5/run_packed_tests.sh
python3 testing/subsystem5/test_b1_contract.py
python3 testing/subsystem5/test_b5_error_contract.py
python3 testing/subsystem5/test_wire_decode.py
python3 -m py_compile tools/s1_wire.py
```

The first two require a C11 compiler. The adapter test also requires `cc`. These tests are **software-only**; they do not confirm physical performance.

## Hardware checks — do not mark PASS until performed

**Connections:** HC-SR04P TRIG GP16, ECHO GP17, VCC 3V3, GND GND; servo signal GP12, with suitable separate servo power and common ground as required by the servo. Use only a 3.3V-safe ECHO sensor. Verify the board and servo power before energizing the wheels.

**Bench conditions:** keep wheels raised for initial tests; clear space around the servo; retain `S5_MOTION_BRIDGE_ENABLED=0` and `ROBOT_ENABLE_BYPASS=0`. The current robot configuration may request guarded forward movement; disable drive or keep wheels raised for initial sensor-only tests. Do not turn on bypass until the team has implemented and validated the B2/B3 bridge and chassis clearance.

1. **Distance accuracy:** place a flat target at measured 10, 20, 30, 50 and 100 cm; record at least five readings per position. Note absolute error and whether any reading was invalid.
2. **Servo repeatability:** observe the 70°, 80°, 90°, 100°, 110° scan sequence; check for physical interference and stable echo readings after the configured settling delay. The code does **not** have servo position feedback: waiting 220 ms is not proof of physical settling.
3. **No-echo fault:** unplug/obstruct the ultrasonic ECHO safely with motors disabled; confirm `FAULT_NO_ECHO` is raised and `0xA1` is reported once per failure episode, then cleared after valid readings return.
4. **Obstacle event:** present a detectable obstacle, verify **one** event per completed scan, correct fields, and STOP when neither side is safe.
5. **Stop response:** with wheels raised, verify B2 actually stops on a close or invalid echo. Measure time from obstacle presentation to stopped motor PWM; do not infer response latency from source code alone.
6. **Decoder:** confirm the laptop prints the nine B5 names, and `NO_ECHO` appears only when B5 raises that bit.

Record results in `testing/subsystem5/hardware_results_template.csv`. The template contains **no fabricated measurements**.

## Integration work requiring other buddies

- The `s5_bridge.c` functions remain deliberately fail-closed placeholders. B2 must provide verified motion completion and stopped acknowledgements; B3 must provide line reacquisition; the Vehicle Controller must arbitrate exclusive motor ownership. **Do not enable `S5_MOTION_BRIDGE_ENABLED` by changing a flag alone.**
- B1/integration lead owns the Wi-Fi task priority change; this B5 patch does not touch it.
- The original Tribunal A2/A3 descriptions differ from the current team mapping. Current mapping is retained; obtain team approval before changing code numbers or documentation.
- Full ARM firmware build and on-robot testing still require the Pico toolchain and the physical robot.
