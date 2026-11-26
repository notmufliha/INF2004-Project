# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | e.g. device reports a reading over WiFi every 10 s | REQ-? | manual | 1. Power on. 2. Watch the dashboard for 2 minutes. | a reading appears every 10 s ± 1 s |
| SYS-02 | | | manual | 1. … 2. … | |
| SYS-T01 | Change Requirement 1: no unacceptable timing loss in any subsystem's changed path (B1 telemetry build, B2 encoder ISR + command slot, B3 snapshot, B4 IMU pipeline, B5 scan hand-over) | CR1 Week 13 item 4 | auto (host, Cortex-M0+ emulator) | `sh testing/timing/run_timing.sh` (see `testing/timing/README.md`) | every path under 2 % CPU at its worst-case rate; results equal before and after |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-26 | SYS-T01 | `3cf1048` (before: `3e34f57`, `7bad6fb`, `34f8528`) | B1 | PASS | `../timing/logs/2026-11-26_m0_cycles.txt` | B1 5.1x faster; B2 ISR 0.84 -> 1.62 us/edge (<= 1.6 % CPU at top speed); B3 +0.2 us; B4 1.9 % faster; B5 +22 us per scan. Instruction-level emulator count, flash wait states not modelled |

## Demo observation, 2026-11-25 (recorded 2026-11-26)

The project user reported: during the demo, the robot started off with barcode reading and decoding, followed by ramp detection, obstacle detection + measurement + bypass (avoiding the obstacle without colliding into it), and perform accordingly to the instruction of the decoded barcode (M -> turn right). 
Points were gain when the robot was able to read and deocde the barcode which was shown in the dashboard, robot is able to drive over the ramp, detect + measure + bypass the obstacle, and perform its action at the junction. 
Points deduction included the failurer to measure the ramp and failure to recover the line after turning right at the junction.  
This records a qualitative hardware demo pass. Earlier pending entries describe
previous evidence; numerical hardware measurements remain unverified.

| ID | Scope | Method | Expected result |
|---|---|---|---|
| SYS-DEMO-01 | Overall integrated demo, including system participation | Manual; retrospective user report | Demonstration behaves as intended by the team |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | SYS-DEMO-01 | Demo build not recorded | Project user (reported 2026-11-26) | PASS (user-reported demo) | [Saved observation](logs/2026-11-25_demo_observation.txt) | Overall demo report; individual steps and measurements not supplied. Does not certify every subsystem case or full system requirements coverage. |
