# Subsystem 2 — test log

**Subsystem:** Buddy 2 motion control. Code: `app_program/subsystem2/`.

The original producer-example cases below are retained as legacy template
checks. They do not establish motor or encoder correctness.

## Motion controller host suite

Run `host/run_tests.ps1` with GCC, or use the direct command in the subsystem
README. The suite compiles the real command/controller implementations with
mocked RTOS, encoder and safety services.

| ID | Coverage |
|---|---|
| S2-H01 | Straight correction, steering, final-distance slowdown |
| S2-H02 | Invalid commands, bounded steering speed, pending STOP priority |
| S2-H03 | Forward/reverse MOVE, individual wheel completion, standstill |
| S2-H04 | Left/right TURN, independent wheel stopping |
| S2-H05 | DRIVE watchdog, one-wheel stall, safety STOP, no auto-resume |
| S2-H06 | Stale authorization, preemption, retained results and FIFO eviction |
| S2-H07 | Feedback history wrap, refresh continuity, invalid sample interval |
| S2-H08 | Motion deadline, wrong direction, failure to reach standstill |
| S2-H09 | Immediate STOP, STOP command ID completion, full steering |
| S2-H10 | Physical-unit APIs, NaN/infinity rejection, immutable terminal result |
| S2-H11 | Hardware test mode command selection, lost-clearance abort, result handling |
| S2-H12 | Invalid encoder transition threshold stops and reports ENCODER |
| S2-I01 | B1 binary counts/s, full signed totals, B2-only fault bits |
| S2-I02 | Legacy B1 API compatibility, error event retry/deduplication |
| S2-I03 | Vehicle Controller percentage mapping: 0/50/100 and invalid input |

| Date | Build | Tests | Result | Evidence |
|---|---|---|---|---|
| 2026-10-28 | Working tree based on `ffa85e6` | S2-H01..H10 | PASS, 1116 assertions | `logs/2026-10-28_host_motion.txt` |
| 2026-10-28 | Working tree with optional motion test task | S2-H01..H11 | PASS, 1133 assertions | `logs/2026-10-28_host_motion_harness.txt` |
| 2026-10-28 | B2/B1 compatibility working tree | S2-H01..H12, S2-I01..I03 | PASS, 1158 controller assertions and both B1 API tests | `logs/2026-10-28_b1_integration.txt` |

Additional on-target checks in `subsystem2_test.c` (not executed on hardware):
S2-U03 straight correction, S2-U04 independent wheel completion, S2-U05 invalid
steering rejection, S2-U06 status availability. Robot-mode `TEST=1` invokes
these checks; they never submit a valid movement command.

Hardware measurements remain pending. See `PID_TUNING_REPORT.md` and
`MOTION_ACCURACY_REPORT.md` for test procedures and provisional acceptance
criteria. No host result should be labelled as a physical robot result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Minimum-bit evidence (Change Requirement 1), 2026-11-25

Measured on the Cortex-M0+ target compiler; details in `DATA_REPRESENTATION.md`.

| Date | Test | Commit | Tester | Result | Command | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S2-MB01 object sizes | `after-minbits` vs `7bad6fb` | B1 | PASS | `python testing/subsystem2/host/measure_sizes.py --before 7bad6fb` | Static + pool storage 668 -> 526 B (-142); PID state 424 -> 392 B. See DATA_REPRESENTATION.md |
| 2026-11-25 | S2-H host suite | `after-minbits` | B1 | PASS | `host/run_tests.ps1` | 43,091 assertions; B1 legacy and binary integration |

## Demo observation, 2026-11-25 (recorded 2026-11-26)

The project user reported: during the demo, the robot was able to drive (using B2's motor logic) forward and perform accordingly to the decoded barcode function (M -> turn right).
This records a qualitative hardware demo pass. Earlier pending entries describe
previous evidence; numerical hardware measurements remain unverified.

| ID | Scope | Method | Expected result |
|---|---|---|---|
| S2-DEMO-01 | Overall integrated demo, including subsystem2 participation | Manual; retrospective user report | Demonstration behaves as intended by the team |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S2-DEMO-01 | Demo build not recorded | Project user (reported 2026-11-26) | PASS (user-reported demo) | [Saved observation](../system/logs/2026-11-25_demo_observation.txt) | Overall demo report; individual steps and measurements not supplied. Does not certify every subsystem case or full system requirements coverage. |
