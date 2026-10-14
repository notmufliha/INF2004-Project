# Subsystem 4 — test log

**Subsystem:** Monitor (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | Mock readings and null handling | FR-B4-1, simulated input only | host auto | `host/imu_mock_test.c` | All 8 mock checks pass |
| S4-U04 | Classification and explicitly unavailable height | FR-B4-5/6/7; height pending | host auto | `host/imu_analysis_test.c` | Acceleration, turn and impact classified; height invalid |
| S4-U05 | Ordered and repeated hump traversals | FR-B4-2/3, detection only | host auto | same host analysis test | One completion pulse per traversal; level alone is not a peak |
| S4-U06 | Noise and impact interruption | NFR-B4-3/4 | host auto | same host analysis test | Isolated slopes rejected; impact cancels hump |
| S4-U07 | Invalid input and configuration | B4 fault handling | host auto | same host analysis test | Invalid/NaN/null inputs rejected; interrupted hump discarded |
| S4-U08 | Sampling gaps and timestamps | NFR-B4-1, host timing validation | host auto | same host analysis test | Wrap accepted; duplicate/backward/gapped readings rejected; hump times out |
| S4-U09 | Tilted gravity and hysteresis | FR-B4-2, NFR-B4-4 | host auto | same host analysis test | Tilted rest does not imply acceleration; intermediate slope retains state |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-14 | S4-U03 through S4-U09 | `c9dca65` + working changes | Codex | PASS | [host log](logs/2026-10-14_host_imu.txt) | GCC C11 with strict warnings; desktop tests only; hardware and height not validated |
