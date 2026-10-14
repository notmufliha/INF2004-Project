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
| S4-U10 | RTOS reading buffer size | B4 internal sampling interface | target auto | `make TEST=1`, `subsystem4_test.c` | Complete `imu_data_t` messages supported |
| S4-U11 | Sampling to analysis task data flow | B4 internal sampling interface | target auto | same target test, wait for a fresh demo completion | Valid hump completion within 9 seconds; height unavailable |
| S4-U12 | Repeating demo sequence | FR-B4-2/5/6, simulated only | host auto | `host/imu_demo_test.c` | Two hump completions across 16 simulated seconds |
| S4-U13 | Filtering, spike preservation and fault recovery | NFR-B4-3/4, B4 fault handling | host auto | `host/imu_processing_test.c` and filtered demo test | Smoothed angles/rate, unchanged impact acceleration, invalid suppression, fault/recovery pulses, timestamp checks |
| S4-U14 | RTOS fault/recovery notifications | B4 fault handling, mock only | target auto | `make TEST=1`, `subsystem4_test.c` | Outage fault then valid recovery within nine seconds |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-14 | S4-U03 through S4-U09 | `c9dca65` + working changes | Codex | PASS | [host log](logs/2026-10-14_host_imu.txt) | GCC C11 with strict warnings; desktop tests only; hardware and height not validated |
| 2026-10-14 | S4-U03 through S4-U09, S4-U12 | Working changes, RTOS demo | Codex | PASS | [demo host log](logs/2026-10-14_host_imu_demo.txt) | Strict desktop compilation; real RTOS execution not tested |
| 2026-10-14 | S4-U10, S4-U11 | Working changes, RTOS demo | Codex | NOT RUN | [test firmware build](logs/2026-10-14_uart_test_build.txt) | ARM UART/no-WiFi test image compiles and links; requires Pico execution |

Firmware validation: normal and test ARM ELF builds both link successfully with
`CONSOLE=uart WIFI=none E2U=`. [Normal build log](logs/2026-10-14_uart_build.txt).
USB/WiFi profile and UF2 conversion were not verified in this environment.
These UART artifacts are compile checks, not the recommended USB demo image.
Do not use the UART console profile with the proposed encoders wired to GP0/GP1.

Processing milestone (2026-10-14, working changes): S4-U03 through S4-U09,
S4-U12 and S4-U13 PASS in strict GCC host tests; see
[processing test log](logs/2026-10-14_host_imu_processing.txt).
Normal and test ARM ELF builds compile and link:
[normal build](logs/2026-10-14_processing_uart_build.txt),
[test build](logs/2026-10-14_processing_uart_test_build.txt).
S4-U10, S4-U11 and S4-U14 remain NOT RUN on hardware. USB/WiFi builds, sensor
performance and actual scheduler timing remain unverified.
