# Subsystem 4 â€” test log

Module retirement (2026-11-10): removed tracked `imu_mock_test` executable,
`imu_height.c/.h`, the geometry-only `imu_odom_init()` helper and isolated
`testhigh_main.c/.h`. Removed height rejection counters and placeholder context;
retained diagnostics, odometry capture/pairing, mock sources and tests. No roll
threshold, detector transition, pitch measurement or public event format changes.
Height remains unavailable. Historical height audits below describe retired code.

Validation: six host test programs passed; native startup/integration tests,
including 15 movement orchestration scenarios and actual classifier diagnostics,
passed. Eight ARM C99 configurations and selected BARR-C style checks passed.
Default USB/Wi-Fi `TEST=1` firmware build passed and generated
`build_make/mtk3pico_smp1_usb_cdc_wifi_test.uf2`.
Diagnostic report is now 200 bytes (previously 300); reading/event layouts remain
128/52 bytes. Physical robot trials were not performed for this cleanup.

Roll-only cleanup (2026-11-10): removed pitch hump state/configuration,
climb/descent fallback, mode switch and pitch-stage/streak diagnostics. Real
and mock operation now share roll recognition; pitch measurement/filtering,
printing and general tilt checks remain. Mock travel is explicitly synthetic.
The robot task no longer initializes or calls the old pitch height estimator;
the unused height module and isolated test are now removed (see retirement below). Height remains unavailable.
Regression tests assert pitch-only profiles cannot generate hump events and
cover either wheel, trends, holds, failures, timing, rearming and mock recovery.
Reading/event/height-context remain 128/52/96 bytes; diagnostics now 300 bytes.
Previous milestone entries below describe historical implementations.
PASS: seven host programs, native startup/queue/odometry/motion/diagnostics,
eight ARM profiles, selected BARR-C checks and USB/Wi-Fi TEST=1 UF2 build.
Physical regression after removal remains pending; roll thresholds and movement
guards were preserved. No teammate implementation files were changed.


B4 roll motion trends (2026-11-05): in roll mode, climbing/peak/descending now
follow confirmed absolute-roll trends, not pitch. Provisional 0.5deg deadband,
two meaningful changes for direction, three stable readings for elevated peak;
two level-return readings report descent before third-reading completion.
Host tests cover both sides, directional jitter, peak/descent, direct level
return and retained pitch measurements. Pitch mode remains available.
Diagnostic roll stage bits added, report=324 bytes. Detection entry/level,
movement guards, height unavailability and packet sizes remain unchanged.
PASS: host and startup/integration regressions, eight ARM profiles, optional
pitch-mode compilation, selected BARR-C checks and USB/Wi-Fi TEST=1 UF2 build.
Hardware trend/noise validation remains pending.

B4 roll-hump detection (2026-11-05): real mode detects either-side roll excursions
after a stable moving baseline; two same-side entry readings and three level
readings complete one event and rearm. Trial thresholds 4deg enter/2deg level are
configurable in s4_runtime.h, not hardware-validated. Pitch motion remains
independent; pitch completion cannot emit or cancel a roll hump. Roll mode
bypasses the pitch height estimator and reports height unavailable.
PASS: host either-side, single-event/rearm, hysteresis, alternate settings,
spike/sign reversal rejection, missing baseline, stationary/fault/invalid data,
encoder holds, roll timeout/wrap and pitch independence regressions.
PASS: native roll diagnostic capture, startup/controller/odometry/motion tests;
eight ARM profiles, selected BARR-C checks and USB/Wi-Fi TEST=1 UF2 build.
Diagnostic report=320 bytes; reading/event/height-context stay 128/52/96 bytes.
No new source modules or teammate implementation changes. NOT RUN: installed
car with one wheel, flat-ground false-positive trials, final threshold selection.
Sampling consistency and one-wheel height accuracy are not claimed fixed.

B4 movement hold (2026-11-04): unchanged paired encoders with fresh, fault-free
forward context preserve terrain for less than 200ms; held samples cannot add
confirmations or complete a hump. Progress resumes the sequence; the 200ms bound,
global terrain timeout and genuine feedback failures cancel it. Diagnostics
split movement reset reasons; report=304 bytes. Reading/event/height-context
sizes remain 128/52/96 bytes. Host tests cover held ascent/descent, resumed
completion, 199/200ms boundary across clock wrap, and each cancellation cause.
Host/startup integration, eight ARM configurations, selected BARR-C checks and
USB/Wi-Fi TEST=1 build PASS; UF2 generated. No B2 source or height limits changed.
Hardware retest remains required.

B4 short-hump recognition (2026-11-04): real-mode level baseline, two samples
per slope, three samples returning level, forward context and encoder progress.
Entry remains +/-4 degrees. A short crest may be skipped; isolated or one-sided
spikes cannot complete a hump. Timing/impact resets discard the candidate.
Diagnostics separate slope/level confirmation counts and movement resets;
report=272 bytes, reading/event/height-context remain 128/52/96 bytes.
Host regressions, native/startup checks, eight ARM profiles, selected BARR-C
checks and USB/Wi-Fi UF2 build PASS. Physical short-hump and flat-ground runs
remain pending. Height retains its 25ms gap and other quality limits; no claim
that hardware timing or peak-height accuracy is fixed.

B4 terrain logic (2026-11-03): independent level/descent streaks; real-mode
forward context plus encoder progress required; elapsed-time pitch filtering
shared by detector and height. Overrun yields 1 ms rather than a full period;
ordinary motion console changes rate limited. Entry remains +/-4 degrees,
three samples; all height/safety limits retained. Host regression, startup/
integration, eight ARM profiles, selected BARR-C checks and USB/Wi-Fi UF2 build
PASS. Physical cadence, false detections and measured-height validation pending.

B4 acceleration priority (2026-11-02): one combined status/XYZ transaction and
configurable idle-only magnetics added. Heading is unavailable during deferred
motion frames. Host/native/ARM checks and UF2 build PASS; physical rate/height
validation remains pending. See [verification](logs/2026-11-02_acceleration_priority.txt).

B4 acceleration pairing (2026-11-02): capture moved to a driver callback just
after acceleration bytes, before math/magnetic work. Burst-end timestamps and
separate sensor-read timings added; 10ms pairing bound retained. Native/host/ARM
checks and USB/Wi-Fi UF2 build PASS; physical validation pending. See
[verification](logs/2026-11-02_acceleration_pairing.txt).

B4 odometry split (2026-11-02): separate acquisition-age, encoder-error-change
and clock-failure counts plus phase timings added without relaxing validity.
Host/native/ARM checks and USB/Wi-Fi build PASS; on-car evidence remains pending.
See [verification](logs/2026-11-02_odometry_diagnostics.txt).

B4 diagnostic summary (2026-11-02): sample-rate raw/filter extrema, reached stages,
threshold streaks and named detector/height rejection counters added. See
[verification](logs/2026-11-02_b4_diagnostics.txt) and the diagnostic section in
[MOTION_TEST.md](../../app_program/subsystem4/MOTION_TEST.md). Physical validation
of this diagnostic firmware is NOT RUN.

B4-owned motion test (2026-11-02): fourteen native orchestration cases, existing
host tests, ARM checks and USB/Wi-Fi UF2 build PASS. Physical execution is NOT RUN.
See [verification](logs/2026-11-02_b4_motion_test.txt) and
[instructions](../../app_program/subsystem4/MOTION_TEST.md).

B4 controller handoff (2026-11-01): production queue/transition tests, existing
host tests, eight startup profiles, eight ARM configurations and changed B4 ARM
objects PASS. Full firmware build remains blocked by B3's size assertion.
Hardware/controller policy remains untested. See the
[verification record](logs/2026-11-01_controller_handoff.txt) and
[controller handoff guide](../../app_program/subsystem4/VC_HANDOFF.md).

BARR-C complete scoped audit (2026-10-28): all 39 numbered sections were
reviewed for B4, and the earlier shared-name/module-name findings were fixed.
All six strict C99 host test programs and seven checker regression tests PASS.
The scanner passes across 38 C/header files and the formatter is stable.
All four ARM syntax configurations PASS (real/mock and normal/test). Normal
and test UART/no-WiFi firmware builds compile and link without warnings.
Evidence: [host](logs/2026-10-28_barr_audit_host.txt),
[checks](logs/2026-10-28_barr_audit_checks.txt),
[normal firmware](logs/2026-10-28_barr_audit_arm_normal.txt),
[test firmware](logs/2026-10-28_barr_audit_arm.txt).
The [audit checklist](../../app_program/subsystem4/CODING_STANDARD.md) records
scope and preferred-practice rationale. No new physical Pico test was run.

BARR-C PDF review validation (2026-10-28): all six C99 desktop test programs
pass, including bounded/non-finite numeric display conversion tests. The selected
style checker passes. The UART/no-WiFi ARM test firmware compiles and links
without compiler warnings. See [PDF-review host log](logs/2026-10-28_barr_pdf_host.txt)
and [PDF-review ARM build log](logs/2026-10-28_barr_pdf_arm.txt).
This is build/desktop evidence, not a new physical Pico test or certification.

BARR-C cleanup validation (2026-10-28): existing host tests pass after formatting,
explicit-width type updates and tolerance-based floating-point expectations.
See [host log](logs/2026-10-28_barr_host.txt) and
[ARM UART/no-WiFi test-build log](logs/2026-10-28_barr_arm_build.txt).
Selected style checks pass; real/mock and test/normal B4 ARM syntax checks pass.
Full coding-standard certification and a fresh on-board run are not claimed.

Further B4 cleanup (2026-10-28): Boolean/pointer/private-global naming, explicit
numeric conditions, named callbacks and function decomposition were checked.
All scoped C functions are at most 100 lines. Expanded selected-rule checks,
existing host regressions and real/mock, normal/test ARM syntax checks pass.
The UART/no-WiFi test firmware links successfully:
[cleanup host log](logs/2026-10-28_barr_cleanup_host.txt),
[cleanup ARM log](logs/2026-10-28_barr_cleanup_arm.txt).
Reading/event layout assertions remain 56/48 bytes. Hardware execution of this
refactor is not yet verified. Shared legacy naming exceptions are documented in
`app_program/subsystem4/CODING_STANDARD.md`.

**Subsystem:** Monitor (example)  Â·  **Student ID:** <student ID>  Â·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `s4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | Mock readings and null handling | FR-B4-1, simulated input only | host auto | `host/imu_mock_test.c` | All 8 mock checks pass |
| S4-U04 | Classification and explicitly unavailable height | FR-B4-5/6/7; height pending | host auto | `host/imu_analysis_test.c` | Acceleration, turn and impact classified; height invalid |
| S4-U05 | Ordered and repeated hump traversals | FR-B4-2/3, detection only | host auto | same host analysis test | One completion pulse per traversal; level alone is not a peak |
| S4-U06 | Noise and impact interruption | NFR-B4-3/4 | host auto | same host analysis test | Isolated slopes rejected; impact cancels hump |
| S4-U07 | Invalid input and configuration | B4 fault handling | host auto | same host analysis test | Invalid/NaN/null inputs rejected; interrupted hump discarded |
| S4-U08 | Sampling gaps and timestamps | NFR-B4-1, host timing validation | host auto | same host analysis test | Wrap accepted; duplicate/backward/gapped readings rejected; hump times out |
| S4-U09 | Tilted gravity and hysteresis | FR-B4-2, NFR-B4-4 | host auto | same host analysis test | Tilted rest does not imply acceleration; intermediate slope retains state |
| S4-U10 | RTOS reading buffer size | B4 internal sampling interface | target auto | `make TEST=1`, `s4_test.c` | Complete `imu_data_t` messages supported |
| S4-U11 | Sampling to analysis task data flow | B4 internal sampling interface | target auto | same target test, wait for a fresh demo completion | Valid hump completion within 9 seconds; height unavailable |
| S4-U12 | Repeating demo sequence | FR-B4-2/5/6, simulated only | host auto | `host/imu_demo_test.c` | Two hump completions across 16 simulated seconds |
| S4-U13 | Filtering, spike preservation and fault recovery | NFR-B4-3/4, B4 fault handling | host auto | `host/imu_processing_test.c` and filtered demo test | Smoothed angles/rate, unchanged impact acceleration, invalid suppression, fault/recovery pulses, timestamp checks |
| S4-U14 | RTOS fault/recovery notifications | B4 fault handling, mock only | target auto | `make TEST=1`, `s4_test.c` | Outage fault then valid recovery within nine seconds |
| S4-U15 | GY-511 driver register protocol and conversions | FR-B4-1/2/7 | host auto | `host/gy511_test.c` | H43/config verified; signed acceleration and X/Z/Y magnetics decoded; tilt and calibration checked; turn rate gated |
| S4-U16 | Real calibrated data arrival | FR-B4-1, NFR-B4-2 | target auto | Default hardware mode, `make TEST=1`, keep level/still | Valid readings reach analysis after calibration within 10s of test start |
| S4-U17 | Real startup readiness | B4 startup | target auto | Real mode, `make TEST=1`, `s4_test.c` | Accepted readings set readiness; native contract also tests timeout and mock rejection |
| S4-U18 | Real/demo isolation | B4 integration | target auto | Real mode, `make TEST=1`, `s4_test.c` | No teaching summary buffer or monitor alarm in real mode |
| S4-H01 | Startup contract and kernel failures | B4 startup | host auto | `startup/run_tests.ps1` | Eight profiles pass task-order, readiness persistence/clear, timeout, mock rejection and error checks |
| S4-H02 | Height estimates and validity | FR-B4 hump height | host auto | `host/testhigh_main.c` | Analytic 5/10/15 cm profiles, highest retention, filter/classifier integration, invalid traversals and queue gaps |
| S4-H03 | Real encoder acquisition adapter | B2/B4 pairing | host auto | `startup/testodom_main.c` | Uses B2 geometry and rejects capture races, stale sampling and clock failure |
| S4-H04 | Read-only B2 snapshots | B2/B4 context | host auto | `startup/testmoti_main.c` | Speed/travel/context, freshness, unavailable feedback, stationary confirmation and independent IMU health |
| S4-M01 | Height accuracy on the car | Real hump estimate | manual | Traverse known-height humps with calibrated encoders; compare serial and B1 output | Record expected/estimated height and error; acceptance needs the team's agreed tolerance |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone appâ€¦).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_â€¦.txt` | |
| 2026-11-01 | S4-H04 and regressions | Working tree | Codex | PASS | [B2 snapshot checks](logs/2026-11-01_b2_snapshots.txt) | B4-only reader, context-gated height and stationary checks; seven host programs, native startup/adapter/reader and eight ARM configurations; physical validation pending |
| 2026-11-01 | S4-H02/H03 and regressions | Working tree | Codex | PASS | [height verification](logs/2026-11-01_height.txt) | Seven host programs, native startup/adapter tests and eight ARM configurations; full build blocked by B3; physical S4-M01 not run |
| 2026-11-01 | S4-H01 and desktop regressions | Working tree | Codex | PASS | [startup verification](logs/2026-11-01_startup.txt) | Eight native startup profiles and eight strict ARM B4 configurations; physical/laptop tests not run; full build blocked by B3 assertion |
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

GY-511 milestone (2026-10-17): S4-U03 through S4-U09, S4-U12, S4-U13 and
S4-U15 PASS on the desktop with strict GCC warnings; see
[GY-511 host log](logs/2026-10-17_gy511_host.txt).
Normal/test UART/no-WiFi ARM ELF build evidence:
[normal](logs/2026-10-17_gy511_uart_build.txt),
[test](logs/2026-10-17_gy511_uart_test_build.txt).
S4-U16 is NOT RUN on a physical Pico. Hardware communication, burst-read STOP
behaviour, startup calibration, mounting directions and sensor accuracy remain
to be tested using [the hardware guide](../../app_program/subsystem4/HARDWARE.md).
Mock-only S4-U11/S4-U14 do not run in default hardware mode. Neither real USB/WiFi
execution nor magnetic calibration, turn-rate accuracy or hump height is claimed.
