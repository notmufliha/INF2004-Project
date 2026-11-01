# Buddy 4: GY-511 hardware input and mock tests

B4 now consumes B2's existing motion snapshots without editing B2. See
[B2_SNAPSHOTS.md](B2_SNAPSHOTS.md) for freshness checks, measured speed/travel,
height gating and tests.

## B1 integration after the merge

B4 reports motion and pitch through `comm_set_imu()`, raises/clears `FAULT_IMU`
on fault/recovery, and reports completed humps through `comm_report_hump()`.
Real paired encoder/pitch readings now feed the [height estimator](HEIGHT.md).
Rejected or missing odometry keeps height explicitly unavailable.
Telemetry conversions clamp to the signed 16-bit B1 range before narrowing.

With `ROBOT_MODE=1`, startup initializes B1 and B2, then starts B4 before starting
motor tasks. Keep the car level and still. B4's normal calibration and recovery
must produce an accepted reading within `ROBOT_IMU_STARTUP_TIMEOUT_MS` (10 seconds
by default). Timeout or B4 startup failure enters the existing disabled-robot
failure path. Mock input cannot satisfy the real-robot readiness gate.

Real mode creates only the sampling/analysis tasks and their queues/ready flag.
It does not create the teaching summary buffer, alarm or legacy monitor, so B4
does not access uninitialized B3 statistics or pause B1's LED. `ROBOT_MODE=0`
retains those demo resources. The monitor clears only its own flag bits.

Readiness is a startup gate, not a motor safety controller. A later sensor fault
clears readiness and updates B1's shared fault mask; it does not directly stop
the motors. Runtime fault-to-motion policy remains with the vehicle controller.

Run the native startup contract tests with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File testing/subsystem4/startup/run_tests.ps1
```

These execute the production startup module against a kernel stub in eight
robot/demo, real/mock and test/normal combinations. They exercise readiness,
timeouts, simulated-input rejection, task order and injected kernel failures.
The stub preserves external kernel/task ABI names; it is not firmware code.

For the end-to-end hardware check, boot level/still and confirm the waiting
message is followed by calibration/recovery and the robot-started message.
Verify laptop pitch/motion against serial output, then verify IMU fault/recovery
with the drive disabled. Completed humps must still show height as unavailable.
These physical/laptop checks have not been performed by the desktop tests.

See [CODING_STANDARD.md](CODING_STANDARD.md) for B4's BARR-C cleanup scope,
rule-by-rule audit, automated checks, formatter profile and platform rationale.

**Firmware now defaults to the real GY-511 input.** See [HARDWARE.md](HARDWARE.md)
for wiring, startup calibration, serial output and the first hardware test.
There is no automatic fallback to simulated data. The driver targets LSM303DLHC
and checks the magnetic H43 ID plus configuration readback before use.

The mock demo described below is available only by setting
`S4_IMU_USE_MOCK` to `1` in `s4_runtime.h`. With the default `0`, TEST=1 runs
S4-U16 (real calibrated readings) instead of mock-only S4-U11/S4-U14.
Mock height remains unavailable. Heading is an uncalibrated magnetic estimate by
default; turn rate remains unavailable until magnetic calibration is configured.
The sections below describe the retained mock demo and processing policy.

`imu_mock.c` supplies processed simulated readings. `imu_analysis.c` consumes
the same `imu_data_t` format without depending on Pico or RTOS headers.
`task_imu.c` now runs a sampling task (priority 18, target 10 ms) and a separate
analysis task (priority 19). An eight-reading message buffer copies readings
between them. Sampling uses actual RTOS timestamps and drops the newest reading
if the queue is full, without blocking. Gaps over the analysis limit reset the
terrain sequence. Delays below that limit still affect confirmation latency.
The legacy monitor runs at priority 19 only in the teaching/demo startup.
`imu_processing.c` validates and filters each received reading before analysis.
Real IMU acquisition is enabled by default; Vehicle Controller and motor
interfaces are not connected yet.

## Run on the Pico

Use the repository's configured build environment (WSL on Windows):

```sh
cd build_make
make -j8
# Optional on-board tests:
make TEST=1 -j8
```

Flash the matching USB/WiFi `.uf2` using BOOTSEL as described in the root README,
then open the USB serial port at 115200 baud. In mock mode, watch for `[B4] Motion=` lines:
unknown/quiet, accelerating, turning, climbing, peak, descending, hump completed,
and suspected-impact. The cycle repeats every eight seconds. Height is explicitly
reported as unavailable. A long impact interval is intentional for this demo.
At 6.5 seconds the mock fails for 200 ms, then provides noisy-level readings.
Look for `sensor fault` followed by `sensor recovered` before the impact phase.
Motion lines print on transitions, not every 10 ms. The existing template LED,
producer, consumer and monitor continue to run; no motor commands are added.

With `TEST=1`, S4-U10 checks reading-buffer size and S4-U11 waits up to nine seconds
for a fresh completed hump through a test-only event buffer. A full test event
buffer drops new observations without blocking analysis. This is not a public
Vehicle Controller event queue. On-board checks must be run on the Pico; desktop
tests exercise the sequence and analysis, not the real scheduler or USB transport.
The firmware link now includes the standard math library required by analysis.
S4-U14 checks the fault/recovery report sequence, with up to nine additional
seconds of waiting in the test build.

## Filtering and fault policy

Defaults in `imu_processing_default_config()` are provisional: smoothing alpha
0.25, fault after three consecutive invalid attempts, recovery after five
consecutive valid readings, and maximum valid-reading gap of 100 ms.
Pitch/roll use angle-aware exponential smoothing and turn rate uses exponential
smoothing. Alpha 1 disables smoothing. Acceleration passes through unchanged so
an impact spike can reach the analysis immediately; acceleration noise rejection
and physical threshold tuning still require hardware experiments.

Invalid/nonfinite readings and angles outside +/-180 degrees produce invalid
output without stale measurements. The first invalid reading resets the filter
and hump sequence, even before the fault threshold. A gap above 100 ms or a
duplicate/backward timestamp starts a fault immediately. A failed recovery
attempt restarts the recovery count. Recovery seeds the filter from a fresh
reading. The analysis task uses a finite receive timeout so absent messages also
enter the fault path. These mechanisms cannot monitor a stalled analysis task.

Fault/recovery reports are USB/console messages and test-tap notifications at
present, not Vehicle Controller commands. A future driver must report acquisition
failure as an invalid reading; sensor reinitialisation and calibration are not
implemented by this hardware-independent processing module.

## Try it

From the project root, in PowerShell with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem4/host/run_tests.ps1
```

Host tests live outside `app_program` because the firmware makefile includes
every C file in that directory and its immediate subdirectories.

Example of calling the analysis without RTOS:

```c
imu_analysis_t analysis;
imu_analysis_config_t config;
imu_data_t data;
imu_event_t event;

imu_analysis_default_config(&config);
if (!imu_analysis_init(&analysis, &config))
{
    /* Report configuration failure. */
}
imu_mock_init();
/* Repeat with successive input readings. */
if ((imu_mock_read(&data)) &&
    (imu_analysis_process(&analysis, &data, &event)))
{
    /* Consume the b_valid snapshot; b_hump_completed is a one-reading pulse. */
}
```

## Behaviour and limits

- Three consecutive qualifying samples confirm terrain transitions by default.
  Slope entry is 8 degrees; level exit is 3 degrees. All defaults are provisional.
- Hump sequence: level, climbing, peak, descending, level. A climb may transition
  directly to descending if no level peak samples are captured.
- `b_hump_detected` identifies an active or just-completed sequence;
  `b_hump_completed` is true only on the reading completing the traversal.
- Impact takes precedence and cancels an in-progress hump. Invalid data, duplicate
  or backward timestamps, gaps over 100 ms, and a 10-second hump timeout also
  break the sequence. Unsigned timestamps support normal 32-bit wraparound.
- Output is a snapshot on every accepted reading, not an event queue notification.
  A future publisher must decide when to forward changes or periodic telemetry.
- Quiet IMU data alone remains UNKNOWN. Fresh nonfaulted B2 physical-stop
  feedback can now confirm STATIONARY; stale or unavailable feedback cannot.
- Acceleration is in m/s^2 and includes gravity; pitch/roll and turn rate are
  supplied in degrees and degrees/second. X is forward, Y sideways, Z upward;
  positive pitch means nose-up. Driver calibration must preserve gravity.
- Magnitude-based acceleration/impact checks are provisional. Sampling must
  provide calibrated readings. Gravity compensation and
  real vibration/impact trials remain necessary; these tests do not establish
  collision-detection reliability.
- Height uses real encoder travel and raw pitch after a confirmed full traversal.
  It is a consistency-checked estimate, not a direct measurement. Highest accepted
  height persists until reboot/context initialization. See [HEIGHT.md](HEIGHT.md)
  for rejection rules, preliminary calibration and physical-validation limits.

## Next integration steps

Verify communication with the installed sensor, actual pins, mounting map and
startup calibration using HARDWARE.md. Collect real-car trials to tune thresholds.
Agree event ownership, overflow handling, fault reporting, and run-reset behaviour
with the Vehicle Controller owner. Reconcile proposed priorities with the kernel's
reserved priorities before starting the tasks. Validate the height method against
known humps and record real timing and detection measurements.
