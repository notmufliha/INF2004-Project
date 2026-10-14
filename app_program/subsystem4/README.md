# Buddy 4: mock analysis and RTOS demo

`imu_mock.c` supplies processed simulated readings. `imu_analysis.c` consumes
the same `imu_data_t` format without depending on Pico or RTOS headers.
`task_imu.c` now runs a sampling task (priority 18, target 10 ms) and a separate
analysis task (priority 19). An eight-reading message buffer copies readings
between them. Sampling uses actual RTOS timestamps and drops the newest reading
if the queue is full, without blocking. Gaps over the analysis limit reset the
terrain sequence. Delays below that limit still affect confirmation latency.
The legacy monitor is retained at priority 19 for Subsystem 3's example interface.
No real IMU, Vehicle Controller or motor interface is connected yet.

## Run on the Pico

Use the repository's configured build environment (WSL on Windows):

```sh
cd build_make
make -j8
# Optional on-board tests:
make TEST=1 -j8
```

Flash the matching USB/WiFi `.uf2` using BOOTSEL as described in the root README,
then open the USB serial port at 115200 baud. Watch for `IMU MOCK:` lines:
unknown/quiet, accelerating, turning, climbing, peak, descending, hump completed,
and suspected-impact. The cycle repeats every eight seconds. Height is explicitly
reported as unavailable. A long impact interval is intentional for this demo.
Motion lines print on transitions, not every 10 ms. The existing template LED,
producer, consumer and monitor continue to run; no motor commands are added.

With `TEST=1`, S4-U10 checks reading-buffer size and S4-U11 waits up to nine seconds
for a fresh completed hump through a test-only event buffer. A full test event
buffer drops new observations without blocking analysis. This is not a public
Vehicle Controller event queue. On-board checks must be run on the Pico; desktop
tests exercise the sequence and analysis, not the real scheduler or USB transport.
The firmware link now includes the standard math library required by analysis.

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
if (!imu_analysis_init(&analysis, &config)) {
    /* Report configuration failure. */
}
imu_mock_init();
/* Repeat with successive input readings. */
if (imu_mock_read(&data) &&
    imu_analysis_process(&analysis, &data, &event)) {
    /* Consume the valid snapshot; hump_completed is a one-reading pulse. */
}
```

## Behaviour and limits

- Three consecutive qualifying samples confirm terrain transitions by default.
  Slope entry is 8 degrees; level exit is 3 degrees. All defaults are provisional.
- Hump sequence: level, climbing, peak, descending, level. A climb may transition
  directly to descending if no level peak samples are captured.
- `hump_detected` identifies an active or just-completed sequence;
  `hump_completed` is true only on the reading completing the traversal.
- Impact takes precedence and cancels an in-progress hump. Invalid data, duplicate
  or backward timestamps, gaps over 100 ms, and a 10-second hump timeout also
  break the sequence. Unsigned timestamps support normal 32-bit wraparound.
- Output is a snapshot on every accepted reading, not an event queue notification.
  A future publisher must decide when to forward changes or periodic telemetry.
- Quiet acceleration is classified as UNKNOWN: IMU readings alone cannot
  distinguish stationary from constant-speed travel. STATIONARY remains in the
  shared enum for later use with independently confirmed motion status.
- Acceleration is in m/s^2 and includes gravity; pitch/roll and turn rate are
  supplied in degrees and degrees/second. X is forward, Y sideways, Z upward;
  positive pitch means nose-up. Driver calibration must preserve gravity.
- Magnitude-based acceleration/impact checks are provisional. Sampling must
  provide calibrated, validated readings. Filtering, gravity compensation and
  real vibration/impact trials remain necessary; these tests do not establish
  collision-detection reliability.
- Height and highest-height fields are zero with `height_valid=false`. No height
  in centimetres is inferred from pitch. Add a validated estimator before using
  these fields in telemetry or claiming the highest-hump requirement is met.

## Next integration steps

Confirm the sensor chip and actual pins. Add the hardware driver and calibration,
then replace mock acquisition in the existing sampling task with real readings.
Agree event ownership, overflow handling, fault reporting, and run-reset behaviour
with the Vehicle Controller owner. Reconcile proposed priorities with the kernel's
reserved priorities before starting the tasks. Validate the height method against
known humps and record real timing and detection measurements.
