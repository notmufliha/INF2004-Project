# Buddy 4: first coding milestone

`imu_mock.c` supplies processed simulated readings. `imu_analysis.c` consumes
the same `imu_data_t` format without depending on Pico or RTOS headers.
The existing monitor task is still the template example; it does not run this
analysis yet. No hardware driver or queues have been connected in this milestone.

## Try it

From the project root, in PowerShell with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem4/host/run_tests.ps1
```

Host tests live outside `app_program` because the firmware makefile includes
every C file in that directory and its immediate subdirectories.

Example of calling the analysis from a future analysis task:

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
/* Repeat after receiving a reading from the future sampling queue. */
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
then the 10 ms sampling task and a separate analysis task with bounded queues.
Agree event ownership, overflow handling, fault reporting, and run-reset behaviour
with the Vehicle Controller owner. Reconcile proposed priorities with the kernel's
reserved priorities before starting the tasks. Validate the height method against
known humps and record real timing and detection measurements.
