# B4 consumption of B2 snapshots

This integration changes B4 only. `imu_motion.c` calls the existing
`motion_get_status()` declared by B2. B4 neither modifies B2 state nor calls its
command, motor, initialization or fault APIs.

The reader captures command identity, measured left/right speed in counts/s,
signed left/right travel and total travel, converted from millimetres to cm.
Travel is relative to the current B2 command; it is not continuous odometry.
Movement is mapped into B4-owned running/forward/reverse/turning/stopped/fault
flags. All values retain B2's sample timestamp.

## Freshness and failures

The snapshot and validation clock are read at IMU acquisition. A snapshot needs:

- A successful B2 read and kernel clock read.
- Age no greater than `IMU_MOTION_MAX_AGE_MS` (100 ms).
- A nonzero control interval no greater than that limit; default/unpublished
  startup state is not treated as current feedback.
- Valid command/status/direction/fault codes, finite distances and plausible
  measured speeds (at most 100,000 counts/s in either direction).

Normal 32-bit clock wrap is supported. A future timestamp is rejected. Failure
clears the entire B4 context; old speeds and a previous valid flag are never
reused. A fresh B2 fault is valid information, with `b_fault=true`, rather than
a healthy movement indication.

## Where B4 uses the result

The normalized context travels in the same queue message as the IMU and encoder
capture. The analysis task does not attach today's B2 status to an older queued
IMU sample. Paired raw encoder counts validate forward progress for roll
detection; B2 command-relative distance is not used as a replacement.
Fresh, nonfaulted forward context and timely paired counts are required.
Reverse, turning, stopped, stale or unavailable context cancels a candidate.
Brief unchanged counts hold confirmations; 200 ms without progress cancels it.
The retired pitch-height module no longer consumes these snapshots.

Fresh nonfaulted B2 physical-stop feedback can confirm `stationary` when the
IMU classifier otherwise has no motion evidence. Quiet IMU data alone remains
`unknown`. Motion context does not override hump, tilt or suspected impact
classification. Periodic console output includes B2 measured speeds or N/A.

An unavailable B2 snapshot does not mark accelerometer data invalid or prevent
the calibrated IMU startup gate from completing. This matters because B4
calibrates before B2's motor tasks start. Height remains unavailable until B2
publishes fresh feedback. B4 still raises/clears only its existing `FAULT_IMU`;
it does not assume ownership of a B2 fault or command a speed change/stop.

## Interface and memory

B4's normalized context is 36 bytes in the checked native layout. The reading
payload originally grew from 80 to 116 bytes with B2 context. Odometry diagnostic
metadata and sensor-read timings add twelve bytes, making the current reading
128 bytes; the event stays 52 bytes. Eight reading messages use 1,056 bytes of
aligned kernel storage: 384 bytes more than the original 80-byte reading,
excluding control blocks.
Queues use sizeof with a 128-byte compile-time payload bound. This is not a
measurement of whole-firmware RAM or maximum call-stack usage.

## Testing and handoff

`startup/run_tests.ps1` runs `testmoti_main` against the real B2 header and a
stub of only its read API. No B2 command/fault-writing API is linked. Tests
cover movement flags, units, unavailable data, bad data, clock errors, freshness
boundaries, clock wrap and stale-stop rejection. Adapter tests verify the raw
encoder capture survives an unavailable snapshot and is not replaced by B2's
snapshot count fields. Height tests verify context rejection and recovery.

The Vehicle Controller notification interface remains a separate integration
task. No controller or B2 files were edited for this snapshot reader. Physical
timing and snapshot values still need checking on the car.
