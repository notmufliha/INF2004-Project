# Real-data hump-height estimator

The B4-only [B2 snapshot reader](B2_SNAPSHOTS.md) now supplies additional movement
context. Production estimates require fresh forward/stopped, nonfaulted context;
the paired raw encoder counts remain the distance source.

The sampling task reads B2 encoder counts alongside the real GY-511 sample.
`imu_odom.c` uses the wheel diameter and separate left/right counts per revolution
from `subsystem2/encoder_control.h`; it does not invent wheel motion or maintain
a second geometry calibration. Capture spans at most one 10 ms acquisition
period. Error counters are read before and after the paired count snapshot.

Each acquisition attempt receives a sequence number, including attempts that
produce no fresh reading or are dropped by a full queue. Pitch, counts, error
counters, timestamp and sequence travel in the same copied message. The analysis
task never reads live encoders for an older queued IMU frame.

After filtering/terrain classification, `imu_height.c` uses the original raw
pitch with its paired counts to avoid angle-filter delay. It integrates:

`height_change_cm = mean_wheel_travel_cm * mean(sin(previous_pitch), sin(pitch))`

Pitch is converted from degrees to radians. The candidate starts above 3 degrees,
before terrain confirmation, so confirmation does not discard early ascent.
Only a confirmed completed traversal with actual descent can produce a valid
estimate. The maximum vertical rise is the estimated hump peak.

## Consistency limits

Defaults are provisional and must be validated on the mounted car:

- Positive calibrated distance per count for both wheels.
- Consecutive acquisition sequence numbers; no queue-drop interpolation.
- Sample intervals at most 25 ms, including normal timestamp wrap.
- No new encoder transition errors, reverse motion or implausible count jumps.
- Forward speed at most 100 cm/s; pitch at most 45 degrees and roll at most 10.
- Acceleration magnitude within 1.5 m/s^2 of standard gravity.
- Per-step wheel disagreement at most max(0.1 cm, 25% of mean travel).
- Completed-traversal wheel disagreement at most max(0.2 cm, 10% of travel).
- Traversal travel at most 200 cm and a positive peak of at least 0.2 cm.
- Return-to-level residual at most min(1 cm, max(0.05 cm, 25% of peak)).

Errors invalidate the entire current traversal until a valid level baseline is
seen again. Impact, nonfinite values, missing odometry or a reset terrain sequence
also cancel the estimate. A stationary manual tilt cannot generate a height.
The highest accepted hump survives rejected traversals/faults and resets only
when the estimator is initialized again, normally at reboot/task startup.

`b_height_valid` means these consistency checks passed. It is not a certificate
of measurement accuracy. Wheel slip, wrong wheel calibration and driving
acceleration can still bias the result. Accelerometer-only pitch assumes nearly
static gravity. Chassis pitch must approximate the road slope; short humps
relative to wheelbase and suspension movement can violate that assumption.
Compare estimates with known-height humps at low speed before claiming accuracy.

## Output and integration

Console output labels the result as an estimate, in whole millimetres. B1's
existing hump event receives height and session maximum in millimetres, with its
existing validity flag. No B1 wire protocol change is required. Internally,
`b_highest_hump_valid` distinguishes an existing maximum from an unknown one;
the console can retain that maximum even when the newest hump is unavailable.
B1's single validity flag still applies to the whole event.

The scripted mock supplies no encoder metadata and continues to report height
as unavailable. Synthetic distances exist only in the desktop test fixtures.

With B2 context, native reading/event sizes are 116/52 bytes. Kernel queues use
sizeof with a 128-byte compile-time bound. B2 context adds 288 queue bytes beyond
the original 80-byte height-reading payload. This excludes stack/context growth
and is not a total RAM measurement. Historical height-only sizes are preserved
in the earlier test record; see B2_SNAPSHOTS.md for the current integration.

## Verification

Run the existing host runner for seven test programs, including `testhigh_main`.
The new tests check analytic 5/10/15 cm profiles, the actual filter/terrain pipeline,
highest-height retention, invalid traversals, clock wrap and sequence gaps.
`startup/run_tests.ps1` additionally exercises the actual encoder adapter with
deterministic kernel/B2 stubs, including capture races, latency and clock errors.

Physical calibration and known-height validation have not been performed by
these desktop tests. The complete firmware build also still depends on resolving
the separate B3 state-size assertion.
