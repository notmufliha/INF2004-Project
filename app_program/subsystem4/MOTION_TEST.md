# B4-owned physical movement test

This optional integration test requests one forward MOVE through B2's existing
`motion_move_cm()` API. B2 still owns motor PWM, encoder feedback and command
execution. B5 must grant and maintain its existing clearance permission. No B2
or B5 implementation is changed.

## Roll-based one-wheel detection (current real mode)

`s4_runtime.h` exposes provisional settings for later flat-ground calibration:

```c
#define S4_ROLL_HUMP_ENTER_DEG 4.0f
#define S4_ROLL_HUMP_LEVEL_DEG 2.0f
#define S4_ROLL_HUMP_TREND_DEG 0.5f
```

These are trial values, not validated limits for the installed car. The level
threshold must be nonnegative and below the entry threshold; invalid settings
fail analysis initialization. Rebuild and reflash after changing them.

With fresh forward encoder progress, three readings within the roll-level band
arm the detector. Two readings above the entry magnitude on the same side start
a candidate. Three readings back within the level band report one completed
roll hump and rearm it. A right-wheel or left-wheel excursion is supported;
positive roll followed by negative roll is not required. Alternating spikes or
large sign reversal before returning level are rejected. The normal movement,
impact, sensor validity and timeout guards still apply; short encoder holds do
not add confirmations.

In roll mode, climbing/peak/descending labels now follow absolute-roll trends.
Candidate entry reports climbing. Two confirmed changes away from level report
climbing; two toward level report descending, for either wheel. Changes below
the provisional trend deadband accumulate relative to the last meaningful
reference, rather than switching the motion state on every noisy difference.
Three stable elevated readings can report peak; a plateau during descent keeps
descending. A short direct drop to level reports descent on the second level
reading, then completes on the third. Pitch is still measured and printed, but
does not override roll motion labels. There is no pitch-based hump or motion-classification fallback.

Completed roll crossings estimate height using peak relative roll and 110 mm
tyre spacing. See [HEIGHT.md](HEIGHT.md) for assumptions and validation. The
retired pitch-height module remains removed.

After flashing, expect `Hump mode=ROLL` at startup. Repeat with either wheel
crossing the hump and on flat ground with ordinary starts/stops. Each full roll
excursion should produce one hump report; flat-ground runs should not. Send the
final `[B4-DIAG]` summary including roll candidates and completions. Compare
repeat runs before selecting final thresholds. Physical validation is pending.

The requested test is enabled in `s4_runtime.h`:

```c
#define S4_MOTION_TEST_ENABLE 1
#define S4_MOTION_TEST_DISTANCE_CM 50U
#define S4_MOTION_TEST_SPEED_CPS 500
```

Distance is centimetres (permitted configuration: 1..150); speed is encoder
counts per second (1..1000), not centimetres per second. Set the enable switch
to 0 before ordinary controller operation. `robot_config.h` must have
`ROBOT_MOTION_TEST=0`; the new test refuses to compile with B2's automatic or
raw wheels-up test enabled. Use this in isolation from other command producers.

Only real-sensor `ROBOT_MODE=1`, `APP_TEST=1` builds run the test. Normal,
mock and teaching-demo builds do not drive through this module. It runs
synchronously after B4's existing checks in `s4_test_run()`, when B2/B5 tasks
are already started. It does not create another RTOS task or kernel queue.

Build the USB firmware from `build_make` using the team's verified toolchain:

```powershell
$env:PICO_SDK_PATH = "C:/Users/engli/.pico-sdk/sdk/2.3.1"
make TEST=1 -j8
```

Flash the new `mtk3pico_smp1_usb_cdc_wifi_test.uf2`. Keep the car level/still
through IMU calibration. Arrange a clear route longer than 50 cm with stopping
space and account for the USB cable. The car can move automatically: after
existing startup checks, the test prints `[B4-TEST] Armed` and waits five seconds.
It then waits up to 30 seconds for fresh valid IMU observations, fresh healthy B2
standstill feedback, and B5 clearance. It submits one command and never retries
movement automatically. Restart to repeat.

During movement the test aborts on lost B5 permission, B2 faults or stale
feedback, an IMU readiness fault, missing recent B4 observations, a reported
suspected impact, roll beyond +/-30 degrees, command replacement or timeout.
The motion phase has a 65-second outer bound; B2's own deadline may stop it
sooner. Every outcome requests a safety STOP through B2. This retains B5's
guards; it is not an independent collision protection system. IMU observations
use the existing best-effort test tap, not a guaranteed safety event channel.

Expected messages include:

```text
[B4-TEST] Armed: forward 50 cm at 500 counts/s; ...
[B4-TEST] Waiting for B5 clearance, B2 standstill and fresh B4 readings ...
[B4-TEST] START id=...
[B4-TEST] MOTION PASS error=0 humps=1 valid_heights=1; STOP requested. ...
```

`MOTION PASS` means B2 reported successful command completion and physical
standstill. It does not establish hump detection. `humps=0` is expected on flat
ground. Completed roll events report geometric estimates; `valid_heights` counts
accepted estimates, not confirmed measurement accuracy.
Full pitch/roll, heading and terrain reports continue separately. Place the
hump early enough to return to level before the drive ends. This test does not
calibrate magnetic heading or provide gyroscope data.

The test consumes only `gh_imu_test_events` after the earlier on-board checks
finish. It does not take messages from acquisition or the controller queue.
Do not add a concurrent consumer of this test tap.

Desktop verification: `testing/subsystem4/startup/run_tests.ps1` runs the actual
orchestrator with deterministic kernel/B2/B5 stubs for success, invalid height,
permission loss, sensor fault, stale IMU/B2 data, command rejection/replacement,
impact, excessive roll, clock failure, cancellation and timeout. These tests do
not establish physical travel accuracy or real-car hazard detection.

## Run diagnostic summary

The analysis task records every processed frame from just before command
submission until the test requests STOP and freezes the report. Collection is
not limited to the once-per-second measurement prints. It starts after the
clearance/standstill preflight; a failure before that prints `No motion capture
started`. A rejected command can produce an empty capture.

At the end, `[B4-DIAG]` lines report:

- Valid raw/filtered sample counts and separate pitch minima/maxima.
- Maximum absolute raw and filtered roll.
- Whether climb, crest and descent were reached, plus completed/accepted counts.
- Roll candidate count, entry/level thresholds and confirmation counts.
- Named nonzero detector reset counts.
  Movement cancellation is split into invalid B2 context, unavailable odometry,
  stale context, B2 fault, reverse context/counts, turning, stopped, not-forward
  and 200 ms without encoder progress. Counts are failed checks per sample,
  not necessarily interruptions of active hump candidates.

Angles use **integer tenths of a degree**: `max=35` means 3.5 degrees;
`max=120` means 12 degrees. `N/A` means no valid angles were captured. An empty
capture is explicitly identified and is not evidence that the car stayed level.
The crest may legitimately be skipped when climb transitions directly to descent.

Reset counts are per-frame failed checks, not separate humps or hardware
faults. They include checks on level ground, not only active candidates.
Interpret them with the stage and angle ranges.

Collection uses a fixed 200-byte report plus lock/bookkeeping, without raw-data
history. A kernel FastLock protects updates and the final copy. Printing occurs
outside the lock after capture ends. Normal `APP_TEST=0` builds do not allocate
the collector or its lock. Readings remain 128 bytes and events remain 52 bytes;
public controller and network formats are unchanged.

The native diagnostic harness exercises the actual roll classifier, resets,
capture lifecycle, invalid angles and raw/filter range separation. It verifies
lock pairing and unlocked printing; it does not simulate SMP scheduling.

### Odometry failure split and phase timing

The summary now separates invalid IMU input, changes in the left/right encoder
error counters during capture, acceleration/encoder window above 10 ms, failure of the
final clock read, and failure of an extra diagnostic clock probe. Conditions can
coexist, so their counts need not sum to the invalid-frame total. B2 encoder
snapshot APIs return no error code; these diagnostics cannot infer a read error
or wiring failure merely from their return.

The 10 ms bound and stable-error-counter checks remain. The current real path
starts the bound immediately before the combined seven-byte status/XYZ burst and reads
encoder counts in a synchronous callback immediately afterward. It excludes the
later angle calculations/magnetic work. Slow bursts and
preemption during that bounded span still invalidate the pair. Missing source
clock brackets invalidate pairing; failed optional timing probes alone do not.
The stored sample timestamp is acceleration burst completion and is never
overwritten after magnetic reads. Magnetic heading history uses its own read
completion time. Exact acceleration conversion time is not known without a
hardware data-ready timestamp; burst completion is a software approximation.

Timing maxima, in milliseconds, are printed for:

- `accel_encoder_window`: acceleration burst start to final pairing clock read.
- `before_encoder_in_window`: burst and dispatch time before the encoder probe,
  derived per frame from the measured phases.
- `encoder_capture`: the three before/count/after encoder snapshots.
- `B2_context`: the optional B2 snapshot and final clock-read interval.
- `acceleration_I2C`: one status/XYZ burst, excluding math/pairing.
- `magnetic_I2C`: optional magnetic status plus burst reads, excluding math.

Each maximum includes its number of available measurements; `N/A` is unknown,
whereas 0 ms means shorter than clock resolution. These maxima may come from
different frames and must not be subtracted from one another. Timing fields
saturate at 65,535 ms (a lower bound for a larger interval); saturated fields are
excluded from the derived pre-encoder measurement. The clock
probes have small overhead, so these are diagnostic timings, not a separate
sensor benchmark. Source collection remains outside B2/B5 code.

Pairing does not guarantee the sampler achieves 100 Hz. Slow optional magnetic
work can still delay the next acceleration sample. These timings identify such
remaining rate problems rather than hiding them with larger limits. Pitch
filtering, slope thresholds, roll limits and B5 safety behavior are unchanged.

### Acceleration-priority acquisition

B4 now reads contiguous accelerometer status and XYZ in a single seven-byte
transaction, using subaddress 0xA7. ST documents address auto-increment and the
contiguous 0x27..0x2D map in the
[LSM303DLHC datasheet, sections 5.1.2 and 6](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf).
This removes one device request per acceleration frame. No-fresh-data status,
clipping, read failures and the 10 ms pairing limit are still checked.

`S4_IMU_MAG_IDLE_ONLY=1` in `s4_runtime.h` enables the real-sensor priority policy.
Magnetic reads require fresh B2 feedback indicating physically stopped,
non-running, non-faulted motion and zero wheel speeds, sustained for one second.
Startup with no B2 context, movement and faults all defer magnetic work. The
same sampling task remains the sole I2C owner: no competing sensor task or
unsynchronized bus access is introduced.

**While deferred, magnetic field, heading and turn rate are unavailable**, not
cached as valid or replaced with fabricated zero readings. A fresh data frame
clears these flags; heading-derivative history is reset, so the first resumed
heading cannot produce a false turn rate across the pause. Normal heading
readings resume at confirmed standstill. An in-flight read or one authorized
by a snapshot just before motion starts can still appear at a transition.

The end-of-run summary prints `Magnetic reads deferred=... frames`. Moving runs
should normally show fewer magnetic timing samples and fewer sample-gap rejects;
this must be measured on the car. `magnetic_I2C=N/A` with deferred frames is
expected. Set `S4_IMU_MAG_IDLE_ONLY=0`, rebuild and reflash to restore full
magnetic acquisition, accepting that its blocking reads may delay acceleration.
The portable driver retains full acquisition by default; B4's real startup
explicitly enables the configured policy.

This is a prioritization tradeoff, not simultaneous high-rate heading and height
sampling. It does not guarantee 100 Hz: kernel I2C latency, scheduling, angle
math and no-fresh-data attempts can still affect cadence. Diagnostic timing,
sequence, gap and gravity checks remain active. Filtering and the historical
hump threshold are unchanged.
