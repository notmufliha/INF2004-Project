# Buddy 2 motion control

Buddy 2 owns motor PWM and encoder feedback. The 10 ms control task executes
DRIVE, distance-limited MOVE, in-place TURN, and STOP. No task automatically
starts driving. Current robot configuration keeps drive and bypass disabled.

## Guarded hardware motion test

The optional `motion_self_test.c` task submits actual motor commands through
the production API. It is disabled by default. In `robot_config.h`, set:

```c
#define ROBOT_ENABLE_DRIVE 1
#define ROBOT_MOTION_TEST 1
#define ROBOT_WHEELS_UP_MOTOR_TEST 0
#define ROBOT_ENABLE_BYPASS 0
```

Choose one mode, rebuild, and flash the newly generated UF2:

| Mode | One-shot test |
|---:|---|
| 0 | Disabled |
| 1 | Straight forward 10 cm |
| 2 | Straight reverse 10 cm |
| 3 | Straight continuous DRIVE for 1 second, refreshed every 20 ms, then STOP |
| 4 | Turn left 90 degrees in place |
| 5 | Turn right 90 degrees in place |
| 6 | Run modes 1..5 in that order, with 3-second pauses |

Start with individual modes. Firmware waits 5 seconds after task startup, then
up to 30 seconds for Buddy 5 clearance and encoder standstill. Motor speed
defaults to 1000 reference counts/s (approximately 81.7 mm/s with the current
preliminary calibration). Each movement is tracked by its command ID. MOVE and
TURN must complete and reach standstill; mode 3 explicitly tests STOP while
DRIVE is active. The sequence runs only once per boot.

Expect `[B2-TEST] Armed`, `waiting for fresh B5 clearance`, `START`, result
lines, `PASS`, and finally `COMPLETE`. The result line reports numeric status,
fault, measured wheel travel in mm, and encoder standstill. PASS establishes
encoder/controller completion; measure real body travel and turn angle too.
On rejection, timeout, cancellation, lost clearance or fault, the test requests
STOP and prints `ABORTED`. There is no automatic restart after an abort. Do not
disable the safety gate to get past a clearance failure.

First verify wheel directions with the wheels raised. Floor tests need a clear
front, rear and turning area with motor power accessible. The forward ultrasonic
sensor cannot verify rear or side clearance. Turns/distances are uncalibrated.
Restore `ROBOT_MOTION_TEST=0` and `ROBOT_ENABLE_DRIVE=0` after testing.

## Command API

Include `motion_command.h` and `b2_motor.h`. All calls are task-context APIs.
Zero-initialize command structures, including the new steering/timeout fields.

```c
motion_command_t command = {0};
uint32_t id;
command.type = MOTION_CMD_DRIVE;
command.direction = MOTION_DIR_FORWARD;
command.target_speed_cps = 1000;
command.steering_permille = 250;
command.safety_generation = b2_safety_stop_generation();
ER error = motion_send_command_with_id(&command, &id);
```

- Speed uses **left-wheel reference counts/s**, 1..5000. The right-wheel
  target is converted using its own encoder resolution. At the preliminary
  geometry, 1000 counts/s is approximately 81.7 mm/s. Confirm these encoder
  calibrations before treating the speed as physically accurate.
- DRIVE steering is -1000..1000: zero is straight; positive increases the left
  wheel speed and reduces the right. Full steering stops the inner wheel.
  Reverse uses the same wheel-speed convention, so its yaw direction reverses.
  Commands whose steering would exceed 5000 reference counts/s are rejected.
- Navigation must refresh DRIVE, for example every **20 ms**, comfortably
  inside the 100 ms watchdog. Every accepted refresh gets a new ID. Check
  errors and retry later if the single pending slot is busy.
- `motion_move_cm(10.0f, 1000, MOTION_DIR_FORWARD, &id)` and the reverse
  equivalent provide distance-limited travel. Raw MOVE distance uses positive
  left-wheel reference counts. Steering must be zero for MOVE and TURN.
- `motion_turn_degrees(90, 1000, MOTION_TURN_LEFT, &id)` turns in place.
- MOVE/TURN default deadline is twice ideal travel time plus 2000 ms, capped
  at 60000 ms. `timeout_ms` can override this to 100..60000 ms. Long moves
  must be split if the 60 s limit is insufficient.
- STOP immediately revokes authorization and zeros PWM, even if submission
  fails because the queue lock is occupied. Retry to obtain a tracked STOP ID.
  STOP replaces a pending movement and records its cancellation. A pending
  STOP cannot be overwritten. Buddy 5 must subsequently confirm clearance.

Every movement still needs acknowledged STOP generation and fresh Buddy 5
clearance. Convenience APIs do not bypass that gate. A cleared obstacle does
not resume an old command: a new movement must be submitted explicitly.

## Results and measured status

`motion_get_status()` returns the current snapshot: ID, command type, direction,
turn direction, measured counts/s, signed wheel travel since command start,
cumulative absolute wheel travel, standstill, fault cause, timestamp, and
current/maximum control interval. Cumulative travel includes reverse and turns;
it is not displacement along the course.

`motion_get_result(id, &snapshot)` returns the first terminal result for an ID.
The last **eight** terminal results are retained FIFO. `E_OBJ` means no retained
result (still pending/running, unknown, or evicted). Use current status and the
submission result to disambiguate. Capture results promptly during DRIVE
refreshes. Replacing an active command reports CANCELLED for its old ID.

RUNNING includes approach and stopping. MOVE/TURN stop each wheel when its own
target is reached, then report COMPLETED only after both encoders are unchanged
for 100 ms with PWM off. A STOP ID also completes only after that standstill.
This is encoder evidence, not an independent body-motion sensor: unplugged
encoders, slip and external pushing still require hardware fault validation.
Terminal results are immutable; use the latest status for later speed changes.

Straight movement corrects the physical left/right travel difference with a
bounded wheel-speed offset. Steering changes reset its origin. Each wheel
slows over its final 30 mm. The remaining-distance speed floor is 5 mm/s,
limited by the requested speed. These are preliminary tuning constants.

## Faults and scheduling

- DRIVE refresh loss: TIMEOUT / WATCHDOG after 100 ms.
- MOVE/TURN deadline: TIMEOUT / DURATION.
- No progress in the expected direction on either commanded wheel:
  FAULT / STALL after 500 ms (5000 ms below 5 mm/s). This latches a safety STOP.
  Before that, a commanded wheel with no progress for 100 ms gets an extra
  8% PWM kick to break static friction. Wheel PWM is feed-forward
  (15% + 0.004% per count/s of target) plus the PI correction, so a steering
  change splits the wheel PWMs immediately. A wheel commanded to 0 is never
  reported as stalled.
- Encoder movement continues for 1500 ms after PWM removal:
  FAULT / STOPPING.
- Buddy 5 STOP, lost clearance or stale generation: CANCELLED / SAFETY.
- Invalid timing (nonpositive or over 20 ms): FAULT / TIMING, latched until
  firmware restart. The 20 ms threshold must be validated under full load.

The control task runs at priority 5 on core 1; ordinary telemetry runs at
priority 14 every 100 ms. Telemetry mutex waits and console diagnostics are
outside the control loop. Robot startup initializes B1's telemetry objects but
does not start B1's synthetic-data demo or its communication tasks.
`ROBOT_ENCODER_DIAGNOSTICS=1` enables the optional encoder console tasks.

The application GPIO dispatcher owns bank IRQ 13. Encoder GP0..3 is its first
subscriber. Buddy 3 must register GP28 through `app_gpio_irq_register()` during
core-0 initialization, acknowledge only its own edges, and avoid reinstalling
the bank handler. Four disjoint pin groups are supported. Keep USB console
with `CRASH_REPORT=0` to leave GP0/GP1 available to the encoders.

## Validation and remaining integration

For the newer B1 binary counts/s API, shared fault bits and error-code mapping,
see [B1_INTEGRATION.md](B1_INTEGRATION.md). The adapter preserves this branch's
legacy JSON/mm/s interface until the B1 header is merged. Encoder faults use a
preliminary eight-invalid-transitions/100 ms limit and latch STOP.

Run host tests using `testing/subsystem2/host/run_tests.ps1` (GCC required), or:

```text
gcc -std=c11 -Wall -Wextra -Werror -pedantic testing/subsystem2/host/motion_test.c app_program/subsystem2/motion_profile.c -lm -o motion_test.exe
motion_test.exe
node tools/check-subsystem2-style.cjs
```

Host tests compile the real controller and command interface with mocked RTOS,
encoder and safety services. They do not prove real interrupt timing, SMP lock
behaviour, physical stopping or motor accuracy. See the hardware acceptance
plan and report templates under `testing/subsystem2/`.

Buddy 5's `s5_bridge.c` remains the integration boundary: navigation must submit
commands, retain their IDs, and read the results. B3 must supply line status.
A separate checked avoidance authorization policy is needed before turning or
reversing around an obstacle can use the safety gate. Do not simply disable
that gate or change drive/bypass flags to claim integration is complete.
# Temporary wheel-speed diagnostics

`B2_SPEED_DIAGNOSTICS` in `pid_control.h` enables `[B2-SPEED]` output at
most twice per second while the PI controller requests wheel PWM. Set it to 0
and rebuild to remove logging and its per-task timestamp. Controller gains,
trim, targets and safety gating are unchanged.

`targetL/R` are signed encoder counts/s after the motion profile, trim and
wheel-specific conversion. `measL/R` are the signed windowed speeds used by PI;
these differ from the instantaneous feedback in `[VC-STEER]`. `pwmReqL/R` are
signed requested duty percentages, not proof of applied output: the B2 safety
gate can independently force zero. `t` is low 32-bit milliseconds and `dt` is
the control interval in ms. Positive means forward. Logging occurs after the
PWM request and outside the safety lock. Console work still adds task overhead.
Targets in counts/s need not be identical for equal linear speeds because the
left/right encoder counts-per-revolution calibration differs. Compare each
measurement with its own target. Correlate timestamps with `[VC-STEER]`.
# Forward feed-forward calibration trial (2026-11-13)

`pid_control.h` now exposes `B2_FORWARD_FEEDFORWARD_TRIAL` (default 1),
`B2_LEFT_BASE_PWM` (6%) and `B2_RIGHT_BASE_PWM` (12%). These are provisional
trial intercepts based on the loaded-robot logs, not measured minimum duties
or a fitted calibration curve. The previous shared base was 15%. Forward
DRIVE uses the selected wheel base plus the existing 0.004% per target count/s,
PI correction and stalled-wheel kick. Thus actual requested PWM is not fixed
at 6/12%. MOVE, TURN and reverse DRIVE retain the original shared base.
Requested speed, trim, PI gains, limits and safety stops are unchanged.

Rebuild/reflash and repeat the plain-white zero-steer test from the same start,
with the same load and battery condition. Compare `[B2-SPEED]` target/measured
speeds during startup and after settling, and note the physical drift. Watch
for slow wheel startup, repeated kicks or a stall; lower base PWM can expose
static friction. Do not conclude calibration is correct from host tests.
Set `B2_FORWARD_FEEDFORWARD_TRIAL` to 0 and rebuild/reflash to restore the old
shared 15% base without removing diagnostics. Record the result before making
another tuning change. Forward line-following DRIVE is affected too; use the
plain-white test first.

Validation: 1,266 B2 host assertions passed with the trial on and off; both B1
integration variants passed. ARM syntax checks passed in both modes. The
network robot build generated `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
These checks establish software compatibility, not physical calibration.

## Change Requirement 1: minimum-bit representation (2026-11-20)

The supplied Change Requirement 1 and CSC25 tribunal documents are the design
references for this change. The tribunal's Buddy 2 decision explicitly retains
32-bit cumulative encoder totals and measured counts/s until a narrower range
is validated, and assigns two bits to each previous quadrature state. Motor
ownership, the 10 ms control period, calibration and safety deadlines continue
to use the existing contracts.

Accepted commands use `motion_stored_command_t` in `motion_storage.h` in both
the pending slot and controller state. Public `motion_command_t` remains a wide
validation input: packing caller input first could turn an invalid speed of
8193 into a valid-looking 1. Submission validates before packing. Irrelevant
command fields are normalized to zero; STOP retains its ID and generation.
These native C structures are never sent as raw MQTT bytes: bit-field order,
padding and enum sizes depend on the compiler ABI. B1 owns binary serialization
and the shared fault mask. B2 continues to raise `FAULT_MOTOR_STALL` and
`FAULT_ENCODER` through the kernel event-flag service.

Data representation table (widths describe logical fields; native alignment
can leave padding):

| Data field | Valid range | Bits | Implementation |
|---|---|---:|---|
| Command type | STOP/DRIVE/TURN/MOVE, 0..3 | 2 | Stored command/status bit-field |
| Direction | stopped/forward/reverse, 0..2 | 2 | Stored command/status bit-field |
| Turn direction | left/right, 0..1 | 1 | Stored command/status bit-field |
| Requested speed | 1..5000 cps; 0 for STOP | 13 | Unsigned stored bit-field |
| MOVE distance | 1..2147483647 counts; 0 otherwise | 31 | Unsigned stored bit-field |
| TURN angle | 1..360 degrees; 0 otherwise | 9 | Unsigned stored bit-field |
| DRIVE steering | -1000..1000 permille | 11 | Signed stored bit-field |
| Explicit timeout/run duration | 0 or 100..60000 ms | 16 each | Stored command bit-fields |
| Derived motion duration | 0..60000 ms | 16 | Controller bit-field |
| Command ID and safety generation | 0..4294967295 | 32 each | Retained uint32_t |
| Motion status | six states, 0..5 | 3 | Status/controller bit-field |
| Terminal fault cause | nine causes, 0..8 | 4 | Status/controller bit-field |
| Shared motor-stall/encoder faults | false/true | 1 each | Existing shared fault mask |
| Previous left/right quadrature state | 0..3 | 2 each | One uint8_t-based packed object |
| Encoder totals/start/history values | signed 32-bit counts | 32 each | Retained INT counters |
| Invalid-transition totals | 0..4294967295 | 32 each | Retained UW counters |
| Published measured wheel speed | signed 32-bit cps | 32 each | Retained INT values |
| Motor API duty input | clamped to -100..100 percent | 8 after clamp | Wide validation/calculation parameter; no retained duty message |
| Controller applied PWM magnitude | 0..35 percent | 6 each | Unsigned controller bit-fields |
| Controller wheel sign | -1, 0, +1 | 2 each | Signed controller bit-fields |
| Speed history count/index | 0..10 | 4 each | Controller bit-fields |
| Diagnostic speed sample count | 0..10 before reset | 4 | Local uint8_t-based bit-field |
| Motor ready | false/true | 1 | Motor-local bit-field |
| Safety clearance/stopped/STOP latch | false/true | 1 each | Packed safety object |
| Controller settling/stopped | false/true | 1 each | Controller-local bit-fields |
| Snapshot standstill | false/true | 1 | Status bit-field |
| Profile reached flags | false/true | 1 each | Task-local bit-fields |
| Demo measurement | 0..100 | 7 | Mailbox message bit-field |
| Demo processor ID | literal 1..2 | 2 | Mailbox message bit-field |
| Sequence/time/interval/generation values | full existing 32-bit range | 32 each | Retained integer fields |
| Absolute RTOS times and time history | existing 64-bit clock domain | 64 each | Retained int64_t |
| PI intermediates, travel, profile speeds | existing floating-point domain | 32 each | Retained float; no new quantization contract |

Queue occupancy/readiness/acknowledgement flags remain separate bool objects
where their synchronization domains differ. Each carries one logical bit;
packing them into a common storage unit would couple independent writers.
RTOS IDs, lock words, mailbox linkage and peripheral registers retain platform
representations. API arguments and arithmetic temporaries remain wide enough
for validation and overflow-safe calculations.

Synchronization: the existing command semaphore protects complete pending-slot
copies. The interrupt/SMP status lock protects snapshots and all eight retained
results. Every runtime update of the packed safety flags holds the existing
safety lock. The encoder ISR updates both phase fields under the encoder lock;
initialization completes before its interrupt sources are enabled. The PID
state and profile belong to the control task. The sample producer transfers
exclusive ownership of each filled block to the mailbox consumer. No packed
object receives an unsynchronized read-modify-write from independent tasks.

Measured with the installed Arm GNU compiler for Cortex-M0+ against baseline
commit `e7fe907` (`CNF_SMP=0`, `APP_TEST=0`, `-O2`):

| Object | Before (bytes) | After (bytes) | Saved |
|---|---:|---:|---:|
| Wide command input | 40 | 40 | 0 |
| Pending command slot | 40 | 24 | 16 |
| Current status snapshot | 60 | 52 | 8 |
| Eight retained results | 480 | 416 | 64 |
| Controller state (stack object) | 424 | 392 | 32 |
| Sample mailbox message, including T_MSG | 20 | 16 | 4 |
| Four sample pool blocks | 80 | 64 | 16 |

The pending slot, current snapshot, history and pool together save **104 bytes**
of declared storage. The controller's local object saves another 32 bytes, but
its configured RTOS stack allocation is unchanged. These measurements include
native structure padding. They are object-level RAM/message comparisons, not
a claim that whole-firmware data+bss or stack reservations fell by that amount.
The kernel's pool allocation alignment can affect actual heap consumption.

Reproduce with the Arm compiler and nm on PATH:

```text
python testing/subsystem2/host/measure_sizes.py --before e7fe907
powershell -ExecutionPolicy Bypass -File testing/subsystem2/host/run_tests.ps1
```

Validation: 10,018 host assertions pass, including round trips through the
real pending slot for every steering value -1000..1000, maximum speed, maximum
MOVE distance, maximum durations, 360-degree right turns and STOP identity.
Invalid values that would alias valid values after truncation are rejected.
Both B1 legacy and binary integration variants pass. ARM syntax checks pass
for all B2 C files, the sample consumer and Vehicle Controller with both
`CNF_SMP=0` and `CNF_SMP=1`, including on-target test code (`APP_TEST=1`).

The complete USB firmware build was attempted but the locally installed Pico
SDK v1.5.1 does not satisfy the repository's SDK include layout (`pico.h` is
missing from the configured include paths). A new UF2 was not produced.
The existing style checker also reports pre-existing branch violations;
this change does not certify full BARR-C compliance.

Hardware acceptance remains necessary: compare loop interval/jitter and
execution time under Wi-Fi load, exercise simultaneous safety STOP and command
submission on SMP, verify forward/reverse encoder totals and invalid-transition
counts, and repeat DRIVE/MOVE/TURN/STOP/stall/watchdog tests. Attach the final
ELF data+bss comparison and measured timing logs before claiming the Week 13
whole-robot correctness/performance evidence is complete.
