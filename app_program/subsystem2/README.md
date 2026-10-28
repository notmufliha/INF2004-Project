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
defaults to 1000 reference counts/s (approximately 8.17 mm/s with the current
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
  geometry, 1000 counts/s is approximately 8.17 mm/s. Confirm these encoder
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
