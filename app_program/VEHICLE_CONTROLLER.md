# Vehicle Controller

`vehicle_controller.c` runs on core 0 at priority 6 with a 20 ms period.
Real robot startup initializes B1, B2, B3, B4 and B5, preserves B4's calibration
gate, starts B3's actual sensor tasks and starts the controller. The old B3
teaching consumer is not used. ROBOT_AUTOSTART=1 arms the mission after fresh
motor/line/IMU inputs and a detected line; set 0 to require remote START.
The default target is 20% (1000 encoder counts/s).

The controller owns normal DRIVE and barcode commands. B5 owns motor commands
while its mission handoff is BUSY, executing encoder-controlled avoidance and
line recovery. The controller still handles hazards and remote STOP during
that handoff. B2 alone applies motor PWM. Automatic
motion tests and the B1 standalone demo are incompatible with controller mode
and produce a compile error. `ROBOT_MOTION_TEST` now defaults to zero.

| Remote command | Execution |
|---|---|
| START | Arm the mission, clear the ESTOP latch; all health/clearance checks still apply. |
| STOP | Immediate PWM stop, cancel navigation, disarm; requires another START. |
| ESTOP | STOP plus a latch that RESUME cannot release. Only START releases it. |
| PAUSE | Stop and discard the active navigation segment; retain mode/speed. |
| RESUME | Continue only after PAUSE, subject to current safety checks. |
| MODE | Stop/disarm, select AUTO, LINE_ONLY or TEST. TEST never drives. |
| SPEED | Set 0..100%; zero stops/cancels navigation. Positive speed resumes an armed zero-speed mission. |
| RESET_PEAK | Request B4's analysis task to clear its highest-hump estimate and B1's peak telemetry. |

Commands are drained in bounded batches. A STOP/ESTOP/PAUSE/MODE prevents later
movement commands in that batch from restarting the car; later stop commands
are still executed. B1's existing ACK confirms queue delivery, not successful
motor execution. Robot-state telemetry describes the controller's resulting
state. Wi-Fi loss alone does not interrupt an already running mission.

DRIVE is submitted every 20 ms while following a detected line, comfortably
inside B2's 100 ms DRIVE watchdog. Queue contention can skip a refresh, but a
60 ms submission gap disarms the mission. B2 retains its independent watchdog.
All commands carry the current B2 safety generation. No controller path grants
clearance: B5 must provide a fresh authorization following every safety STOP.

States use the existing B1 wire enum: INIT, IDLE, LINE_FOLLOW, NAV_COMMAND,
HUMP, OBSTACLE_SCAN, OBSTACLE_AVOID, LINE_RECOVERY and SAFE_STOP. B5 scans,
profiles, bypasses and reacquires the original line/heading before returning
control. On lost clearance the controller stops and waits for B5. Hump observations halve the following speed. On line loss the
car enters LINE_RECOVERY and searches for up to 8 seconds: it creeps forward
at half speed with 25% steering, first toward the side the line was last
seen, in widening sweeps (600 ms, then 1200 ms the other way, 1800 ms back,
and so on). If the line is not found it stops but stays armed, and resumes
line following by itself as soon as B3 sees the line again; no new START is
needed. Dashboard STOP/ESTOP, faults and stale data still disarm. PAUSE during a turn discards that turn rather than
blindly replaying it on RESUME.

B3's timestamped snapshot must be at most 100 ms old, B2 feedback at most
100 ms old, and B4's latest valid notice at most 1500 ms old (quiet B4 notices
are generated every second). Sensor faults, impact observations, roll beyond
25 degrees, pitch beyond 40 degrees, stale data and motion timeout/fault disarm
the mission. Hazard observations retained in a coalesced B4 notice stop the
current tick even if its latest snapshot has recovered. Restart requires a new
START after healthy data returns. Thresholds need on-car validation.

AUTO consumes B3 navigation messages: LEFT/RIGHT turn 90 degrees, UTURN turns
180 degrees, STRAIGHT moves 10 cm. STOP/standstill and renewed B5 clearance
precede submission. The exact B2 command result must indicate successful
completion and physical standstill. Failed/cancelled moves stop the mission.
Navigation messages received while idle, paused or already executing another
segment are discarded. LINE_ONLY ignores navigation. B3 decodes Code 39
barcodes (`b3_code39.c`, full Code 39 set) read in either direction; the
`b3_routes` table maps `A` to STRAIGHT and `Z` to RIGHT; other letters are
logged and ignored. A queued command waits for a confirmed junction and the
car slows to 75% while one is pending.

Two-sensor steering is proportional: the difference between the two line
sensors, ignoring the first 50 ADC codes, sets up to 80% steering toward the
stronger side. It applies whenever the line is detected, including at a
junction (balanced sensors give zero steering on their own). Curve speed:
the base speed drops in proportion to the hardest recent steering, down to
60% at full steer, and recovers over about 0.4 s after the curve
(`VC_CURVE_MIN_PERMILLE`, `VC_CURVE_DECAY_PER_TICK`). Motion is smoothed:
steering changes by at most 150 permille per 20 ms tick, and speed rises by
at most 100 counts/s per tick (also from standstill); slowing is immediate.
No DRIVE asks a wheel to run between 0 and `VC_MIN_WHEEL_CPS` (300 counts/s,
about 25 mm/s; calibrate on the car): a curve's inner wheel a little below it
is raised to it, far below it is set to 0 so the car pivots around that wheel,
keeping the outer wheel's speed. This prevents B2 stall faults on curves and
at low dashboard speeds. B3 treats raw ADC readings below 300 as lost, then
normalises the remaining readings per sensor (0 = off-line, 1000 = black);
a sensor is on the line at 201 or more. While a barcode is under the sensor
(a GP28 edge within 400 ms) the speed is frozen at the value it had when the
first bar arrived: no junction, curve or barcode slowdown, because a speed
change mid-strip distorts the timed bar widths.
The base speed is reduced only when the outer wheel would otherwise exceed
B2's 5000 counts/s limit. Sensor thresholds, steering direction/gain, junction travel and turn
accuracy must be calibrated on the actual track. B5's physical avoidance
bridge is not implemented: on an obstacle the car stops and waits.

B5 clearance: an ultrasonic ping that returns nothing within range (echo
longer than the 30 ms window, or beyond 400 cm) counts as clear at 400 cm when
`ROBOT_NO_ECHO_MEANS_CLEAR` is 1 (the default in `robot_config.h`), so the car
can drive in open space. A sensor that never responds is still a NO_ECHO fault
and keeps the car stopped. Set it to 0 to stop whenever no echo returns.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File
testing/controller/run_tests.ps1` from the project root. Tests execute the real
controller policy and task adapter with simulated producer interfaces, including
commands, watchdog cadence, sensor staleness, faults, navigation and clock wrap.
Hardware scheduling, stop distance and full Pico firmware execution require
separate validation.

See [current build, recovery and calibration notes](../ROBOT_FIX_NOTES.md).
# Temporary steering diagnostic

`VC_STEERING_DIAGNOSTICS` in `vehicle_controller.h` defaults to 1 for the
rightward-drift investigation. Set 0 and rebuild to compile out the diagnostic
and its four-byte per-controller timestamp. At most one `[VC-STEER]` line is
printed per 250 ms, after an accepted forward DRIVE, including line recovery.

Fields: `t` is unsigned milliseconds; `state` is the controller state;
`R26` and `L27` are normalized physical right/left sensor values (0 white,
1000 black); `flags` is the B3 line bitmask. `steer` is the accepted command in
permille: positive turns right, negative left, zero straight. `base` is the
final command speed in encoder counts/s before B2 steering/trim distribution,
not PWM. `measL`/`measR` are the existing measured wheel speeds in counts/s;
`age` is their snapshot age in milliseconds. Feedback precedes the new command,
so compare successive records, not a single record as an instantaneous response.
No sensor thresholds, steering gains or motor targets are changed by logging.

## Line following v4 (`VC_FOLLOW_V4`, default on)

A clean rewrite of following, line search, junction turns, barcode reading,
obstacle bypass, ramp handling and lift-stop. `VC_FOLLOW_V4 0` and
`ROBOT_BOX_BYPASS 0` bring back the previous controller and B5 bypass.

| Behaviour | v4 |
|---|---|
| Speed | cruise 120 mm/s (dashboard 29 %), slow 100 mm/s near an obstacle and for 150 mm after a bypass, barcode reads at 60 mm/s |
| Steering (`vc_follow.c`) | error = left − right (normalised); target = e·\|e\|/1000; smoothed 1/4 per 10 ms; no deadband or slew limiter |
| Curves | base speed cut up to 50 % with \|steering\|; wheel split ±50 mm/s at full error; inner wheel ≥ 1/3 of base |
| Line leaves a sensor | coast 120 ms (400 ms if it left on the other side last time), then an arc toward that side growing 20 ‰ per 10 ms |
| Lost | both white ≥ 200 ms **and** ≥ 60 mm after the line went black under one sensor; centred (both white after only a grey graze) is never lost |
| Line search | spin 60° toward the side last seen, 120° back, 60° again; either sensor on tape = found; 5 s → run stops |
| Junction | nL + nR ≥ 1400 for ≥ 10 mm, classified 30 mm later: both sensors ≥ 900 = cross-junction |
| Barcode turn | decoded action waits for a cross-junction: stop, forward 60 mm, turn 90° (U-turn 180° left), follow |
| Barcode read | on the first bars (≥ 3 GP28 edges): stop, reverse past the strip start (+110 mm), wait 600 ms, read at 60 mm/s; failed pass → back to its start (+10 mm), up to 2 retries |
| Obstacle (`subsystem5/s5_box.c`) | slow at 60 cm, stop at 40 cm (2 agreeing pings), creep to 25 cm, scan; single narrow echo = ramp glint → continue; bypass on the side needing the smaller shift: turn 90 out, lateral leg, turn 90 back, pass leg (distance + 15 cm depth + 10 cm tail), turn 45 in, accept a 15–75 mm wide tape crossing 50–120 % of lateral·√2 along, forward 60 mm, turn 45 back; lateral > 30 cm or pass > 80 cm, side echo < 6 cm, or 20 s without the line → stop |
| Ramp | while B4 reports a hump / pitch ≥ 2° (held 480 ms and 150 mm after) the obstacle stop distance is 25 cm |
| Lift-stop | both line sensors raw ≥ 3500 for 300 ms, or the IMU reads lifted/tilted for 60 ms → stop, START needed |

Host tests: `testing/controller/vc_follow_test.c` (in `run_tests.ps1`).
