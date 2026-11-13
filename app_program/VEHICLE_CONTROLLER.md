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
sensors, with a 40-count normalized deadband, sets up to 80% steering toward the
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
at low dashboard speeds. B3 uses the reference branch's fixed per-channel
high-is-black normalization: GP26 white 245 / black 346, GP27 white 212 /
black 777 (0 = off-line, 1000 = black); a sensor is on the line at 201 or
more. The curve controller and bounded line-loss search are unchanged;
the reference's drive-straight-when-lost option is not imported.
B2 uses reference encoder resolutions (2534 left / 2530 right counts/rev)
and forward DRIVE feed-forward bases (10% left / 15% right), plus the existing
speed term, PI correction and breakaway kick. MOVE, TURN and reverse DRIVE
retain the original 15% bases; turn-angle correction scales are unchanged.
While a barcode is under the sensor (a GP28 edge within 400 ms) the speed
is frozen at the value it had when the first bar arrived: no junction, curve
or barcode slowdown, because a speed change mid-strip distorts the timed
bar widths.
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
