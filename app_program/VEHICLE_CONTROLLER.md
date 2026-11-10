# Vehicle Controller

`vehicle_controller.c` runs on core 0 at priority 6 with a 20 ms period.
Real robot startup initializes B1, B2, B3, B4 and B5, preserves B4's calibration
gate, starts B3's actual sensor tasks and starts the controller. The old B3
teaching consumer is not used. Boot state is Idle; movement requires START.
The default target is 20% (1000 encoder counts/s).

The controller is the only mission command producer. B5 retains its independent
immediate STOP and clearance heartbeat; B2 alone applies motor PWM. Automatic
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
HUMP, OBSTACLE_SCAN, LINE_RECOVERY and SAFE_STOP. OBSTACLE_AVOID is reserved;
physical bypass remains disabled. On lost clearance the controller stops and
waits for B5. Hump observations halve the following speed. On line loss the
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
barcodes (`b3_code39.c`): `A` queues STRAIGHT and `Z` queues RIGHT; other
symbols are ignored. A queued command waits for a confirmed junction and the
car slows to 75% while one is pending.

Two-sensor steering is proportional: the difference between the two line
sensors, ignoring the first 50 ADC codes, sets up to 50% steering toward the
stronger side. A sensor counts as on the line between 301 and 4100 ADC codes.
The base speed is limited so the outer wheel never exceeds B2's 5000 counts/s
limit. Sensor thresholds, steering direction/gain, junction travel and turn
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
