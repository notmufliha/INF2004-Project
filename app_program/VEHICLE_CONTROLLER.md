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
waits for B5. Hump observations halve the following speed. Fresh line loss stops
in LINE_RECOVERY for at most three seconds; the current recovery policy waits
for detection and does not perform an uncalibrated moving search. Timeout
disarms in SAFE_STOP. PAUSE during a turn discards that turn rather than
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
segment are discarded. LINE_ONLY ignores navigation. B3's barcode decoder is
still disabled until the team configures the actual barcode protocol.

Two-sensor steering is deliberately provisional: left-only detection applies
35% left steering, centre/junction detection goes straight. The base speed is
limited so the outer wheel never exceeds B2's 5000 counts/s limit. Sensor
thresholds, steering direction/gain, junction travel and turn accuracy must be
calibrated on the actual track. This controller does not complete B3's barcode
protocol or B5's physical avoidance bridge.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File
testing/controller/run_tests.ps1` from the project root. Tests execute the real
controller policy and task adapter with simulated producer interfaces, including
commands, watchdog cadence, sensor staleness, faults, navigation and clock wrap.
Hardware scheduling, stop distance and full Pico firmware execution require
separate validation.
