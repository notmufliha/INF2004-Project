# Spread sensors + barcode re-read (branch `line-barcode-spread`)

Firmware: `line_barcode_spread.uf2`. Based on `line-follow-smooth` with
`barcode-robust` merged in.

**Sensor layout.** `VC_LINE_STRADDLE 1` (vehicle_controller.h): the two line
sensors are spread out so the 18 mm line runs BETWEEN them. Both on floor =
centred (drive straight); one on black = steer toward it; both black = a
crossing tape (junction). Before, both-on-floor counted as "line lost", so the
car ran the hard line search between every correction: the jerkiness. The
line counts as lost only when it went out past a sensor and was not found
within 0.7 s, or no sensor touched it for 3 s. For the old close-together
layout set `VC_LINE_STRADDLE 0`.

**Sensor ranges.** `B3_LINE_LEARN_RANGE 1`: each sensor uses its own learned
floor/tape readings once it has seen both, because moving the sensors changes
them. After power-up, slide the car across the line once (both sensors over
the tape), then put it down; don't lift it while running. `[B3-RANGE]` on the
serial log shows the range in use.

**Barcode feedback.** GP22 buzzer (Robo Pico; buzzer switch ON): one high beep
= barcode read (the letter is in the `[B3] barcode *X* read` log line), two
low beeps = bars crossed but rejected. In AUTO mode a rejected scan makes the
car stop, reverse to 5 cm before the strip and drive over it again (once per
barcode, `VC_BARCODE_RESCANS`).

Tuning: weaving on straights -> raise `VC_STRADDLE_FULL_SCALE`; curves missed
-> lower it, or lower `VC_STRADDLE_DEEP`.

# Line-following stability build

Branch: `line_follow_v3`. Firmware: `build_make/robot_line_turn_v2.uf2`.
Serial identifier: `[FW-ID] ... LINE-TURN-V2`.

Normalized sensor-difference deadband is 60 and full correction starts at
650 instead of 400. Maximum correction remains 800 for sharp curves.
Weaker corrections release immediately. On a sign change the old correction
is discarded and the opposite correction ramps from zero, instead of keeping
the robot turning away from the newly observed line.

Corrections through 500 permille preserve the requested wheel-speed ratio at
low speed by raising the base to the sustainable minimum. They no longer
become sudden pivots. Strong corrections and lost-line search retain their
existing behavior. Sensor polarity, measured black/white calibration, motor
speed settings and turn calibration are unchanged.

Validation: `testing/controller/run_tests.ps1` passes, including centering,
direction reversal, moderate curves, sharp curves, lost-line recovery, sensor
calibration and stale-data/fault stops. Network-enabled UF2 build succeeds.
These tests verify command behavior, not physical course tracking.

Flash the UF2 and test a straight section, then curves. Start with the line
centered beneath the two sensors. Inspect `[VC-STEER]` values if weaving
continues; floor/load-specific tuning still requires physical feedback.

Line recovery always searches for the tape when the line is lost.
Captured log held left briefly then steer0 while both sensors reported lost.
The default now uses last-side turn recovery for1500 ms, followed by bounded
widening search; total search timeout remains8000 ms. An exhausted search
stops but remains armed for line reacquisition. Initial placement must expose
the black line to the sensors before startup. V1 steering stability changes
remain. Controller tests exercise the production default; optional clear-path
tests explicitly opt in. Controller suite and all18 branch B5 groups pass.
Physical course completion still requires testing.


## Integrated obstacle bypass

The robot starts with smooth line following (`ROBOT_OBSTACLE_TEST=0`). Buddy 5 pauses the vehicle controller on an
obstacle and runs the bypass imported from `Buddy_5_Obstacle_Avoidance`.
The straight-driving mode has been removed; startup follows visible tape or performs a bounded line search.

With `ROBOT_BYPASS_LINE_HANDOFF=1` (the normal default), once the robot has
left the original tape and completed the outward left/right turn, fresh black
on either line sensor stops the bypass immediately throughout all remaining
movement legs, later turns, and recovery states. Three distinct fresh black
samples at physical rest confirm the handoff. No additional bypass alignment
turn is issued; the vehicle controller resumes smooth line following after
front clearance is confirmed. Stale line samples stop motion. The original
bounded search remains available if the first contact is lost while stopping.

Floor validation (not performed by host tests):
1. Start on black tape with a clear front path; verify smooth following.
2. Place an obstacle ahead; verify following pauses, scan completes, and the
   robot chooses a clear side. Repeat for left and right bypasses.
3. Put black tape across the post-turn route; verify motion stops on contact,
   then following resumes without completing the remaining bypass turns.
4. Repeat with both sensors on black and with a short transient black contact.
5. Keep the front blocked during reacquisition; verify forward motion waits.

Host checks: `python testing/subsystem5/run_complete_tests.py`,
`testing/controller/run_tests.ps1`, and `testing/subsystem2/host/run_tests.ps1`.

Startup no-target policy: `ROBOT_NO_TARGET_FORWARD=1` and
`ROBOT_COMPLETED_FAR_ECHO=1` allow line following when there is no ultrasonic
target. Logs label this `OPEN_SPACE_ASSUMED`; it is not a measured distance.
A disconnected echo wire is indistinguishable from no target. Real close
echoes still stop motion, and malformed/stuck-high/no-fall pulses remain faults.
Scan boundary rays retain their separate strict policy.

Startup line acquisition no longer waits for black before enabling navigation.
Fresh line/IMU/motor samples and confirmed front clearance arm the controller;
it follows black tape immediately, or uses the existing bounded steering search
when initially off tape. The search stops on its existing timeout. STOP/ESTOP,
stale sensors, impacts, motor faults and close obstacles still prevent movement.
