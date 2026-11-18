# Spread sensors + barcode re-read (branch `line-barcode-spread`)

## Run log (send us real data)

The car records the last ~24 s of every run (one row per 20 ms: raw and
normalised line sensors, GP28, flags, junction evidence, state, steering,
speed, pending/active barcode turn) plus events: `BARCODE_READ`,
`BARCODE_REJECT`, `GP28_RUN` (mm the barcode sensor was on black),
`JUNCTION`, `NAV_WAITING` (tape seen with a turn pending but not accepted:
arg = curve_steer), `NAV_START`, `NAV_DONE`, `LINE_LOST`, `RESCAN`.

1. Drive the car over the barcode + junction (no USB needed), stop it.
2. Plug it into the laptop, close other serial monitors, then from the repo:
   `powershell -ExecutionPolicy Bypass -File tools\capture_run_log.ps1 -Note "B then junction, sensors together"`
3. `git add logs` / `git commit -m "Run log: ..."` / `git push`.

## Junctions

Layout: `VC_LINE_STRADDLE 1` (spread) is the default again; if the line sits
under both sensors for 10 cm the car switches itself to the together layout
(`[VC] both line sensors on black ...`). The boot log says which is active.
Spread: when the line leaves a sensor the car coasts 120 ms (400 ms if the
exit is on the other side from the last one), then turns toward it in a
growing arc (the 2026-11-18 logs zig-zagged: an immediate hard turn pushed
the line across to the other sensor every 0.27 s). A rejected barcode is
re-read only once GP28 has seen floor for 4 cm (the strip is passed).

Sensors together (`VC_LINE_STRADDLE 0`): both line sensors are on
the line all the time, so a junction is the barcode sensor GP28 on black
for >= 13 mm. B3's balanced both-black flag is no longer required (an
off-centre car failed it), and only a hard curve (`curve_steer` >= 600, was
300: normal corrections exceeded 300) blocks it. If GP28 does not pass over
the junction tape at all (branch on the other side), the log shows no
`GP28_RUN` of ~18 mm there: then we need a different signal.

Spread layout (`VC_LINE_STRADDLE 1`): junction = both line sensors touched
black within the last 25 mm of travel (not necessarily at the same moment),
or the GP28 tape with either sensor black in that window (side branch).
`[VC] junction` in the log when detected. Buzzer is OFF by default
(`B3_BUZZER_ENABLE` in b3_buzzer.h). After a junction with a barcode turn
pending, the car keeps following for `VC_JUNCTION_APPROACH_MM` (50 mm;
MEASURE: line sensors to wheel axle, minus ~10 mm) so it turns with its
wheels on the junction instead of into the floor beside the new line.
Bends: the line leaving the same sensor again within 1 s turns back toward
it at once (arc from 350), no coasting.
Offset after a bend: an exit counts as "line may be outside" from 450
(was 700: GP26 reads tape at only ~450-750), and with both sensors on floor
for 0.4 s the car drifts gently toward the side that last touched the line
(seek, up to 450 permille). Junction evidence is ignored while reversing
for a barcode re-read and cleared afterwards.

2026-11-18 rework: line following is back to commit 3ada3f5 (the version
that followed best). Commits 3e849eb/78c697d made the car zig-zag full
left/full right on straights (run log: every 300-450 ms touch counted as a
"bend" and pivoted). Kept from them: a junction counts only after 8 cm of
steady following (no fake junction after the barcode re-read reverse), and
steering is held within +-400 while crossing barcode bars. For the double
turn (hump) only: the inner wheel may stop (pivot) when one sensor has held
the line for >= 700 ms and steering is >= 700.

Barcode turns (spread): B2 turns 20 deg short (70 / 160 / 340), then the car
keeps turning the same way until a line sensor finds the line, then follows
it (a U-turn that stopped short used to drive off straight: both sensors on
floor looked "centred"). Barcode beside the line: a line sensor that turns
black twice on a barcode strip (GP28 >= 3 changes) while the other does not
is reading bars and is ignored until the strip is passed (`[VC] left line
sensor is reading barcode bars` in the log); no junction on a strip.

Firmware: `line_barcode_spread.uf2`. Based on `line-follow-smooth` with
`barcode-robust` merged in.

**Sensor layout.** `VC_LINE_STRADDLE 1` (vehicle_controller.h): the two line
sensors are spread out so the 18 mm line runs BETWEEN them. Both on floor =
centred (drive straight); one on black = steer toward it; both black = a
crossing tape (junction). Before, both-on-floor counted as "line lost", so the
car ran the hard line search between every correction: the jerkiness. The
line counts as lost only when it went out past a sensor and was not found
within 0.7 s, or no sensor touched it for 3 s. Steering strength is the same
as `barcode-robust` (deadband 60, full 800 at 650). If both sensors stay on
black for 6 cm (a crossing tape is 18 mm), the sensors are close together and
the car switches to `barcode-robust`'s close-layout following until reboot
(`[VC] both line sensors on black ...` in the log). `VC_LINE_STRADDLE 0`
forces the close layout from the start.

**Sensor ranges.** Fixed endpoints as in `barcode-robust` (default
`B3_LINE_LEARN_RANGE 0`). Setting it to 1 learns each sensor's own floor/tape
readings instead (slide the car across the line once after power-up; don't
lift it), for when moved sensors no longer match the fixed values.

**Barcode feedback.** GP22 buzzer (Robo Pico; buzzer switch ON): one high beep
= barcode read (the letter is in the `[B3] barcode *X* read` log line), two
low beeps = bars crossed but rejected. In AUTO mode a rejected scan makes the
car stop, reverse to 5 cm before the strip and drive over it again (once per
barcode, `VC_BARCODE_RESCANS`).

Tuning: curves missed -> lower `B3_LINE_STEER_ERROR_FULL_SCALE` (both
layouts) or `VC_STRADDLE_DEEP`; weaving on straights -> raise it.

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
