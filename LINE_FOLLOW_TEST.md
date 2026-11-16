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

V2 turn recovery: ROBOT_CLEAR_PATH_FORWARD defaults to0 on line_follow_v3.
Captured log held left briefly then steer0 while both sensors reported lost.
The default now uses last-side turn recovery for1500 ms, followed by bounded
widening search; total search timeout remains8000 ms. An exhausted search
stops but remains armed for line reacquisition. Initial placement must expose
the black line to the sensors before startup. V1 steering stability changes
remain. Controller tests exercise the production default; optional clear-path
tests explicitly opt in. Controller suite and all18 branch B5 groups pass.
Physical course completion still requires testing.
