# Subsystems 1, 2, 3 and 5: build and test

Current route marker: `B5-BLUE-ROUTE45`. Bypass turns remain 45 degrees as
requested for the blue detour in the photo. Sequence: turn away from the
obstacle -> diagonal forward -> opposite 45-degree turn to face parallel ->
forward past obstacle -> 45-degree inward turn -> find black -> restore
forward heading and resume line steering. Side selection still requires
measured clearance; the drawing does not supply a physical scale or sensor map.
The parallel leg now subtracts the diagonal's forward progress, avoiding the
previous excessive straight travel. For an 8-cm profile and 25-cm front range,
the offset is 28.28 cm diagonally (20 cm lateral/forward) and the parallel
leg is 63 cm instead of 83 cm. Depth remains bounded at 40 cm, not measured
from the photograph. Timer fix, trim 90, and stop/resume 16/21 cm remain active.

Latest range adjustment: stop/scan at **16 cm** (previously 18 cm), and
resume at **21 cm** (previously 23 cm). Minimum turning clearance remains
14 cm. Confirm `stop_cm=16 release_cm=21` in the `B5-TIMER1MHZ-SMOOTH`
boot line. Verify braking clearance at low speed on the actual floor.

Current boot marker: `B5-TIMER1MHZ-SMOOTH`. Startup now initializes RP2040
WATCHDOG_TICK to ENABLE | 12 after selecting the 12-MHz reference crystal.
The hardware TIMER uses that tick; SysTick/RTOS milliseconds use a separate
clock. The uninitialized zero divider corresponds to 512 reference cycles,
so a nominal 65,000-us trigger interval took about 2.77 seconds. This matches
the 2720..2780 ms poll gaps and repeatedly expired the unchanged 350 ms motor
clearance lease, causing short motor bursts followed by stops.

Confirm `[S5-CLOCK] timer_tick_cycles=12 expected=12 (1 MHz)` at boot. Outside
stationary scans, repeated multi-second `S5-POLL-GAP` messages should disappear
and encoder counts should progress continuously while following a clear path.
The driver regression exercises the production clock initializer and checks
no-target reads remain below 130 ms in its simulated register/timer model.
Physical cadence still needs verification; host timing is not hardware proof.
The motor watchdog remains 350 ms. Fixed sensor settling is reduced from
2000 to 500 ms, and the extra servo wait is reduced from 600 to 200 ms.
IMU calibration and three fresh clearance confirmations still precede motion.

Current boot marker: `B5-TURN45-APPROACH-SLOW`. The latest turn was interrupted
by INSUFFICIENT_CLEARANCE, with scan rays at 13.65..14.59 cm, rather than
finishing at the wrong encoder angle. Valid front readings <=30 cm now
reduce VC approach speed before the unchanged 18 cm stop point. Missing
readings do not release an already active slowdown. This aims to reduce
coasting into the 14 cm turn envelope; verify braking distance physically.
Viewpoint fallback no longer always starts left. It chooses a side whose
two rays both exceed 14 cm, preferring the clearer side. If neither side
qualifies, `S5-TURN-WAIT` keeps it stopped and the normal rescan continues.
All bypass/viewpoint commands remain 45 degrees and straight trim remains 90.
Test with the robot initially farther than 30 cm from the obstacle so the
slower approach can take effect. A robot already too close must be moved
back with motor power off; this firmware does not add blind reverse.

Current boot marker: `B5-TURN45-FORWARD-RECHECK`. The latest capture completed
the left turn, then reported about 17 cm ahead and latched
INSUFFICIENT_CLEARANCE before submitting forward travel. Before starting a
straight leg, a close post-turn echo now holds STOP and rechecks instead of
permanently aborting. Three fresh front readings >=23 cm (including configured
open-space/no-target readings) release the forward MOVE without another turn.
A close reading resets that confirmation. `[S5-FORWARD-WAIT]` reports the
current range and confirmation count. Persistent close echoes keep the robot
stopped within the existing maneuver deadlines; do not treat 17 cm as a clear
path. Invalid sensing and close readings during active travel still fault and
stop immediately. Check the physical sensor direction and beam if an apparently
clear heading repeatedly reports a nearby surface.

Current boot marker: `B5-TURN45-IMU-GAP250`. The latest capture never starts
driving: `B4-FAULT-DETAIL raw_valid=0 sample_gap_ms=110 failures=1` repeatedly
blocks boot with `imu=0 faults=4`. Acquisition no-data timeout, processing
timeout and terrain timing now use the same 250 ms robot limit. Fresh samples
110 ms apart are accepted; absence beyond 250 ms still faults, invalid values
remain invalid, and recovery still requires five fresh good samples. This
does not bypass the IMU gate or authorize motion from stale measurements.
The default non-robot processing configuration retains its 100 ms limit.
IMU host tests include the captured 110 ms gap and >250 ms failure/recovery.
Verify `VC-BOOT` appears after valid sensors and clearance on the real robot.

Latest straight-drive adjustment: `ROBOT_STRAIGHT_TRIM_PERMILLE=90`, reduced
from 100 to counter the reported right drift. Confirm `straight_trim=90`
in the boot log. This is a floor-test tuning trial between the earlier
80 (left drift) and 100 (right drift); verify on the same surface and load.

Current boot marker: `B5-TURN45-IMU-RETRY`. The latest no-movement log already
accepted no-target echoes as `OPEN_SPACE_ASSUMED`; startup was blocked by
`imu=0 faults=4`, not by the ultrasonic policy. I2C0 transaction timeout is
now 20 ms instead of 5 ms, and GY-511 register reads/writes retry a timeout
or I/O error once. Persistent errors and short transfers remain invalid.
`B4-I2C` reports bus errors after retry, and `B4-FAULT-DETAIL` distinguishes
invalid raw data from timing gaps. Keep the chassis still through calibration.
If the fault continues, save these lines with `VC-BOOT-WAIT`; no-target
ultrasonic readings cannot establish that the IMU or its wiring is healthy.

Current boot marker: `B5-TURN45-CLEARANCE14`. The latest COM6 capture selected
GO_RIGHT from a 15.36 cm scan but rejected the live 14.50 cm reading before
submitting a motor turn. Minimum rotation clearance is now 14 cm, above
the configured chassis rotation radius (~12.8 cm for half-width 8 cm and
half-length 10 cm). Verify actual dimensions and clearance on the floor.
Forward stopping remains 18 cm, resume remains 23 cm, and turns remain 45
degrees. Expect a right-turn `S5-MOVE` at the captured 14.50 cm reading,
then a positive-distance MOVE when the new heading is clear. Readings at
or below 14 cm, invalid sensing, and STOP still prevent rotation.

Latest boot marker: `B5-TURN45-ONCE-ADVANCE`. The new COM6 capture showed
72..74 cm clear after the first left viewpoint turn, but the old logic kept
scanning and restoring heading. A completed left or right viewpoint turn now
commits directly to forward bypass travel after three clear front readings.
It skips the remaining viewpoint scans and does not repeat the outward turn.
`S5-LOOK-ADVANCE` must be followed by `S5-MOVE` with positive `cm_x100` and
`turn_deg=0`. Width uses the conservative 30 cm course bound in this shortcut;
it is not a newly measured width. The later opposite turn remains part of
the bypass, after the diagonal offset is completed.

Three clear front readings during coarse/fine planning also resume normal
driving, even if a side scan is invalid. A temporary clear reading followed
by a close/invalid front reading cancels that release and requests a fresh
stopped scan. Obstacle/fault stop protections remain enabled.

Latest range adjustment: stop/scan at **18 cm** (previously 20 cm), and
require **23 cm** to resume (previously 25 cm). The 15 cm minimum turning
clearance remains. Confirm `stop_cm=18 release_cm=23` in the boot log.

Current boot marker: `B5-TURN45-CLEAR-FORWARD`.
Bypass and viewpoint turns now request 45 degrees. The bypass offset is
diagonal, so its encoder distance is lateral clearance / sin(45 degrees).
With healthy sensors and clear front space, the default policy drives
straight at reduced speed when black is absent and uses line steering as
soon as black is detected. Startup no longer requires placement on tape.
After the bounded bypass search, it restores forward heading and continues
straight if tape is still absent. STOP, PAUSE, E-STOP, close obstacles,
stale/invalid sensors and motor faults still stop movement.
After a completed turn, the robot advances when the next section is clear.
If the full width/depth leg is not visible, it moves at most 5 cm (less if
needed to preserve chassis clearance and margin), stops, and rechecks.
Completed encoder travel accumulates toward the original bypass distance;
acceptance alone does not advance the plan. Close or invalid front readings
still stop immediately. The 10% turn reduction remains active.

Both encoder turn scales retain the prior 10% reduction after floor
feedback that the robot turns too far. This reduces wheel travel for all chassis turns,
including bypass and viewpoint turns. Verify completed left/right turns on
the same floor and load; this is a provisional adjustment without a measured
overshoot angle. Flash the current `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.

## Current obstacle bypass (12 November 2026)

Flash `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`; confirm
`B5-TURN45-CLEAR-FORWARD` at boot. The COM6 capture showed every 70..110-degree
ray hitting the obstacle at 16..18 cm, so neither edge was visible.
The servo retains those limits. If the normal scan cannot find a route,
the chassis turns left 45 degrees, scans while stopped, turns right 45
degrees to center, then another right 45 degrees, scans while stopped,
then turns left 45 degrees to restore its
heading. `[S5-LOOK]` reports these turns, and `[S5-WIDE]` reports their
combined bearings relative to the original heading. Encoder turn calibration
therefore affects both the scan geometry and bypass accuracy.

After choosing an opening, the sequence is turn out 45 degrees, travel the
width-based diagonal distance, turn the opposite 45 degrees, travel beyond the
obstacle's configured depth bound, turn inward, and search for the black line.
The first fresh black detection stops the robot; three distinct stationary
samples confirm it. It then restores forward heading and hands control back
to the VC: follow detected tape, otherwise continue straight on a clear path.
If the search limit is reached without tape, restore forward heading and use
the same clear-path policy instead of parking. `[S5-ROUTE]` reports
`offset_cm_x100`, pass distance and search limit. The old fixed one-second
offset is removed. A single central hit in the wide view uses the clear
bracketing angles as a conservative visible-width estimate.

Use wheels raised for initial direction checks. On the floor, test a small
and large obstacle with a clear left route, then repeat with a clear right
route. Expect a larger measured lateral offset for a larger obstacle. The
current course limits remain width <=30 cm and depth <=40 cm; depth is a
configured bound, not measured by the frontal scan. With both sides blocked,
invalid sensing, or an object <=15 cm away during a turn, the robot stops.
Host tests and a successful build do not establish
physical clearance or successful bypass.

## Boot arming fix (12 November 2026)

The supplied COM6 log stops on `imu_valid=0 hazard=1 faults=4` before
front sensing finishes startup. Later `OPEN_SPACE_ASSUMED` readings cannot
restart that stopped mission. Autostart now stays pending until healthy sensors
AND B5 front clearance are ready. B5 can establish clearance while boot is
arming; a warmup IMU failure keeps motors stopped and allows boot arming to
continue once readings recover. STOP, PAUSE, MODE and E-STOP cancel boot arming.
Once START fires, faults still stop the mission and require an explicit restart.
The radio runs at priority 24 and real-robot IMU sampling/analysis at 8/9.
The startup handshake has a host regression test; physical recovery remains to be tested.
Sensor fault, impact, tilt, STOP and watchdog protections remain enabled.

Flash `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` with motor power off and wheels
raised. Confirm the current `B5-TURN45-CLEAR-FORWARD` marker in the boot log. Keep the
chassis level during calibration, then check for fresh B4 readings, `mission=1`
and changing encoder counts when motor power is applied. During warmup,
`VC-BOOT-WAIT` identifies the blocking sensor and `VC-BOOT` confirms START.
If IMU faults or
multi-second `S5-POLL-GAP` messages persist, save the new boot log; do not
interpret clear ultrasonic readings as proof the IMU recovered. Wi-Fi still
requires the configured SSID to be available on 2.4 GHz.

The supplied ten-page Project Briefing has been reviewed. See
[BRIEFING_COMPLIANCE.md](BRIEFING_COMPLIANCE.md) for the requirement-to-code
mapping and remaining physical evidence. These steps verify the integrated
implementation. Host tests cannot certify physical tracking, wiring or
obstacle clearance.

## 1. Repeat the automated checks

Open PowerShell in `C:\Users\lucas\Downloads\INF2004-Project`:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_robot.ps1
powershell -ExecutionPolicy Bypass -File tools/build_robot.ps1 -Network
```

The test runner saves a transcript in `testing/integration/logs/` and stops
on the first failing suite. The build uses the existing local SDK and MSYS2.
`-Network` uses your local `config/wifi_secrets.h`; verify its WiFi and broker
settings locally. Do not publish that file. Firmware:
`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
For USB-only operation omit `-Network`; flash `mtk3pico_smp0_usb_cdc.uf2`.
The USB-only image cannot connect to the dashboard.

## 2. Flash with the wheels raised

1. Switch motor power off and support the chassis with both wheels clear.
2. Hold BOOTSEL, connect USB, release BOOTSEL, and copy the newly built UF2
   to `RPI-RP2`. The drive disappears when the board reboots.
3. Open its new COM port in a serial monitor at 115200 baud and enable logging.
4. Keep the chassis level and still until IMU calibration completes. Expect
   recurring `[ROBOT-HEALTH]`, `[LINE]`, and `[S5-LIVE]` output. Repeated
   `STARTUP FAILED` is a failure: save its error and stop testing.
5. This profile can autostart when inputs become ready. Keep the wheels raised
   throughout setup. Use dashboard STOP before moving sensors by hand.

## 3. Subsystem 1: dashboard and commands

1. Put laptop and Pico W on the configured WiFi. Start the configured MQTT
   broker with TCP port 1883 and WebSockets port 9001. The listener setup is
   in `tools/dashboard/README.md`.
2. Open `tools/dashboard/index.html`, use `ws://localhost:9001` and prefix
   `inf2004/g25/car` unless your local configuration specifies another prefix.
3. Connect to the real robot (leave Demo data off). Confirm live telemetry age,
   link state and acknowledgements; send STOP and confirm both wheels stop.
4. Test START, PAUSE, RESUME, STOP and E-STOP on the stand. RESUME must not
   release E-STOP; explicit START releases it. Acknowledgement alone is
   insufficient: observe the wheels and controller state too.
5. Disconnect WiFi for 30 seconds, restore it and confirm reconnection and
   renewed telemetry. Send STOP again and confirm its physical effect.
6. Run telemetry for ten minutes, recording frame gaps, drops and command
   round-trip times. The briefing sets no numerical latency or frame-loss
   limits; record the actual results and any agreed team acceptance target.

## 4. Subsystems 2 and 3: encoder and line setup

1. With motor power off, rotate each wheel forward by hand. Its corresponding
   encoder total must increase; the other wheel's total should stay still.
   Ten measured revolutions give counts/revolution = count difference / 10.
   Compare against `ENC_LEFT_COUNTS_PER_REV` and `ENC_RIGHT_COUNTS_PER_REV`
   in `app_program/subsystem2/encoder_control.h`; correct measured values and
   rebuild if necessary. Wrong resolution produces unequal physical speeds.
2. Slide the course under each IR sensor at its actual operating height so
   each sees white and solid black several times. Do this after every reboot,
   with the robot stopped and motor power off. Hold each surface under each
   sensor for at least 0.1 seconds, without changing sensor height. Look for
   `[B3-CAL] learned GP26 white=... black=...; GP27 white=... black=...`.
   The current fixed profile expects black to produce LOWER raw ADC than
   white (black <=1500, white >=1999). Valid low black readings below 300
   are accepted; zero remains excluded. Record actual readings for each
   surface and sensor. `[B3-CAL] observed` reports extrema and does not
   change the fixed thresholds. Opposite polarity or inadequate contrast
   needs correction before a floor test.
3. GP26 is physically the RIGHT line sensor; GP27 is physically the LEFT.
   Confirm by covering one sensor at a time and observing dashboard raw ADC.
   `[LINE]` in USB logs contains normalized controller values (0 white,
   1000 black); the dashboard bars contain raw ADC values.
4. Leave a sensor on the same grey/edge surface for at least one minute.
   Its normalized value should remain stable apart from sensor noise. The
   updated calibration preserves observed endpoints instead of shrinking
   the range while stationary. Reboot and resweep if light or surface changes.
5. Put the line under the sensors and provide valid front clearance. Apply
   motor power on the stand, send SPEED 10 (or use the speed control), then
   START. Both wheels must rotate physically forward. Place the line toward
   GP26: left wheel should run faster; toward GP27: right should run faster.
   STOP immediately if the direction is wrong; correct wiring/mapping and
   rebuild before proceeding.

## 5. Line-following floor test

1. Start with a flat straight line, then broad left/right curves. Keep a
   reachable power switch and clear test area. Perform the stand calibration
   first, send STOP, then lower the robot and send START at low speed.
   Confirm startup marker `B5-TURN45-CLEAR-FORWARD`. In clear space the VC
   should stay in FOLLOW instead of alternating FOLLOW/SCAN. Isolated
   invalid echoes have a 150 ms grace only while previously clear and
   already following; they never refresh the clearance watchdog. Actual
   close readings still stop immediately. For a remaining interruption,
   save `[VC-CLEARANCE-WAIT]`, `[S5-FRONT-INVALID]`, and `[S5-POLL-GAP]`.
2. Record video and serial/dashboard logs. Run the straight and both curves
   three times. Require no departure from the line and no oscillation growing
   from side to side. Record actual speed and line width with each result.
3. If it steers away, repeat the GP26/GP27 and wheel direction check. If it
   drifts on a straight despite balanced IR readings, verify physical wheel
   speed and encoder resolution before changing steering gain.
   Straight forward DRIVE now applies `ROBOT_STRAIGHT_TRIM_PERMILLE=100`
   in `robot_config.h` to counter the reported left drift: left travel
   target +10%, right -10%. Confirm `straight_trim=100` in the startup log.
   This is the midpoint of 80 (left drift) and 120 (slight right drift).
   Encoder synchronization follows that ratio rather than cancelling the
   trim. Commanded curves, in-place turns, reverse, and distance MOVE legs
   retain their existing profiles. Validate the correction on the floor;
   this value is a tuning trial, not a measured wheel calibration.
4. If it oscillates with correct mapping, reduce requested speed first.
   Steering constants are in `b3_line_barcode.h`; change one at a time and
   repeat the same course. Minimum sustainable wheel speed is provisional
   (`VC_MIN_WHEEL_CPS`), so low percentages still have a speed floor.
   This image uses the weighted-position PD controller from `line-follow-pd`.
   The fixed trim trials have been removed. `B3_LINE_KP_X10` and
   `B3_LINE_KD_X10` are the PD tuning settings; verify physical sensor,
   encoder and motor mapping before tuning gains.
5. Remove the line with the path ahead clear: the robot should drive straight
   at reduced speed for more than eight seconds, then use line steering when
   tape reappears. Send STOP and verify it stays stopped even with a clear
   path. Send STOP before repositioning it.
   Brief ultrasonic clearance stops must not restart this timer or the
   initial turn direction. With no line, repeated clearance interruptions
   must still exhaust the search and leave both wheels stopped.
   If it consistently turns one way, use `[LINE-RAW]` with motors off to
   record GP26 and GP27 on white and black separately. The current tracking
   uses fixed low-ADC-is-black thresholds; `[B3-CAL] observed` reports raw
   extrema only and does not calibrate those tracking thresholds.
6. Test Code 39 A and Z in both travel orientations. A queues STRAIGHT and Z
   queues RIGHT; the action executes at a confirmed junction. Check the event
   and the physical route. Other letters decode but have no configured route.

## 6. Subsystem 5: stop, scan and bypass

1. On the stand, move a large flat box from beyond 25 cm to below 20 cm.
   Require both wheels to stop on a valid close reading, then observe coarse
   and fine scans. Verify servo direction, measured distance and scan results.
2. ECHO must be 3.3 V safe. A normal 5 V HC-SR04 needs level shifting; confirm
   common ground and adequate motor/servo power before testing.
3. Current configuration assumes open space for a missing echo. Disconnecting
   ECHO therefore is not a valid stop test in this profile. For a strict
   sensor-failure test, set BOTH `ROBOT_NO_TARGET_FORWARD` and
   `ROBOT_NO_TARGET_SCAN` to 0 in `robot_config.h`, rebuild, reflash and verify
   missing returns prevent motion. Record which policy was tested.
4. Before floor bypass, measure chassis geometry, obstacle depth and width,
   counts/revolution, and completed 45-degree turns. Current bounds assume
   depth <=40 cm and width <=30 cm; turn scales are provisional. Servo travel
   is restricted to the original 70/80/90/100/110-degree coarse scan.
5. On a controlled course, test one left bypass and one right bypass. Require
   stop before contact, plausible scan, clearance for the whole chassis,
   bounded travel, line reacquisition and resumed following. An unobservable
   route must stop. Exhausted tape search restores forward heading and uses
   clear-path forward driving. Do not score simulation as a physical
   bypass pass.
   The sequence is: STOP -> coarse/fine profile (chassis-assisted view if needed)
   -> turn out -> travel the width-based offset -> stop -> turn the opposite direction
   to face forward -> pass the depth bound -> turn inward -> search
   across the black line -> STOP -> face forward -> follow tape if detected,
   otherwise continue straight while the path is clear.
   A usable coarse profile now commits directly to the turn: fine scanning
   only runs if the coarse sweep cannot produce a route. Each scan angle
   uses three agreeing pings and 200 ms settling. Servo movements are
   gentle 2-degree steps every 35 ms to reduce mount vibration. A blocked
   route waits about three seconds before repeating the sweep. A usable
   route still turns immediately after the first coarse sweep.
   The chooser compares both candidate side openings before committing;
   a larger left opening must not be ignored just because the right side
   has two clear rays. `[S5-CHOICE]` reports the direction and side ranges.
   During turns the servo stays forward while ultrasonic checks continue
   about every 65 ms as the chassis changes heading. `[S5-TURN-GUARD]`
   reports these checks; a close or invalid reading must stop the turn.
   If the fine scan clips the obstacle, a course-bound route may use the
   configured 30 cm width and 40 cm depth bounds. It requires a clear side
   ray to select a turn direction. The sideways leg is encoder measured;
   both it and the subsequent pass require head-on clearance for the next
   movement plus chassis length and margin after turning. When the whole
   leg is not visible, advance in steps of at most 5 cm and recheck.
   `[S5-ROUTE] course_bounds=1` identifies this route; telemetry reports its
   width as unknown rather than claiming the bound is a measurement.
   Both the inner (80/100-degree) and outer (70/110-degree) rays can supply
   an opening. A valid side reading inside the 15 cm turn clearance blocks
   that side. Openings within 10 cm of each other prefer LEFT. The
   candidate opening threshold is now 25 cm, separate from the 40 cm
   silhouette classification. Scanning retains the obstacle latch through
   the 20..25 cm stop/release band. Minimum turn clearance is 15 cm;
   verify the chassis and completed turn clearance on the actual course.
   Every forward leg retains live front checks and stops on a close or
   invalid reading. Obstacle handling never requests the old reverse fallback.
   If neither side is open, expect `WAIT_ROUTE`: it stays stopped and
   retries the scan. Three clear front readings after obstacle removal
   allow line following to resume without rebooting.
   The first fresh tape detection brakes immediately. Three distinct line
   samples while physically stopped must confirm it before the alignment
   turn; a transient detection resumes the remaining bounded search.
   Search continues up to 40 cm past the predicted crossing, in 2 cm legs.
   If the alignment turn moves the sensors off the tape, a bounded
   +/-15/30/45-degree heading sweep tries to reacquire it before resuming.
   Watch `[S5-ROUTE]` for the selected side and planned distances, and
   `[S5-MOVE]` for each encoder-tracked leg. No line or no clear route means
   STOP, not unlimited travel.
   On handoff, the vehicle controller clears its pre-obstacle steering and
   speed history and accelerates from fresh sensor readings. Also test an
   obstacle encountered while searching for a lost line: after bypass, a
   still-missing line must receive a fresh eight-second recovery window.

   Reference behavior and implementation mapping:
   [video review](testing/integration/VIDEO_REFERENCE_NsEfamcjgF8.md).
6. During bypass test PAUSE, STOP and E-STOP separately. Each must stop actual
   motion; RESUME must not release E-STOP. Reverse requires an independent
   rear-clearance provider and remains unavailable without one.

For each physical test save date, firmware SHA256, settings, expected result,
observed result and log/video reference in the relevant `testing/` TEST_LOG.
The older `README_START_HERE_REAL_ROBOT.md` describes an obsolete forward-only
profile; use this guide for the currently built firmware.
