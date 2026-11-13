# Subsystems 1, 2, 3 and 5: build and test

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
   Black must produce the higher
   raw ADC reading; measured contrast must be at least 64 counts and clearly
   greater than the stationary noise on each surface. Opposite
   polarity or inadequate contrast needs correction before a floor test.
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
2. Record video and serial/dashboard logs. Run the straight and both curves
   three times. Require no departure from the line and no oscillation growing
   from side to side. Record actual speed and line width with each result.
3. If it steers away, repeat the GP26/GP27 and wheel direction check. If it
   drifts on a straight despite balanced IR readings, verify physical wheel
   speed and encoder resolution before changing steering gain.
4. If it oscillates with correct mapping, reduce requested speed first.
   Steering constants are in `b3_line_barcode.h`; change one at a time and
   repeat the same course. Minimum sustainable wheel speed is provisional
   (`VC_MIN_WHEEL_CPS`), so low percentages still have a speed floor.
   This image uses the weighted-position PD controller from `line-follow-pd`.
   The fixed trim trials have been removed. `B3_LINE_KP_X10` and
   `B3_LINE_KD_X10` are the PD tuning settings; verify physical sensor,
   encoder and motor mapping before tuning gains.
5. Remove the line briefly: recovery should first steer hard toward the last
   seen side for one second, then alternate in widening sweeps and stop after
   eight seconds total. It stays armed
   and may resume when a line reappears. Send STOP before repositioning it.
6. Test Code 39 A and Z in both travel orientations. A queues STRAIGHT and Z
   queues RIGHT; the action executes at a confirmed junction. Check the event
   and the physical route. Other letters decode but have no configured route.

## 6. Subsystem 5: stop, scan and bypass

1. On the stand, move a large flat box from beyond 45 cm to below 35 cm.
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
   counts/revolution, and completed 90-degree turns. Current bounds assume
   depth <=40 cm and width <=30 cm; turn scales are provisional. Servo travel
   is restricted to 70..110 degrees and may not observe both obstacle edges.
5. On a controlled course, test one left bypass and one right bypass. Require
   stop before contact, plausible scan, clearance for the whole chassis,
   bounded travel, line reacquisition and resumed following. An unobservable
   route or exhausted search must stop. Do not score simulation as a physical
   bypass pass.
   The sequence is: STOP -> coarse/fine profile -> turn out -> lateral
   offset -> turn forward -> pass the depth bound -> turn inward -> search
   across the black line -> STOP -> face forward -> confirm line -> resume PD.
   Search continues up to 40 cm past the predicted crossing, in 2 cm legs.
   If the alignment turn moves the sensors off the tape, a bounded
   +/-15/30/45-degree heading sweep tries to reacquire it before resuming.
   Watch `[S5-ROUTE]` for the selected side and planned distances, and
   `[S5-MOVE]` for each encoder-tracked leg. No line or no clear route means
   STOP, not unlimited travel.
6. During bypass test PAUSE, STOP and E-STOP separately. Each must stop actual
   motion; RESUME must not release E-STOP. Reverse requires an independent
   rear-clearance provider and remains unavailable without one.

For each physical test save date, firmware SHA256, settings, expected result,
observed result and log/video reference in the relevant `testing/` TEST_LOG.
The older `README_START_HERE_REAL_ROBOT.md` describes an obsolete forward-only
profile; use this guide for the currently built firmware.
