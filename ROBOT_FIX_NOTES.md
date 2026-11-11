# Integrated robot firmware

The default profile now starts line following automatically once B2 motor
feedback, B3 line samples and B4 IMU data are fresh. B4 must first finish its
level/still calibration. B5 then requires three clear front readings and an
established line reference before it releases motor permission.

The vehicle controller uses weighted line-position PD steering, curve
slowdown, widening line search and bidirectional Code 39 decoding. B5 owns
obstacle bypass and its line recovery; B2 remains the only motor PWM owner.

## Build and flash

From PowerShell in the project directory:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_robot.ps1
```

Flash `build_make/mtk3pico_smp0_usb_cdc.uf2` with BOOTSEL. The default script
builds USB diagnostics with networking off. It uses this machine's existing
MSYS2, Arm compiler and `.codex_tmp/pico-sdk`, or the SDK selected using
`-SdkPath` / `PICO_SDK_PATH`. A build does not itself flash the board.

Use `-Smp 1` for the dual-core USB image. For real telemetry, configure the
gitignored `config/wifi_secrets.h` from the example and build with `-Network`.
`-ExampleNetwork` checks the WiFi/lwIP/MQTT build using example credentials;
its `_example.uf2` is for compilation verification, not a configured network.
A bare `make` also builds offline when credentials are absent.

After flashing, open the newly enumerated USB COM port at 115200 baud. Its
number may change from COM6. Expect `[ROBOT-HEALTH]` and `[LINE]` every two
seconds, even when the terminal opens after boot. The health line includes
mission permission, B5 ownership (0=FOLLOW, 1=BUSY, 2=FAULT), motor status and
encoders. `[S5-LIVE]` reports the front echo and controller state once per
second. Repeated `[ROBOT] STARTUP FAILED` means a subsystem failed to start.

## Follow, avoid, reacquire

1. Follow the black line with the existing B3/VC steering. While moving,
   remain authorized above 35 cm; require 45 cm to restart after a STOP.
2. Stop immediately on a close reading. Wait for encoder-confirmed standstill.
3. Scan five coarse rays, then five fine rays around the obstacle. Estimate
   obstacle location, width and lateral clearance.
4. Select a clear side. Turn out, offset sideways, restore the travel heading,
   pass the configured obstacle-depth bound, then turn toward the original line.
5. Turn toward the line and search in bounded 2 cm legs, continuing up to
   40 cm beyond the predicted crossing. After leaving the original line,
   accept three fresh black detections anywhere in that corridor, including
   broad tape that sets the junction flag. Stop and restore forward heading.
6. Reconfirm the line while stopped. If rotation moved the sensors off it,
   sweep headings +/-15, +/-30 and +/-45 degrees relative to the restored
   heading. Three fresh detections complete the handoff to PD line following.
   Fresh front clearance is required again before following resumes.

Every movement uses its own B2 command result, not elapsed time alone. STOP,
ESTOP, PAUSE, zero speed, stale inputs, IMU hazards and motor faults can stop
B5 movement too. Failed searches or unobservable routes stop the robot.
Reverse remains unavailable without an independent rear-clearance provider.

## Why a clear path previously prevented movement

- Startup required remote START, which was unavailable in an offline build.
- The physical motion bridge was a set of placeholders and bypass was disabled.
- Open-space interpretation differed between the front guard and consensus.
- Long ultrasonic batches could outlast the safety heartbeat.
- The release threshold stopped the car before reaching the scan threshold.

The live guard now runs once per approximately 65 ms cycle. VC refreshes its
DRIVE commands every 20 ms independently. The 100 ms DRIVE watchdog and the
350 ms clearance watchdog remain enabled. Same-direction PWM updates keep
the active output continuous; direction changes clear the opposite input.

## Open-space policy and calibration

`ROBOT_NO_TARGET_FORWARD=1` treats NO_RISE and a completed out-of-range pulse
as **assumed open space**, logged as `OPEN_SPACE_ASSUMED`. Scans can use the
same assumption for clear boundary rays with `ROBOT_NO_TARGET_SCAN=1`, while
retaining three-reading consensus. A real close echo overrides front clearance;
a close dissenting scan echo cannot authorize a clear consensus. Stuck-high,
unfinished and malformed pulses remain faults. Unlike the previous code, an
unfinished high pulse is never converted to clearance.

A disconnected ECHO wire can look like NO_RISE. This policy cannot detect
that fault or guarantee detection of soft/angled objects. Set both options
to 0 to require return echoes, and verify that a real box produces a close
reading before putting the robot on the floor.

Current hardware assumptions:

| Connection | GPIO |
|---|---|
| Left encoder A/B | GP0 / GP1 |
| Right encoder A/B | GP2 / GP3 |
| Motor left forward/reverse | GP9 / GP8 |
| Motor right forward/reverse | GP10 / GP11 |
| Ultrasonic TRIG / ECHO | GP16 / GP17 |
| Servo | GP12 |
| B3 analog line channels | GP26 / GP27 |
| B3 digital barcode sensor | GP28 |
| B4 I2C SDA / SCL | GP4 / GP5 |

B3 normalizes each sensor using its observed white/black endpoints once
at least 64 ADC counts of contrast have been seen. Sweep both sensors over
white and black while stopped after every reboot. Learned endpoints no longer
shrink while following or parked; normalized values >=201 mean line-active.
Three overlapping samples confirm calibration extremes and reject isolated
spikes. Until the range is learned, the PD branch fallback is 220..620 raw
ADC, with raw 301 as the first line-active reading. This fallback is not
measured calibration for a particular robot. `[B3-CAL] learned ...` prints
both sensors' first accepted ranges after a sweep.
The dashboard shows raw ADC; USB `[LINE]` shows normalized values.
Its historical channel names
are physically reversed: the channel named `left` is mounted on the right.
Retain that mapping unless the wiring differs. Motor pin definitions now
actually control the driver. UART0 console/mirroring is rejected in robot
mode because GP0/GP1 belong to the encoder.

Servo travel remains 70..110 degrees for the existing restricted mount. This
may not observe both edges of a wide obstacle. Bypass assumes obstacle depth
at most 40 cm and width at most 30 cm, with provisional chassis geometry and
turn calibration in `robot_config.h` and `encoder_control.h`. These bounds
must match the course; the ultrasonic sensor does not measure hidden depth.

## Verification and remaining hardware work

Verified: all 16 B5 host test groups; VC handoff, delayed-sample autostart,
remote STOP and hazards; bidirectional barcodes; B2's 1158 host assertions;
B1 wire/reconnect/dashboard tests; B4 IMU/terrain tests. USB firmware builds
for both SMP profiles, and the WiFi/lwIP/MQTT example profile builds.

No COM port was visible during verification. Physical flashing, motor power,
sensor polarity, PID/turn calibration, stopping distances and course recovery
remain unverified. First check motor direction, encoder counts and IR values
with the wheels raised, then perform a controlled floor test with an accessible
power switch. Use a 3.3 V-safe ECHO connection and a common ground.
