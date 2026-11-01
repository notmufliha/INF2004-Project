# INF2004 real robot: motor diagnostic + Buddy 5

## What this version actually implements
- Buddy 2: two-motor PWM, single motor owner, 1.2-second command watchdog, explicit RP2040 PWM reset release.
- Buddy 5: GP16/GP17 ultrasonic, GP12 servo, startup servo test, stop on first valid close reading, stop on invalid reading, coarse scan, fine scan, width/clearance estimate, suggested bypass side, forward again only after three valid clear readings.
- The motor direction is taken from the team's `motor_control.c` reference: M1 forward GP9, M2 forward GP10. If your physical wiring differs, swap that motor's forward and reverse pins in `app_program/robot_config.h`.
- Buddy 2/3 in the uploaded project do NOT provide verified encoder-controlled turning, line sensing and route validation. Autonomous *physical bypass + line reacquisition* is deliberately disabled. The code reports the suggested direction but does not blindly drive around an obstacle. No guarantee of straight tracking without encoder correction.

## FIRST: motor diagnostic (do this if wheels never moved)
1. Battery OFF. Put robot on stable stand so BOTH wheels are lifted. Keep fingers, cables and clothing clear.
2. Open `app_program/robot_config.h` and set `ROBOT_WHEELS_UP_MOTOR_TEST 1`. Keep `ROBOT_ENABLE_DRIVE 1` and `ROBOT_ENABLE_BYPASS 0`.
3. Rebuild and flash (commands below). Keep the robot raised. Connect USB to a PC for serial diagnostics *only if your board supports USB + battery simultaneously*.
4. Power robot. The motor task waits 3 seconds, drives LEFT only for 1.2 s, stops, drives RIGHT only for 1.2 s, stops, drives BOTH for 1.2 s, then remains stopped. This diagnostic ignores obstacle distance, so NEVER use on the floor.
5. Check serial `[B2] PWM READY ...` and `[B2-TEST] LEFT only / RIGHT only / BOTH forward`.
6. If nothing turns but serial logs show the pulses, verify motor supply voltage at the motor driver, power switch, M1/M2 terminals, motor driver type, and correct GPIO assignments. Do not keep increasing PWM indefinitely. If only one wheel turns, check the other motor's connection and pin pair. If one wheel turns backward, swap only that motor's FWD and REV pin defines.
7. **Set `ROBOT_WHEELS_UP_MOTOR_TEST 0` and REFLASH before any floor test.**

## SECOND: actual ultrasonic-controlled movement
1. Set `ROBOT_WHEELS_UP_MOTOR_TEST 0`, `ROBOT_ENABLE_DRIVE 1`, `ROBOT_ENABLE_BYPASS 0`.
2. Verify GP16 TRIG, GP17 ECHO, GP12 servo; standard 5V HC-SR04 ECHO MUST be level shifted to 3.3V for Pico. A documented 3.3V-compatible HC-SR04P can be used per its datasheet. Use an adequately regulated servo supply and common ground.
3. Place robot on stand; put no obstacles in front (clear path >=45 cm and within ultrasonic range). After startup servo sweep and three valid clear measurements, BOTH wheels should spin forward. Put a box at ~25 cm: both wheels should stop, servo coarse scan (30,60,90,120,150), fine scan, print width and side. Remove box: after 3 valid clear measurements, both wheels resume.
4. Only after this passes, try on a flat floor in an open area at low speed, with a reachable battery disconnect. Start with a large stable box at least 60 cm ahead. Stop distance is configured to 35 cm, release at 45 cm; tune from real measured stopping distance.
5. If serial says `ECHO_INVALID`, check ultrasonic power and echo voltage; the robot intentionally does not move. If `[B2] status=STOPPED` despite valid clear readings, check B5 prints and watchdog. If `[B2] FORWARD` but motors don't turn, diagnose hardware power/wiring/PWM using the motor test.

## Build and flash from WSL
```bash
cd /mnt/c/Users/lucas/Downloads/INF2004-Project
bash testing/subsystem5/run_host_tests.sh
cd build_make
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 clean
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 -j8
ls -lh mtk3pico_smp0_usb_cdc.uf2
```
Flash the NEWLY BUILT `.uf2` via BOOTSEL. The ZIP is source-only; no UF2 is included because an old UF2 would not contain the new changes.

## Battery-only clarification
USB powering the Pico does not guarantee the motor supply is energized; conversely a battery connected only to the motor driver may not power the Pico. With battery-only, you cannot read USB serial until you also connect a supported USB diagnostic connection. Confirm the Pico LED and servo startup; if neither happens, diagnose power distribution before changing code. Use the Maker Pi Pico Base documentation for battery voltage range, switch and motor terminals. Never connect battery voltage directly to GPIO or Pico 3V3.

## Expected serial
```
[ROBOT] Guarded real-hardware mode; drive=1 bypass=0
[B2] PWM READY M1 GP9/GP8 M2 GP10/GP11 duty=350/999 drive=1 diagnostic=0
[S5-SERVO] SELF TEST 90 -> 60 -> 120 -> 90
[S5-DETECT] sample=... cm_x100=... threshold_cm=35 hits=0/3
[B2] FORWARD left=350 right=350
[S5-STOP] obstacle=1; motor PWM=0 acknowledged
[S5-COARSE] sweep=30,60,90,120,150 (real servo)
[S5-FINE] center_deg=... sweep=+/-20 deg in 10 deg steps
[S5-PROFILE] valid=... width_cm_x100=... side=...
```
The lines above are examples, not recorded measurements from the robot.

## Verification performed in this environment
- 14/14 Buddy 5 host simulation tests passed.
- ARM Cortex-M0+ clang syntax checks passed for modified application C sources.
- Full firmware link/build and physical motor tests were NOT available here. A successful build and on-robot calibration are still required.
