# Buddy 2 standalone movement demo

From `build_make` in the project's configured MSYS/build shell:

```sh
make B2_DEMO=1 SMP=0 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 -j8
```

Requires the Arm compiler and `PICO_SDK_PATH`. Flash the newly built
`build_make/mtk3pico_smp0_usb_cdc_b2demo.uf2` via BOOTSEL.
The demo has a separate image suffix and build profile. Normal builds default
to `B2_DEMO=0`; switching profiles invalidates old objects automatically.

**Supervised floor test: no obstacle sensing.** Clear front/rear/sides and
keep motor power accessible. Verify wheel directions with wheels raised first.
The sequence starts automatically five seconds after B2 startup, runs once,
then stops. Reboot to repeat. Unexpected failure aborts without retry.

| Test | Movement |
|---|---|
| 1 | Encoder-based forward 10 cm |
| 2 | Encoder-based reverse 10 cm |
| 3 | Continuous straight DRIVE one second, then STOP |
| 4 | Left 90-degree encoder turn |
| 5 | Right 90-degree encoder turn |
| 7 | Left-steered DRIVE one second, then STOP |
| 8 | Right-steered DRIVE one second, then STOP |
| 9 | Deliberately omit DRIVE refresh; expect WATCHDOG timeout and standstill |

There are three-second pauses. PI control runs throughout movement. Console
logs show command results and encoder travel; bounded-motion progress includes
measured wheel counts/s. Different encoder resolutions mean equal physical
speeds need different raw counts/s. Measure body distance and angle separately:
PASS indicates controller completion, not independently measured accuracy.
The watchdog demonstrates software fault handling, not physical stall detection.
Do not restrain powered wheels to demonstrate a stall.

No Wi-Fi START, line sensors, IMU or ultrasonic sensor is required. Only B2
tasks run; B1 initializes shared fault/telemetry storage without starting its
tasks. B3/B4/B5 and the vehicle controller are not initialized or started.
The demo supplies supervised authorization through B2's existing safety API.
STOP acknowledgement, generation checks, PWM limits, stall/encoder checks,
control timing checks and DRIVE watchdog remain active.

For an individual test, change `ROBOT_MOTION_TEST` in `robot_config.h` to its
test number and rebuild. Restore its conditional default of 10 for the complete
demo. Mode 6 retains the original tests 1..5. Ordinary builds use mode 0.

## Rightward drift tuning trial

The right wheel PI base PWM is 21% versus the left's 20%: a provisional one
percentage point increase for the observed slower right motor. Feedback,
straight-line correction and the 35% ceiling remain active. Turn calibration
scales are unchanged. The bias applies to normal mode and both directions;
retest reverse and both turns as well as forward travel.

Repeat measured runs on the same surface, battery and load. Record lateral
drift, body travel and encoder wheel travel. If encoder distances agree but
the body curves, check encoder calibration, wheel diameter, slip and alignment
before increasing the bias. `PID_RIGHT_BASE_PWM` in `pid_control.c` controls
the trial; 20.0 restores the old symmetric base. Hardware improvement remains
unverified until measured on the car.

Run `testing/subsystem2/host/run_tests.ps1` for controller assertions, the full
demo sequence against a simulated encoder/motor plant, expected watchdog stop,
and both B1-interface suites. Host results do not establish hardware accuracy.
