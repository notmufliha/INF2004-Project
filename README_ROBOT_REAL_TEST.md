# INF2004 — real robot integration from the 21 Oct uploaded ZIP

This is a **guarded real-hardware milestone**, not an end-to-end autonomous bypass.
It integrates B2's motor PWM and B5's real ultrasonic/servo scanning, with a
motor stop request on the first close echo. B5's coarse/fine scan, obstacle
profile and side selection are real. The uploaded B2 and B3 files are still
teaching sample producer/consumer tasks, NOT encoder motor control and IR line
following. Consequently no physical bypass, reversing, line search, or line
reacquisition is enabled. No program can verify these on the supplied code.

## IMPORTANT: confirm your actual board and wiring

* Raspberry Pi Pico WH + Maker Pi Pico Base or other motor driver: **verify the
  physical driver pin mapping before enabling the wheels**. The code assumes
  **GP8/GP9 -> left motor driver direction inputs**, **GP10/GP11 -> right**.
  The Pico GPIOs drive the **motor driver INPUTS**, never the motors directly.
  The driver and motors require a suitable separate motor supply and common
  GND. If your board uses different pins, edit `app_program/robot_config.h`.
  Do not connect two drivers to the same motors or GPIOs.
* Ultrasonic **TRIG GP16**, **ECHO GP17**, GND common, VCC according to sensor.
  Standard 5V HC-SR04 ECHO is **5V**: use a resistor divider or level shifter
  before GP17. Only a sensor explicitly rated for 3.3V GPIO may connect ECHO
  directly. **No ECHO connection means no distance readings and no driving.**
* Servo signal **GP12**, servo power from an appropriate regulated supply,
  servo GND tied to Pico GND. Never power the servo from Pico GP12 or 3V3.
  Fix the ultrasonic sensor securely to the servo so it rotates with it.
* Test with a stable battery, a nearby power disconnect, a wide open area,
  and someone ready to cut motor power. Avoid stairs, people, pets and roads.

## Configuration: `app_program/robot_config.h`

- `ROBOT_MODE 1`: only the B2 motor task and B5 sensor task are started. The
  old S1-S4 demo tasks remain in the project but don't run in this mode.
- `ROBOT_ENABLE_DRIVE 0`: initial state; **all motor PWM stays zero**.
- `ROBOT_ENABLE_DRIVE 1`: forward PWM after three valid CLEAR readings.
  Any close reading (<35cm), invalid echo, or ambiguous reading stops PWM.
  Three valid readings >=45cm are needed before forward resumes.
- `ROBOT_ENABLE_BYPASS 0`: mandatory until real encoders and IR line tracking
  are implemented; a build-time guard rejects 1.
- `ROBOT_FORWARD_PWM 240` out of `ROBOT_PWM_TOP 999` (24% duty).
  Start with wheels raised. Reverse a motor's two GPIO assignments if it
  spins backward; DO NOT swap wires while powered.
- `ROBOT_MOTOR_WATCHDOG_MS 900`: if the B2 task receives no fresh forward
  request for 900ms, it sets motor PWM to zero. This is a SOFTWARE watchdog;
  it cannot protect against a stalled kernel or broken motor driver.

**A 0% PWM stop acknowledgement means PWM outputs were set to zero; it does
not verify physical wheel velocity. There are no encoder readings yet.**

## Install

1. Back up the current `INF2004-Project` folder.
2. Extract `INF2004_21Oct_RealRobot_ChangedFiles.zip` **into the project root**,
   merging `app_program/` and `testing/`. Keep the folder structure.
3. Open `app_program/robot_config.h`, check the motor GPIO pins against the
   actual robot, and leave `ROBOT_ENABLE_DRIVE 0` for the first test.
4. Leave `S5_SIMULATION 0`, `S5_HARDWARE_ENABLED 1`, `S5_SERVO_ENABLED 1`,
   `S5_MOTION_BRIDGE_ENABLED 0`, `S5_SERVO_PIN 12` in `subsystem5.h`.
5. Do not use `TEST=1` for the robot drive firmware.

## Build in WSL, from VS Code terminal

```bash
cd /mnt/c/Users/lucas/Downloads/INF2004-Project
bash testing/subsystem5/run_host_tests.sh
cd build_make
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 clean
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 -j8
ls -lh mtk3pico_smp0_usb_cdc.uf2
```

If the compiler prints an `error:`, fix that before flashing; never flash an
old `.uf2` after a failed build. The expected image is in `build_make/`.

## Flash

1. **Power OFF the motor and servo supply** while checking wiring.
2. Connect the Pico USB cable while holding BOOTSEL. Release when `RPI-RP2`
   appears in Windows Explorer.
3. Copy `build_make/mtk3pico_smp0_usb_cdc.uf2` to `RPI-RP2`. The drive should
   disappear automatically.
4. Reconnect normally (without BOOTSEL) and open VS Code Serial Monitor,
   choose the Pico's COM port, 115200 baud, and start monitoring.
5. Power the motor/servo supplies only after wiring checks are complete.

## Physical test A — real sensor and servo, no motors

1. Keep `ROBOT_ENABLE_DRIVE 0`; build and flash as above.
2. Confirm logs `[ROBOT] Guarded real-hardware mode; drive=0` and
   `[B2-MOTOR] ... drive_enabled=0`.
3. Servo self-test should move 90 -> 60 -> 120 -> 90 degrees. If it does not,
   verify GP12 signal, servo power and common ground.
4. Place a flat box at about 20-30 cm straight ahead. Three close readings
   should cause `[S5-STOP]`, `[S5-COARSE]`, `[S5-FINE]`, `[S5-PROFILE]`,
   and `[S5-PLAN]` logs. Servo must visibly rotate during scanning.
5. Remove box. With a wall/background inside the ultrasonic sensor's useful
   range, expect valid clear readings. With no echo the code fails safe:
   it treats the path as UNKNOWN and does NOT allow driving.
6. Confirm motor outputs never energize.

## Physical test B — wheels OFF the floor, forward and stop

1. Disconnect power. Secure the robot on a stable stand with both drive wheels
   freely suspended. Verify no loose wires can reach moving wheels.
2. Set `ROBOT_ENABLE_DRIVE 1`, keep `ROBOT_ENABLE_BYPASS 0`.
3. Rebuild (use `make ... clean` then `make ...`), reflash the NEW UF2.
4. With a clear valid forward reading >=45cm, expect `[B2-MOTOR] FORWARD`;
   both wheels must rotate in the intended forward direction.
5. Put a box 20-30cm in front of the ultrasonic sensor; expect
   `[B2-MOTOR] STOP` and BOTH wheels to stop, then a servo scan.
6. Block ECHO or disconnect sensor ONLY after removing motor power, then
   restart: invalid readings should keep motors stopped.
7. If a wheel rotates backwards, power off and correct the forward/reverse
   motor GPIO mapping in `robot_config.h`, rebuild and retest.
8. If a wheel never turns, check driver motor power, enable pins/jumpers,
   GPIO mapping, and common GND. A printed FORWARD command alone is not proof
   of correct physical motor operation.

## Physical test C — slow straight-line floor trial

ONLY after Test B passes. Choose an open flat floor with no drop-offs,
place a large stable box in the path, and have a power cut-off within reach.
Keep `ROBOT_ENABLE_BYPASS 0`. Set `ROBOT_FORWARD_PWM` low enough for slow
movement. Start with the box >45cm away; the robot should drive straight.
When it approaches the stop threshold, it should stop, scan, profile and
remain stopped until the path is verified clear again. Move the box away
and verify forward motion resumes after three valid clear readings.

The robot may drift because encoder closed-loop correction is NOT in this
uploaded project. Never assume a fixed 24% duty makes both wheels travel at
identical speed. Stop if the robot veers, overshoots, or becomes unstable.

## What is still needed for the FULL diagram

1. **Buddy 2:** actual encoder GPIO interrupts, wheel-speed PID, distance and
   turn primitives, motion-complete and stop acknowledgements. Calibrate
   wheel circumference, counts/revolution, track width, motor polarity.
2. **Buddy 3:** actual line IR ADC/digital drivers, calibrated line detection,
   line-following, search and reacquisition confirmation.
3. **Vehicle Controller:** sole arbiter of B2 motion requests and B3 steering;
   STOP preempts all; explicit bounded timeout on each motion.
4. **Buddy 5:** bridge those real APIs to its existing `S5_CMD_GO_LEFT/RIGHT`,
   `S5_CMD_REVERSE`, `S5_CMD_FIND_LINE`, `S5_CMD_RESUME` states; test with
   wheels raised, then measured obstacle geometry, and only then on floor.

Do NOT change `S5_MOTION_BRIDGE_ENABLED` to 1 or
`ROBOT_ENABLE_BYPASS` to 1 to try to bypass these dependencies.

## Expected log fragments (illustrative)

```text
[ROBOT] Guarded real-hardware mode; drive=1 bypass=0
[B2-MOTOR] pins 8/9 10/11; PWM 1kHz; drive_enabled=1; watchdog=900ms
[S5-BOOT] Buddy 5 task running, hardware profile GP16/17/12
[S5-DETECT] sample=... cm_x100=... threshold_cm=35 hits=0/3
[B2-MOTOR] FORWARD duty=240/999
[S5-DETECT] sample=... cm_x100=2200 threshold_cm=35 hits=1/3
[B2-MOTOR] STOP command/watchdog
[S5-STOP] obstacle=1; motor PWM=0 acknowledged
[S5-COARSE] sweep=30,60,90,120,150 (real servo)
[S5-FINE] center_deg=... sweep=+/-20 deg in 10 deg steps
[S5-PROFILE] valid=... width_cm_x100=... side=...
[S5-PLAN] direction=... action=BYPASS_REQUEST_ONLY
[S5-RECOVER] physical bypass NOT enabled: no calibrated encoder/IR bridge
```
