# Forward / obstacle STOP firmware

The default `ROBOT_FORWARD_STOP_MODE=1` runs both motors forward at 50%
duty after three stable front-distance batches at or above 45 cm. It stops
on the first moving reading at or below 35 cm or a sensor fault. Clearance older
than 350 ms also stops the motors. It resumes after confirmed clearance.
Startup takes several seconds for the servo and ultrasonic sensor to settle.

This mode uses B2's STOP generation and sensor watchdog, without requiring
encoders or PI tuning. B2 owns motor PWM; B5 supplies clearance. The sensor
faces forward while driving. A confirmed obstacle triggers a stopped coarse
and fine servo scan (70–110 degrees), once per obstacle episode. Each scan
returns the sensor to 90 degrees; resuming still requires three confirmed
front-clear batches. Bypass and
motion-command tests are disabled. Motor direction pins and forward duty
are configured in `app_program/robot_config.h`.

Build in Ubuntu WSL:

```sh
cd /mnt/c/Users/lucas/Downloads/INF2004-Project
python3 testing/subsystem2/host/test_forward_stop.py
python3 testing/subsystem2/host/test_continuous_pwm.py
bash testing/subsystem5/run_host_tests.sh
cd build_make
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 -j8
```

Flash `build_make/mtk3pico_smp0_usb_cdc.uf2` through BOOTSEL. First verify
with wheels raised: clear the front sensor, check both wheels turn forward,
then place an object about 25 cm ahead and check both wheels stop. Remove
the object and check they resume. Physical power, wiring, sensor accuracy
and wheel direction still require this on-robot check; software host tests
cannot validate them. Missing echoes intentionally keep the robot stopped.

## B5 USB serial diagnostics

Open the Pico USB serial port at 115200 baud after flashing. B5 continuously
prints diagnostics, including if the monitor was opened after startup:

- `[S5-ECHO]`: each stationary ping's status, actual pulse duration in
  microseconds, distance multiplied by 100, and whether it was filtered.
- `[S5-BATCH]`: too few usable readings or readings that disagree.
- `[S5-CHECK]`: every completed stationary batch and clearance counters.
- `[S5-FRONT]`: periodic front distance while driving.
- `[S5-GUARD]`: the exact echo failure that stopped a moving robot.

`cm_x100=0` in a failed CHECK is an unknown distance, not an object at zero cm.
`ECHO_NO_RISE` means GP17 did not receive an echo pulse. Check the actual
TRIG=GP16 / ECHO=GP17 connections, sensor power, common ground, and a large
flat target 50–100 cm ahead. `ECHO_STUCK_HIGH` or `ECHO_NO_FALL` means the
echo signal stayed high; check sensor power and the echo level shifter.
A 5 V HC-SR04 echo must be reduced to a Pico-safe 3.3 V signal. Valid close
echoes are retained and stop the robot.

## Open-space forward policy

`ROBOT_NO_TARGET_FORWARD=1` is enabled by default as requested. Repeated
NO_RISE results and completed out-of-range pulses are accepted as assumed
open space by the forward controller. Serial prints `OPEN_SPACE_ASSUMED`;
400 cm in this case is a control sentinel, not a measured distance. The
same three-batch restart confirmation applies. A real close echo overrides
assumed-open samples in a stationary batch. Scanning uses actual echoes
only, with no assumed distances in width or side profiles.

Echo capture normally waits 45 ms, then allows one additional 15 ms window
for a late falling edge. A completed late pulse can be treated as out of
range by the forward policy; serial reports `[S5-ECHO-RECOVER]`. A signal
still high after roughly 60 ms, a stuck-high signal before triggering, and
short glitches remain faults and stop the robot. The driver never drives
the ECHO pin or retriggers while it is high. The independent motor watchdog
remains active.

This single-sensor policy cannot distinguish open space from a disconnected
or failed sensor that produces no pulse; either can permit movement. Set
`ROBOT_NO_TARGET_FORWARD=0` to restore stop-on-missing-echo behavior.

Motor updates preserve the active PWM input during same-direction refreshes.
Front polling is 80 ms, leaving more margin under the 350 ms watchdog. Valid
readings between 35 and 45 cm keep an already moving robot moving; they do
not restart a stopped robot. Sensor faults still stop immediately.

The firmware cannot establish whether a missing pulse is caused by wiring,
power or a faulty sensor; inspect the new raw status before adjusting filters.

Set `ROBOT_FORWARD_STOP_MODE=0` to restore the encoder command controller
and B5's command-refresh adapter. Earlier robot README files describe that
controller and its obstacle profiling behavior.
