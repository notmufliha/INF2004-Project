# GY-511: first real-data test

The driver targets the LSM303DLHC on a GY-511. Register configuration, byte order
and sensitivities follow [ST DS7202 Rev 2](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf).
It checks magnetic identification bytes H43 at 7-bit address 0x1e and verifies
accelerometer configuration at 0x19. The accelerometer has no documented WHO_AM_I
register: this is a compatibility check, not proof of the exact chip marking.

## Connections and mounting

Connect SDA to GP4, SCL to GP5, and GND to Pico ground. Provide appropriate power
for your breakout. VIN and 3.3V are not interchangeable without the module's
schematic: confirm which is the regulated input/output before powering it, and
do not connect both supplies. I2C pull-ups must go to 3.3V, not 5V. INT1, INT2 and
DRDY are not required. I2C0 now belongs exclusively to the IMU sampling task;
the kernel driver configures GP4/GP5 at boot. Other tasks must not access I2C0.
Its transaction-phase timeout is now 5ms. The RP2040 I2C receive handler also
queues STOP on the final byte of a burst; physical bus behaviour needs validation.

Default mapping is sensor X=car forward, sensor Y=car sideways, sensor Z=up.
The chip must produce +1g on robot Z when upright. Check the PCB axis markings
and actual outputs; edit `config.axis[]` in `hardware_read()` in `task_imu.c`
if the board is mounted differently. Values are signed axis numbers 1=X, 2=Y,
3=Z; for example {-2,1,3} rotates the horizontal axes. Use a physically consistent
right-handed mapping. Calibration cannot correct an incorrect mounting map.

## Build, flash, observe

Use the configured environment from the root README (WSL on Windows):

```sh
cd build_make
make -j8
# Optional real-data arrival test:
make TEST=1 -j8
```

Hold BOOTSEL while plugging the Pico in, then copy the matching USB/WiFi UF2 to
RPI-RP2. Open the USB COM port at 115200 baud. Expected startup:

```text
IMU: REAL GY-511 on I2C0 GP4/GP5; no motor commands
IMU REAL: sensor configured; keep level and still for 200 fresh samples
IMU REAL: level calibration complete; magnetic heading uncalibrated, turn rate unavailable
```

Keep the vehicle stationary and level throughout startup, approximately two
seconds or longer. Startup averages 200 fresh samples and checks per-axis variance
and proximity to {0,0,+9.81}. It removes offsets while retaining gravity on Z.
If calibration is rejected, check mounting and keep still; it retries. This is a
one-position offset calibration, not a full multi-position sensitivity calibration.
Calibration/initialisation produces invalid readings until ready, so a temporary
unavailable/fault report followed by recovery is expected.

If `GY-511 check failed` repeats, check power, SDA/SCL, ground and module identity.
The task retries initialisation once per second. Ten acquisition failures trigger
reinitialisation and a new level calibration. Do not move during recalibration.
No fresh acceleration for over 100 ms enters the failure path. Magnetic read
failure/overflow only invalidates magnetic/heading fields; acceleration continues.

Serial reports include pitch/roll, acceleration in mg (1000mg approximately 1g),
magnetic field in microtesla, heading and explicit availability/calibration flags.
Numeric unavailable fields may be zero: only interpret them when their flags are
true. Reports are periodic (about one second) and on motion-state changes.

1. Level and still: expect approximately {0,0,1000}mg and pitch/roll near zero.
2. Slowly raise the car's nose: pitch should increase; lower it: pitch decreases.
3. Slowly roll the car sideways: roll should change with orientation.
4. Traverse a known hump only after mounting/direction checks; verify one completion.
5. Disconnect the sensor while observing output: expect unavailable/fault; reconnect
   and keep level/still for reinitialisation, calibration and recovery.

TEST=1 checks for fresh valid data using S4-U16; it does not establish sensor
accuracy, collision reliability, or height measurement. Do not deliberately crash
the robot to test impacts. The software does not command motors.

## Implemented measurements and remaining validation

- Acceleration: 100Hz, +/-4g, high-resolution, 2mg/LSB; clipping is rejected.
- Pitch/roll: accelerometer gravity-vector estimates. Driving acceleration and
  vibration affect them; they are not gyroscope-fused orientation measurements.
- Magnetic field: 75Hz, +/-1.3 gauss, X/Y=1100 and Z=980 LSB/gauss.
- Heading: tilt-compensated magnetic estimate for near-1g readings. `heading_valid`
  means calculable, not calibrated or accurate. No declination correction.
- Magnetic calibration defaults OFF. Supply robot-frame `mag_offset_ut[]` and
  `mag_scale[]` in `hardware_read()` after collecting calibration data; set
  `mag_calibrated=true` only after testing near the installed motors/wiring.
  Diagonal correction is supported; full soft-iron matrix calibration is not.
- Estimated turn rate uses fresh calibrated heading changes, shortest-angle wrap
  and a maximum 100ms interval. Without calibration, `turn_rate_valid=false` and
  turning classification is disabled. It remains an estimate, not a gyro reading.
- Existing hump and impact algorithms consume real data but thresholds still
  require real-car tests. Height in centimetres and highest height remain unavailable.

Desktop regression command (no board required):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File testing/subsystem4/host/run_tests.ps1
```

It includes a fake-register-bus driver test. Passing it and cross-compiling do not
prove physical I2C communication. Record your serial output and actual hardware
test results in `testing/subsystem4/TEST_LOG.md`.
