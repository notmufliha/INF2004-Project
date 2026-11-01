# Buddy 5: safer servo scan and battery-only motor diagnosis

## Physical mount FIRST

1. Switch off all power. Secure the servo body to the bracket and secure the plastic horn to the servo output shaft with its center screw (if the model uses one). Secure the ultrasonic module to the horn/bracket. Do not rely on friction, loose tape, or jumper-wire tension.
2. Route ultrasonic and servo wires with a small slack loop and strain relief. Ensure the sensor cannot hit the chassis, IR modules or wires at 70 and 110 degrees. If it still pulls loose, reduce `S5_SAFE_MIN_DEG` / `S5_SAFE_MAX_DEG` further (e.g. 80..100) and adjust the coarse/fine angle arrays accordingly.
3. This firmware limits hardware motion to 70..110 degrees, uses gradual 3-degree movements with 35ms between steps, and starts with a smaller 80/100 degree test. This makes the scan less wide: width and side-clearance estimates may become invalid, and physical bypass is deliberately disabled.

## Portable battery: why Pico powers on but motors don't

A USB power bank connected to the Pico USB port can power the microcontroller while the onboard motor driver receives no usable motor supply. Inspect the exact motor-driver board silk-screen/manual for its motor-power input (often marked VM, VIN, MOTOR POWER or battery terminal). Never assume a USB power bank powers the motor supply rail. Never connect a battery directly to a GPIO pin or an unverified connector. Check motor-driver supply voltage and polarity with a meter against its rated input, and check the power switch/enable. A photo cannot confirm the motor supply wiring or the exact GPIO mapping.

## Stage A: servo and ultrasonic only

- Raise both wheels, keep `ROBOT_WHEELS_UP_MOTOR_TEST 0`, and set `ROBOT_ENABLE_DRIVE 0` in `app_program/robot_config.h`.
- Rebuild/reflash. Observe small 90 -> 80 -> 100 -> 90 startup motion. Hold an obstacle about 20-30cm ahead. Servo should scan only 70..110 degrees.
- If horn detaches, immediately power off and fix the mount before further testing.

## Stage B: isolated motor diagnostic, WHEELS RAISED ONLY

- Confirm rated motor supply connected to the actual motor driver and common ground with Pico. Turn off battery while checking wiring.
- Set `ROBOT_ENABLE_DRIVE 1`, `ROBOT_WHEELS_UP_MOTOR_TEST 1` in `app_program/robot_config.h`.
- Rebuild/reflash, raise wheels, then power on. After about 3 seconds: left wheel 1.2 seconds, right wheel 1.2 seconds, both wheels 1.2 seconds, then all stop.
- If neither wheel moves, check motor supply rail, switch, motor output wiring, driver enable and GPIO pin mapping. USB serial can display `[B2] PWM READY` / `[B2-TEST]` logs, but USB-only power may not run motors.
- If wheels move the wrong direction, stop and swap the corresponding FWD/REV pin defines only after verifying the driver pinout.

## Stage C: obstacle-gated forward drive

- Restore `ROBOT_WHEELS_UP_MOTOR_TEST 0`, keep `ROBOT_ENABLE_DRIVE 1` and `ROBOT_ENABLE_BYPASS 0`. Rebuild/reflash.
- Wheels raised: clear path >=45cm for 3 stable readings -> wheels forward; obstacle <35cm -> both stop -> servo coarse/fine scan -> print profile and side proposal. Invalid echo -> stop. Remove obstacle -> forward after 3 valid clear readings.
- After successful raised-wheels test, test at low speed on flat ground with a power disconnect ready. Do not assume a fixed 35cm stop threshold is adequate without measuring actual stopping distance.

## Build (WSL)

```sh
cd /mnt/c/Users/lucas/Downloads/INF2004-Project
bash testing/subsystem5/run_host_tests.sh
cd build_make
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 clean
make CONSOLE=usb_cdc WIFI=none SMP=0 TEST=0 -j8
ls -lh mtk3pico_smp0_usb_cdc.uf2
```

Flash the NEW UF2 with BOOTSEL. USB serial is for logs. No full robot bypass or line reacquisition is claimed: encoder/IR hardware integration and clearance validation remain outstanding.
