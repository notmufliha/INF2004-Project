# B5 guarded straight DRIVE — V2 (4 Nov 2026)

## Root cause from the new COM6 log

- `[B2] Distance calibration test starting` is **an encoder print task**, not a motor command.
- `Encoder L=0 R=0` means no wheel encoder motion was measured; the log does not prove that the motors received a DRIVE command.
- `b2_request_forward()` only grants safety clearance. It does **not** submit `MOTION_CMD_DRIVE` to Buddy 2. The earlier B5 "fix" lacked this motor-command integration.
- The uploaded source also had `ROBOT_MOTION_TEST=4` (one-shot 90-degree left turn) and `ROBOT_ENCODER_DIAGNOSTICS=1` (repetitive console prints). These are not normal forward-drive settings.
- `[S5-STOP] confirmed close obstacle` and the scan indicate that B5 also saw repeated close echoes. It must not ignore those without evidence that they are false.
- `s1: EVENT obstacle at 511 cm, width 127` represents B1 invalid field sentinels, **not** a real obstacle 511 cm away.

## Changes

1. `task_obstacle.c`: removed `<math.h>`/`<stddef.h>` (the source of ARM/newlib vs micro T-Kernel `size_t` conflicts), and uses `__builtin_isfinite`.
2. `task_obstacle.c`: added a **forward-only demonstration adapter**. Once B5 confirms a clear path, a second B5 task sends `MOTION_CMD_DRIVE` via B2's public `motion_send_command()` API. B2 still owns PWM, encoder checks, safety generation and STOP acknowledgement.
3. The adapter refreshes B2's 100-ms DRIVE watchdog every 35 ms **only if** B5's most recent valid clear echo is at most 350 ms old **and** B2 grants motion clearance. If the echo is stale, B2 reports a fault/timeout, or the encoder fails, B5 STOPs and latches the drive off until reset.
4. On a close/invalid echo, B5 revokes its clearance and calls `b2_request_stop()` immediately; it never treats an invalid echo as a clear path.
5. `robot_config.h`: `ROBOT_MOTION_TEST=0` prevents the separate B2 one-shot turn from competing for motor commands; `ROBOT_ENCODER_DIAGNOSTICS=0` stops unrelated encoder-calibration log spam.
6. Logs now include `[S5-FW] B5-GUARDED-DRIVE-2026-11-04-V2`, `[S5-DRIVE] ... B2 permission granted`, and `[S5-B2] guarded DRIVE command accepted ...`.

**This is not autonomous obstacle avoidance, line reacquisition or a full Vehicle Controller.** Do not use it alongside another controller that owns B2's command queue. To restore sensor-only mode, set `S5_DEMO_SENSOR_ONLY=1`. To restore team ownership, disable `S5_DEMO_B2_FORWARD_COMMANDS` before connecting a real VC.

## Build on Windows WSL Ubuntu

From your `INF2004-Project/build_make` directory:

```bash
make clean
make -j8
ls -lt *.uf2
explorer.exe .
```

Flash the **new** `.uf2` via BOOTSEL and the `RPI-RP2` drive. Verify the V2 firmware banner. The previous source included `<math.h>`, which caused the `size_t` error; this version removes it. Do not edit micro T-Kernel headers.

## Safety test procedure

1. **Motor power off**: flash the Pico, open USB serial, verify `[S5-FW] B5-GUARDED-DRIVE-2026-11-04-V2`.
2. **Wheels raised**: power motors and verify `[S5-DRIVE]` and `[S5-B2]` appear after 3 stable clear batches (>=45 cm). Check that the encoders actually count and the wheels turn.
3. Place a flat box 20–30 cm from the sensor; expect immediate STOP, followed by scanning. No bypass is commanded.
4. Remove the box; after 3 stable clear batches B5 may authorize forward again. If `[S5-CHECK] stable=0` or a stable close distance appears, **do not force drive**; inspect GP16/17 wiring, common ground, servo/motor power noise and possible sensor reflections.
5. **Only after raised-wheel tests pass**, do a low-speed controlled floor test with a reachable motor-power cutoff. Keep the 35-cm STOP threshold and the watchdogs enabled.

If `[S5-B2-FAULT] B2 controller/encoder fault or timeout` appears, the drive remains locked until reset. Do not raise speed or disable encoder safety to work around this.

## Tests and limits

```bash
python testing/subsystem5/test_guarded_drive.py
python testing/subsystem5/test_guard_heartbeat.py
python testing/subsystem5/moving_preflight.py
```

Host-side syntax and policy tests cannot verify RP2040 compilation, motor wiring, electrical echo integrity, real-time scheduling or actual wheel motion. A new Pico UF2 build and physical test are required.
