# Buddy 5 — Wednesday demonstration package (B5 files only)

## What is actually ready
- Real HC-SR04P distance acquisition (TRIG GP16, ECHO GP17, 3.3 V compatible).
- Real servo on GP12 (mechanically constrained to 70–110 degrees).
- B5 stable echo filtering, coarse/fine scanning, width/clearance estimation.
- B1 front-distance telemetry, scan events, `FAULT_NO_ECHO` and B5 errors.
- Real B2 STOP request and STOP acknowledgement checks.
- A **new, concrete B5 bridge handoff contract** that replaces the old weak
  placeholders. The Vehicle Controller must install callbacks before full
  motion mode is enabled; otherwise B5 fails closed.

**Not ready / not claimed:** physically driving around an obstacle, reversing
without rear clearance, B3 line reacquisition, full mission autonomy. These
require the team Vehicle Controller, a started and validated B3 task, motor
ownership, calibrated geometry, and an independently checked route.

## Install onto latest `main` (do not overwrite teammate code)

1. Create a new branch from the latest `origin/main`.
2. Copy the B5 files in the changed-files ZIP into the corresponding paths.
   **Do not replace `task_obstacle.c` from an old ZIP.**
3. From the repository root run:

   ```powershell
   python testing/subsystem5/apply_demo_guard.py
   python testing/subsystem5/demo_preflight.py
   ```

4. Review `git diff` before committing. All modified files should be under
   `app_program/subsystem5/` or `testing/subsystem5/`.
5. Ask the B2/VC owner to set `ROBOT_MOTION_TEST=0` for the demo; it currently
   defaults to **4**, so its own task otherwise prints a 30-second blocked
   motion-test attempt. The B5 demo STOP latch prevents B2 clearance.
6. Rebuild and reflash using the team's Pico toolchain and board selection.
   **The supplied ZIP is source code, not a compiled UF2.**

## Preflight tests (host)

```powershell
bash testing/subsystem5/run_host_tests.sh
bash testing/subsystem5/run_packed_tests.sh
bash testing/subsystem5/run_bridge_tests.sh
python testing/subsystem5/test_b1_contract.py
python testing/subsystem5/test_b5_error_contract.py
python testing/subsystem5/demo_preflight.py
```

## Hardware demonstration (2–4 minutes)

**Preparation:** support/raise the wheels or disconnect motor power for the
sensor-only demonstration. Use a separate correctly rated servo supply with
common ground if the servo draws more than the board can supply. Keep HC-SR04P
ECHO at 3.3 V; do not connect a 5 V HC-SR04 ECHO directly to the Pico.
Secure the servo horn and wiring. Do not power-cycle while hands are in the
motor/servo path.

1. Flash the verified firmware. On USB serial, look for
   `[S5-BOOT]` and `[S5-SERVO] SELF TEST` (90 -> 80 -> 100 -> 90).
2. Put a flat target at a **measured** 50–70 cm in front. Show that the
   readings are valid and the motors stay stopped in `DEMO STOP-ONLY` mode.
3. Move the target to around 20–30 cm (below `ROBOT_STOP_CM=35`). Wait for
   three stable close readings. Show `[S5-STOP]`, `[S5-COARSE]`, `[S5-FINE]`,
   `[S5-PROFILE]`, `[S5-PLAN]`, and `[S5-METRIC]` if the scan obtains
   sufficient valid measurements. Not all target shapes produce a valid width;
   invalid scans correctly result in STOP rather than a fabricated route.
4. Remove the target. Show that confirmed clear readings release the obstacle
   latch **but not B2's motor safety STOP** in this demonstration mode.
5. Show the B1 wire event / error decoder only if the actual build enables
   Wi-Fi/MQTT (`NET` and `MQTT` options) and the network connection is up.
   USB logs work without networking.

## New B5 bridge contract for Vehicle Controller developer

`S5BridgeDriver` in `s5_bridge.h` contains callbacks for readiness, commands,
completion, line detection and emergency STOP. VC must call
`s5_bridge_install(&callbacks)` **during initialization, before B5 starts**.
B5 always calls the real `b2_request_stop()` on STOP/emergency, even without
VC. `s5_bridge_stopped()` reads the real B2 stop acknowledgement.

VC callback responsibilities:
- Exclusive motor-command ownership; accept a command only after B2 queue
  submission succeeds and its safety generation is correct.
- `motion_complete()` only after the matching B2 command ID is COMPLETED.
- `line_found()` only from a fresh, debounced B3 sensor reading.
- Refuse reverse unless rear clearance is independently established.
- Refuse GO_LEFT/GO_RIGHT unless the *whole* corridor and calibrated chassis
  geometry are verified; ultrasonic side rays alone are insufficient.
- Cancel queued/active movement on emergency STOP.

Do **not** set `S5_MOTION_BRIDGE_ENABLED=1` or `ROBOT_ENABLE_BYPASS=1` for the
Wednesday sensor-only demo. Both require team integration and safety tests.

## Scope and limits

This package modifies only Buddy 5 source and Buddy 5 tests. It does not
change B1, B2, B3, B4, VC, `robot_config.h`, `error_codes.h`, or the wire
protocol. The existing nine B5 error codes remain unchanged. The host tests
verify software behavior; no Pico firmware build or physical robot test was
performed in the preparation environment.
