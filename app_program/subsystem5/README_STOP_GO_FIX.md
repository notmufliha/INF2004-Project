> **Superseded for the moving robot by [README_GUARDED_DRIVE_V2.md](README_GUARDED_DRIVE_V2.md).** The earlier version only granted B2 clearance and did not send B2 DRIVE commands.

# Buddy 5 — STOP/FORWARD oscillation fix (4 Nov 2026)

## Why the robot kept alternating STOP and FORWARD

The supplied COM6 log shows repeated `ECHO_NO_FALL` immediately after `[B2] FORWARD`, plus occasional `OK` echoes at 11–19 cm while most clear readings were about 310–312 cm. It also shows `[B2] STOP: forward heartbeat expired`.

The previous `task_obstacle.c` did a fast single-ping guard **followed by** a slow five-ping consensus (sometimes retried) **on every moving loop**. This could starve B2's 1200-ms clearance heartbeat, and the first invalid ping stopped the robot only for the next stable-clear batch to re-arm it. A real obstacle, intermittent echo, and electrical noise are indistinguishable from the log alone.

## What changed (B5 only)

- **While moving:** use one bounded ultrasonic ping per loop. If valid and at least `ROBOT_RELEASE_CM` (45 cm), refresh `b2_request_forward()` immediately, then wait `S5_FRONT_POLL_MS` (200 ms). The slow 5/10-ping consensus no longer blocks the heartbeat while moving.
- **Any invalid, suppressed, close (<35 cm), or ambiguous (35–45 cm) moving reading:** immediately request B2 STOP, invalidate drive state and verify the next readings while stopped. The last known 310-cm reading is **not** reused as permission to move.
- **While stopped:** retain the existing 3-of-5 echo consensus and require `ROBOT_CLEAR_CONFIRM` (3) clear batches before granting forward permission again. Check that B2 has actually granted motion permission before setting the `driving` state.
- **Chatter lockout:** after three guard-induced STOPs within 15 seconds, latch STOP until reboot. This avoids repeated motor pulses when the sensor cannot reliably certify a clear path. It is intentionally **not** an override that makes the robot drive with `ECHO_NO_FALL`.
- Preserve the existing 35-cm stop threshold, 45-cm release threshold, B5-only `FAULT_NO_ECHO`, telemetry, scanning, servo range and disabled physical bypass.
- `S5_DEMO_SENSOR_ONLY` is **0** in this moving-test version; use **1** if you want to demonstrate sensors with no motor movement.

## Important: serial log vs supplied source

The supplied project has `S5_DEMO_SENSOR_ONLY=1` but the serial log shows `[S5-DRIVE]` and motor movement. Those cannot come from the same compiled configuration. Rebuild and flash the **new UF2**, and verify the boot banner:

```
[S5-FW] B5-STOP-GO-FIX-2026-11-04; single-ping moving guard + chatter lockout
```

Do not flash an old `.uf2` from the ZIP's `build_make` directory.

## Before testing moving wheels

1. Run `python testing/subsystem5/test_guard_heartbeat.py` and `python testing/subsystem5/moving_preflight.py`.
2. Build a new UF2 with your **own installed ARM toolchain**; the package does not include a newly built firmware image.
3. With wheels **raised**, confirm the firmware banner and STOP on a box at 20–30 cm. Confirm no forward permission on invalid echoes.
4. For floor driving, first ask B2/VC to set `ROBOT_MOTION_TEST=0` in `app_program/robot_config.h`; your uploaded version has test 4 enabled. Do not edit other subsystems as part of the B5-only patch.
5. Use a low speed, controlled clear track, accessible physical emergency STOP and someone ready to disconnect motor power. Verify the sensor's echo behaviour with motors powered. Do not increase watchdog duration or disable STOP to make the robot move.

## Expected log

On a healthy clear path, the serial monitor should show the banner and one `[S5-DRIVE] confirmed clear=...` when forward permission begins. While the path remains valid and clear, the code refreshes B2's clearance heartbeat without repeatedly printing DRIVE/STOP.

If the sensor reports invalid or close echoes, the robot **must** STOP. If this happens three times within 15 seconds, the log should show:

```
[S5-HOLD] repeated unsafe/invalid readings: STOP for at least 8000 ms; require 5 stable clear batches to re-arm
```

This means the code has prevented chatter, **not** that the underlying sensor fault has been repaired. Check the HC-SR04P's 3.3-V-safe ECHO wiring, shared ground, servo/motor supply separation and power noise. Repeated `ECHO_NO_FALL` is not a reliable clear-path reading. Reboot only after investigating and correcting the issue.

## Verification limits

Host tests exercise the guard-history policy, stopped consensus and source-level heartbeat structure. They do **not** reproduce the real sensor electrical faults or guarantee a complete Pico firmware build. A full moving-robot test and confirmation of the B2/VC ownership model are still required.


## V3 correction (4 November 2026)

The V2 permanent sensor lock was removed. After three unsafe moving echoes
within 15 seconds, B5 keeps the motors stopped for **at least 8 seconds**
and requires **five consecutive, agreeing stationary clear batches** before
allowing B2 forward permission again. Any invalid or close batch resets
the recovery streak. This does not mask a missing echo. The serial log now
reports `[S5-HOLD]` and `[S5-REARM]`, and `ECHO_NO_FALL` includes a PWM
`pulse_us` diagnostic. A B2 encoder/PID fault still remains latched.

If `ECHO_NO_FALL` repeats when the wheels turn, investigate HC-SR04P
TRIG=GP16, ECHO=GP17, 3V3/GND, shared grounds, motor power sag, servo
power and motor electrical noise. No software-only change can guarantee
continuous safe driving with an unreliable forward sensor.
