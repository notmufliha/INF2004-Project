# Buddy 5 — Wednesday demo verification record

Date prepared: 2026-11-02. Scope: Buddy 5 source only.

## Host-side checks performed in preparation environment

| Check | Result |
|---|---|
| B5 controller/simulation scenarios | PASS: 14/14 |
| B5 42-bit packed representation and version | PASS |
| B5-to-B1 wire adapter | PASS: 14 assertions |
| B5 error/fault decoder contract | PASS |
| New B5 bridge fail-closed and B2 STOP test | PASS |
| B5 demo preflight static checks | PASS |
| GCC C syntax check for B5 bridge/init and demo task (host include approximation) | PASS |
| Actual ARM/Pico firmware build | NOT RUN — ARM toolchain not installed here |
| Actual HC-SR04P and servo hardware test | NOT RUN |
| Wi-Fi/MQTT delivery on the robot | NOT RUN |
| Moving robot physical bypass | NOT IMPLEMENTED / NOT TESTED |

## Fill in during the actual demonstration

| Test | Observed result | Evidence/log |
|---|---|---|
| B5 boots with GP16/17/12 | | |
| Servo 90 -> 80 -> 100 -> 90 self-test | | |
| Clear target 50–70 cm | | |
| Close target below 35 cm triggers STOP | | |
| B2 acknowledges STOP | | |
| Coarse and fine scans | | |
| Profile / route recommendation or explicit STOP | | |
| Removing target resets obstacle latch | | |
| B5 demo mode never releases motor STOP | | |
| B1 laptop telemetry, if networking enabled | | |
| Error / no-echo test, if safely reproducible | | |

## Team-owned blockers

- Vehicle Controller callback registration, command arbitration, and B2
  motion ID tracking are required for actual bypass.
- B3 must be started and its line-detection freshness validated before
  line reacquisition can be claimed.
- The B2 motion self-test defaults to 4 in robot_config.h; request the
  B2/VC owner set it to 0 for the sensor-only demo.
- Hardware calibration of servo angle, obstacle width and stopping latency
  must be measured on the physical robot.
