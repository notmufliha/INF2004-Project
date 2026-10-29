# Buddy 2 response to Buddy 1 integration requirements

## Implemented

- `subsystem2_init()` already calls `encoder_init()`; robot mode initializes
  and starts B2. Keep this startup path when merging with B1.
- The low-priority telemetry task reads B2's measured signed speeds and full
  signed encoder totals. With the binary B1 header (`COMM_SPEED_MAX` defined),
  `b2_integration.c` calls `comm_set_motion(left_cps, right_cps, left, right)`.
  B2 does not truncate totals to 24 bits; B1 owns packing and the laptop owns
  reconstruction. B2's 5000 limit applies to requested speed, not encoder totals.
- The current branch still has the older JSON B1 API. The adapter retains
  its five-argument mm/s call until B1 is merged, then selects the counts/s
  interface automatically. Do not describe this branch's old JSON speed as
  counts/s. The binary decoder in `tools/s1_wire.py` is for the new B1 format.
- A wheel failing to progress in its commanded direction raises
  `fault_raise(FAULT_MOTOR_STALL)` and stops the motion.
- Encoder initialization failure or eight impossible A/B transitions on
  either encoder within the 100 ms observation window raises
  `fault_raise(FAULT_ENCODER)`. Runtime detection latches STOP and reports
  `MOTION_FAULT_ENCODER`. The transition threshold is preliminary; validate
  noise and edge capture at the maximum wheel speed before deployment.
- Shared bits are latched. B2 clears only `FAULT_MOTOR_STALL | FAULT_ENCODER`
  during its initialization. Fix the cause and restart to recover. B2 never
  clears IMU, line, ultrasonic or Wi-Fi faults.
- No encoder pulses cannot distinguish a stalled motor from a disconnected
  encoder; that case reports MOTOR_STALL, not a proven electrical diagnosis.
- Error events run in the telemetry task, outside the 10 ms motor loop.
  The same ID/fault is reported once after successful queueing; a full queue
  causes a retry on the next telemetry update. Current fault bits are raised
  immediately without waiting for that event.

## B2 error code allocation

| Code | Header constant | Laptop decoder name |
|---|---|---|
| 0x41 | ERR_S2_MOTOR_STALL | motor stall |
| 0x42 | ERR_S2_ENCODER | encoder fault |
| 0x43 | ERR_S2_LOOP_OVERRUN | 10 ms loop overrun |
| 0x44 | ERR_S2_MOTION_TIMEOUT | motion deadline exceeded |
| 0x45 | ERR_S2_STOP_TIMEOUT | standstill timeout |
| 0x46 | ERR_S2_COMMAND | motion command fault |
| 0x47 | ERR_S2_DRIVE_WATCHDOG | B2 DRIVE refresh missed |

Shared `error_codes.h`, `fault_flags.h`, and `tools/s1_wire.py` were taken from
`origin/b1-wifi-mqtt` and B2's new codes/names appended. Existing allocations
were preserved. Resolve these shared files together when merging; do not
renumber existing codes.

## Merge checks owned jointly with B1 / Vehicle Controller

1. Initialize B1 (or the future Vehicle Controller's shared flag service)
   before B2. The new B1 branch creates `flg_faults`; the compatibility
   implementation in `b2_integration.c` compiles out when that header is used.
2. Set `S1_POLL_B2_ENCODERS=0` in the B1 configuration when B2 supplies motion
   telemetry. Otherwise B1 also calculates and overwrites those values.
3. Robot-mode networking still needs the B1 tasks/startup/build configuration.
   This patch does not merge or enable B1's entire Wi-Fi/MQTT implementation.
4. The Vehicle Controller owns receiving the remote 4-byte struct and
   command arbitration. B2 provides `motion_speed_from_percent()`:
   0 -> 0, 50 -> 2500, 100 -> 5000 counts/s; out-of-range input is rejected.
   Zero percent must become STOP, not a zero-speed DRIVE. The VC must still
   supply direction/steering, observe B5 clearance and refresh DRIVE <100 ms.
5. Keep the existing turn calibration and hardware test selection until tests
   finish. This work does not alter PWM gains, speed selection or turn scales.
6. Moving Wi-Fi below the other subsystems is a team scheduling change;
   verify priorities after merging. B2 motion control is currently priority 5.

## Validation

Host tests cover both telemetry signatures, signed counts/s, untruncated
encoder totals, fault ownership, event retry/deduplication, encoder fault stop,
and percentage mapping. ARM syntax checks cover the actual new B1 public header
as well as the current application. Physical fault injection, Wi-Fi publishing,
24-bit wire reconstruction and full-load timing remain integration tests.
