# Subsystem 4 — Minimum-bit data representation (Change Requirement 1)

Evidence for Buddy 4 (IMU, hump and lift detection).
"Before" = commit `34f8528` (2026-11-21), the parent of the B4 change
(`d4c6499`). "After" = tag `after-minbits`. Design notes and synchronisation:
`app_program/subsystem4/DATA_REPRESENTATION.md`.

## 1. Data representation table

| Field or group | Valid range or meanings | Before | After | Bits |
|---|---|---|---|---|
| Motion state in `imu_event_t` | 8 states, stationary … unknown | enum (32) | enum bit-field | 3 |
| Event flags (hump active / completed, tilt, valid, height / peak valid, fault, recovery, turn rate, heading valid / calibrated, impact, lift) | false / true | 13 × `bool` (8) | 13 Boolean bit-fields | 1 each |
| Sample validity flags (reading, turn rate, magnetics, heading, calibrated heading, odometry) | false / true | 6 × `bool` (8) | 6 Boolean bit-fields | 1 each |
| Odometry reasons | 6 independent reasons, mask 0–63 | `uint8_t` (8) | unsigned bit-field | 6 |
| Timing availability | 6 independent sources, mask 0–63 | `uint8_t` (8) | unsigned bit-field | 6 |
| B2 motion-context flags (valid, running, forward, reverse, turning, stopped, fault) | false / true | 7 × `bool` (8) | 7 Boolean bit-fields | 1 each |
| Controller notice mask | 12 observations + queue overflow, 0–8191 | `uint32_t` (32) | unsigned bit-field | 13 |
| Completion height valid | false / true | `bool` (8) | Boolean bit-field | 1 |
| Notice tracker flags | 10 flags | 10 × `bool` (8) | 10 Boolean bit-fields | 1 each |
| Analysis roll state / terrain stage | 0–2 / 0–3 | enum (32) each | enum bit-fields | 2 each |
| Analysis and processing state flags | false / true | `bool` (8) each | Boolean bit-fields | 1 each |
| Sensor values, filters, tilt, heading, height | calibrated floating-point domain | `float32_t` | kept | 32 |
| Timestamps, IDs, sequence / error counters | 0–4,294,967,295, wrap | `uint32_t` | kept | 32 |
| B2 encoder counts and speed snapshots (input) | signed 32-bit at the interface | `int32_t` | kept | 32 |
| MQTT motion / lift / height valid (sent by B1) | 0–7 / false-true / false-true | — | explicit masks and shifts | 3 / 1 / 1 |
| MQTT pitch / roll | ±51.1° / ±180.0° in 0.1° | — | signed two's complement | 10 / 12 |
| MQTT latest / peak hump height | 0–126 mm | — | unsigned | 7 each |

Sensor and estimator values keep full precision: the change applies to
bounded states and flags only.

## 2. Measured size comparison (Cortex-M0+)

Measured 2026-11-25 with `arm-none-eabi-gcc` (Cortex-M0+) by compiling
`testing/subsystem4/measure_sizes.c` against the `34f8528` headers and the
current headers, then `arm-none-eabi-nm -S -t d`. Queue storage is
`depth × (4 + align4(payload))`.

| Payload, object or queue storage | Before (B) | After (B) | Saved (B) |
|---|---:|---:|---:|
| IMU motion context (inside a sample) | 36 | 32 | 4 |
| Sampling → analysis message | 128 | 112 | 16 |
| Analysis event (inside a controller notice) | 52 | 32 | 20 |
| Analysis → controller notice | 88 | 64 | 24 |
| Static notice tracker | 108 | 76 | 32 |
| Processing context, including its previous sample | 160 | 144 | 16 |
| Sampling queue, 8 slots | 1,056 | 928 | 128 |
| Controller notice queue, 8 slots | 736 | 544 | 192 |
| **Queue reservations + static notice tracker** | **1,900** | **1,548** | **352** |
| Analysis task context (task-local) | 176 | 236 | −60 |

The analysis context measured 156 B straight after the change. The later
hump-detection fix (`47c87aa`, 25 Nov) added fields to it (156 → 236 B), so
it is now larger than before; it lives in the analysis task's existing stack
reservation and does not change static RAM.

## 3. Correctness evidence

B4 host suites (`testing/subsystem4/host/run_tests.ps1`) exercise time gaps,
wraparound, roll classification, hump height, impact and lift with the packed
types; the startup suite exercises encoder pairing, controller notice delivery,
queue saturation and recovery. See `TEST_LOG.md`.

## 4. Timing (performance)

Measured 2026-11-26 on the Cortex-M0+ instruction set (arm-none-eabi-gcc `-O2`, cycles counted in an emulator with RP2040 timings) by `sh testing/timing/run_timing.sh`; log `testing/timing/logs/2026-11-26_m0_cycles.txt`, method and limits in `testing/timing/README.md`.

| Path | Before (`34f8528`) | After (`d4c6499`) | Budget |
|---|---:|---:|---|
| Filter + terrain/motion analysis, per IMU sample (20 samples over a hump) | 7,457 cycles / 59.7 µs | 7,318 cycles / 58.5 µs | every 10 ms → 0.59 % CPU |

The narrower types make the per-sample pipeline **1.9 % faster**; both
builds accept all 20 samples.
