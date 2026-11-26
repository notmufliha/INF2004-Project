# Subsystem 3 — Minimum-bit data representation (Change Requirement 1)

Evidence for Buddy 3 (IR line sensing and Code 39 barcodes).
B3's shared snapshot `B3_STATE` was written as a packed bit-field structure
from the start, so "Before" is the plain design it replaces: the same six
fields as ordinary 32-bit integers. "After" = tag `after-minbits`.
Layout and locking: `app_program/subsystem3/b3_line_barcode.h` and
`app_program/subsystem3/B3_IMPLEMENTATION.md`.

## 1. Data representation table

| Field | Valid range | Values | Before | After | Bits |
|---|---|---|---|---|---|
| IR left (GP26), normalised | 0 floor … 1000 tape (raw ADC 0–4095 fits too) | ≤ 4096 | `int` (32) | `unsigned : 12` | 12 |
| IR centre (GP27), normalised | as above | ≤ 4096 | `int` (32) | `unsigned : 12` | 12 |
| IR right / barcode input (GP28) | black / white | 2 | `int` (32) | `unsigned : 1` | 1 |
| Line flags | detected, lost, junction (bitmask) | 2³ | `int` (32) | `unsigned : 3` | 3 |
| Last barcode | none, A–Z | 27 | `int` (32) | `unsigned : 5` | 5 |
| Last navigation command | none, left, right, straight, U-turn, stop, slow, normal, spin, finish | 10 | `int` (32) | `unsigned : 4` | 4 |
| Navigation command message (B3 → controller) | `B3_NAV_COMMAND` values | 10 | — | one byte per message (`UB`) | 8 |
| Barcode letter → action table | 26 entries | 10 actions | — | `uint8_t[26]` | 8 each |

`B3_STATE` holds 37 meaningful bits in 5 bytes (3 bits unused). The header
asserts `sizeof(B3_STATE) == 5` because packed bit-field layout is
compiler-specific; the structure is only shared in RAM under `mtx_b3_state`
and is never sent as raw bytes (B1 packs the telemetry itself).

## 2. Measured size comparison (Cortex-M0+)

Measured 2026-11-25 with `arm-none-eabi-gcc` (Cortex-M0+) by
`sh testing/measure_minbits.sh` (`testing/minbits_probe.c`):

| Object | Before (B) | After (B) | Saved (B) |
|---|---:|---:|---:|
| `B3_STATE` snapshot | 24 | 5 | 19 (79 %) |

## 3. Correctness evidence

The size is enforced at compile time by the `_Static_assert` in
`b3_line_barcode.h`, so a layout change fails the build. The host tests that
compile the real B3 sources (run by `testing/controller/run_tests.ps1`):

- `code39_test.c`: C39-01 … C39-10 (full table, both directions and
  polarities, speed drift and timer wrap, chatter, rejects, blur, partial
  reads).
- `line_cal_test.c`: S3-PD-C01 … C10 and the measured per-channel contrast
  and floor-tracking cases (output bounded to 0–1000).
- `testing/subsystem5/test_line_input.c`: B3 line input through the packed
  snapshot (GP26/27 channel mapping, polarity, timestamp, invalidation,
  recovery, bounded locking).

See `TEST_LOG.md`.

## 4. Timing (performance)

Measured 2026-11-26 on the Cortex-M0+ instruction set (arm-none-eabi-gcc `-O2`, cycles counted in an emulator with RP2040 timings) by `sh testing/timing/run_timing.sh`; log `testing/timing/logs/2026-11-26_m0_cycles.txt`, method and limits in `testing/timing/README.md`.

| Path | Before (plain 32-bit fields) | After (packed `B3_STATE`) | Budget |
|---|---:|---:|---|
| Write one line-sensor sample into the snapshot | 25 cycles / 0.20 µs | 50 cycles / 0.40 µs | every 20 ms → 0.002 % |
| Controller reads the whole snapshot | 25 cycles / 0.20 µs | 36 cycles / 0.29 µs | every 20 ms → 0.0015 % |

The packed snapshot is under 0.5 µs per access, so the time spent holding
`mtx_b3_state` stays negligible.
