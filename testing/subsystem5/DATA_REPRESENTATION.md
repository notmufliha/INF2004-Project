# Subsystem 5 — Minimum-bit data representation (Change Requirement 1)

Evidence for Buddy 5 (ultrasonic, servo scan, obstacle bypass).
B5's minimum-bit change is in the data it hands on: a scan result leaves B5
as narrow fixed-width fields instead of the native `S5Profile` structure.
"Before" = the native `S5Profile` (floats and 16-bit angles).
"After" = tag `after-minbits`: the 42-bit B5 packed profile
(`app_program/subsystem5/s5_packed.c`) and the fields B1 sends for B5
(`app_program/subsystem1/MQTT.md`). Interface notes:
`app_program/subsystem5/README_BARRC_CR1_B5_ONLY.md`.

## 1. Data representation table

| Field | Valid range | Invalid code | Before | After | Bits |
|---|---|---|---|---|---|
| Obstacle (nearest) distance | 0–510 cm | 511 | `float` (32) | unsigned field | 9 |
| Obstacle width | 0–126 cm | 127 | `float` (32) | unsigned field | 7 |
| Left clearance | 0–510 cm | 511 | `float` (32) | unsigned field | 9 |
| Right clearance | 0–510 cm | 511 | `float` (32) | unsigned field | 9 |
| Servo angle of the nearest point | 0–254° | 255 | `uint16_t` (16) | unsigned field | 8 |
| Packed format version | 1 | other values rejected | — | unsigned field | 4 |
| Padding | 0 | non-zero rejected | — | zero bits | 2 |
| Avoid action (B1 obstacle event) | 6 actions | — | `UB` + name text | B1 field | 3 |
| Front distance (B1 telemetry) | 0–510 cm | 511 = no echo | `H` mm (16) | B1 field | 9 |
| No-echo fault | clear / set | — | — | bit in the shared fault mask | 1 |

The packed profile holds 42 data bits + 4 version bits + 2 zero pad bits in
6 bytes, in a fixed bit order independent of C structure layout. B1's
obstacle event carries 9 + 7 + 9 + 9 + 3 = **37 application bits**
(distance, width, two clearances, action); B1's encoder is the one sent over
MQTT, and the 42-bit B5 format is host-tested for B5 itself.

## 2. Measured size comparison (Cortex-M0+)

Measured 2026-11-25 with `arm-none-eabi-gcc` (Cortex-M0+) by
`sh testing/measure_minbits.sh` (`testing/minbits_probe.c`):

| Object | Before (B) | After (B) | Saved (B) |
|---|---:|---:|---:|
| Scan result handed on: native `S5Profile` → `S5PackedProfile` | 56 | 6 | 50 (89 %) |
| Decoded form of the packed profile (`S5DecodedProfile`) | — | 10 | — |

`S5Profile` stays the in-RAM working form inside B5's own task while it plans
a bypass; the saving applies to every copy that leaves B5.

## 3. Correctness evidence

- `testing/subsystem5/test_packed.c` ("B5 42-bit packed format + version"):
  pack/unpack round trips with rounding, 6-byte size, zero padding;
  invalid source values, an unsupported version, non-zero padding and the
  511 invalid-distance code are rejected.
- `testing/subsystem5/test_b1_contract.py`: the wire-adapter functions in
  `task_obstacle.c` that convert B5 values to B1's cm fields and ranges.
- All 21 B5 host groups: `python testing/subsystem5/run_complete_tests.py`.

See `TEST_LOG.md`.
