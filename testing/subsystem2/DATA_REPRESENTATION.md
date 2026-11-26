# Subsystem 2 — Minimum-bit data representation (Change Requirement 1)

Evidence for Buddy 2 (motion control: motors, encoders, PID).
"Before" = commit `7bad6fb` (2026-11-20), the parent of the B2 change
(`b5ca3b7`, `dc2cc5a`). "After" = tag `after-minbits`. Full field list and
synchronisation notes: `app_program/subsystem2/README.md`, section
"Change Requirement 1: minimum-bit representation".

## 1. Data representation table

| Field | Valid range | Values | Before | After | Bits |
|---|---|---|---|---|---|
| Command type | STOP / DRIVE / TURN / MOVE | 4 | enum (32) | stored bit-field | 2 |
| Direction | stopped / forward / reverse | 3 | enum (32) | stored bit-field | 2 |
| Turn direction | left / right | 2 | enum (32) | stored bit-field | 1 |
| Requested speed | 1–5000 counts/s; 0 for STOP | 5001 | `INT` (32) | unsigned bit-field | 13 |
| MOVE distance | 1–2,147,483,647 counts | — | `INT` (32) | unsigned bit-field | 31 |
| TURN angle | 1–360° | 360 | `INT` (32) | unsigned bit-field | 9 |
| DRIVE steering | −1000…+1000 permille | 2001 | `INT` (32) | signed bit-field | 11 |
| Timeout / run duration | 0 or 100–60,000 ms | — | `uint32_t` each | bit-field | 16 each |
| Motion status | 6 states | 6 | enum (32) | bit-field | 3 |
| Terminal fault cause | 9 causes | 9 | enum (32) | bit-field | 4 |
| Encoder totals / history | −8,388,608…8,388,607 counts | 2²⁴ | `INT` (32) | explicit 3-byte value | 24 each |
| Published wheel speed | −8192…8191 counts/s | 2¹⁴ | `INT` (32) | signed bit-field (B1 sends 14 bits) | 14 each |
| Previous quadrature state L/R | 0–3 | 4 | `UINT` (32) each | one packed byte | 2 each |
| Applied PWM magnitude | 0–35 % | 36 | `INT` (32) each | bit-field | 6 each |
| Wheel sign | −1, 0, +1 | 3 | `INT` (32) each | signed bit-field | 2 each |
| Speed-history count / index | 0–10 | 11 | `UINT` (32) each | bit-field | 4 each |
| Motor stopped / STOP latch | false / true | 2 | `bool` each | packed safety object | 1 each |
| Command ID, safety generation | 0–4,294,967,295 | — | `uint32_t` | kept | 32 each |
| RTOS times, PI terms, travel | existing 64-bit / float domains | — | kept | kept | 64 / 32 |

Command IDs, generations, RTOS times and PI arithmetic keep their full width:
they wrap or carry precision that a narrower type would lose.

## 2. Measured size comparison (Cortex-M0+)

Measured 2026-11-25 with `arm-none-eabi-gcc` (Cortex-M0+, `-O2`) by
`python testing/subsystem2/host/measure_sizes.py --before 7bad6fb`:

| Object | Before (B) | After (B) | Saved (B) |
|---|---:|---:|---:|
| Pending command slot | 40 | 24 | 16 |
| Current status snapshot | 60 | 48 | 12 |
| Eight retained results | 480 | 384 | 96 |
| Four sample pool blocks (one mailbox message is 20 → 16 B) | 80 | 64 | 16 |
| ISR encoder totals, both wheels | 8 | 6 | 2 |
| **Static and pool storage** | **668** | **526** | **142** |
| PID controller state (task-local) | 424 | 392 | 32 |
| … of which encoder speed history, both wheels | 88 | 66 | 22 |
| Command input (API argument, kept wide for validation) | 40 | 40 | 0 |

The PID state measured 368 B straight after the change; the encoder-dropout
handling added on 24 Nov (open-loop drive while an encoder is silent) brings it
to 392 B, still 32 B under the original. It lives in the task's existing stack
reservation, so it does not change the firmware's static RAM.

## 3. Correctness evidence

`testing/subsystem2/host/run_tests.ps1` (43,091 host assertions on 2026-11-25): round trips
through the real pending slot for every steering value −1000…1000, maximum
speed, MOVE distance, maximum durations and 360° turns; signed 24-bit
boundary and sign-extension tests; all 16,384 published speed values; encoder
rollover; values that would alias after truncation are rejected. See
`TEST_LOG.md`.

## 4. Timing (performance)

Measured 2026-11-26 on the Cortex-M0+ instruction set (arm-none-eabi-gcc `-O2`, cycles counted in an emulator with RP2040 timings) by `sh testing/timing/run_timing.sh`; log `testing/timing/logs/2026-11-26_m0_cycles.txt`, method and limits in `testing/timing/README.md`.

| Path | Before (`7bad6fb`) | After (`dc2cc5a`) | Budget |
|---|---:|---:|---|
| Encoder GPIO ISR, per edge | 106 cycles / 0.84 µs | 202 cycles / 1.62 µs | ≤ 10,000 edges/s (both wheels at 5000 counts/s) → ≤ 1.6 % CPU; 100 µs between edges |
| Store + load one motion command | 52 cycles / 0.42 µs | 162 cycles / 1.30 µs | at most once per 20 ms controller cycle → 0.007 % |

The 3-byte encoder total and packed command slot cost extra shift/mask
instructions, but the ISR still finishes in under 2 % of the time between
edges at top speed, so no edge is missed and the 10 ms PID loop is unaffected.
