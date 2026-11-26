# Change Requirement 1 — timing and performance evidence

Week 13 item 4 asks for evidence that the minimum-bit representation causes
"no unacceptable degradation in timing or system performance". This folder
measures, for every subsystem, the code path the change touched, **before and
after** the change, on the real target instruction set.

## Method

1. `build_elf.sh` compiles the subsystem's own sources with the project's
   `arm-none-eabi-gcc` for Cortex-M0+ (`-mcpu=cortex-m0plus -mthumb -O2`),
   plus a small harness (`*_timing.c`) that calls the path with typical
   values. "Before" is built from a git worktree of the commit before the
   change.
2. `m0_cycles.py` runs the function in a Cortex-M0+ emulator (Unicorn),
   counts every executed instruction and converts them to cycles with the
   Cortex-M0+ timings (TRM table 3-1: loads/stores and taken branches
   2 cycles, `BL` 3, `PUSH`/`POP` 1 + registers, RP2040 single-cycle
   multiplier). Time = cycles at 125 MHz.
3. Each harness also prints a result (frame length, decoded value, accepted
   samples) so a broken build cannot produce a fake fast time.

Run everything: `PY=<python with unicorn capstone pyelftools> sh testing/timing/run_timing.sh`.
Log of the run below: `logs/2026-11-26_m0_cycles.txt`.

**Limits.** Flash wait states/XIP cache misses and interrupt latency are not
modelled (both builds are measured the same way, so the comparison holds).
This is an instruction-level measurement, not a stopwatch on the car; the
budgets below are the periods the firmware runs these paths at.

## Results (2026-11-26, arm-none-eabi-gcc 16.1.0)

| Sub-system | Path (how often it runs) | Before | After | Change | Share of CPU after |
|---|---|---:|---:|---|---|
| B1 | build one telemetry message (every 500 ms) | 21,078 cyc / 168.6 µs, 299 B JSON | 4,100 cyc / 32.8 µs, 30 B binary | **5.1× faster**, 10× fewer bytes to send | 0.007 % (was 0.034 %) |
| B2 | encoder GPIO ISR, per edge (≤ 10,000 edges/s, both wheels at 5000 counts/s) | 106 cyc / 0.84 µs | 202 cyc / 1.62 µs | +0.77 µs per edge | ≤ 1.6 % at top speed (was 0.8 %) |
| B2 | store + load one motion command (at most once per 20 ms controller cycle) | 52 cyc / 0.42 µs | 162 cyc / 1.30 µs | +0.88 µs | 0.007 % |
| B3 | write one line-sensor sample into the shared snapshot (every 20 ms) | 25 cyc / 0.20 µs | 50 cyc / 0.40 µs | +0.20 µs | 0.002 % |
| B3 | controller reads the whole snapshot (every 20 ms) | 25 cyc / 0.20 µs | 36 cyc / 0.29 µs | +0.09 µs | 0.0015 % |
| B4 | filter + terrain/motion analysis, per IMU sample (10 ms) | 7,457 cyc / 59.7 µs | 7,318 cyc / 58.5 µs | **1.9 % faster** | 0.59 % |
| B5 | hand on one obstacle scan result (once per servo scan) | 195 cyc / 1.56 µs (56 B copy) | 2,928 cyc / 23.4 µs (pack + unpack, 6 B) | +21.9 µs per scan | < 0.12 % even if it ran every 20 ms cycle |

B4 per-sample figures are the 20-sample run divided by 20.

## Conclusion

Packing costs a few shift/mask instructions per access, so the paths that
only read or write fields (B2 ISR and command slot, B3 snapshot, B5
hand-over) got slower by **at most 22 µs per event**. Each path still uses
under 2 % of the CPU at its worst-case rate, and none comes close to its
period (the B2 ISR, the tightest, takes 1.6 µs per edge against
100 µs between edges at top speed). The paths that build or process messages
got **faster**: B1's binary telemetry is 5× quicker to build than the JSON
it replaced and puts 10× fewer bytes on the WiFi link, and B4's analysis is
2 % quicker. No subsystem loses functionality: the correctness tests are
listed in each `testing/subsystemN/DATA_REPRESENTATION.md`.
