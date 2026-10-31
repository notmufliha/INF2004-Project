# Subsystem 3: sample consumer and Buddy 3 module

This folder contains the consumer side of the current project example. It
receives bounded sample messages from Subsystem 2, updates shared statistics,
returns the sample block and producer slot, and sends a short report to
Subsystem 4 every five samples.

It also contains a separate Buddy 3 line/barcode module in
`b3_line_barcode.c` / `b3_line_barcode.h`. The existing sample consumer remains
the active integration example; the B3 module is not started by `app_main.c`
until the team agrees on the shared startup and Vehicle Controller queue
interfaces. See [B3_IMPLEMENTATION.md](./B3_IMPLEMENTATION.md) for setup,
queue, threshold, and barcode protocol details.

## Task and objects

| Item | Definition |
|---|---|
| Task | `consumer_task`, priority 16 (Subsystem 3 band 16–17), pinned to core 0 on SMP builds |
| Startup | `subsystem3_start()` calls `app_start_task()`, which performs `tk_cre_tsk()` then `tk_sta_tsk()` |
| IF-01 mailbox | `mbx_samples`, created and owned by S3; carries `SAMPLE_MSG *` from S2 |
| IF-02 return path | S3 calls `tk_rel_mpf(mpf_samples, msg)` and `tk_sig_sem(sem_slots, 1)`; both objects are owned by S2 |
| IF-03 report | S3 sends copied text to S4's `mbf_summary`, then sets `FLG_BATCH` in `flg_events` |
| IF-04 statistics | `app_stats` is protected by S3's `mtx_stats`; S4 must hold the mutex to read it |

`APP_STATS` contains the received count, minimum, maximum, and signed sum.
The current producer generates integer values in the range 0–100. The mutex
guards every shared read/write; reporting and sample release happen outside
the critical section.

## Execution and checks

The consumer blocks in `tk_rcv_mbx(..., TMO_FEVR)`. For each message it locks
the statistics mutex, updates the aggregate, unlocks, optionally reports each
fifth sample with a nonblocking message-buffer send, and returns the memory
block and slot. Kernel-call errors are logged; failures to lock still release
the producer-owned resources.

Automated tests live in `subsystem3_test.c`: S3-U01 checks count/min/max/sum
using known values, S3-U02 checks the live statistics and mutex. INT-01 checks
S2-to-S3 sample flow. Run from `build_make` with `make TEST=1 -j8`, flash the
resulting Pico W UF2, and save console output under `testing/subsystem3/logs/`.

```sh
cd build_make
make clean && make all
```

This repository is the micro T-Kernel Pico W firmware project. It builds a
`.elf` and `.uf2`; it does not produce `sysmain.exe` or support a `./sysmain.exe`
invocation. Follow the root README for toolchain, flash, and console steps.

## Design review alignment

The supplied Week 6 design review assigns Buddy 3 to analogue IR line
following (GP26/GP27), digital right-side barcode sensing (GP28), junction and
line-loss reporting, and navigation command generation. It proposes a 20 ms
line update interval and timestamped barcode edge capture. Physical thresholds
and the A-D pulse-width encoding are not specified in the current repository;
the new module leaves barcode classification disabled until those values are
confirmed.
