# Subsystem 3: sample consumer (current integrated example)

This folder contains the consumer side of the current project example. It
receives bounded sample messages from Subsystem 2, updates shared statistics,
returns the sample block and producer slot, and sends a short report to
Subsystem 4 every five samples.

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
line update interval and timestamped barcode edge capture, with threshold
values and decoding limits still subject to calibration/testing. The teaching
consumer remains available. The optional line-only integration below provides
Buddy 5 with real line samples; full barcode and mission navigation integration
is separate.

## Buddy 5 autonomous integration

`b3_line.c` / `b3_line.h` provide the timestamped GP26/GP27 line input used by
Buddy 5 autonomous mode. The pin mapping, low-on-line polarity, 2048 ADC
threshold and 20 ms sampling interval come from this project's B3 code at
commit `912c369`. Only the line sampler was extracted; this does not restore
that branch's barcode/navigation changes. Robot startup initializes and
starts this sampler only when `ROBOT_ENABLE_BYPASS=1`. The teaching consumer
retains its original lifecycle when `ROBOT_MODE=0`.

B3 owns the ADC handle and state mutex. `b3_get_timed_state()` returns a
complete successful ADC pair and its timestamp, or an error; failed ADC
reads invalidate availability. B5 rejects samples older than 100 ms. The
2048 threshold and geometry still require physical calibration. See
`../subsystem5/README_BUDDY5.md` for the integrated feature and build checks.
