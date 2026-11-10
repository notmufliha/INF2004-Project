# Subsystem 3 — test log

**Current implementation:** sample consumer example · **Priority:** 16 ·
**Core:** 0 on SMP · **Code:** `app_program/subsystem3/`

The attached Week 6 design review assigns Buddy 3 to IR line following and
barcode decoding. Those robot requirements are not yet implemented in this
checkout; see `app_program/subsystem3/README.md`. Do not record them as passed
based on the sample-consumer tests below.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Count, min, max, and sum for a known sequence | Statistics helper | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | Live statistics bounds and mutex usability | IF-04 | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| INT-01 | Samples move from S2 to S3 and update count | IF-01, IF-02 | auto | `subsystem3_test.c`; integration log | `[TEST] INT-01 PASS` |

Test implementation is built with `make TEST=1 -j8` from `build_make`, then
flashed to the Pico W. This firmware uses a `.uf2` image, not `sysmain.exe`.

## Results

No hardware test results have been recorded in this change. Add one row for
each actual run with date, tested commit, tester, result, and saved evidence.

## Calibration host checks, 2026-11-10

The integrated robot now runs the B3 line/barcode implementation. The sample
consumer checks above describe the legacy demonstration profile.

| ID | Check | Expected result |
|---|---|---|
| S3-C01 | Different sensor sensitivities | Equal normalized surfaces |
| S3-C02 | Grey floor after a white/black sweep | Below line threshold |
| S3-C03 | Parked on black | Contrast retained; white recognized immediately |
| S3-C04 | 12-bit ADC range | Output stays within 0..1000 |
| S3-C05 | Low/high contrast first passage | Black recognized immediately |
| S3-C06 | 30,000 identical samples after a sweep | Endpoints and normalized values remain fixed |
| S3-C07 | White/black ranges below provisional raw threshold 300 | Both sensors normalize white/black to 0/1000 after sweep |

| Date | Test IDs | Source | Tester | Result | Evidence |
|---|---|---|---|---|---|
| 2026-11-10 | S3-C01 through S3-C06 | Working tree based on `62467ca5a8e84b14ebbcc3d1cef91a188af7dbb2`, including calibration changes | Codex host runner | PASS | [Transcript](../integration/logs/2026-11-10_143429_host.txt) |

No physical line-tracking result is claimed. See `ROBOT_TEST_STEPS.md` at the
project root for calibration, direction checks and controlled floor testing.

| Date | Test IDs | Source | Tester | Result | Evidence |
|---|---|---|---|---|---|
| 2026-11-10 | S3-C01 through S3-C07; VC/B5 snapshot timing | Working tree based on `62467ca5a8e84b14ebbcc3d1cef91a188af7dbb2`, including 64-count learning and clock-after-snapshot fixes | Codex host runner | PASS | [Transcript](../integration/logs/2026-11-10_150149_host.txt) |
