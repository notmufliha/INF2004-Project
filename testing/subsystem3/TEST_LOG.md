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
