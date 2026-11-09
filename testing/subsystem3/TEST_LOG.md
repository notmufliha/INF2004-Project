# Subsystem 3 test log

Subsystem 3 reads GP26/ADC0 and GP27/ADC1 for line following and captures
Code 39 barcode transitions on GP28. Host decoder tests do not verify ADC
pin mapping, sensor polarity, thresholds, GPIO timing, or robot motion.

## Automated checks

| Test | Coverage | Method |
|---|---|---|
| Code 39 host test | Forward/reverse `*A*` decoding and rejection of an incomplete capture | `testing/subsystem3/code39_host_test.c` |
| Vehicle Controller test | Navigation queue commands, hazards, DRIVE cadence, and task adapter | `testing/controller/run_tests.ps1` |
| Source syntax check | Changed B3 sources against host-visible T-Kernel declarations | GCC syntax-only check; not a firmware build |

On-target helper tests are available through `test_line_run()` and
`test_barcode_run()` when `APP_TEST` is enabled. `test_line_run()` tests the
pure `line_track` helpers; neither test validates real sensors.

## Hardware validation

No hardware results are recorded yet. Before enabling robot movement, test
each analog sensor separately on white and black, tune
`B3_LINE_LEFT_THRESHOLD` and `B3_LINE_CENTRE_THRESHOLD` in `cfg_line.h`, then
verify turns, barcode capture, and the ultrasonic clearance gate. Record the
date, firmware revision, tester, result, and evidence for each physical test.
