# B3 integration notes

- `b3_init()` runs after B1 and B2 initialization. It opens the ADC device,
  creates the shared B3 state mutex and the navigation message buffer, then
  initializes barcode capture.
- `b3_start()` starts the 20 ms line-sampling task and the GP28 barcode task.
- GP26/ADC0 is reported as `ir_left`; GP27/ADC1 is reported as `ir_centre`.
  Their active-high thresholds are independently configurable with
  `B3_LINE_LEFT_THRESHOLD` and `B3_LINE_CENTRE_THRESHOLD` in `cfg_line.h`;
  both default to 2048. B3 sets `DETECTED` when either sample is at/above its
  threshold, `LOST` when neither is, and `JUNCTION` when both are.
- `comm_set_line()` publishes B1 telemetry. The Vehicle Controller reads the
  timestamped B3 snapshot and navigation queue. Decoded letters also go to
  `comm_report_barcode()`; route selection remains in the Vehicle Controller.
- GP28 is digital barcode input, not a third analog line sensor. Its callback
  acknowledges only its own GPIO edge bits and records bounded edge samples;
  decoding and message delivery happen in task context.

The line threshold and steering are provisional. Verify both ADC channels
separately on white and black, then calibrate sensor placement and turns on the
track. The ultrasonic task is separately disabled by default for stationary
line tests; with it disabled, B2's fail-closed clearance gate must prevent
movement. Do not enable movement until valid ultrasonic clearance and both
line sensors have been checked.
