# Buddy 3 line sensing and barcode input

Robot mode starts the B3 line and barcode tasks from `app_main.c`. The line
task reads GP26/ADC0 and GP27/ADC1 through the `adca` device, publishes raw
readings to B1 telemetry, and exposes a timestamped state snapshot to the
Vehicle Controller. GP26/left is active-low (black at or below
`B3_LINE_LEFT_THRESHOLD`, default 1000); GP27/centre is active-high (black at
or above `B3_LINE_CENTRE_THRESHOLD`, default 2100). Both thresholds are in
`cfg_line.h` and should be rechecked against the physical track.

The barcode task captures both-edge transitions from the digital sensor on
GP28 using the shared GPIO IRQ dispatcher. It decodes Code 39 `*A*` through
`*Z*` and sends each letter to the Vehicle Controller's `mbf_nav_cmd` queue.
B1 also receives a barcode event. Route assignment remains owned by the
Vehicle Controller and its MQTT `MAP` commands.

The controller consumes B3 readings for line-loss detection, steering, and
the two-sensor junction flag. This is only an interface integration: the
threshold, sensor mounting/pin mapping, steering behavior, and barcode timing
must still be verified on the actual robot. Keep the motors disabled until
both sensors respond distinctly over black and white and the ultrasonic
clearance task is enabled and verified.

The independent `line_track` helpers and Code 39 decoder have on-target test
entry points `test_line_run()` and `test_barcode_run()` when `APP_TEST` is
enabled. The host decoder test can be compiled with:

```text
gcc -std=c11 -Wall -Wextra -Werror testing/subsystem3/code39_host_test.c app_program/subsystem3/code39.c -o code39_host_test
```
