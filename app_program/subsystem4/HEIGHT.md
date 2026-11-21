# One-wheel hump height estimate

Completed roll crossings now estimate peak wheel height difference:
`height_mm = wheel_track_mm * sin(peak_relative_roll)`.
The measured tyre centre spacing is **110 mm**, configured by
`S4_WHEEL_TRACK_MM` in `s4_runtime.h`. The analysis default is also 110 mm.

The mean roll of the existing progressing level-baseline readings supplies the
reference. The largest absolute excursion is retained from the first entry
confirmation through the active crossing. Either wheel is supported. Existing
roll thresholds, forward checks and completion rules are unchanged.

Height becomes valid only on a completed event. Interrupted crossings do not
publish a height. Excursions above 90 degrees leave height unavailable.
The highest completed estimate persists through detector resets until analysis
initialization/reboot. Existing centimetre event fields carry estimates; console
and B1 paths convert them to millimetres. Public reading/event layouts stay the
same. Diagnostics count accepted estimates.

## Peak roll (edge jolt rejection)

The peak is not the single highest sample. When a wheel hits the hump edge
the chassis jolts; the IMU sits above the roll axis, so the jolt reads as
several degrees of extra roll for a few samples (a 10 mm hump measured
17 mm this way). The peak is now the trimmed mean of the last
`S4_ROLL_HUMP_PEAK_SAMPLES` (7) unfiltered accelerometer rolls, dropping the
top and bottom third, so jolts of up to 2 samples are ignored and the plateau
on top of the hump is kept. The level baseline also follows slow drift while
driving level between humps. If narrow humps now read low, lower the window
(the wheel must stay on top for about half of it); 1 restores the old peak.

## Assumptions and validation

One wheel must stay on level ground while the other crosses the hump, with both
tyres maintaining contact. This estimates wheel height difference, not directly
the surface height. Tyre deformation, chassis flex, acceleration, filtering and
sparse samples affect accuracy. It does not model both wheels rising together.

Test both wheel sides against a ruler-measured hump. Start level and return to
level before the movement test stops. Compare the printed `Hump estimate` over
repeated trials. Host tests verify math and event handling; physical accuracy
remains unvalidated. For reference, 30 degrees with 110 mm spacing gives 55 mm.

The retired pitch-height module stays removed. This model lives within
`imu_analysis.c`; it adds no new module. Mock heights use synthetic readings.
