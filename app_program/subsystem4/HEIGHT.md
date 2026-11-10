# Height estimation status

The current robot detects one-wheel humps from roll. Height and highest height
are unavailable because a validated one-wheel height model is not implemented.
Pitch is still measured, filtered and printed; it does not drive hump detection
or climbing/peak/descending classification.

The old pitch hump state machine and its fallback switch have been removed.
The robot analysis task no longer initializes or calls the pitch/wheel-path
height estimator. There is no setting that restores pitch hump detection.

## Retired pitch-height module

The unused `imu_height.c/.h`, its isolated test and the geometry-only
`imu_odom_init()` helper have been removed. Git history retains the old math.
The old tracked `imu_mock_test` executable has also been removed; mock source
and tests remain available through the host test script.

Encoder acquisition and timing checks remain in `imu_odom` for roll detection
and diagnostics. Public height availability fields remain for integration,
but current robot hump events report height unavailable.

A future one-wheel estimator needs a suitable geometric model, measured wheel
track and validation against known heights before reporting valid estimates.
See [MOTION_TEST.md](MOTION_TEST.md) for the active roll detector and trials.
