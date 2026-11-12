# Compare both sides and monitor obstacles during turns

The user confirmed the sensor does not point right at the logged 70-degree
position. Servo orientation and the logical 70=left, 110=right mapping are
therefore unchanged; no speculative inversion was applied.

The measured-profile chooser previously returned as soon as the two-ray
planner found a route. This could approve RIGHT when its two rays were
clear while ignoring a larger LEFT outer opening whose inner ray clipped
the obstacle. The controller now compares the valid outer candidate
openings first and selects the one with more room. An oversized measured
obstacle still prevents a route, and the course-bound fallback retains its
explicit width/depth assumptions. A tie retains the left preference.

Obstacle readings continue on every approximately 65 ms control cycle
while an asynchronous encoder turn is running. The servo remains at the
forward 90-degree position; this is continuous ranging as the chassis
turns, not a blocking servo sweep during motor movement. Turn clearance
remains 15 cm and invalid readings stop the maneuver. The one-second
forward leg retains the 20 cm forward stop threshold.

Added `S5-CHOICE` direction/side-range output and rate-limited
`S5-TURN-GUARD` readings during active turns. Regression tests reproduce
the previously wrong-right choice with LEFT=200 cm, RIGHT=150 cm and a
clipped inner left ray, then verify GO_LEFT. Other tests keep turning
through multiple clear samples, introduce a close obstacle mid-turn, and
verify immediate STOP for both turn directions. Invalid mid-turn samples
also stop. Existing tests retain both turn signs and the forward sequence.

All host suites passed, with the B5 suite rerun after the additional
mid-turn checks. The network UF2 built successfully. Physical operation
still requires verification on the robot.

Startup marker: `B5-SIDE-CHOICE-LIVE-GUARD`, `straight_trim=100`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `F51DBABF306DF18D60DB6F5FEDECAFDA1DE18F7E4CA615206A9D0C50C0607061`.
