# Bypass with the original servo range

The later `SCAN_TURN_FORWARD_2026-11-12.md` updates side selection: an outer
opening selects a candidate turn, then fresh head-on sensing verifies the
forward leg. It also replaces autonomous reversing with stopped rescans.

Restored the 70–110-degree servo limits and 70/80/90/100/110-degree coarse
scan at the user's request. The wide-scan profile fallback was removed.

For a valid but clipped fine scan, the controller may instead use the
existing course limits: obstacle width <=30 cm and depth <=40 cm. These
are explicit assumptions, not ultrasonic measurements. The fallback
requires all scan samples valid, a contiguous clipped obstacle segment,
a frontal obstacle within the stop threshold but beyond the minimum turn
clearance, and an outer coarse ray with sufficient projected lateral room.

With the supplied coarse readings 24.30/23.52/24.81/23.52/160.29 cm,
the right outer ray projects to approximately 54.8 cm of lateral room.
The route requires 30 + 8 + 8 = 46 cm. It therefore chooses RIGHT, performs
the existing encoder-controlled outward turn, and verifies enough fresh
front clearance for the entire lateral leg plus chassis and margin before
translating. Every subsequent forward leg also receives that check.
Live front monitoring continues during movement. Unknown/blocked routes,
motion faults, stale line data, and expired search deadlines still stop.

The full course width is used laterally because the frontal ray may hit
an obstacle edge; no lateral centre or measured silhouette is inferred.
`[S5-ROUTE] course_bounds=1` identifies the fallback and obstacle telemetry
reports width as unknown. Measured, bracketed profiles keep their normal
route planning. The line-search/stop/alignment/handoff sequence remains.

Validation: all host suites passed, including replay of the supplied
narrow scan leading to GO_RIGHT, mirrored left preference, invalid/blocked
scan rejection, left/right full-width maneuver legs, post-turn obstruction
rejection, and telemetry distinguishing a width bound from a measurement.
The network UF2 built successfully; physical operation remains unverified.

Firmware startup marker: `B5-NARROW-COURSE-BYPASS`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `7898FE145A64BE93ECEE58189F46B08AC40E7920CAF5D5D60FBA3580AA61A6B8`.
