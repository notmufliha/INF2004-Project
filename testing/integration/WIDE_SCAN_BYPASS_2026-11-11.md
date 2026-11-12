# Obstacle bypass blocked by the scan window

Superseded on 12 November 2026: the user requested restoring 70–110 degrees.
The current implementation uses course bounds and a post-turn clearance
check instead; this file records the earlier reverted approach.

The supplied COM6 capture shows the robot moving, detecting an obstacle
around 23 cm, stopping, scanning, and latching `FINE_SCAN`. No `[S5-ROUTE]`
or `[S5-MOVE]` occurs: the bypass executor never receives a route.

The old coarse rays were 70/80/90/100/110 degrees. Four rays still hit the
obstacle; the fine scan at 70/75/80/85/90 also clips its silhouette. A valid
profile needs observable boundaries, so changing stop distance alone cannot
repair this failure.

The coarse scan now uses 30/60/90/120/150 degrees. Servo limits are 30–150
degrees, using the existing gentle stepping and pulse mapping (1166–1833 us).
Check physical mount clearance across this range before floor testing.

A clipped but valid fine scan can use the wide coarse profile. When only
the centre coarse ray hits, four valid clear side rays are required. The
adjacent clear-ray angles bound the visible width at the frontal distance
using x = y * tan(angle). This deliberately uses the full angular gap,
rather than reporting a precise width from a single hit. The existing size,
side-clearance, live-front, encoder and line-confirmation checks still apply.
This remains a visible silhouette estimate, not a full depth measurement.

Regression coverage includes the wide servo angles, a clipped 22 cm
obstacle progressing through STOP, scans, GO_LEFT, FIND_LINE and RESUME,
right-side preference, invalid rays, an unbracketed obstacle, and an
oversized width bound. Existing executor tests cover left/right physical
command sequences, bounded line search and alignment confirmation.

Firmware startup now prints `B5-WIDE-SCAN-BYPASS` with compile date/time,
scan range, stop distance (25 cm), and release distance (30 cm).
Generated network UF2 SHA256:
`B9037A087849BB6746441795A27DD355FFE17CA26EFE10E23C803634573696AF`.

Physical acceptance requires `[S5-ROUTE]` followed by encoder-tracked
`[S5-MOVE]` legs, tape reacquisition, and resumed following, plus a STOP
check during bypass. If the wider scan still cannot observe a route, stop
and inspect the new scan readings rather than authorizing blind travel.
The course bounds remain width <=30 cm and depth <=40 cm. No physical
robot pass is claimed from host tests.
