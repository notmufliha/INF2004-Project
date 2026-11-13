# Exact capture replay: inner opening was ignored

The supplied capture identifies the older B5-SIDE-CHOICE-LIVE-GUARD image
with 120 ms settling. The current build incorporates gentle 2-degree/35 ms
servo steps and 200 ms settling, within the same 70–110-degree range.

Actual coarse readings from the capture:

| Logical angle | Range (cm) |
| --- | --- |
| 70 | 16.94 |
| 80 | 152.39 |
| 90 | 20.46 |
| 100 | 152.79 |
| 110 | 18.86 |

The old decision only considered the outer 70/110 rays as turn candidates.
It therefore ignored the two inner openings. The fragmented silhouette
also failed both the measured profile and contiguous course-bound fallback,
leading to repeated coarse/fine scans and WAIT_ROUTE.

The decision now considers both rays on each side. A small difference
within 10 cm is treated as a tie and prefers LEFT; the captured 0.40 cm
side difference no longer flips the choice. When a fragmented silhouette
has a usable opening, the existing course-width/depth assumptions supply
the unknown-size bound instead of requiring a contiguous measured object.
Invalid rays are never treated as openings. A valid side ray within 15 cm
blocks that side even if the other ray sees far background. The later
5.33 cm frame in the capture still cannot authorize any turn.

Tests replay the exact five ranges and assert GO_LEFT directly after the
coarse scan. A test couples the decision to the actual motion adapter and
verifies a left turn command, timed forward motion, the opposite turn,
and a further forward leg. Additional tests cover a missing inner ray,
an 8 cm side obstruction, and the close 5.33 cm frame. All host suites
passed (`2026-11-12_121735_host.txt`). Network firmware built successfully.
This is software replay verification, not a physical course pass.

The one-second forward leg, continuous front monitoring during turns,
20/25 cm stop/release thresholds, and straight trim 100 remain.

Distinct flash image: `build_make/robot_inner_ray_gentle.uf2`.
Startup marker: `B5-INNER-RAY-TURN`.
SHA256: `49C64F22C210C78F1CE4F236B4ACA07967CB6520E9F87FF604B014EB19B88DA4`.
