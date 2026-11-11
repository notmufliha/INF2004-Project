# Buddy 2 PI tuning report — hardware measurements pending

Current controller uses PI (PID with derivative gain zero). Encoder quantization
and noise motivate starting with D=0; retaining that choice requires measured
stability and accuracy. Do not report preliminary gains as experimentally tuned.

| Parameter | Initial value | Validation |
|---|---:|---|
| Period | 10 ms | Record maximum interval under Wi-Fi/sensor load |
| P gain | 0.006 PWM-percent per count/s (was 0.002) | Pending |
| I gain | 0.005 PWM-percent per count | Pending |
| Integral bound | +/-10 PWM-percent | Pending |
| Output bound | 0..35 percent | Pending |
| Feed-forward | 15 + 0.004 x target counts/s percent (was fixed 20) | Measure loaded deadband (15) and PWM at a known speed (slope) |
| Static-friction kick | +8 percent after 100 ms without encoder progress | Check it frees a stuck wheel without lurching |
| Speed history | 100 ms | Check transient response and quantization |
| Straight correction | 2 (mm/s)/mm, capped at 25% base speed | Pending |
| Approach zone | Final 30 mm per wheel | Measure overshoot |

1. Verify each wheel's motor direction and encoder sign with wheels raised.
2. Measure loaded deadband, speed versus PWM, and usable target-speed range.
3. Confirm counts/revolution across at least ten marked wheel revolutions.
4. Log reference/measured wheel speeds and supply voltage for speed steps in
   both directions. Tune P first, then I; keep output/integral limits active.
5. Measure steady error, settling time, overshoot and straight-line deviation.
6. Repeat on battery and under sensor/Wi-Fi load. Record control interval maxima.

| Date/build | Supply/load | Gains | Target mm/s | Steady error % | Settling ms | Overshoot % | Max interval ms | Evidence |
|---|---|---|---:|---:|---:|---:|---:|---|
| Pending | | | | | | | | |

Provisional acceptance: steady wheel-speed error within 10% over the validated
operating range after 1 second, no sustained oscillation, and no control interval
above 20 ms under full load. Replace these targets with justified team-approved
values if measurements show a different operating envelope.

## Straight-line trim trial, 2026-11-10

User reports gradual right drift on a straight line. No simultaneous sensor
and wheel-speed measurements were supplied. Encoder resolution, turn scales
and PI gains remain unchanged. VC now applies a provisional -25 permille
steering trim near straight ahead (L=0.975v, R=1.025v in physical-speed targets).
It fades to zero at 200 permille steering and is disabled at junctions and
during barcode crossing. `ROBOT_LINE_STEER_TRIM_PERMILLE=0` disables the trial.

The low-speed limiter also preserves gentle steering ratios instead of
turning both sub-minimum targets into a one-wheel pivot. Host tests cover
both trim directions, trim disabled, mild steering from -200..200, wheel
minimums and maximum targets; the full host suite passed.
[Transcript](../integration/logs/2026-11-10_152135_host.txt).

Physical result pending: after a white/black sensor sweep, measure signed
lateral deviation in three one-metre straight runs at the same speed, surface,
payload and battery level. Record normalized IR values and wheel speeds.
Adjust trim from repeatable results, not by changing encoder geometry.

Follow-up: the user reports slight right drift with the latest image.
The next trial changes trim from -25 to -40 permille (L=0.960v,
R=1.040v near centre). Curve fade, junction/barcode exclusions, encoder
geometry and PI gains are unchanged. Physical tracking remains to be measured.

Further feedback: drift worsened after the -40 trial. Trial trim is now
disabled (0). A nonzero trim also bypasses `motion_profile_targets()`'s
zero-steering straight-travel correction, so increasing it did not establish
sensor direction or actual wheel imbalance. Verify physical channel mapping
before another directional adjustment.
