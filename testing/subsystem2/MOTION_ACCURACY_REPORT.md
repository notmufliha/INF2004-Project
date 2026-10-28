# Buddy 2 motion accuracy — hardware measurements pending

Geometry is preliminary: wheel diameter 6.5 cm, wheel track 9.7 cm,
left 2499 counts/revolution, right 2341. Validate scale independently for each
wheel. Tune effective track width with repeated left and right turns; wheel
slip means a ruler measurement alone is insufficient.

2026-10-28 user observation: 90-degree commands produced approximately
65..75-degree left/right body turns. Completion logs and repeated measurements
were not supplied. Provisional left/right turn scales are now 90/70 = 1.2857143,
equivalent to an effective turn track of about 12.47 cm. Physical wheel track
and straight-distance calibration remain unchanged. Both turn distance and
the derived deadline use the correction. This is a calibration trial, not a
verified accuracy result. If a run reports cancellation/timeout/fault, resolve
that first rather than increasing the scale.

Follow-up user evidence: right 90-degree turn is more accurate, while left
still turns approximately 60..70 degrees. Mode 4 reports
`id=1 result=2 fault=0 L_mm=-104 R_mm=106 stopped=1`, followed by PASS;
the user confirms both wheels turn throughout the manoeuvre. This confirms
encoder/controller completion, not the true body angle. Left scale is now
provisionally 1.2857143 * 90/65 = 1.7802198. Right stays at 1.2857143.
That trial subsequently overshot to a user-estimated 110..120 degrees.
The next provisional left scale is 1.7802198 * 90/115 = 1.3932155, using
the midpoint of that range. Right remains 1.2857143. Completion status for
the overshoot trial was not supplied. Measure several completed trials
before further tuning; these estimates do not establish repeatability.
No motor PWM/gains or straight-distance constants changed.

Further user observation: left scale 1.3932155 improved the turn but still
produced approximately 100 degrees. New provisional left scale is
1.3932155 * 90/100 = 1.2538940. Right remains 1.2857143. Completion logs
and repeated measurements for this trial were not supplied. Retest at least
three completed turns under identical conditions. Earlier similar scales
produced different reported angles, so check measurement repeatability,
surface, battery and wheel slip before another single-trial correction.

Retest modes 4 and 5 individually at the same speed, surface and load. Measure
at least five completed turns per direction; update each direction separately:
`new_scale = old_scale * requested_angle / mean_measured_angle`.

Use a marked flat surface and repeat each distance/turn at least ten times in
each direction. Record supply voltage, surface, payload, target speed, build,
signed wheel travel, actual body travel/angle and stopping distance. Keep motor
authorization and STOP active throughout tests.

| Case | Provisional acceptance | Measured result/evidence |
|---|---|---|
| Forward/reverse 20 cm | At least 9/10 within +/-1 cm; lateral drift <=1 cm | Pending |
| Left/right 90 degrees | At least 9/10 within +/-10 degrees | Pending |
| Left/right 180 degrees | At least 9/10 within +/-15 degrees | Pending |
| Asymmetric load | Straight correction reduces deviation without oscillation | Pending |
| One wheel stalls/missing encoder | FAULT/STALL; no false COMPLETED | Pending |
| DRIVE refresh removed | PWM zero within 100 ms plus one control period | Pending |
| STOP during MOVE/TURN | PWM off; old ID CANCELLED; no automatic restart | Pending |
| Obstacle at maximum validated speed | No contact; stop travel <=10 cm | Pending |
| Standstill confirmation | COMPLETED only after 100 ms encoder quiet with PWM off | Pending |
| Full load timing | Maximum control interval <=20 ms | Pending |

The 10 cm stop-travel target is provisional, not a measured guarantee. Measure
sensing-to-PWM latency and PWM-to-standstill distance separately, then select
obstacle thresholds and maximum speed with clearance margin. The configured
35 cm obstacle threshold alone does not establish worst-case stop performance.

| Date/build | Test | Requested | Measured | Error | Supply/surface | Evidence |
|---|---|---:|---:|---:|---|---|
| Pending | | | | | | |
