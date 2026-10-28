# Buddy 2 PI tuning report — hardware measurements pending

Current controller uses PI (PID with derivative gain zero). Encoder quantization
and noise motivate starting with D=0; retaining that choice requires measured
stability and accuracy. Do not report preliminary gains as experimentally tuned.

| Parameter | Initial value | Validation |
|---|---:|---|
| Period | 10 ms | Record maximum interval under Wi-Fi/sensor load |
| P gain | 0.002 PWM-percent per count/s | Pending |
| I gain | 0.005 PWM-percent per count | Pending |
| Integral bound | +/-10 PWM-percent | Pending |
| Output bound | 0..35 percent | Pending |
| Base output | 20 percent | Measure motor deadband first |
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
