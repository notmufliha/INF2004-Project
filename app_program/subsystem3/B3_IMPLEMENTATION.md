# Buddy 3 line and barcode module

`b3_line_barcode.c` / `b3_line_barcode.h` add a separate Buddy 3 module in
the current Subsystem 3 folder. The existing sample consumer is intentionally
left intact; `app_main.c` is a shared integration point and does not start
the Buddy 3 tasks yet.

## What the module does

- Reads ADC channel 0 (GP26) and channel 1 (GP27), nominally every 20 ms.
- Treats readings from 301 through 4100 on either GP26/left or GP27/centre as
  line-active. It reports `DETECTED` if either sensor is active, `LOST` if
  neither is active, and `JUNCTION` if both are active.
- While following, adjusts steering proportionally to the raw ADC difference.
  The ADC named `left` is physically on the robot's right, and the ADC named
  `centre` is physically on its left. Similar readings drive straight; a
  stronger right sensor steers right and a stronger left sensor steers left.
  A 50-code deadband filters small differences, with steering capped at
  800 permille. The correction reaches its cap at a 500-code difference.
  Steering applies whenever the line is detected, including when both
  sensors see it. The car slows on curves (down to 50% at full steer) and
  only speeds back up gradually after the curve.
- If line flags report `LOST`, the controller searches at half the selected
  speed, initially toward the last detected side, in widening sweeps for up
  to 8 seconds, then stops but stays armed until the line is seen again.
  It never searches without fresh ultrasonic clearance.
- `JUNCTION` means both sensors read at least `B3_LINE_JUNCTION_MIN` (3500)
  and within `B3_LINE_JUNCTION_BALANCE` (300) of each other. Calibrate the
  minimum from the dashboard IR values over a real junction. Both sensors
  merely above 300 is normal line following, not a junction.
- In AUTO mode, a barcode route is held as pending until three consecutive
  junction samples confirm both sensors are on the junction. The robot slows
  to 75% after receiving a route, then drives straight at half speed through a
  junction without a route. After three non-junction samples the junction
  latch clears. A pending route executes at the confirmed junction.
- Reads GP28 as the digital barcode input and registers both-edge GPIO
  interrupts through the shared `app_gpio_irq_dispatch()` subscriber.
- The GP28 callback only timestamps and records edge levels in a bounded
  ring, sets the pending/overflow flags, and acknowledges GP28's edge bits.
  It does not call the ADC, decode a barcode, print, or send a kernel message.
- A separate task drains GP28 edges and decodes the Code 39 widths, then
  pushes valid `uint8_t` navigation command values to `mbf_nav_cmd` with
  `tk_snd_mbf(..., TMO_POL)`. μT-Kernel's message buffer serializes concurrent
  senders and copies the one-byte message. A full queue is reported, not
  silently treated as a successful send.
- Uses the packed five-byte `B3_STATE` snapshot under `mtx_b3_state`, and
  updates the existing B1 telemetry/event APIs from task context.
- Publishes line readings and status only; it does not issue B2 motion
  commands. The Vehicle Controller must own steering and safety arbitration.

## Integration steps

1. Agree and claim GP26, GP27, GP28 and the ADC peripheral in `TEAM.md`.
   This change enables ADC pin initialization for channels 0 and 1 in
   `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`. Channel 2 / GP28 remains
   disabled because GP28 is a digital interrupt input in this module.
2. The Vehicle Controller must receive one `uint8_t` from `mbf_nav_cmd`.
   Values are 0 none, 1 left, 2 right, 3 straight, 4 U-turn. The queue is
   created by `b3_init()`; receive with `tk_rcv_mbf(mbf_nav_cmd, &command,
   TMO_FEVR)` or poll with `TMO_POL`.
3. Coordinate task ownership and priority with the current Subsystem 3 demo:
   this module uses priorities 16 and 17 on core 0, while the example
   `consumer_task` already uses priority 16. Do not start both at the same
   priority without agreeing on the project task table.
4. After the integration point is agreed, initialize Subsystem 1 first so
   its telemetry mutex and event queue exist; then call `b3_init()` before
   starting tasks and `b3_start()` from core 0. The sample `subsystem3_init()`
   and `subsystem3_start()` lifecycle currently do not call either function.
5. Test ADC readings with the actual line sensors and floor. Readings at or
   below 300 or above 4100 count as inactive; verify that white-surface readings
   stay outside the active range before relying on line-loss detection.
   The steering deadband and correction scale are initial tuning values; test
   with the wheels raised and adjust them only after confirming turn direction.

## Code 39 barcode behavior

The GP28 digital IR input records both edges as the robot crosses the Code 39
strip. The decoder groups the nine alternating bar/space widths in each
character and separates narrow from wide elements by their relative widths,
so decoding does not depend on a fixed robot speed. It requires the standard
`*` start/stop characters and accepts one-character payloads only, scanned
in either direction (a strip crossed from the far end arrives reversed and
is matched against the mirrored patterns):

| Code 39 payload | Recorded barcode | Navigation |
|---|---|---|
| `*A*` | A | Straight |
| `*Z*` | Z | Right |

Other Code 39 payloads and malformed scans are ignored. A bounce on GP28
(rise and fall in one interrupt) records the settled level and merges into
the surrounding element instead of discarding the barcode. The decoder currently
uses alternating edge levels, so either black/white electrical polarity is
supported. The track barcode is approximately 144 mm long; this length is not
used as a decode threshold.

The controller retains a decoded navigation command until three consecutive
junction samples confirm the junction. Test the scan and route first on the
track at low speed; reject incorrect or incomplete decodes rather than
turning from a partial barcode.

## Minimum-bit RAM estimate

`B3_STATE` contains 34 meaningful bits: 12 + 12 + 1 + 3 + 3 + 3. With the
requested `unsigned int` bit-fields and `packed` attribute, the current RP2040
GCC toolchain makes the structure 5 bytes (40 bits); six ordinary 32-bit integer
members would use 24 bytes. This saves **19 bytes (about 79%)** per snapshot.
Six bits remain unused. The header asserts the 5-byte size because this layout
is compiler-specific. It must not be used as a serialized wire format.
