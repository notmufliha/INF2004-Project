# Buddy 3 line and barcode module

`b3_line_barcode.c` / `b3_line_barcode.h` add a separate Buddy 3 module in
the current Subsystem 3 folder. The existing sample consumer is intentionally
left intact; `app_main.c` is a shared integration point and does not start
the Buddy 3 tasks yet.

## What the module does

- Reads ADC channel 0 (GP26) and channel 1 (GP27), nominally every 20 ms.
- Auto-calibrates each sensor (`b3_line_cal.c`): it learns that sensor's own
  white and black readings (widening at once, slowly forgetting old extremes,
  never narrower than 300 counts; starts from 100..500, whose midpoint is the
  team's tested raw threshold of 300, and only widens from there) and normalises to 0 = white .. 1000 =
  black. The controller receives normalised values; the dashboard still shows
  raw ADC. This removes the bias between two sensors of different
  sensitivity, and a grey floor beside the course no longer reads as line.
- A sensor is line-active at normalised 500 or more (half way between its
  own white and black). It reports `DETECTED` if either sensor is active and
  `LOST` if neither is.
- While following, adjusts steering proportionally to the normalised difference.
  The ADC named `left` is physically on the robot's right, and the ADC named
  `centre` is physically on its left. Similar readings drive straight; a
  stronger right sensor steers right and a stronger left sensor steers left.
  A 40-point deadband filters small differences, with steering capped at
  800 permille, reached at a 400-point difference. Steering applies whenever
  the line is detected, including when both sensors see it. The car slows on
  curves (down to 60% at full steer) and only speeds back up gradually.
- If line flags report `LOST`, the controller searches at half the selected
  speed, initially toward the last detected side, in widening sweeps for up
  to 8 seconds, then stops but stays armed until the line is seen again.
  It never searches without fresh ultrasonic clearance.
- `JUNCTION` means both sensors read at least `B3_LINE_JUNCTION_MIN` (800
  normalised) and within `B3_LINE_JUNCTION_BALANCE` (150) of each other. Both
  sensors merely on the line is normal following, not a junction.
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
5. Before a run, slide the car across the line once or twice so each sensor
   sees white and black; calibration then settles on the course. Check on the
   dashboard that black reads at least ~300 counts above white.
   The steering deadband and correction scale are initial tuning values; test
   with the wheels raised and adjust them only after confirming turn direction.

## Code 39 barcode behavior

The GP28 digital IR input records both edges as the robot crosses the Code 39
strip. The decoder groups the nine alternating bar/space widths in each
character and separates narrow from wide elements by their relative widths,
so decoding does not depend on a fixed robot speed. It requires the standard
`*` start/stop characters and one-character payloads, decodes the full Code
39 set (0-9, A-Z, - . space $ / + %), and reads either direction: a strip
crossed from the far end has each character's elements reversed, so the
mask is mirrored and looked up in the same table. The narrowest wide element
must be at least 1.4x the widest narrow one. A level change that flips back
within 3 ms (sensor chatter, or a spike inside a bar) is merged into the
surrounding element. While bars are passing (an edge within 400 ms) the
controller holds a steady 500 counts/s (~41 mm/s), as the teams with
reliable on-car reads did.

Every decode and every rejection is printed on the USB serial console, for
example `[B3] barcode *A* read (forward) -> STRAIGHT at next junction`, or
`[B3] barcode after start '*' rejected: <reason>`. Letters act only if they
are in the `b3_routes` table in `b3_line_barcode.c`:

| Code 39 payload | Recorded barcode | Navigation |
|---|---|---|
| `*A*` | A | Straight |
| `*Z*` | Z | Right |

Other letters are logged as "not in route table". A bounce on GP28 (rise and
fall in one interrupt) records the settled level instead of discarding the
barcode. If nothing is ever logged when crossing a barcode, check the GP28
module's potentiometer: its LED must switch on every black bar and off on
every white gap (the dashboard's "IR right" shows this raw 0/1 level). The decoder currently
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
