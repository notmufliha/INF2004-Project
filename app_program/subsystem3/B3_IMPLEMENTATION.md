# Buddy 3 line and barcode module

`b3_line_barcode.c` / `b3_line_barcode.h` add a separate Buddy 3 module in
the current Subsystem 3 folder. The integrated robot startup starts the
Buddy 3 tasks and the Vehicle Controller consumes their samples.

## What the module does

- Reads digital comparator outputs GP26/GP27 every 20 ms. The default polarity
  is active-low (`B3_LINE_ACTIVE_LEVEL=0`): black is reported as 1000 and
  white as 0. Override `B3_LINE_ACTIVE_LEVEL` to `1` at build time if the
  installed modules assert HIGH over black. Adjust each sensor's comparator
  potentiometer so it switches cleanly at the course edge. The controller and
  dashboard receive these binary 0/1000 values; no ADC calibration sweep is
  required.
- Reports `DETECTED` if either sensor sees black and `LOST` if neither does.
  GP27 is physically on the robot's left, and GP26 is physically on its right.
- While following, uses weighted line position and PD steering from commit
  `503ed6e` on `line-follow-pd`, integrated into the current robot branch.
  GP26 named `left` is physically on the robot's right, and GP27 named
  `centre` is physically on its left. Equal readings drive straight; a line
  under only the right sensor steers right and under only the left sensor
  steers left. The weighted position is 0 left, 500 centre, or 1000 right.
  Steering is `(16*error + 10*delta_error)/10`,
  capped at 800 permille. Steering applies whenever
  the line is detected, including when both sensors see it. The car slows on
  curves (down to 60% at full steer) and only speeds back up gradually.
- If line flags report `LOST`, the controller first steers hard toward the
  last seen side for one second, then searches in widening sweeps for up
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
  It does not sample the line sensors, decode a barcode, print, or send a
  kernel message.
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

1. Agree and claim GP26, GP27, and GP28 in `TEAM.md`. GP26/GP27 are configured
   as digital inputs for the line sensors; GP28 is the digital barcode
   interrupt input.
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
5. Before a run, lift the drive wheels and move each line sensor across white
   and black. Confirm its digital output changes state at the line edge and
   that the dashboard reports black as 1000. If it reports black as 0, build
   with `B3_LINE_ACTIVE_LEVEL=1`. Verify that a line under the left sensor
   commands a left correction and a line under the right sensor commands a
   right correction before running on the course. Tune the comparator
   potentiometers and PD gains only after confirming turn direction.

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
