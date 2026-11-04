# Buddy 3 line and barcode module

`b3_line_barcode.c` / `b3_line_barcode.h` implement the Buddy 3 line and
barcode-sensor module in the Subsystem 3 folder. The existing sample consumer
is intentionally left intact for teaching-demo mode. Robot-mode startup in
`app_main.c` initializes and starts the B3 tasks.

## What the module does

- Reads ADC channel 0 (GP26) and channel 1 (GP27), nominally every 20 ms.
- Treats readings at or below `B3_LINE_THRESHOLD` as line-active. It reports
  `DETECTED` if either sensor is active, `LOST` if neither is active, and
  `JUNCTION` if both are active.
- Reads GP28 as the digital barcode input and registers both-edge GPIO
  interrupts through the shared `app_gpio_irq_dispatch()` subscriber.
- The GP28 callback only timestamps and records edge levels in a bounded
  ring, sets the pending/overflow flags, and acknowledges GP28's edge bits.
  It does not call the ADC, decode a pulse, print, or send a kernel message.
- A separate task turns the edge widths into Code 39 elements, decodes A-Z
  in either scan direction, and pushes the decoded letter value (1..26) to
  `mbf_nav_cmd` with `tk_snd_mbf(..., TMO_POL)`. A full queue is reported.
- Uses the eight-byte `B3_STATE` snapshot under `mtx_b3_state`, and
  updates the existing B1 telemetry/event APIs from task context.
- Publishes line readings and status only; it does not issue B2 motion
  commands. The Vehicle Controller must own steering and safety arbitration.

## Integration steps

1. Agree and claim GP26, GP27, GP28 and the ADC peripheral in `TEAM.md`.
   This change enables ADC pin initialization for channels 0 and 1 in
   `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`. Channel 2 / GP28 remains
   disabled because GP28 is a digital interrupt input in this module.
2. The Vehicle Controller receives a `uint8_t` barcode code from
   `mbf_nav_cmd`: 1 is A, ..., 26 is Z. It stores that character and waits
   for the next rising edge of the B3 junction flag before applying its
   configured route.
3. Coordinate task ownership and priority with the current Subsystem 3 demo:
   this module uses priorities 16 and 17 on core 0, while the example
   `consumer_task` already uses priority 16. Do not start both at the same
   priority without agreeing on the project task table.
4. In robot mode, `app_main.c` initializes Subsystem 1 before calling
   `b3_init()`, then calls `b3_start()` from core 0 after B4 readiness. The
   separate sample `subsystem3_init()` / `subsystem3_start()` lifecycle remains
   for the teaching demo and does not start this B3 module.
5. Test ADC polarity and thresholds with the actual line sensors and floor.
   The provisional threshold is 2048; it is not a validated track setting.

## Code 39 decoding and route mapping

The scanner recognizes Code 39 start/stop sentinels, classifies narrow/wide
elements per character, and tries both scan directions. It accepts a single
uppercase A-Z payload character per barcode. The route is not hard-coded:
the Vehicle Controller receives four unique letter-to-route assignments from
the laptop over MQTT before START. Commands are sent on `<prefix>/cmd`:

```text
MAP_RESET
MAP A FORWARD
MAP B UTURN
MAP C LEFT
MAP D RIGHT
```

Send one command per letter; all four routes (FORWARD, UTURN, LEFT, RIGHT)
must be assigned to distinct letters before START is accepted. `MAP_RESET`
clears the current table. The seven-bit MAP argument stores the letter index
in its upper five bits and route code in its lower two bits. A barcode with
no configured route causes a safe stop at the next junction.

The provided dimensions are an 18 mm line, about 20 mm gap between line and
barcode, and about 141 mm barcode length. They do not specify sensor mounting,
scan speed, line reflectance threshold, or junction geometry. The ADC
threshold remains provisional; validate both scan directions and junction
detection on the physical track before enabling floor movement.

**WARNING:** Pulse-width bounds, active polarity, line threshold, and the
20 ms sampling cadence depend on sensor response, barcode geometry, robot
speed, surface, and lighting. Measure representative tracks and test queue
handling before allowing the Vehicle Controller to act on decoded commands.

## B3 state size

`B3_STATE` contains 36 meaningful bits: 12 + 12 + 1 + 3 + 5 + 3. The RP2040
GCC toolchain allocates the `unsigned int` bit-fields in two 32-bit units, so
the state occupies 8 bytes. The header asserts that size because bit-field
layout is compiler-specific; the structure must not be used as a serialized
wire format.
