# B3 integration notes

- Reads ADC channel 0 (GP26, left) and channel 1 (GP27, centre) nominally
  every 20 ms. Both raw readings are published for diagnosis.
- Uses independent thresholds and polarities from `cfg_line.h`: GP26/left is
  active-low at or below `B3_LINE_LEFT_THRESHOLD` (default 1000), while
  GP27/centre is active-high at or above `B3_LINE_CENTRE_THRESHOLD`
  (default 2100). It reports
  `DETECTED` if either sensor is active, `LOST` if neither is active, and
  `JUNCTION` if both are active. These are provisional thresholds based on
  current bench readings; recalibrate if readings change on the track.
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
2. Line following and junction detection require both sensors to respond
   correctly. Confirm each raw ADC channel changes between white and black
   before enabling movement; barcode decoding alone does not validate line
   following or junction detection.
3. The Vehicle Controller receives a `uint8_t` barcode code from
   `mbf_nav_cmd`: 1 is A, ..., 26 is Z. It stores that character and waits
   for a junction signal before applying its configured route.
4. Coordinate task ownership and priority with the current Subsystem 3 demo:
   this module uses priorities 16 and 17 on core 0, while the example
   `consumer_task` already uses priority 16. Do not start both at the same
   priority without agreeing on the project task table.
5. In robot mode, `app_main.c` initializes Subsystem 1 before calling
   `b3_init()`, then calls `b3_start()` from core 0 after B4 readiness. The
   separate sample `subsystem3_init()` / `subsystem3_start()` lifecycle remains
   for the teaching demo and does not start this B3 module.
6. Test ADC polarity and thresholds with the actual line sensors and floor.
   The initial thresholds are 1000 for the active-low left sensor and 2100
   for the active-high centre sensor; they are not validated track settings.
   For stationary line-sensor testing, `ROBOT_ENABLE_ULTRASONIC` in
   `robot_config.h` can be set to 0; this skips the ultrasonic/servo task.
   B2's clearance interlock remains fail-closed, so this must not be used to
   drive the robot. Re-enable and verify the ultrasonic task before any
   movement test. For a line-only run, select `MODE LINE` so barcode routes
   are not required; start at low speed only after clearance is available.

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

For autonomous barcode routing, send one command per letter; all four routes
(FORWARD, UTURN, LEFT, RIGHT) must be assigned to distinct letters before
`MODE AUTO` START is accepted. `MODE LINE` does not require mappings and
ignores barcode navigation. `MAP_RESET` clears the current table. The seven-bit
MAP argument stores the letter index in its upper five bits and route code in
its lower two bits. A barcode with no configured route causes a safe stop at
the next junction in AUTO mode.

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
