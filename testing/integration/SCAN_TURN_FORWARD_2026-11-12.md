# Obstacle decision: pause, scan, turn, forward

Removed the autonomous reverse fallback from `s5_controller_step`.
Previously a valid obstacle profile whose projected two-ray corridor was
too small produced REVERSE. The navigation adapter rejects reversing
without an independent rear sensor, so that path ended at emergency STOP.

The servo limits remain 70–110 degrees. A measured profile still uses the
normal route planner first. If that planner has no route but a valid outer
ray shows an opening, the controller selects the side with more observed
room. A clipped profile can use the existing explicit course bounds of
30 cm width and 40 cm depth. An outer ray is now treated as a candidate
turn direction, rather than a certification of the entire lateral corridor.
After the encoder-controlled turn, a fresh head-on reading must clear the
entire next forward leg plus chassis length and margin. Live front checks
continue during the leg. Line following remains paused until bypass and
line confirmation finish.

When both sides are blocked, the controller enters WAIT_ROUTE, keeps the
motors stopped, and retries the scan after approximately one second of
outer task ticks. This does not latch a permanent navigation fault.
Three valid front readings at or above the release distance while stopped
also allow an obstacle that was removed to release the pause. The adapter
only accepts this handoff from a physically stopped, idle avoidance state.
Actual sensor, motion and stale-data faults retain their stop behavior.

Validation:

- All host suites passed (`2026-11-12_111503_host.txt`).
- Left/right tests cover valid fine profiles rejected by the old corridor
  screen, without emitting REVERSE.
- A test couples the actual decision code to the navigation adapter and
  verifies GO_RIGHT -> completed turn -> positive forward MOVE.
- Simulations assert no autonomous REVERSE, exercise blocked rescans and
  a subsequently available route; adapter tests cover release after obstacle
  removal. Existing tests retain the line reacquisition and handoff checks.
- Network firmware compiled and generated the UF2. Hardware motion remains
  unverified; no floor-test pass is claimed.

Firmware startup marker: `B5-SCAN-TURN-FORWARD`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `7FA54C51C25C17346FA6876BA54EC7B91ECB805024B4930DF9704EC502642B28`.
