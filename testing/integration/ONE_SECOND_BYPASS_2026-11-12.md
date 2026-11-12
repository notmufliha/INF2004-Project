# One-second forward leg between opposite turns

Requested behavior: turn away from the obstacle, move forward for one
second, turn the opposite direction to restore the original heading, then
move forward again. The original 70–110-degree servo limits remain.

The first sideways MOVE is replaced with a timed, straight, forward DRIVE
at the existing bypass speed (600 counts/s). Its nominal duration is
`ROBOT_BYPASS_OFFSET_DRIVE_MS = 1000`. Navigation refreshes this command
on each approximately 65 ms sensor cycle, keeping B2's 100 ms DRIVE
watchdog satisfied. Refreshes do not restart the maneuver timer.

At the deadline B5 revokes movement and submits STOP, waits for the motion
snapshot to confirm physical stopping, then issues the opposite 90-degree
turn. The existing distance-controlled pass and bounded tape reacquisition
follow. Line following stays paused throughout this ownership interval.

The old initial 20/46 cm corridor reservation is no longer applied to the
timed leg. Live close-obstacle, invalid front sensor, motion fault, stale
line and mission STOP checks remain active. Later distance MOVE legs retain
their clearance checks. The timed leg's actual displacement depends on
wheel speed, acceleration and calibration; one second does not guarantee
the former planned lateral distance or clearance of every allowed box.

Timing is RTOS/sensor-cycle based: STOP is requested on the first cycle
at or after 1000 ms (normally within one additional 65 ms period), followed
by physical deceleration. No exact physical one-second timing is claimed.

Tests cover both initial turn signs, repeated DRIVE refreshes, the timer
deadline and clock rollover, waiting for physical STOP, the opposite turn
and subsequent forward MOVE through the actual navigation adapter. A
50 cm front reading now permits the timed leg where the old 46 cm MOVE
reservation would reject it; a new close reading still stops immediately.

Startup marker: `B5-ONE-SECOND-BYPASS`.
Runtime marker: `[S5-DRIVE] forward duration_ms=1000 speed_cps=600`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `82FA4436374A90CE9A1845876B138B9D7D203ACAD3E8D8BF7409088BFA1F4A4C`.
The firmware built successfully. Physical operation requires a floor test.
