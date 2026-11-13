# Clear-path stop/start cycling

The supplied COM6 capture shows line detected on both sensors, valid
front returns about 1.4–1.6 m, B5 staying FOLLOW, and VC alternating states
2 and 5. No obstacle scan or hazard stop occurs. Those transitions identify
clearance loss (or NO_ECHO), rather than lost-line recovery or bypass.
The old periodic log cannot distinguish a transient invalid echo from an
expired clearance lease between its once-per-second snapshots.

Two sources of avoidable cycling were addressed:

- An isolated invalid front reading previously revoked clearance and set
  NO_ECHO immediately, then required three clear readings to restart. While
  already following with previously confirmed clearance, at most two
  invalid readings can now retain the existing lease for no more than
  150 ms after the latest valid clear reading. Invalid readings do not
  refresh permission or manufacture a distance. Three failures, an expired
  grace period, and any real close reading stop movement. Stuck-high and
  incomplete pulses still revoke permission immediately at the task layer.
- B5's priority was 20, below telemetry, despite maintaining a 350 ms
  clearance lease. Robot mode now runs B5 at priority 7, below motor control
  (5) and VC (6). Millisecond echo/trigger-gap waits yield via 1 ms RTOS
  sleeps so line sampling and other tasks still run. Hardware PWM captures
  pulse duration while the task yields; the 12 us trigger stays bounded.

The actual interruption reason is now visible via `VC-CLEARANCE-WAIT`
(allowed flag, NO_ECHO flag, generation), `S5-FRONT-INVALID` (echo status,
pulse width, failure count), and `S5-POLL-GAP` (delayed sensor polling).
The obstacle thresholds remain 25/30 cm; scan range stays 70–110 degrees.

Validation: all host suites passed, including repeated clear/invalid
sequences without STOP, immediate close STOP, bounded missing-data STOP,
and hardware pulse capture through simulated task sleeps. The network UF2
built successfully. This does not claim a physical floor-test pass or prove
which of the two causes occurred in the old, undersampled capture.

Firmware startup marker: `B5-STEADY-LINE-FOLLOW`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `1AC0997167E7FAE09A1CCEF998869AD3F250AEFF29E7514BA7603A66978DF3AD`.
