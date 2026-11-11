# Subsystem 1: requirements, traceability and test plan

Lectures 5 and 6: every requirement has an ID, uses "shall", has a number
with a tolerance where it can, traces to design elements (`DESIGN_B1.md`), to
code, and to at least one test. Test IDs match `TEST_LOG.md`.

The FR/NFR IDs are those used in the Week 6 design and the Week 8 Tribunal.
The wording below restates them in testable form; **check it against the Week
6 document before submission**. NFR-B1-6, NFR-B1-7 and FR-B1-8 are new
(found while fixing reconnect, see section 4).

## 1. Requirements

| ID | Requirement | Rationale |
|---|---|---|
| FR-B1-1 | The car shall join the configured 2.4 GHz WiFi network within 30 s of power-on when the AP is in range. | demo start-up |
| FR-B1-2 | The car shall open an MQTT session to the configured broker within 30 s of power-on (with FR-B1-1). | |
| FR-B1-3 | While connected, the car shall publish a telemetry frame every 500 ± 50 ms (period settable 100 ms to 10 s by the TELEM command). | live monitoring |
| FR-B1-4 | Each barcode, hump, obstacle and error event shall be published once, within 1 s of being reported, with QoS 1. | |
| FR-B1-5 | Each valid command shall reach the Vehicle Controller queue and be acknowledged within 500 ms of arrival at the car. | remote STOP |
| FR-B1-6 | The car shall publish a heartbeat every 5 ± 0.5 s while connected, and the broker shall show "offline" within 20 s of the car losing power. | dashboard knows if car is alive |
| FR-B1-7 | After a WiFi or broker loss the car shall reconnect without a reset, and set the WiFi-loss fault bit while disconnected. | |
| FR-B1-8 | The status LED shall show the link state (on, 1 Hz, 2 Hz, blip) within 250 ms of a state change. | no-laptop diagnosis |
| NFR-B1-1 | A network fault shall not stop, delay or block any other subsystem's task. | safety of motion |
| NFR-B1-2 | Every call another subsystem makes into B1 shall return within 5 ms, including when queues are full. | |
| NFR-B1-3 | B1's own task shall run at the lowest application priority and shall not busy-wait. | |
| NFR-B1-4 | Telemetry shall use at most 28 bytes per frame, commands and acks 3 bytes (Change Requirement 1). | bandwidth, RAM |
| NFR-B1-5 | Malformed commands (length, version, reserved bits, id, argument range) shall be rejected and never forwarded. | |
| NFR-B1-6 (new) | After the AP returns, the car shall re-join within 10 s, whether the link was lost by failure or by the AP disappearing; a join stuck for 20 ± 1 s shall be restarted. | Tribunal "still weak" item; bug found |
| NFR-B1-7 (new) | If a broker host name is configured, a failed or slow DNS lookup (> 5 s) shall not stop reconnection: the last working address is used. | DNS concern raised by team |

## 2. Traceability matrix

| Req | Design | Code | Verified by | Status |
|---|---|---|---|---|
| FR-B1-1 | D1.8 | `cyw43_utk.c` | S1-U09, S1-S01 | U09 not yet run on Pico |
| FR-B1-2 | D1.7 | `mqtt_utk.c` `connect_broker()` | S1-U09 | not yet run |
| FR-B1-3 | D1.1, D1.2, D1.3 | `s1_comm.c` `publish_telemetry()` | S1-U10, S1-U11, S1-S04 | not yet run |
| FR-B1-4 | D1.1, D1.2 | `comm.c` `post_event()` | S1-U06, S1-U12 | not yet run |
| FR-B1-5 | D1.2, D1.3, I1.3 | `s1_comm.c` `handle_command()` | S1-U03, S1-U08, S1-U13, S1-S02 | U03/U08 logic host-checked |
| FR-B1-6 | D1.2, D1.7 | `publish_status()`, Last Will | S1-U15, S1-U18 | not yet run |
| FR-B1-7 | D1.6, D1.7, D1.8, I1.8 | `mqtt_utk_link_down()`, `s1_link.c` | S1-U16, S1-U17, S1-U21, S1-S03 | policy host-tested (H02-H04) |
| FR-B1-8 | D1.4 | `s1_led_level()` | S1-U23 | not yet run |
| NFR-B1-1 | D1.5 (TMO_POL), D1.2 | `comm_bridge.c` | S1-U17, S1-S05 | not yet run |
| NFR-B1-2 | D1.1 | `comm.c` | S1-U06 | not yet run |
| NFR-B1-3 | D1.2 | `tk_wai_flg` in `s1_comm_task()` | S1-U02, S1-S05 | not yet run |
| NFR-B1-4 | D1.3 | `comm_pack.c` | S1-U05, S1-H01, S1-H06 | H01, H06 PASS |
| NFR-B1-5 | D1.3 | `comm_pack_cmd_decode()` | S1-U04, S1-U14, S1-H01 | H01 PASS |
| NFR-B1-6 | D1.6, D1.8 | `s1_link_wifi_action()` | S1-H02, S1-H03, S1-S03 | H02/H03 PASS |
| NFR-B1-7 | D1.6, D1.7 | `s1_link_broker_choice()`, `resolve_broker()` | S1-H05, S1-S06 | H05 PASS |

Gaps the matrix shows (Lecture 6 step 5):

- No requirement covers what the robot should do when WiFi is lost
  **during** a mission. The Vehicle Controller keeps driving
  (`VEHICLE_CONTROLLER.md`), so an E-STOP cannot reach it. That is a team
  safety decision: either a requirement "on WiFi loss > N s the VC shall stop"
  or a written acceptance of the risk. B1 already raises `FAULT_WIFI_LOSS`, so
  the VC can act on it.
- The ack confirms delivery to the VC queue, not execution. If the demo needs
  "STOP executed", add an execution-result event from the VC.
- Every hardware (U/S) test is still open; only the host tests have run.

## 3. Test plan by level (Lecture 6)

**Unit, host (no Pico)**: `sh testing/subsystem1/host/run_host_tests.sh`

| ID | What | Boundaries tested |
|---|---|---|
| S1-H01 | C packer vs laptop decoder, 18 frames, 24 command frames | min, max, saturation, invalid markers, 24-bit wrap |
| S1-H02 | re-join for DOWN/FAIL/NONET/BADAUTH/unknown | 4999 ms (no), 5000 ms (yes), UINT32_MAX |
| S1-H03 | UP never acts; JOIN/NOIP reset only when stuck | 5000, 19999 (wait), 20000 (reset) |
| S1-H04 | MQTT backoff | attempts 0..6 -> 1,1,2,4,8,16,16 s; 7 -> OFFLINE 60 s; huge count |
| S1-H05 | DNS fallback choice | all four input combinations |
| S1-H06 | dashboard decoder (JavaScript) vs the same 18 frames, bad commands rejected | as H01 |

**Unit, on target** (`make NET=lwip MQTT=1 TEST=1`, `ROBOT_MODE 0`): S1-U01..U10,
U20, U23 (LED pattern at tick 0, 1, 2, 7, 8 and an out-of-range state).

**Integration** (interfaces I1.1-I1.8, fault injection as in the Lecture 6 I1 example):

| ID | Interface | Inject | Expected |
|---|---|---|---|
| S1-I01 | I1.3 | 9 commands in one burst (queue holds 8) | 8 acks OK, 1 ack BUSY, `fwd_failed` + 1 |
| S1-I02 | I1.2 | 40 events in < 20 ms | all calls return < 5 ms, extras E_TMOUT (U06) |
| S1-I03 | I1.8 | turn the AP off while CONNECTED | `mqtt: WiFi lost, session closed` within one poll, not after 15 s |
| S1-I04 | I1.6 | send `SPEED 101`, `2e 00`, `e2 00 00` | REJECTED with reason; VC never sees them |
| S1-I05 | I1.7 | B5 unplugs the ultrasonic sensor | `NO_ECHO` bit in the next telemetry, other bits unchanged |

**System, on the car** (record in `TEST_LOG.md`, evidence in `logs/`):

| ID | Requirement | Method | Pass |
|---|---|---|---|
| S1-S01 | FR-B1-1/2 | power on 10 times, read "connected after N ms" | all 10 <= 30 s |
| S1-S02 | FR-B1-5 | dashboard ack round-trip for 20 STOP commands | every one <= 500 ms |
| S1-S03 | FR-B1-7, NFR-B1-6 | hotspot off 30 s, on; repeat 5 times | each time CONNECTED <= 10 s after the AP is back, `reconn` + 1 |
| S1-S04 | FR-B1-3 | 10-minute soak on the dashboard | "Missing frames" 0 (or < 0.5 %), period 500 ± 50 ms |
| S1-S05 | NFR-B1-1/3 | B2 logs its 10 ms loop period with WiFi idle vs. dashboard connected and TELEM 100 | no extra overruns with WiFi load (open Tribunal item A1b) |
| S1-S06 | NFR-B1-7 | set `MQTT_BROKER_HOST`, then stop the laptop's DNS / use a wrong name after a first good connect | reconnects to the cached address; console shows `DNS failed, using last address` |

**Acceptance**: a teammate who did not write B1 starts, steers (SPEED/MODE) and
E-STOPs the car from the dashboard without instructions, and reads its state.

## 4. What changed for these requirements (2026-11-02)

- **Bug (NFR-B1-6):** when the hotspot disappeared, the cyw43 driver reported
  DOWN (0), but re-join only happened on negative codes, so the car stayed
  offline until reset. Fixed in `s1_link.c` / `cyw43_utk.c`; boundary tests
  S1-H02/H03.
- **FR-B1-7:** WiFi loss now closes the MQTT session at once (`I1.8`) instead
  of after the ~15 s keepalive, and resets the backoff.
- **DNS (NFR-B1-7):** the default build connects to `MQTT_BROKER_IP`, an IP
  literal, so it never uses DNS. With `MQTT_BROKER_HOST` set, the lookup is
  redone on every attempt, guarded by a 5 s timeout and a generation number,
  and falls back to the last working address. See SD-4 in `DESIGN_B1.md`.

## 5. Sensor fusion and optimisation notes (Lectures 9 and 10)

- **Fusion level (L9):** B1 transmits *decision/feature-level* results (state,
  speeds, barcode letter, hump height, obstacle action), not raw samples. That
  is the distributed fusion model, which keeps bandwidth low; the trade-off is
  that the laptop cannot re-filter raw data.
- **Memory (L10):** minimum-bit fields and frames (Change Requirement 1);
  `static const` name/argument tables in flash; fixed-width unsigned types; no
  floating point in B1; structs passed by pointer.
- **Performance (L10):** bounded drain loops, the comm task sleeps on an
  event flag (interrupt-style wake-up, not polling), and the LED uses a short
  cyclic handler that only wakes a task (short ISR).
- **Power (L10), open item:** the kernel WiFi task polls the radio every 2 ms
  at priority 4. That is the polling pattern Lecture 10 calls least
  power-efficient, and its cost to B2's 10 ms loop is unmeasured (S1-S05).
  Options to evaluate once S1-S05 has a number: a longer poll period while
  idle, or the CYW43 power-save mode. Measure the current draw before and
  after with a USB power meter.
