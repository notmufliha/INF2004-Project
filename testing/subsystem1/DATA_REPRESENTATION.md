# Subsystem 1 — Minimum-bit data representation (Change Requirement 1)

Week 13 evidence for Buddy 1 (WiFi Communication, Command & Telemetry).
"Before" = commit `ec0b859` (JSON/text MQTT, recorded in
`logs/before_ram.txt`). "After" = this branch. Wire layouts: `app_program/subsystem1/MQTT.md` section 2.

## 1. Data representation table

| Field | Owner | Valid range | Values | Before | After | Bits |
|---|---|---|---|---|---|---|
| WiFi link state | B1 | INITIALISING … DISABLED | 7 | `volatile INT` (32) | enum in `volatile UB`*; 3-bit field on the wire | 3 |
| MQTT client state | B1 | IDLE … OFFLINE | 5 | `volatile INT` (32) | enum in `volatile UB`* | 3 |
| Online / offline flag | B1 | offline / online | 2 | JSON text `{"online":true}` (15 B) | bit 4 of a 1-byte frame (`0x30`/`0x20`) | 1 |
| WiFi-loss fault | B1 | clear / set | 2 | not represented | bit 4 (`0x10`) of the shared fault event flag | 1 |
| Fault bitmask | all | 8 faults | 2⁸ | separate variables per subsystem | `flg_faults` event flag, 8 bits on the wire | 8 |
| Telemetry period | B1 | 100–10,000 ms, 100 ms steps | 100 | `UW` ms (32) | `UB` × 100 ms, 7-bit field | 7 |
| Command type | B1 / VC | 11 commands | 11 | text verb (≤ 12 chars) / `UB` | 4-bit id; `REMOTE_CMD.type : 4` | 4 |
| Command argument | B1 / VC | speed 0–100 %, mode 0–2, telem 1–100 | 101 | decimal text / `H` (16) | 7-bit field; `REMOTE_CMD.arg : 7` | 7 |
| Command sequence no. | B1 | 0–255, wraps | 256 | `#n` text up to 8 digits / `UW` (32) | 8-bit field; `REMOTE_CMD.seq : 8` | 8 |
| Ack accepted flag | B1 | rejected / accepted | 2 | JSON `"ok":false` | 1 bit | 1 |
| Ack result | B1 | 7 accept/reject reasons + spare | 15 | reason text (≤ 31 chars) | 5-bit code | 5 |
| Event type | B1 | barcode, hump, obstacle, error | 4 | text `"type":"obstacle"` / `UB` | 2-bit field | 2 |
| Event sequence no. | B1 | 0–255, wraps | 256 | `UW` (32), JSON digits | 8-bit field | 8 |
| Error code | all | 8 subsystems × 32 errors | 256 | subsystem `UB` + code `H` + 24-char text | 1 byte: 3-bit subsystem + 5-bit number (`error_codes.h`) | 8 |
| Telemetry / status seq | B1 | 0–65535, wraps | 65,536 | `UW` (32) | 16-bit field | 16 |
| Time since boot | B1 | 0–29 h in 0.1 s steps | ~1 M | `UW` ms, JSON digits | 20-bit field, 0.1 s units | 20 |
| Robot state | VC | 9 states | 9 | `UB` + name text | `COMM_TELEMETRY.robot_state : 4` | 4 |
| Wheel speed L/R | B2 | ±5000 counts/s (B2 rejects targets > 5000) | — | `H` mm/s (wrong unit) | `INT speed : 14`, ±8191, saturated | 14 each |
| Encoder total L/R | B2 | B2 keeps signed 32-bit | — | `W` (32) | low 24 bits, laptop unwraps (28 min at max speed per wrap) | 24 each |
| Distance travelled | B2 | — | — | `W` mm (B2 has no mm yet) | **removed** (derive on laptop once wheel calibration exists) | 0 |
| IR left / centre | B3 | 0–4095 | 4096 | `UH` (16) | `: 12` | 12 each |
| IR right | B3 | 0/1 | 2 | `UB` (8) | `: 1` | 1 |
| Line flags | B3 | detected/lost/junction | 2³ | `UB` (8) | `: 3` bitmask | 3 |
| Last barcode | B3 | none, A–D | 5 | ASCII `UB` (8) | `: 3` (0 none, 1–4) | 3 |
| Last nav command | B3 | none, L, R, straight, U-turn | 5 | `UB` (8) | `: 3` | 3 |
| IMU motion state | B4 | 8 states (`imu_motion_state_t`) | 8 | `UB` own 7-state list | `: 3`, B4's own enum | 3 |
| Pitch | B4 | ±51.1° in 0.1° | 1023 | `H` (16) | `INT : 10` | 10 |
| Hump height / peak | B4 | 0–12.6 cm in 0.1 cm | 127 | `H` mm (16) each | `: 7` mm each + 1-bit valid | 7 each, 1 |
| Obstacle distance | B5 | 0–510 cm, + no-echo | 512 | `H` mm (16) | `: 9` cm, 511 = no echo | 9 |
| Obstacle width | B5 | 0–126 cm, + invalid | 128 | `H` mm (16) | `: 7` cm, 127 = invalid | 7 |
| Clearance L / R | B5 | 0–510 cm, + invalid | 512 | `H` mm (16) | `: 9` cm, 511 = invalid | 9 each |
| Avoid action | B5 | 6 actions | 6 | `UB` + name text | `: 3` | 3 |
| Link counters (tx, drop, …) | B1 | wrap, laptop uses differences | — | `UW` (32) ×8 | `UH` (16) ×8 | 16 |

\* The two link states are written by one task and read by others without a
lock. They stay in a whole byte: a byte store is atomic on the RP2040,
whereas storing a 3-bit bit-field is a read-modify-write of the whole word.
They are still 3 bits on the wire.

### Safe access to shared packed data

| Packed data | Writers | Protection |
|---|---|---|
| `COMM_TELEMETRY` bit-fields (24 B) | B2 (via S1 poll), B3, B4, B5, VC, comm_task | every read and write holds `mtx_telem` (`TA_INHERIT`, 5 ms limit); values are saturated before taking the lock |
| Fault bitmask | B1, B2, B3, B4, B5 tasks and ISRs | kernel event flag: `tk_set_flg` / `tk_clr_flg` are atomic and ISR-safe — no read-modify-write in application code. Test S1-U20 |
| `COMM_EVENT`, `REMOTE_CMD` bit-fields | one task each | built on the caller's stack and copied whole by `tk_snd_mbf`; never shared while being written |

## 2. Message size comparison

Measured by `testing/subsystem1/host/run_host_tests.sh` (S1-H01) and on
target by S1-U05. "Before" = MQTT.md's example messages (typical) and every
field at its widest value (worst), from the old `comm_json.c`.

| Message | Before (typical / worst) | After | Reduction |
|---|---|---|---|
| Telemetry | 290 / 374 B | **28 B** | 90 % / 93 % |
| Event: barcode / hump / obstacle / error | 60 / 55 / 86 / 78 B (worst 117) | **5 / 6 / 9 / 7 B** | ~90 % |
| Status / heartbeat | 157 / 233 B | **23 B** | 85 % / 90 % |
| Online / Last Will | 15 / 16 B | **1 B** | 94 % |
| Command | 11 / 64 B | **3 B** | 73 % / 95 % |
| Ack | 60 / 143 B | **3 B** | 95 % / 98 % |

Inter-task messages:

| Structure | Before | After |
|---|---|---|
| `COMM_TELEMETRY` snapshot | 40 B | 24 B |
| `COMM_EVENT` (mbf_events) | 44 B | 8 B |
| `REMOTE_CMD` (mbf_remote_cmd → VC) | 8 B | 4 B |
| `COMM_STATS` | 32 B | 16 B |
| TX record header (mbf_tx) | 2 B | 1 B (topic 2 bits + retain 1 bit) |

## 3. RAM usage comparison

**Measured** with `arm-none-eabi-size` and `arm-none-eabi-nm` on two builds
that differ only by this change (`logs/before_ram_main.txt` = `main`
20e3981, `logs/after_ram.txt` = this branch). Both: pico-sdk 2.2.0,
`make SMP=0 NET=lwip MQTT=1`, every subsystem started (`ROBOT_MODE` 0 for
the build, so subsystem 1 is linked in both). SMP=0 because `main`'s
`b2_motor.c` no longer builds with SMP=1, so `logs/before_ram.txt`
(ec0b859, SMP=1, before B4/B5 were merged) cannot be compared directly.

| | Before | After | Change |
|---|---|---|---|
| Whole firmware `.data + .bss` | 81,012 B | 79,452 B | **−1,560 B** |
| Subsystem 1 static RAM (symbols below) | 1,778 B | 192 B | **−1,586 B** |
| Flash (`.text`) | 369,078 B | 370,546 B | +1,468 B (packing code, fault mask, hooks) |

| Subsystem 1 symbol | Before | After |
|---|---|---|
| `json_out` → `frame` (comm_task) | 401 | 32 |
| `rec` in `s1_tx_enqueue` + `comm_bridge_tx_pop` | 402 + 402 | 33 + 33 |
| `pend_payload` (mqtt_utk.c, sized by `COMM_PAYLOAD_MAX`) | 400 | 32 |
| `rx_data` (mqtt_utk.c, sized by `COMM_CMD_MAX`) | 65 | 4 |
| `s1_telem` | 40 | 24 |
| `s1_stats` | 32 | 16 |
| state, sequence numbers, period, counters (9 variables) | 36 | 18 |

**Kernel message-buffer memory** is not in `.bss`: the kernel takes it from
its own memory area when `tk_cre_mbf` runs, so `size` cannot show it. It is
the `bufsz` each buffer is created with:

| Buffer | Before | After |
|---|---|---|
| `mbf_events` 16 × (msg + 8) | 16 × (44 + 8) = 832 | 16 × (8 + 8) = 256 |
| `mbf_tx` | 3,072 | 640 |
| `mbf_rx` | 4 × (65 + 8) = 288 | 8 × (4 + 8) = 96 |
| `mbf_remote_cmd` 8 × (msg + 8) | 8 × (8 + 8) = 128 | 8 × (4 + 8) = 96 |
| **Total** | **4,320 B** | **1,088 B** (−3,232 B) |

In total subsystem 1 uses about **4.8 KB less RAM** (1.6 KB measured static
+ 3.2 KB kernel buffers). The Week 8 estimate was ~6 KB.

To repeat in Ubuntu: check out each commit, set `ROBOT_MODE` to 0 in
`app_program/robot_config.h` (do not commit it), then
`make clean && make SMP=0 NET=lwip MQTT=1 -j8` and
`arm-none-eabi-size *_lwip_mqtt.elf`. A different compiler version gives
slightly different totals, but the difference should be about the same.

## 4. Correctness and performance

| Evidence | Test |
|---|---|
| Pico packer and laptop decoder agree on every field at min / max / saturated / invalid values | S1-H01 (host) |
| Valid commands decoded; malformed, out-of-range and unknown frames rejected and never forwarded | S1-U03, S1-U04, S1-H01, S1-U14 |
| Frame sizes as documented; packing never overruns its buffer | S1-U05 |
| Out-of-range values saturate instead of wrapping; encoder totals survive the 24-bit wrap | S1-U07, S1-H01 |
| Event reporting still never blocks (40 events < 20 ms) | S1-U06 |
| Fault-mask bits from different writers never lost | S1-U20 |
| WiFi-loss bit set and cleared with the link | S1-U21 |
| End-to-end, real data from B4/B5 | S1-U22 |
| Timing: radio time per telemetry frame drops ~10×; the 10 ms motor loop must be measured with MQTT on/off (Tribunal test B1 + B2) | to do with B2 |
