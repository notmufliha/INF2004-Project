# Subsystem 1 — MQTT interface (Buddy 1: WiFi Communication, Command & Telemetry)

This is the contract between the robot car and anything on the laptop side
(`tools/mqtt_monitor.py`, `tools/mqtt_send_cmd.py`, a dashboard). All topics
start with `MQTT_TOPIC_PREFIX` from `config/wifi_secrets.h` (default
`inf2004/g25/car`).

Since Change Requirement 1 (minimum-bit data representation) every payload
is a **compact binary frame**, not JSON text. Each value uses only the bits
its valid range needs, packed MSB-first with shifts and masks
(`comm_pack.c`). `mosquitto_sub` will show unreadable bytes; use
`tools/mqtt_monitor.py`, which decodes them with `tools/s1_wire.py`.

**Keep three files in step:** `comm_pack.c` (Pico), `tools/s1_wire.py`
(laptop) and this document. `testing/subsystem1/host/run_host_tests.sh`
checks the first two agree bit for bit.

## 1. Topics

| Topic | Direction | QoS | Retained | Rate | Frame |
|---|---|---|---|---|---|
| `<prefix>/telemetry` | car → laptop | 0 | no | every 500 ms (change with `TELEM`) | 28 B |
| `<prefix>/event` | car → laptop | 1 | no | when it happens | 5–9 B |
| `<prefix>/status` | car → laptop | 0 (heartbeat), 1 (online/offline) | online/offline: **yes** | every 5 s | 23 B / 1 B |
| `<prefix>/cmd` | laptop → car | 1 | no | on demand | 3 B |
| `<prefix>/cmd/ack` | car → laptop | 1 | no | one per command | 3 B |

**QoS:** telemetry is superseded twice a second, so QoS 0; events and acks
happen once and must not be lost, so QoS 1.

**Online/offline:** on connecting, the car publishes the 1-byte frame `0x30`
(version 1, online) retained on `<prefix>/status`, and registers the Last
Will `0x20` (version 1, offline), which the broker publishes ~15 s after the
car vanishes. These are the first byte of the heartbeat frame, so the
decoder tells them apart by length.

## 2. Frame layouts (car → laptop)

Fields in order, most significant bit first; frames are padded with zero
bits to a whole byte. Every frame starts with a 3-bit wire **version = 1**.
`s` = signed (two's complement). Values outside a field's range are
**saturated** by the Pico, never wrapped, except the encoder totals.

### Telemetry — 223 bits → 28 bytes (was 290 B typical, 374 B worst as JSON)

| # | Field | Bits | Range / unit | Source |
|---|---|---|---|---|
| 1 | ver | 3 | 1 | S1 |
| 2 | seq | 16 | 0–65535, wraps; a gap = lost frames | S1 |
| 3 | time | 20 | 0.1 s since boot (wraps after 29 h) | S1 |
| 4 | robot state | 4 | 0–8: INIT IDLE LINE_FOLLOW NAV_COMMAND HUMP OBSTACLE_SCAN OBSTACLE_AVOID LINE_RECOVERY SAFE_STOP | VC |
| 5 | faults | 8 | shared fault bitmask (`fault_flags.h`) | all |
| 6–7 | speed L, R | 14 s each | ±8191 counts/s (B2 max is 5000) | B2 |
| 8–9 | encoder L, R | 24 s each | low 24 bits of B2's total; the laptop unwraps | B2 |
| 10–11 | IR left, centre | 12 each | raw ADC 0–4095 | B3 |
| 12 | IR right | 1 | digital 0/1 | B3 |
| 13 | line flags | 3 | bit0 detected, bit1 lost, bit2 junction | B3 |
| 14 | last barcode | 3 | 0 none, 1–4 = A–D | B3 |
| 15 | last nav | 3 | 0–4: NONE LEFT RIGHT STRAIGHT UTURN | B3 |
| 16 | motion state | 3 | B4's `imu_motion_state_t` 0–7 | B4 |
| 17 | pitch | 10 s | ±511 = ±51.1° (0.1° steps) | B4 |
| 18 | hump valid | 1 | 0 = heights not measured yet | B4 |
| 19–20 | hump last, peak | 7 each | 0–126 mm (0.1 cm steps) | B4 |
| 21 | obstacle distance | 9 | 0–510 cm, **511 = no echo / not measured** | B5 |
| 22 | obstacle width | 7 | 0–126 cm, 127 = not measured | B5 |
| 23–24 | clearance L, R | 9 each | 0–510 cm, 511 = not measured | B5 |

**Encoder totals.** B2 keeps signed 32-bit counters (Tribunal decision:
retain until the range is validated). Only the low 24 bits are sent. At B2's
maximum accepted speed of 5000 counts/s that is 28 min of continuous driving
before the field wraps, and the decoder (`Unwrap24`) rebuilds the full total
anyway, because consecutive frames (≤ 10 s apart → ≤ 50,000 counts) always
differ by far less than 2²³.

### Event — header 33 bits + body

Header: ver 3 · **type 2** (0 barcode, 1 hump, 2 obstacle, 3 error) · seq 8 · time 20 (0.1 s).

| Type | Body | Total |
|---|---|---|
| barcode | code 3 (1–4 = A–D) · nav 3 | 39 bits → 5 B |
| hump | valid 1 · height 7 (mm) · peak 7 (mm) | 48 bits → 6 B |
| obstacle | distance 9 · width 7 · left 9 · right 9 (cm, all-ones = invalid) · action 3 (NONE STOP CONTINUE LEFT RIGHT REVERSE) | 70 bits → 9 B |
| error | code 8 · detail 8 | 49 bits → 7 B |

**Error codes** (`app_program/error_codes.h`) replace error text: 1 byte =
3-bit subsystem (0 system/VC, 1–5 buddy) + 5-bit error number. The Pico
keeps them as `#define`s (0 bytes of RAM); the names exist only in
`tools/s1_wire.py` (`ERROR_NAMES`). B5's code numbers are its `S5Error` values.

### Heartbeat / status — 179 bits → 23 bytes

ver 3 · online 1 · link 3 (INITIALISING CONNECTING_WIFI CONNECTING_MQTT
CONNECTED RECONNECTING OFFLINE DISABLED) · demo 1 · seq 16 · time 20 ·
IP 32 (first octet in the low byte) · telemetry period 7 (×100 ms) ·
faults 8 · tx 16 · drop 16 · evdrop 16 · rx 16 · rej 16 · reconnects 8.

Counters wrap at 16 (8) bits; compare successive heartbeats.

### Acknowledgement — 24 bits → 3 bytes

ver 3 · seq 8 (copied from the command) · command id 4 · ok 1 · result 5 · pad 3.

| Result | Meaning | | Result | Meaning |
|---|---|---|---|---|
| 0 | forwarded to the Vehicle Controller | | 8 | frame is not 3 bytes |
| 1 | pong | | 9 | unknown wire version |
| 2 | status sent | | 10 | reserved bits not zero |
| 3 | telemetry period set | | 11 | unknown command |
| | | | 12 | argument out of range |
| | | | 13 | unexpected argument |
| | | | 14 | controller busy (VC queue full) |

## 3. Commands — `<prefix>/cmd`, 24 bits → 3 bytes

ver 3 · **id 4** · **arg 7** · seq 8 · reserved 2 (must be 0).

| Id | Command | Arg | Goes to |
|---|---|---|---|
| 1 | START | 0 | Vehicle Controller |
| 2 | STOP | 0 | Vehicle Controller |
| 3 | ESTOP | 0 | Vehicle Controller |
| 4 | PAUSE | 0 | Vehicle Controller |
| 5 | RESUME | 0 | Vehicle Controller |
| 6 | MODE | 0 AUTO, 1 LINE, 2 TEST | Vehicle Controller |
| 7 | SPEED | 0–100 % (VC maps 100 % to B2's 5000 counts/s) | Vehicle Controller |
| 8 | RESET_PEAK | 0 | Vehicle Controller |
| 9 | PING | 0 | S1 (ack result "pong") |
| 10 | STATUS | 0 | S1 (publishes a heartbeat now) |
| 11 | TELEM | 1–100 (× 100 ms) | S1 |

Any frame that is not exactly 3 bytes, has another version, non-zero
reserved bits, an unknown id, or an argument out of range is **rejected
with a reason** and never reaches the Vehicle Controller (NFR-B1-5).

Send from the laptop:

```
python tools/mqtt_send_cmd.py SPEED 40 --seq 5      # prints the ack
python tools/mqtt_monitor.py                        # then type: STOP, MODE LINE, TELEM 300 ...
```

The Vehicle Controller receives `REMOTE_CMD` (4 bytes: type 4 bits, arg 7,
seq 8) from `mbf_remote_cmd`.

## 4. Inside the car (for the other buddies)

```
 B2/B3/B4/B5/VC tasks                     Subsystem 1                  kernel WiFi task (prio 4)
 -------------------                      -----------                  -------------------------
 comm_set_xxx()   --> s1_telem (mutex) --> comm_task (prio 13) --+
 comm_report_xxx() -> mbf_events -------->   packs binary frames +--> mbf_tx --> mqtt_utk.c --> broker
 fault_raise/clear -> flg_faults ------->                                         pump, 20 ms
 Vehicle Controller <-- mbf_remote_cmd <-- validates <-- mbf_rx <---------------- mqtt_utk.c <-- broker
```

Connected today: **B2** — `comm_task` reads `encoder_get_counts()` every
telemetry period and derives counts/s (`S1_POLL_B2_ENCODERS`); **B4** —
`task_imu.c` sets motion state and pitch, reports humps, and owns
`FAULT_IMU`; **B5** — `task_obstacle.c` sets the front distance, reports
each obstacle profile, and owns `FAULT_NO_ECHO`. **B3** has no line/barcode
code yet: it should call `comm_set_line()` and `comm_report_barcode()`.

- Every `comm_xxx()` call returns immediately and never waits for the
  network (NFR-B1-2). If a queue is full, the item is dropped and counted.
- The telemetry snapshot is packed C bit-fields; several tasks write fields
  that share a 32-bit word, so every access holds `mtx_telem`
  (`TA_INHERIT`). The fault mask is a kernel event flag, so `tk_set_flg` /
  `tk_clr_flg` update it atomically, even from an ISR.
- Only the kernel's WiFi task calls lwIP/MQTT (TEAM.md section 3).
- A network failure only pauses telemetry and remote commands, and sets
  `FAULT_WIFI_LOSS`. It never stops the robot (NFR-B1-1).

## 5. Connection recovery (statechart)

| Situation | What happens | `link` state | `FAULT_WIFI_LOSS` |
|---|---|---|---|
| WiFi down | kernel re-joins the AP every 5 s | `CONNECTING_WIFI` (first time) / `RECONNECTING` | set once a link was lost |
| WiFi down > 30 s | telemetry paused, re-join continues | `OFFLINE` | set |
| Broker refuses / unreachable | retry after 1, 2, 4, 8, 16, 16 s | `RECONNECTING` | set |
| 6 retries failed | wait 60 s, then start again | `OFFLINE` | set |
| Broker connection drops | noticed within ~15 s (keepalive 10 s), then retries | `RECONNECTING` | set |
| Connected again | heartbeat sent at once | `CONNECTED` | cleared |

The status LED on GP19 shows the state: fast blink = joining WiFi, slow
blink = connecting/reconnecting to the broker, solid = connected, short blip
every 2 s = offline.
