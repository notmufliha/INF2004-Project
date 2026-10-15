# Subsystem 1 — MQTT interface (Buddy 1: WiFi Communication, Command & Telemetry)

This is the contract between the robot car and anything on the laptop side
(dashboard, `mosquitto_sub`, `tools/mqtt_monitor.py`). All topics start with
`MQTT_TOPIC_PREFIX` from `config/wifi_secrets.h` (default `inf2004/g25/car`).

## 1. Topics

| Topic | Direction | QoS | Retained | Rate | Content |
|---|---|---|---|---|---|
| `<prefix>/telemetry` | car → laptop | 0 | no | every 500 ms (change with `TELEM`) | robot state snapshot |
| `<prefix>/event` | car → laptop | 1 | no | when it happens | barcode / hump / obstacle / error |
| `<prefix>/status` | car → laptop | 0 (heartbeat), 1 (online/offline) | online/offline: **yes** | every 5 s | heartbeat and link statistics |
| `<prefix>/cmd` | laptop → car | 1 | no | on demand | plain-text command |
| `<prefix>/cmd/ack` | car → laptop | 1 | no | one per command | accepted / rejected + reason |

**Why these QoS levels:** telemetry is superseded 2× per second, so a lost
frame does not matter and QoS 0 keeps the radio load low. Events and command
acknowledgements happen once and must not be lost, so they use QoS 1 (the
broker confirms each one).

**Online/offline:** on connecting, the car publishes `{"online":true}`
retained on `<prefix>/status`. It also registers a *Last Will*
`{"online":false}` (retained). If the car loses power or WiFi, the broker
publishes that itself after ~15 s (1.5 × the 10 s keepalive). A dashboard
opened at any time therefore shows whether the car is reachable.

## 2. Message formats (JSON)

All numbers are integers (no floating point on the wire). Units are in the
field comments below.

### Telemetry — `<prefix>/telemetry`

```json
{"seq":42,"t":21000,"state":"LINE_FOLLOW",
 "spd":[100,98],"enc":[4200,4197],"dist":2100,
 "line":{"l":812,"c":2400,"r":0,"det":true,"lost":false,"jct":false},
 "imu":{"ev":"MOVING","pitch":-4},
 "hump":{"last":23,"peak":31},
 "bc":{"code":"B","nav":"RIGHT"},
 "obs":{"d":250,"w":120,"lc":300,"rc":90}}
```

| Field | Meaning | Unit | Source |
|---|---|---|---|
| `seq` | increments by 1 per telemetry message (gaps = lost frames) | — | S1 |
| `t` | time since boot | ms | S1 |
| `state` | `INIT` `IDLE` `LINE_FOLLOW` `NAV_COMMAND` `HUMP` `OBSTACLE_SCAN` `OBSTACLE_AVOID` `LINE_RECOVERY` `SAFE_STOP` | — | Vehicle Controller |
| `spd` | [left, right] wheel speed | mm/s | B2 |
| `enc` | [left, right] encoder counts | counts | B2 |
| `dist` | distance travelled | mm | B2 |
| `line.l` / `line.c` | left / centre IR, raw ADC | 0–4095 | B3 |
| `line.r` | right IR (barcode sensor), digital | 0/1 | B3 |
| `line.det/lost/jct` | line detected / lost / junction | bool | B3 |
| `imu.ev` | `STATIONARY` `MOVING` `ACCELERATING` `TURNING` `CLIMBING` `DESCENDING` `IMPACT` | — | B4 |
| `imu.pitch` | pitch angle | 0.1° | B4 |
| `hump.last` / `hump.peak` | last hump height / **highest hump this run** | mm | B4 |
| `bc.code` / `bc.nav` | last barcode letter (`-` = none) / `LEFT` `RIGHT` `STRAIGHT` `UTURN` | — | B3 |
| `obs.d/w/lc/rc` | obstacle distance, width, left & right clearance (0 = none) | mm | B5 |

Worst-case size: 374 bytes (limit `COMM_PAYLOAD_MAX` = 400).

### Events — `<prefix>/event`

```json
{"seq":7,"t":16000,"type":"barcode","code":"A","nav":"LEFT"}
{"seq":8,"t":26000,"type":"hump","height":23,"peak":31}
{"seq":9,"t":42000,"type":"obstacle","d":250,"w":120,"lc":300,"rc":90,"action":"LEFT"}
{"seq":10,"t":43000,"type":"error","sub":2,"code":17,"text":"encoder stalled"}
```

`action` is one of `NONE` `STOP` `CONTINUE` `LEFT` `RIGHT` `REVERSE`.
`sub` is the subsystem number that reported the error.

### Heartbeat / status — `<prefix>/status`

```json
{"online":true,"seq":12,"up":60000,"link":"CONNECTED","ip":"192.168.1.23",
 "telem_ms":500,"tx":151,"drop":0,"evdrop":0,"rx":3,"rej":1,"reconn":0,"demo":true}
```

| Field | Meaning |
|---|---|
| `tx` | messages handed to the MQTT client since boot |
| `drop` | messages dropped because the outgoing queue was full |
| `evdrop` | `comm_report_xxx()` calls refused because the event queue was full |
| `rx` / `rej` | commands accepted / rejected |
| `reconn` | times the link came back after being lost |
| `demo` | `true` while `S1_STANDALONE_DEMO` = 1 (fake data!) |

## 3. Commands — `<prefix>/cmd`

Plain text, case-insensitive: `VERB [ARG] [#SEQ]`. The optional `#SEQ` is
echoed in the acknowledgement so the sender can match replies.

| Command | Argument | Goes to | Effect |
|---|---|---|---|
| `START` | — | Vehicle Controller | begin / continue mission |
| `STOP` | — | Vehicle Controller | controlled stop → SAFE_STOP |
| `ESTOP` | — | Vehicle Controller | emergency stop, motors off |
| `PAUSE` / `RESUME` | — | Vehicle Controller | hold / continue |
| `MODE` | `AUTO` \| `LINE` \| `TEST` | Vehicle Controller | change operating mode |
| `SPEED` | 0–100 | Vehicle Controller | target speed in percent |
| `RESET_PEAK` | — | Vehicle Controller | forget the highest hump |
| `PING` | — | S1 | replies `pong` |
| `STATUS` | — | S1 | publishes a status message now |
| `TELEM` | 100–10000 | S1 | telemetry period in ms |

### Acknowledgement — `<prefix>/cmd/ack`

```json
{"cmd":"SPEED 40 #5","seq":5,"ok":true,"result":"forwarded"}
{"cmd":"SPEED 400 #6","seq":6,"ok":false,"reason":"argument out of range"}
{"cmd":"FLY","seq":0,"ok":false,"reason":"unknown command"}
{"cmd":"STOP #7","seq":7,"ok":false,"reason":"controller busy"}
```

Rejected commands are **never** forwarded to the Vehicle Controller (NFR-B1-5).
`controller busy` means the command was valid but the controller's queue
(8 commands) was full.

## 4. Inside the car (for the other buddies)

```
 B2/B3/B4/B5/VC tasks                      Subsystem 1                  kernel WiFi task (prio 4)
 -------------------                      -----------                  -------------------------
 comm_set_xxx()   --> s1_telem (mutex) --> comm_task (prio 13) --+
 comm_report_xxx() -> mbf_events -------->   formats JSON        +--> mbf_tx --> mqtt_utk.c --> broker
                                                                                  pump, 20 ms
 Vehicle Controller <-- mbf_remote_cmd <-- validates <-- mbf_rx <---------------- mqtt_utk.c <-- broker
```

- Every `comm_xxx()` call returns immediately. Nothing ever waits for the
  network (NFR-B1-2). If a queue is full, the item is dropped and counted.
- Only the kernel's WiFi task calls lwIP/MQTT (TEAM.md section 3). Subsystem 1
  talks to it through the three `comm_bridge_xxx()` functions in
  `comm_bridge.h`, which use only `TMO_POL` kernel calls.
- A network failure only pauses telemetry and remote commands. It never
  stops the robot (NFR-B1-1).

## 5. Connection recovery (statechart)

| Situation | What happens | `link` state |
|---|---|---|
| WiFi down | kernel re-joins the AP every 5 s | `CONNECTING_WIFI` (first time) / `RECONNECTING` |
| WiFi down > 30 s | telemetry paused, re-join continues | `OFFLINE` |
| Broker refuses / unreachable | retry after 1, 2, 4, 8, 16, 16 s | `RECONNECTING` |
| 6 retries failed | wait 60 s, then start again | `OFFLINE` |
| Broker connection drops | noticed within ~15 s (keepalive 10 s), then retries | `RECONNECTING` |

The status LED on GP19 shows the state: fast blink = joining WiFi, slow
blink = connecting/reconnecting to the broker, solid = connected, short blip
every 2 s = offline.
