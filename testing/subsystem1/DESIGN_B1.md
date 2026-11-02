# Subsystem 1 (WiFi, Command & Telemetry): high-level design

Lecture 5 format: every box (D) and arrow (I) has an ID; one sequence diagram
per scenario, including the error cases; one unified statechart. Requirements
and the trace to code and tests are in `REQUIREMENTS_TRACE.md`.

## 1. Architecture: boxes and arrows

| ID | Design element | File(s) | Runs on |
|---|---|---|---|
| D1.1 | Public API: telemetry snapshot, event queueing, link status | `comm.c` / `comm.h` | caller's task |
| D1.2 | Comm task: link statechart, command validation, framing, heartbeat | `s1_comm.c` | task, priority 13 |
| D1.3 | Wire format: bit packing, command decoding | `comm_pack.c` | pure functions |
| D1.4 | Status LED | `s1_led.c` | task 12 + 250 ms cyclic handler |
| D1.5 | Bridge to the kernel network task | `comm_bridge.c` / `.h` | both sides, TMO_POL only |
| D1.6 | Reconnect policy: WiFi re-join, MQTT backoff, DNS fallback | `s1_link.c` | pure functions |
| D1.7 | MQTT client: connect, publish, subscribe, DNS lookup | `lib/liblwip/.../mqtt_utk.c` | kernel WiFi task |
| D1.8 | WiFi driver loop: poll, join, re-join | `lib/libwifi/.../cyw43_utk.c` | kernel WiFi task, 2 ms |
| D1.9 | Shared fault mask | `app_program/fault.c` | any task or ISR |
| D1.10 | Laptop tools: monitor, command sender, dashboard | `tools/` | laptop |

| ID | Interface | From -> to | Contract |
|---|---|---|---|
| I1.1 | IF-S1-TLM `comm_set_xxx()` | B2/B3/B4/B5/VC -> D1.1 | non-blocking, saturates, <= 5 ms mutex wait |
| I1.2 | IF-S1-EVT `comm_report_xxx()` | any task -> D1.1 -> D1.2 | returns E_OK / E_PAR / E_TMOUT, never waits |
| I1.3 | IF-S1-CMD `mbf_remote_cmd` | D1.2 -> Vehicle Controller | 4-byte validated command, queue of 8 |
| I1.4 | link status `comm_get_state()` | D1.2 -> anyone | 7 states |
| I1.5 | tx / rx message buffers | D1.2 <-> D1.7 via D1.5 | TMO_POL both ways |
| I1.6 | MQTT topics `inf2004/g25/car/...` | D1.7 <-> broker <-> D1.10 | binary frames, MQTT.md section 2 |
| I1.7 | fault mask `fault_raise/clear/get` | every subsystem <-> D1.9 | event flag, ISR-safe |
| I1.8 | link-down notice `mqtt_utk_link_down()` | D1.8 -> D1.7 | called once when the link leaves UP |

```mermaid
flowchart LR
  B2[B2 motion] -- I1.1 --> D11[D1.1 API]
  B3[B3 line] -- I1.1/I1.2 --> D11
  B4[B4 IMU] -- I1.1/I1.2 --> D11
  B5[B5 obstacle] -- I1.1/I1.2 --> D11
  VC[Vehicle Controller] -- I1.1 --> D11
  D11 --> D12[D1.2 comm task]
  D12 -- uses --> D13[D1.3 wire format]
  D12 -- I1.3 --> VC
  D12 -- I1.4 --> D14[D1.4 LED]
  D12 <-- I1.5 --> D15[D1.5 bridge]
  D15 <--> D17[D1.7 MQTT client]
  D18[D1.8 WiFi loop] -- I1.8 --> D17
  D18 -- uses --> D16[D1.6 policy]
  D17 -- uses --> D16
  D17 <-- I1.6 --> BR[(broker)] <--> D110[D1.10 laptop / dashboard]
  ALL[all subsystems] -- I1.7 --> D19[D1.9 fault mask]
```

## 2. Sequence diagrams

### SD-1 Telemetry (FR-B1-3). Pre: connected. Post: one 28-byte frame at the laptop.

```mermaid
sequenceDiagram
  participant B2 as B2/B3/B4/B5
  participant API as D1.1 API
  participant CT as D1.2 comm task
  participant MQ as D1.7 MQTT (WiFi task)
  participant L as Laptop
  B2->>API: comm_set_motion(...) (saturate, lock, store)
  Note over CT: every 500 ms while CONNECTED
  CT->>API: comm_get_telemetry() (copy under lock)
  CT->>CT: comm_pack_telemetry() -> 28 B
  CT->>MQ: tx mbf (TMO_POL)
  MQ->>L: publish .../telemetry QoS 0
```

### SD-2 Remote command (FR-B1-5, NFR-B1-5). Post: command in the VC queue and an ack at the laptop.

```mermaid
sequenceDiagram
  participant L as Laptop / dashboard
  participant MQ as D1.7 MQTT
  participant CT as D1.2 comm task
  participant VC as Vehicle Controller
  L->>MQ: .../cmd  3 bytes (QoS 1)
  MQ->>CT: rx mbf + flag (TMO_POL)
  CT->>CT: comm_pack_cmd_decode()
  alt invalid frame
    CT->>MQ: ack REJECTED + reason
  else PING / STATUS / TELEM
    CT->>MQ: ack OK (handled by S1)
  else robot command
    CT->>VC: mbf_remote_cmd (TMO_POL)
    CT->>MQ: ack OK, or BUSY if the VC queue is full
  end
  MQ->>L: .../cmd/ack 3 bytes (QoS 1)
```

### SD-3 WiFi lost and back (FR-B1-7, NFR-B1-1, NFR-B1-6). Edge case.

```mermaid
sequenceDiagram
  participant AP as Hotspot
  participant WF as D1.8 WiFi loop
  participant P as D1.6 policy
  participant MQ as D1.7 MQTT
  participant CT as D1.2 comm task
  AP--xWF: AP disappears (DISASSOC)
  WF->>MQ: mqtt_utk_link_down() (I1.8)
  MQ->>MQ: close session, cancel timers, retries = 0
  CT->>CT: state RECONNECTING, raise FAULT_WIFI_LOSS
  loop every poll
    WF->>P: s1_link_wifi_action(DOWN, elapsed)
    P-->>WF: REJOIN at >= 5 s (RESET if stuck in JOIN/NOIP >= 20 s)
    WF->>AP: cyw43_wifi_join()
  end
  AP-->>WF: associated, DHCP done (UP)
  WF->>MQ: mqtt_utk_start() -> connect at once
  CT->>CT: CONNECTED, clear FAULT_WIFI_LOSS, reconnects + 1, status frame
```

The robot keeps driving throughout: no S1 call waits on the network.

### SD-4 Broker host name lookup (NFR-B1-7). Only with `MQTT_BROKER_HOST`.

```mermaid
sequenceDiagram
  participant MQ as D1.7 MQTT
  participant DNS as lwIP DNS
  participant P as D1.6 policy
  MQ->>MQ: generation++, pending = true, arm 5 s timeout
  MQ->>DNS: dns_gethostbyname()
  alt cached or numeric
    DNS-->>MQ: ERR_OK + address
  else no DNS server yet (just after a re-join)
    DNS-->>MQ: callback(NULL) INSIDE the call, then ERR_INPROGRESS
  else normal
    DNS-->>MQ: ERR_INPROGRESS ... later callback(address or NULL)
  else no answer
    MQ->>MQ: timeout: generation++ (late answer ignored)
  end
  MQ->>P: s1_link_broker_choice(dns_ok, have_cached)
  P-->>MQ: RESOLVED / CACHED (last good address) / RETRY (backoff)
```

## 3. Unified statechart (comm task, `comm_link_state_t`)

```mermaid
stateDiagram-v2
  [*] --> INITIALISING
  INITIALISING --> DISABLED: radio failed / no NET build
  INITIALISING --> CONNECTING_WIFI: radio ready
  CONNECTING_WIFI --> CONNECTING_MQTT: link UP
  CONNECTING_MQTT --> CONNECTED: CONNACK
  CONNECTED --> RECONNECTING: WiFi or broker lost / FAULT_WIFI_LOSS
  RECONNECTING --> CONNECTED: link + session back / clear fault
  CONNECTING_WIFI --> OFFLINE: WiFi down > 30 s
  RECONNECTING --> OFFLINE: WiFi down > 30 s or 6 MQTT retries used
  OFFLINE --> RECONNECTING: link UP again / 60 s pause over
```

The MQTT client (D1.7) has its own sub-states: IDLE, RESOLVING,
CONNECTING, CONNECTED, BACKOFF (1, 2, 4, 8, 16, 16 s), OFFLINE (60 s).

## 4. Data and resources

Message sizes and RAM: `DATA_REPRESENTATION.md`. After the BARR-C rewrite the
B1 application state is unchanged (about 190 bytes); the DNS fallback adds 10
bytes in `mqtt_utk.c`, and one lwIP timeout slot.
