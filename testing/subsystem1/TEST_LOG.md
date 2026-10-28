# Subsystem 1 — test log

**Subsystem:** WiFi Communication, Command & Telemetry  ·  **Student ID:** 2502999  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results, and
`app_program/subsystem1/MQTT.md` for the topics and message formats.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | LED cyclic handler is running | — | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | comm_task sleeps between loops (no busy-wait) | NFR-B1-3 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | valid 3-byte binary commands accepted and decoded (id, arg, seq) | FR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | invalid frames rejected: wrong length, version, reserved bits, id, range, extra arg (10 cases) | NFR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | binary frames have the documented sizes (28/5-9/23/3 B) and packing never overruns (was: JSON writer bounded) | CR1, robustness | auto | `subsystem1_test.c` | `[TEST] S1-U05 PASS` + size info line |
| S1-U06 | 40 events posted in < 20 ms, never blocks when full | NFR-B1-2 | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | packed telemetry setters/getter consistent, out-of-range values saturated, encoder low 24 bits | IF-S1-TLM, CR1 | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | received binary frames validated (1 ok, 1 rejected) | FR-B1-5, NFR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | WiFi + MQTT connect within 30 s | FR-B1-1, FR-B1-2 | auto (needs AP + broker) | `make NET=lwip MQTT=1 TEST=1` | `[TEST] S1-U09 PASS` |
| S1-U10 | telemetry published for 3 s with no drops | FR-B1-3 | auto (needs AP + broker) | `make NET=lwip MQTT=1 TEST=1` | `[TEST] S1-U10 PASS` |
| S1-U11 | telemetry visible on laptop at 2 Hz | FR-B1-3 | manual | 1. Flash `..._lwip_mqtt.uf2`. 2. `python tools/mqtt_monitor.py` | `telemetry` line every 0.5 s, `#seq` +1 each |
| S1-U12 | events published (barcode/hump/obstacle) | FR-B1-4 | manual | Build with `S1_STANDALONE_DEMO=1` (or trigger B4/B5 for real), watch `event` lines in `mqtt_monitor.py` for 30 s | demo: barcode every 8 s, hump every 13 s, obstacle every 21 s |
| S1-U13 | remote command round trip | FR-B1-5 | manual | `python tools/mqtt_send_cmd.py STOP --seq 1` (demo mode on) | ack `#1 STOP OK: forwarded`, console `controller got STOP`, telemetry state `SAFE_STOP` |
| S1-U14 | invalid command rejected on the wire | NFR-B1-5 | manual | `python tools/mqtt_send_cmd.py --hex 2e 00` and `--hex e2 00 00` | ack `REJECTED` with reason (length / version); not forwarded |
| S1-U15 | heartbeat every 5 s | FR-B1-6 | manual | watch `.../status` | heartbeat `seq` +1 every 5 s |
| S1-U16 | broker loss → reconnect | FR-B1-7 | manual | stop Mosquitto 30 s, restart | console: retry 1,2,4,8 s…; LED slow blink; reconnects; `reconn` +1 |
| S1-U17 | WiFi loss → reconnect, robot unaffected | FR-B1-7, NFR-B1-1 | manual | switch hotspot off 20 s, on again | LED fast blink, `demo_task` keeps running, reconnects |
| S1-U18 | Last Will "offline" | FR-B1-6 | manual | unplug the Pico while `mqtt_monitor.py` runs | within ~15 s `status (retained) OFFLINE (last will)` (byte `0x20`) |
| S1-U19 | 10-minute soak, count lost frames | Assumption A1 | manual | `python3 tools/mqtt_monitor.py --quiet-telemetry --log logs/<date>_soak.jsonl`, 10 min, Ctrl-C | summary: telemetry missing ≈ 0 %, events/acks 0 missing |
| S1-U20 | shared fault bitmask: 1000 rounds of set/clear on two bits, the other bit never disturbed | CR1 (safe shared packed data) | auto | `subsystem1_test.c` | `[TEST] S1-U20 PASS` |
| S1-U21 | WiFi-loss bit in the shared fault mask | CR1, FR-B1-7 | manual | USB console open; switch the hotspot off 20 s, then on | console `s1: link CONNECTED -> RECONNECTING  faults=0x10`, then `... -> CONNECTED  faults=0x00` |
| S1-U22 | real B4/B5 data reaches the laptop | integration | manual | robot build with `NET=lwip MQTT=1`; tilt the car, put a hand 20 cm in front of the ultrasonic sensor | telemetry `imu=` and `pitch=` change; `obs=20cm`; an `OBSTACLE` event after the scan; `NO_ECHO` fault when the sensor is unplugged |
| S1-H01 | Pico packer and laptop decoder agree bit for bit (18 frames at min/max/invalid values, 24 command frames, 24-bit unwrap) | CR1, R2 (decoder mismatch) | auto (host) | `sh testing/subsystem1/host/run_host_tests.sh` | `ALL HOST TESTS PASSED` |
| S1-M01 | RAM before/after the change | CR1 Week 13 | manual | see `DATA_REPRESENTATION.md` section 3 | `.bss` lower by ~1.6 KB, kernel buffers by 3,232 B |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, laptop terminal, broker log).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-28 | S1-H01 | PR #7 | B1 | PASS | `logs/2026-10-28_host_S1-H01.txt` | 18 frames + 24 command frames + unwrap; run on the laptop, no Pico |
| 2026-10-28 | S1-M01 | PR #7 | B1 | DONE | `logs/before_ram_main.txt`, `logs/after_ram.txt` | data+bss 81,012 -> 79,452 B (-1,560); B1 symbols 1,778 -> 192 B. Build only, not run |
