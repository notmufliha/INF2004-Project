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
| S1-U03 | valid commands accepted and decoded | FR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | invalid commands rejected (12 cases) | NFR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | JSON writer bounded, escapes quotes | robustness | auto | `subsystem1_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | 40 events posted in < 20 ms, never blocks when full | NFR-B1-2 | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | telemetry setters/getter consistent | IF-S1-TLM | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | received commands validated (1 ok, 1 rejected) | FR-B1-5, NFR-B1-5 | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | WiFi + MQTT connect within 30 s | FR-B1-1, FR-B1-2 | auto (needs AP + broker) | `make NET=lwip MQTT=1 TEST=1` | `[TEST] S1-U09 PASS` |
| S1-U10 | telemetry published for 3 s with no drops | FR-B1-3 | auto (needs AP + broker) | `make NET=lwip MQTT=1 TEST=1` | `[TEST] S1-U10 PASS` |
| S1-U11 | telemetry visible on laptop at 2 Hz | FR-B1-3 | manual | 1. Flash `..._lwip_mqtt.uf2`. 2. `mosquitto_sub -v -t 'inf2004/g25/car/#'` | `telemetry` line every 0.5 s, `seq` +1 each |
| S1-U12 | events published (barcode/hump/obstacle) | FR-B1-4 | manual | Demo mode on, watch `.../event` for 30 s | barcode every 8 s, hump every 13 s, obstacle every 21 s |
| S1-U13 | remote command round trip | FR-B1-5 | manual | `mosquitto_pub -t inf2004/g25/car/cmd -m "STOP #1"` | ack `ok:true`, console `controller got STOP`, telemetry `state` = `SAFE_STOP` |
| S1-U14 | invalid command rejected on the wire | NFR-B1-5 | manual | publish `SPEED 400`, `FLY` | ack `ok:false` with reason; not forwarded |
| S1-U15 | heartbeat every 5 s | FR-B1-6 | manual | watch `.../status` | heartbeat `seq` +1 every 5 s |
| S1-U16 | broker loss → reconnect | FR-B1-7 | manual | stop Mosquitto 30 s, restart | console: retry 1,2,4,8 s…; LED slow blink; reconnects; `reconn` +1 |
| S1-U17 | WiFi loss → reconnect, robot unaffected | FR-B1-7, NFR-B1-1 | manual | switch hotspot off 20 s, on again | LED fast blink, `demo_task` keeps running, reconnects |
| S1-U18 | Last Will "offline" | FR-B1-6 | manual | unplug the Pico while subscribed | within ~15 s `status` = `{"online":false}` (retained) |
| S1-U19 | 10-minute soak, count lost frames | Assumption A1 | manual | `python3 tools/mqtt_monitor.py --quiet-telemetry --log logs/<date>_soak.jsonl`, 10 min, Ctrl-C | summary: telemetry missing ≈ 0 %, events/acks 0 missing |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, laptop terminal, broker log).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
