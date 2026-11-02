# Robot car dashboard (Subsystem 1)

A single web page that shows everything the car publishes over MQTT and sends
commands back.  It decodes the same binary frames as `tools/s1_wire.py`
(`s1_wire.js` here is a line-by-line copy, checked against the C packer by
host test S1-H06).

| Panel | Shows | From |
|---|---|---|
| Tiles | robot state, the car's own link state + IP, telemetry age and period, missing telemetry frames (sequence gaps), fault bits | telemetry + heartbeat |
| Motion | left/right wheel speed chart (last 60 s), unwrapped encoder totals | B2 |
| Line & barcode | IR left/centre bars, right sensor, line flags, last barcode and its command | B3 |
| IMU & hump | motion state, pitch, last/peak hump height | B4 |
| Obstacle | front distance chart, width, left/right clearance | B5 |
| Link health | frames queued/dropped, commands ok/rejected, reconnects, heartbeat age | B1 heartbeat |
| Commands | START / PAUSE / RESUME / STOP / E-STOP, speed, mode, reset peak, ping, status, any typed command; every ack with its round-trip time | B1 -> Vehicle Controller |
| Events | barcode / hump / obstacle / error events with names | all |

Space bar = E-STOP (when the cursor is not in a text box).

## Try it without the car

Open `tools/dashboard/index.html` in a browser and press **Demo data**.  A fake
car inside the page produces real binary frames and answers commands, so the
whole decode path is exercised.

## Use it with the car

The browser can only speak MQTT over WebSockets, so Mosquitto needs a second
listener.  Add to `mosquitto.conf` (Windows: `C:\Program Files\mosquitto\`,
Ubuntu: `/etc/mosquitto/conf.d/dashboard.conf`):

```
listener 1883
listener 9001
protocol websockets
allow_anonymous true
```

Restart Mosquitto, open `index.html`, keep `ws://localhost:9001` (or use the
laptop's IP from another device) and the topic prefix `inf2004/g25/car`, then
press **Connect**.  The car still connects on port 1883 as before.

The page loads mqtt.js from unpkg.com, so the laptop needs internet the first
time the page is opened (the browser caches it afterwards).

## What to look at during tests

- **WiFi reconnect (S1-S03):** switch the hotspot off for 30 s.  The car badge
  turns *OFFLINE (last will)* after ~15 s, and *Missing frames* stays still,
  because telemetry pauses while offline.  Switch it back on: the badge
  returns to *online* and *Reconnects* goes up by one.
- **Command latency (S1-S02):** the ack log shows the round-trip time of every
  command (laptop -> broker -> car -> broker -> laptop).
- **No dropped telemetry (A1):** leave it connected for 10 minutes; *Missing
  frames* should stay at 0.
