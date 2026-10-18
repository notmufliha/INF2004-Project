#!/usr/bin/env python3
"""
mqtt_monitor.py -- laptop-side monitor for the Group 25 robot car (Subsystem 1).

Shows everything the car publishes, lets you type commands, and counts
missing sequence numbers so you can prove "no dropped messages" for
Assumption A1 (run it for 10+ minutes and read the summary).

Setup (once):   pip install paho-mqtt
Run:            python3 tools/mqtt_monitor.py                  # broker on this PC
                python3 tools/mqtt_monitor.py --broker 192.168.1.10
                python3 tools/mqtt_monitor.py --log run1.jsonl  # also save every message

While running, type a command and press Enter, e.g.
    STOP        SPEED 40        MODE LINE        PING        TELEM 250
Ctrl-C prints the summary and exits.
"""

import argparse
import json
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is not installed.  Run:  pip install paho-mqtt")


class Stats:
    def __init__(self):
        self.count = {}       # topic suffix -> messages received
        self.last_seq = {}    # topic suffix -> last "seq" seen
        self.missing = {}     # topic suffix -> sequence numbers skipped
        self.started = time.time()
        self.lock = threading.Lock()

    def seen(self, kind, seq):
        with self.lock:
            self.count[kind] = self.count.get(kind, 0) + 1
            if seq is None:
                return
            prev = self.last_seq.get(kind)
            if prev is not None and seq > prev + 1:
                self.missing[kind] = self.missing.get(kind, 0) + (seq - prev - 1)
            if prev is not None and seq < prev:
                print(f"  ({kind} seq restarted at {seq}: car rebooted?)")
            self.last_seq[kind] = seq

    def summary(self):
        mins = (time.time() - self.started) / 60.0
        lines = [f"\n=== summary after {mins:.1f} min ==="]
        for kind in sorted(self.count):
            n = self.count[kind]
            lost = self.missing.get(kind, 0)
            pct = 100.0 * lost / (n + lost) if (n + lost) else 0.0
            lines.append(f"  {kind:10s} received {n:6d}   missing {lost:4d}  ({pct:.2f} %)")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default="localhost", help="broker address (default localhost)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="inf2004/g25/car",
                    help="MQTT_TOPIC_PREFIX from wifi_secrets.h")
    ap.add_argument("--quiet-telemetry", action="store_true",
                    help="count telemetry but do not print every frame")
    ap.add_argument("--log", help="append every message to this file (JSON lines)")
    args = ap.parse_args()

    stats = Stats()
    logf = open(args.log, "a", encoding="utf-8") if args.log else None

    def on_connect(client, userdata, flags, rc, *extra):
        print(f"connected to broker {args.broker}:{args.port}, listening on {args.prefix}/#")
        client.subscribe(args.prefix + "/#", qos=1)

    def on_message(client, userdata, msg):
        kind = msg.topic[len(args.prefix) + 1:] if msg.topic.startswith(args.prefix + "/") else msg.topic
        text = msg.payload.decode("utf-8", errors="replace")
        seq = None
        try:
            data = json.loads(text)
            if isinstance(data, dict) and "seq" in data and kind != "cmd/ack":
                seq = int(data["seq"])
        except ValueError:
            data = None
        if kind != "cmd":                      # our own commands echo back
            stats.seen(kind, seq)
        if logf:
            logf.write(json.dumps({"t": time.time(), "topic": msg.topic, "payload": text}) + "\n")
            logf.flush()
        if kind == "telemetry" and args.quiet_telemetry:
            return
        stamp = time.strftime("%H:%M:%S")
        retained = " (retained)" if msg.retain else ""
        print(f"{stamp} {kind:10s}{retained} {text}")

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)   # paho-mqtt 2.x
    except AttributeError:
        client = mqtt.Client()                                   # paho-mqtt 1.x
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()

    seq = 0
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            seq += 1
            cmd = f"{line} #{seq}" if "#" not in line else line
            client.publish(args.prefix + "/cmd", cmd, qos=1)
            print(f"  -> sent \"{cmd}\"")
    except KeyboardInterrupt:
        pass
    finally:
        print(stats.summary())
        client.loop_stop()
        if logf:
            logf.close()


if __name__ == "__main__":
    main()
