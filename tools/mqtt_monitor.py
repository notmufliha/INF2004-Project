#!/usr/bin/env python3
"""
mqtt_monitor.py -- laptop-side monitor for the Group 25 robot car (Subsystem 1).

Decodes the car's binary frames (tools/s1_wire.py), lets you type commands
(sent as 3-byte binary frames), and counts missing sequence numbers so you
can prove "no dropped messages" for Assumption A1 (run it for 10+ minutes
and read the summary).

Setup (once):   pip install paho-mqtt
Run:            python3 tools/mqtt_monitor.py                  # broker on this PC
                python3 tools/mqtt_monitor.py --broker 192.168.1.10
                python3 tools/mqtt_monitor.py --log run1.jsonl  # also save every message
                python3 tools/mqtt_monitor.py --raw             # also print the hex bytes

While running, type a command and press Enter, e.g.
    STOP        SPEED 40        MODE LINE        PING        TELEM 300
(TELEM is in ms, a multiple of 100.)  Ctrl-C prints the summary and exits.
"""

import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s1_wire  # noqa: E402

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is not installed.  Run:  pip install paho-mqtt")

# Sequence numbers wrap: 16 bits for telemetry/status, 8 bits for events
SEQ_BITS = {"telemetry": 16, "status": 16, "event": 8}


class Stats:
    def __init__(self):
        self.count = {}       # topic suffix -> messages received
        self.last_seq = {}    # topic suffix -> last "seq" seen
        self.missing = {}     # topic suffix -> sequence numbers skipped
        self.bad = 0          # frames that did not decode
        self.started = time.time()
        self.lock = threading.Lock()

    def seen(self, kind, seq):
        with self.lock:
            self.count[kind] = self.count.get(kind, 0) + 1
            if seq is None:
                return
            bits = SEQ_BITS[kind]
            prev = self.last_seq.get(kind)
            if prev is not None:
                gap = (seq - prev) % (1 << bits)
                if gap == 0 or gap > (1 << bits) // 2:
                    print(f"  ({kind} seq jumped back to {seq}: car rebooted?)")
                elif gap > 1:
                    self.missing[kind] = self.missing.get(kind, 0) + gap - 1
            self.last_seq[kind] = seq

    def summary(self):
        mins = (time.time() - self.started) / 60.0
        lines = [f"\n=== summary after {mins:.1f} min ==="]
        for kind in sorted(self.count):
            n = self.count[kind]
            lost = self.missing.get(kind, 0)
            pct = 100.0 * lost / (n + lost) if (n + lost) else 0.0
            lines.append(f"  {kind:10s} received {n:6d}   missing {lost:4d}  ({pct:.2f} %)")
        if self.bad:
            lines.append(f"  undecodable frames: {self.bad}")
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
    ap.add_argument("--log", help="append every decoded message to this file (JSON lines)")
    ap.add_argument("--raw", action="store_true", help="also print each frame in hex")
    args = ap.parse_args()

    stats = Stats()
    logf = open(args.log, "a", encoding="utf-8") if args.log else None
    enc_l, enc_r = s1_wire.Unwrap24(), s1_wire.Unwrap24()

    def on_connect(client, userdata, flags, rc, *extra):
        print(f"connected to broker {args.broker}:{args.port}, listening on {args.prefix}/#")
        client.subscribe(args.prefix + "/#", qos=1)

    def on_message(client, userdata, msg):
        kind = msg.topic[len(args.prefix) + 1:] if msg.topic.startswith(args.prefix + "/") else msg.topic
        raw = bytes(msg.payload)
        try:
            data = s1_wire.decode(kind, raw)
            text = s1_wire.describe(kind, data)
        except (KeyError, ValueError) as e:
            data, text = None, f"UNDECODABLE ({e}): {raw.hex(' ')}"
            with stats.lock:
                stats.bad += 1
        if data is not None and kind == "telemetry":
            data["enc_l_total"] = enc_l.update(data["enc_l"])   # undo the
            data["enc_r_total"] = enc_r.update(data["enc_r"])   # 24-bit wrap
        if kind != "cmd":                      # our own commands echo back
            seq = data.get("seq") if (data and kind in SEQ_BITS) else None
            stats.seen(kind, seq)
        if logf:
            logf.write(json.dumps({"t": time.time(), "topic": msg.topic, "len": len(raw),
                                   "hex": raw.hex(), "data": data}) + "\n")
            logf.flush()
        if kind == "telemetry" and args.quiet_telemetry:
            return
        stamp = time.strftime("%H:%M:%S")
        retained = " (retained)" if msg.retain else ""
        hexdump = f"  [{len(raw)} B: {raw.hex(' ')}]" if args.raw else ""
        print(f"{stamp} {kind:10s}{retained} {text}{hexdump}")

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
            seq = (seq + 1) & 0xFF
            try:
                frame = s1_wire.encode_command(line, seq)
            except ValueError as e:
                print(f"  not sent: {e}")
                continue
            client.publish(args.prefix + "/cmd", frame, qos=1)
            print(f"  -> sent {line.upper()} #{seq}  [{frame.hex(' ')}]")
    except KeyboardInterrupt:
        pass
    finally:
        print(stats.summary())
        client.loop_stop()
        if logf:
            logf.close()


if __name__ == "__main__":
    main()
