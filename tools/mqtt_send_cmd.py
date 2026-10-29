#!/usr/bin/env python3
"""
mqtt_send_cmd.py -- send ONE binary command to the Group 25 car and print
its acknowledgement (Subsystem 1).

Commands are 3-byte frames (tools/s1_wire.py, MQTT.md section 3).  Type
them as text; this tool encodes them, publishes on <prefix>/cmd and waits
for the matching ack on <prefix>/cmd/ack.

Setup (once):   pip install paho-mqtt
Examples:       python3 tools/mqtt_send_cmd.py STOP
                python3 tools/mqtt_send_cmd.py SPEED 40 --seq 7
                python3 tools/mqtt_send_cmd.py TELEM 300 --broker 192.168.1.10
                python3 tools/mqtt_send_cmd.py --hex 2e 00 2c     # raw frame, for
                                                                   # rejection tests
Exit status: 0 accepted, 1 rejected, 2 no ack within --timeout.
"""

import argparse
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s1_wire  # noqa: E402

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is not installed.  Run:  pip install paho-mqtt")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="*", help="e.g. STOP, SPEED 40, MODE LINE, TELEM 500")
    ap.add_argument("--hex", nargs="+", help="send these raw bytes instead (hex)")
    ap.add_argument("--seq", type=int, default=1, help="sequence number 0..255 (default 1)")
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="inf2004/g25/car")
    ap.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for the ack")
    args = ap.parse_args()

    if args.hex:
        frame = bytes.fromhex("".join(args.hex))
    else:
        try:
            frame = s1_wire.encode_command(" ".join(args.command), args.seq)
        except ValueError as e:
            sys.exit(f"not sent: {e}")
    # Match the ack by sequence number; a malformed --hex frame is acked
    # with seq 0, so accept any ack then
    seq = s1_wire.decode_command(frame)["seq"] if len(frame) == 3 else None

    got = threading.Event()
    result = {}

    def on_connect(client, userdata, flags, rc, *extra):
        client.subscribe(args.prefix + "/cmd/ack", qos=1)

    def on_subscribe(client, userdata, *extra):
        client.publish(args.prefix + "/cmd", frame, qos=1)
        print(f"sent {len(frame)} B [{frame.hex(' ')}]")

    def on_message(client, userdata, msg):
        try:
            d = s1_wire.decode_ack(msg.payload)
        except ValueError as e:
            print(f"undecodable ack: {e}")
            return
        if seq is None or d["seq"] == seq:
            result.update(d)
            got.set()

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)   # paho-mqtt 2.x
    except AttributeError:
        client = mqtt.Client()                                   # paho-mqtt 1.x
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()
    ok = got.wait(args.timeout)
    client.loop_stop()

    if not ok:
        print(f"no ack within {args.timeout} s (car offline?)")
        sys.exit(2)
    print("ack " + s1_wire.describe("cmd/ack", result))
    sys.exit(0 if result["ok"] else 1)


if __name__ == "__main__":
    main()
