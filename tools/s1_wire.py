"""
s1_wire.py -- laptop side of the Group 25 binary MQTT format (Subsystem 1).

The car sends fixed-width fields packed MSB-first (Change Requirement 1:
minimum-bit data representation).  This module is the ONLY place that
knows the layouts on the laptop; it mirrors
app_program/subsystem1/comm_pack.c field for field.  Change both, and
MQTT.md section 2, together.

    import s1_wire
    s1_wire.decode("telemetry", payload)   -> dict
    s1_wire.encode_command("SPEED 40", seq=5) -> 3 bytes

Used by tools/mqtt_monitor.py, tools/mqtt_send_cmd.py and the host tests
in testing/subsystem1/host/.  Needs only the Python standard library.
"""

WIRE_VERSION = 1

# --------------------------------------------------------------------------
#  Names.  The Pico sends numbers only; the text lives here.
# --------------------------------------------------------------------------
ROBOT_STATES = ["INIT", "IDLE", "LINE_FOLLOW", "NAV_COMMAND", "HUMP",
                "OBSTACLE_SCAN", "OBSTACLE_AVOID", "LINE_RECOVERY", "SAFE_STOP"]
LINK_STATES = ["INITIALISING", "CONNECTING_WIFI", "CONNECTING_MQTT", "CONNECTED",
               "RECONNECTING", "OFFLINE", "DISABLED"]
# B4's imu_motion_state_t (subsystem4/imu_types.h)
MOTION_STATES = ["STATIONARY", "ACCELERATING", "TURNING", "CLIMBING_HUMP",
                 "HUMP_PEAK", "DESCENDING_HUMP", "SUDDEN_IMPACT", "UNKNOWN"]
NAV = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"]
AVOID = ["NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"]
BARCODES = ["-", "A", "B", "C", "D"]
EVENT_TYPES = ["barcode", "hump", "obstacle", "error"]
MODES = ["AUTO", "LINE", "TEST"]

# 4-bit command ids: 1..8 go to the Vehicle Controller, 9..11 to S1
COMMANDS = {"START": 1, "STOP": 2, "ESTOP": 3, "PAUSE": 4, "RESUME": 5,
            "MODE": 6, "SPEED": 7, "RESET_PEAK": 8,
            "PING": 9, "STATUS": 10, "TELEM": 11}
COMMAND_NAMES = {v: k for k, v in COMMANDS.items()}

RESULTS = {0: "forwarded", 1: "pong", 2: "status sent", 3: "telemetry period set",
           8: "frame is not 3 bytes", 9: "unknown wire version",
           10: "reserved bits not zero", 11: "unknown command",
           12: "argument out of range", 13: "unexpected argument",
           14: "controller busy"}

# Shared fault bitmask (app_program/fault_flags.h)
FAULT_BITS = {0x01: "MOTOR_STALL", 0x02: "ENCODER", 0x04: "IMU",
              0x08: "NO_ECHO", 0x10: "WIFI_LOSS", 0x20: "LINE_LOST"}

# 1-byte error codes: 3-bit subsystem + 5-bit number (app_program/error_codes.h)
ERROR_NAMES = {
    (0, 1): "watchdog: DRIVE refresh missed",
    (1, 1): "S1 outgoing queue full", (1, 2): "S1 controller queue full",
    (1, 31): "S1 self-test",
    (2, 1): "motor stall", (2, 2): "encoder fault", (2, 3): "10 ms loop overrun",
    (2, 4): "motion deadline exceeded", (2, 5): "standstill timeout",
    (2, 6): "motion command fault", (2, 7): "B2 DRIVE refresh missed",
    (3, 1): "line lost", (3, 2): "bad barcode",
    (4, 1): "IMU fault",
    # B5: numbers match S5Error in subsystem5/s5_logic.h.
    # 0xA1: detail = forward servo angle / 2 (90 / 2 = 45).
    # Do not relabel 0xA2 as clearance or 0xA3 as servo-not-settled:
    # those would conflict with the team's existing firmware codes.
    (5, 1): "B5 no reliable ultrasonic echo / front sensor failure",
    (5, 2): "B5 motor stop acknowledgement failed",
    (5, 3): "B5 scan timeout",
    (5, 4): "B5 no safe obstacle bypass route",
    (5, 5): "B5 fine scan failed",
    (5, 6): "B5 motion timeout",
    (5, 7): "B5 line reacquisition timeout",
    (5, 8): "B5 insufficient clearance",
    (5, 9): "B5 reverse timeout",
}

DIST_INVALID = 0x1FF    # 9-bit all ones: no echo / not measured
WIDTH_INVALID = 0x7F

# --------------------------------------------------------------------------
#  Layouts: (name, bits, signed).  Same order as comm_pack.c.
# --------------------------------------------------------------------------
TELEMETRY = [
    ("ver", 3, False), ("seq", 16, False), ("t_ds", 20, False),
    ("state", 4, False), ("faults", 8, False),
    ("spd_l", 14, True), ("spd_r", 14, True),          # counts/s (B2)
    ("enc_l", 24, True), ("enc_r", 24, True),          # counts, low 24 bits
    ("ir_l", 12, False), ("ir_c", 12, False), ("ir_r", 1, False),
    ("line", 3, False), ("barcode", 3, False), ("nav", 3, False),
    ("motion", 3, False), ("pitch_dd", 10, True),      # 0.1 deg (B4)
    ("hump_valid", 1, False), ("hump_last_mm", 7, False), ("hump_peak_mm", 7, False),
    ("obs_d_cm", 9, False), ("obs_w_cm", 7, False),    # B5
    ("obs_lc_cm", 9, False), ("obs_rc_cm", 9, False),
]
EVENT_HEADER = [("ver", 3, False), ("type", 2, False), ("seq", 8, False),
                ("t_ds", 20, False)]
EVENT_BODY = {
    0: [("code", 3, False), ("nav", 3, False)],
    1: [("valid", 1, False), ("height_mm", 7, False), ("peak_mm", 7, False)],
    2: [("d_cm", 9, False), ("w_cm", 7, False), ("lc_cm", 9, False),
        ("rc_cm", 9, False), ("action", 3, False)],
    3: [("err", 8, False), ("detail", 8, False)],
}
STATUS = [
    ("ver", 3, False), ("online", 1, False), ("link", 3, False), ("demo", 1, False),
    ("seq", 16, False), ("t_ds", 20, False), ("ip", 32, False),
    ("telem_ds", 7, False), ("faults", 8, False),
    ("tx", 16, False), ("drop", 16, False), ("evdrop", 16, False),
    ("rx", 16, False), ("rej", 16, False), ("reconn", 8, False),
]
ACK = [("ver", 3, False), ("seq", 8, False), ("cmd", 4, False), ("ok", 1, False),
       ("result", 5, False), ("pad", 3, False)]
COMMAND = [("ver", 3, False), ("cmd", 4, False), ("arg", 7, False),
           ("seq", 8, False), ("reserved", 2, False)]


def nbytes(layout):
    return (sum(b for _, b, _ in layout) + 7) // 8


# --------------------------------------------------------------------------
#  Bit packing (MSB first), the same as bits_put / bits_get in comm_pack.c
# --------------------------------------------------------------------------
def unpack(layout, data, pos=0):
    """Returns (dict, bit position after the last field)."""
    value = int.from_bytes(data, "big")
    total = len(data) * 8
    out = {}
    for name, bits, signed in layout:
        if pos + bits > total:
            raise ValueError(f"frame too short for field '{name}'")
        v = (value >> (total - pos - bits)) & ((1 << bits) - 1)
        if signed and v & (1 << (bits - 1)):
            v -= 1 << bits
        out[name] = v
        pos += bits
    return out, pos


def pack(layout, values):
    acc = 0
    total = 0
    for name, bits, _ in layout:
        v = values.get(name, 0)
        acc = (acc << bits) | (v & ((1 << bits) - 1))
        total += bits
    pad = (-total) % 8
    return (acc << pad).to_bytes((total + pad) // 8, "big")


def _name(table, i):
    return table[i] if 0 <= i < len(table) else f"?{i}"


def fault_names(mask):
    return [n for bit, n in FAULT_BITS.items() if mask & bit] or ["none"]


def error_text(code):
    sub, num = (code >> 5) & 7, code & 31
    return f"S{sub}#{num} " + ERROR_NAMES.get((sub, num), "unknown")


def _cm(v, invalid=DIST_INVALID):
    return None if v == invalid else v


def _check_version(d):
    if d["ver"] != WIRE_VERSION:
        raise ValueError(f"wire version {d['ver']}, expected {WIRE_VERSION}")


# --------------------------------------------------------------------------
#  Decoders: raw fields plus readable extras
# --------------------------------------------------------------------------
def decode_telemetry(data):
    d, _ = unpack(TELEMETRY, data)
    _check_version(d)
    d["t_s"] = d["t_ds"] / 10
    d["state_name"] = _name(ROBOT_STATES, d["state"])
    d["fault_names"] = fault_names(d["faults"])
    d["line_flags"] = [n for bit, n in ((1, "DETECTED"), (2, "LOST"), (4, "JUNCTION"))
                       if d["line"] & bit]
    d["barcode_name"] = _name(BARCODES, d["barcode"])
    d["nav_name"] = _name(NAV, d["nav"])
    d["motion_name"] = _name(MOTION_STATES, d["motion"])
    d["pitch_deg"] = d["pitch_dd"] / 10
    d["obs_d_cm"] = _cm(d["obs_d_cm"])
    d["obs_w_cm"] = _cm(d["obs_w_cm"], WIDTH_INVALID)
    d["obs_lc_cm"] = _cm(d["obs_lc_cm"])
    d["obs_rc_cm"] = _cm(d["obs_rc_cm"])
    return d


def decode_event(data):
    d, pos = unpack(EVENT_HEADER, data)
    _check_version(d)
    body, _ = unpack(EVENT_BODY[d["type"]], data, pos)
    d.update(body)
    d["t_s"] = d["t_ds"] / 10
    d["type_name"] = EVENT_TYPES[d["type"]]
    if d["type"] == 0:
        d["code_name"] = _name(BARCODES, d["code"])
        d["nav_name"] = _name(NAV, d["nav"])
    elif d["type"] == 2:
        d["action_name"] = _name(AVOID, d["action"])
        for k, inv in (("d_cm", DIST_INVALID), ("w_cm", WIDTH_INVALID),
                       ("lc_cm", DIST_INVALID), ("rc_cm", DIST_INVALID)):
            d[k] = _cm(d[k], inv)
    elif d["type"] == 3:
        d["err_text"] = error_text(d["err"])
    return d


def decode_status(data):
    if len(data) == 1:                  # retained online / Last Will byte
        d = {"ver": data[0] >> 5, "online": (data[0] >> 4) & 1}
        _check_version(d)
        return d
    d, _ = unpack(STATUS, data)
    _check_version(d)
    d["link_name"] = _name(LINK_STATES, d["link"])
    d["ip_str"] = ".".join(str((d["ip"] >> s) & 0xFF) for s in (0, 8, 16, 24))
    d["telem_ms"] = d["telem_ds"] * 100
    d["fault_names"] = fault_names(d["faults"])
    d["t_s"] = d["t_ds"] / 10
    return d


def decode_ack(data):
    d, _ = unpack(ACK, data)
    _check_version(d)
    d["cmd_name"] = COMMAND_NAMES.get(d["cmd"], f"?{d['cmd']}")
    d["result_text"] = RESULTS.get(d["result"], f"?{d['result']}")
    return d


def decode_command(data):
    d, _ = unpack(COMMAND, data)
    d["cmd_name"] = COMMAND_NAMES.get(d["cmd"], f"?{d['cmd']}")
    return d


DECODERS = {"telemetry": decode_telemetry, "event": decode_event,
            "status": decode_status, "cmd/ack": decode_ack, "cmd": decode_command}


def decode(kind, data):
    """kind = topic suffix ("telemetry", "event", "status", "cmd/ack", "cmd")."""
    return DECODERS[kind](bytes(data))


# --------------------------------------------------------------------------
#  Commands: text the user types -> 3-byte frame
# --------------------------------------------------------------------------
def encode_command(text, seq=0):
    """'SPEED 40', 'MODE LINE', 'TELEM 500' (ms), 'STOP' -> bytes.
    Raises ValueError for anything the car would reject anyway."""
    words = text.upper().split()
    if not words or words[0] not in COMMANDS:
        raise ValueError(f"unknown command: {text!r}")
    verb, args = words[0], words[1:]
    arg = 0
    if verb == "SPEED":
        arg = _one_int(verb, args, 0, 100)
    elif verb == "MODE":
        if len(args) != 1 or args[0] not in MODES:
            raise ValueError("MODE needs AUTO, LINE or TEST")
        arg = MODES.index(args[0])
    elif verb == "TELEM":
        ms = _one_int(verb, args, 100, 10000)
        if ms % 100:
            raise ValueError("TELEM period must be a multiple of 100 ms")
        arg = ms // 100
    elif args:
        raise ValueError(f"{verb} takes no argument")
    return pack(COMMAND, {"ver": WIRE_VERSION, "cmd": COMMANDS[verb],
                          "arg": arg, "seq": seq & 0xFF})


def _one_int(verb, args, lo, hi):
    if len(args) != 1 or not args[0].isdigit():
        raise ValueError(f"{verb} needs one number {lo}..{hi}")
    v = int(args[0])
    if not lo <= v <= hi:
        raise ValueError(f"{verb} {v} is outside {lo}..{hi}")
    return v


# --------------------------------------------------------------------------
#  Encoder totals arrive as the low 24 bits.  Between two telemetry frames
#  (<= 10 s at B2's 5000 counts/s = 50,000 counts << 2^23) the change is
#  always small, so the full count can be rebuilt on the laptop.
# --------------------------------------------------------------------------
class Unwrap24:
    def __init__(self):
        self.total = None

    def update(self, low24):
        low24 &= 0xFFFFFF
        if self.total is None:
            self.total = low24 - (1 << 24) if low24 & 0x800000 else low24
        else:
            delta = (low24 - self.total) & 0xFFFFFF
            if delta & 0x800000:
                delta -= 1 << 24
            self.total += delta
        return self.total


def describe(kind, d):
    """One readable line for the console."""
    if kind == "telemetry":
        obs = "-" if d["obs_d_cm"] is None else f"{d['obs_d_cm']}cm"
        hump = (f"{d['hump_last_mm']}/{d['hump_peak_mm']}mm" if d["hump_valid"]
                else "n/a")
        return (f"#{d['seq']} t={d['t_s']:.1f}s {d['state_name']} "
                f"spd={d['spd_l']},{d['spd_r']}c/s enc={d['enc_l']},{d['enc_r']} "
                f"ir={d['ir_l']},{d['ir_c']},{d['ir_r']} line={'+'.join(d['line_flags']) or '-'} "
                f"bc={d['barcode_name']}/{d['nav_name']} imu={d['motion_name']} "
                f"pitch={d['pitch_deg']:+.1f} hump={hump} obs={obs} "
                f"faults={','.join(d['fault_names'])}")
    if kind == "event":
        head = f"#{d['seq']} t={d['t_s']:.1f}s {d['type_name'].upper()}"
        if d["type"] == 0:
            return f"{head} {d['code_name']} -> {d['nav_name']}"
        if d["type"] == 1:
            return (f"{head} height={d['height_mm']}mm peak={d['peak_mm']}mm"
                    if d["valid"] else f"{head} (height not measured)")
        if d["type"] == 2:
            return (f"{head} d={d['d_cm']} w={d['w_cm']} lc={d['lc_cm']} "
                    f"rc={d['rc_cm']} cm -> {d['action_name']}")
        return f"{head} {d['err_text']} detail={d['detail']}"
    if kind == "status":
        if "link" not in d:
            return "ONLINE" if d["online"] else "OFFLINE (last will)"
        return (f"#{d['seq']} {d['link_name']} ip={d['ip_str']} telem={d['telem_ms']}ms "
                f"tx={d['tx']} drop={d['drop']} evdrop={d['evdrop']} rx={d['rx']} "
                f"rej={d['rej']} reconn={d['reconn']} faults={','.join(d['fault_names'])}"
                + (" DEMO-DATA" if d["demo"] else ""))
    if kind == "cmd/ack":
        return (f"#{d['seq']} {d['cmd_name']} {'OK' if d['ok'] else 'REJECTED'}: "
                f"{d['result_text']}")
    if kind == "cmd":
        return f"#{d['seq']} {d['cmd_name']} arg={d['arg']}"
    return str(d)
