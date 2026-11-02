/*
 * s1_wire.js -- browser/Node copy of tools/s1_wire.py (Subsystem 1 binary
 * MQTT format).  Mirrors app_program/subsystem1/comm_pack.c field for field;
 * change it together with comm_pack.c, s1_wire.py and MQTT.md section 2.
 * testing/subsystem1/host/test_dashboard_decoder.js checks it against the
 * frames the C packer produces (S1-H06).
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.S1Wire = factory();
  }
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  const WIRE_VERSION = 1;

  const ROBOT_STATES = ["INIT", "IDLE", "LINE_FOLLOW", "NAV_COMMAND", "HUMP",
    "OBSTACLE_SCAN", "OBSTACLE_AVOID", "LINE_RECOVERY", "SAFE_STOP"];
  const LINK_STATES = ["INITIALISING", "CONNECTING_WIFI", "CONNECTING_MQTT",
    "CONNECTED", "RECONNECTING", "OFFLINE", "DISABLED"];
  const MOTION_STATES = ["STATIONARY", "ACCELERATING", "TURNING",
    "CLIMBING_HUMP", "HUMP_PEAK", "DESCENDING_HUMP", "SUDDEN_IMPACT", "UNKNOWN"];
  const NAV = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"];
  const AVOID = ["NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"];
  const BARCODES = ["-", "A", "B", "C", "D"];
  const EVENT_TYPES = ["barcode", "hump", "obstacle", "error"];
  const MODES = ["AUTO", "LINE", "TEST"];
  const COMMANDS = { START: 1, STOP: 2, ESTOP: 3, PAUSE: 4, RESUME: 5,
    MODE: 6, SPEED: 7, RESET_PEAK: 8, PING: 9, STATUS: 10, TELEM: 11 };
  const COMMAND_NAMES = Object.fromEntries(
    Object.entries(COMMANDS).map(([k, v]) => [v, k]));
  const RESULTS = { 0: "forwarded", 1: "pong", 2: "status sent",
    3: "telemetry period set", 8: "frame is not 3 bytes",
    9: "unknown wire version", 10: "reserved bits not zero",
    11: "unknown command", 12: "argument out of range",
    13: "unexpected argument", 14: "controller busy" };
  const FAULT_BITS = [[0x01, "MOTOR_STALL"], [0x02, "ENCODER"], [0x04, "IMU"],
    [0x08, "NO_ECHO"], [0x10, "WIFI_LOSS"], [0x20, "LINE_LOST"]];
  const ERROR_NAMES = {
    "0,1": "watchdog: DRIVE refresh missed",
    "1,1": "S1 outgoing queue full", "1,2": "S1 controller queue full",
    "1,31": "S1 self-test",
    "2,1": "motor stall", "2,2": "encoder fault", "2,3": "10 ms loop overrun",
    "2,4": "motion deadline exceeded", "2,5": "standstill timeout",
    "2,6": "motion command fault", "2,7": "B2 DRIVE refresh missed",
    "3,1": "line lost", "3,2": "bad barcode", "4,1": "IMU fault",
    "5,1": "B5 no reliable ultrasonic echo",
    "5,2": "B5 motor stop acknowledgement timeout",
    "5,3": "B5 obstacle scan timeout", "5,4": "B5 no safe obstacle bypass route",
    "5,5": "B5 fine scan failed", "5,6": "B5 obstacle avoidance motion timeout",
    "5,7": "B5 line reacquisition timeout",
    "5,8": "B5 insufficient obstacle clearance",
    "5,9": "B5 reverse manoeuvre timeout" };
  const DIST_INVALID = 0x1FF;
  const WIDTH_INVALID = 0x7F;

  // [name, bits, signed] -- same order as comm_pack.c
  const TELEMETRY = [["ver", 3], ["seq", 16], ["t_ds", 20], ["state", 4],
    ["faults", 8], ["spd_l", 14, true], ["spd_r", 14, true],
    ["enc_l", 24, true], ["enc_r", 24, true], ["ir_l", 12], ["ir_c", 12],
    ["ir_r", 1], ["line", 3], ["barcode", 3], ["nav", 3], ["motion", 3],
    ["pitch_dd", 10, true], ["hump_valid", 1], ["hump_last_mm", 7],
    ["hump_peak_mm", 7], ["obs_d_cm", 9], ["obs_w_cm", 7], ["obs_lc_cm", 9],
    ["obs_rc_cm", 9]];
  const EVENT_HEADER = [["ver", 3], ["type", 2], ["seq", 8], ["t_ds", 20]];
  const EVENT_BODY = [
    [["code", 3], ["nav", 3]],
    [["valid", 1], ["height_mm", 7], ["peak_mm", 7]],
    [["d_cm", 9], ["w_cm", 7], ["lc_cm", 9], ["rc_cm", 9], ["action", 3]],
    [["err", 8], ["detail", 8]]];
  const STATUS = [["ver", 3], ["online", 1], ["link", 3], ["demo", 1],
    ["seq", 16], ["t_ds", 20], ["ip", 32], ["telem_ds", 7], ["faults", 8],
    ["tx", 16], ["drop", 16], ["evdrop", 16], ["rx", 16], ["rej", 16],
    ["reconn", 8]];
  const ACK = [["ver", 3], ["seq", 8], ["cmd", 4], ["ok", 1], ["result", 5],
    ["pad", 3]];
  const COMMAND = [["ver", 3], ["cmd", 4], ["arg", 7], ["seq", 8],
    ["reserved", 2]];

  function unpack(layout, bytes, startBit) {
    const out = {};
    let pos = startBit || 0;
    const total = bytes.length * 8;
    for (const [name, bits, signed] of layout) {
      if (pos + bits > total) {
        throw new Error(`frame too short for field '${name}'`);
      }
      let v = 0;
      for (let i = 0; i < bits; i++, pos++) {
        v = v * 2 + ((bytes[pos >> 3] >> (7 - (pos & 7))) & 1);
      }
      if (signed && v >= 2 ** (bits - 1)) v -= 2 ** bits;
      out[name] = v;
    }
    return [out, pos];
  }

  function pack(layout, values) {
    const bits = [];
    for (const [name, width] of layout) {
      let v = values[name] || 0;
      v = ((v % 2 ** width) + 2 ** width) % 2 ** width; // two's complement
      for (let i = width - 1; i >= 0; i--) {
        bits.push(Math.floor(v / 2 ** i) % 2);
      }
    }
    const out = new Uint8Array(Math.ceil(bits.length / 8));
    bits.forEach((b, i) => { if (b) out[i >> 3] |= 0x80 >> (i & 7); });
    return out;
  }

  const name = (table, i) => (i >= 0 && i < table.length ? table[i] : `?${i}`);
  const cm = (v, invalid) => (v === invalid ? null : v);
  function faultNames(mask) {
    const n = FAULT_BITS.filter(([bit]) => mask & bit).map(([, s]) => s);
    return n.length ? n : ["none"];
  }
  function errorText(code) {
    const sub = (code >> 5) & 7, num = code & 31;
    return `S${sub}#${num} ` + (ERROR_NAMES[`${sub},${num}`] || "unknown");
  }
  function checkVersion(d) {
    if (d.ver !== WIRE_VERSION) {
      throw new Error(`wire version ${d.ver}, expected ${WIRE_VERSION}`);
    }
  }

  function decodeTelemetry(bytes) {
    const [d] = unpack(TELEMETRY, bytes);
    checkVersion(d);
    d.t_s = d.t_ds / 10;
    d.state_name = name(ROBOT_STATES, d.state);
    d.fault_names = faultNames(d.faults);
    d.line_flags = [[1, "DETECTED"], [2, "LOST"], [4, "JUNCTION"]]
      .filter(([b]) => d.line & b).map(([, s]) => s);
    d.barcode_name = name(BARCODES, d.barcode);
    d.nav_name = name(NAV, d.nav);
    d.motion_name = name(MOTION_STATES, d.motion);
    d.pitch_deg = d.pitch_dd / 10;
    d.obs_d_cm = cm(d.obs_d_cm, DIST_INVALID);
    d.obs_w_cm = cm(d.obs_w_cm, WIDTH_INVALID);
    d.obs_lc_cm = cm(d.obs_lc_cm, DIST_INVALID);
    d.obs_rc_cm = cm(d.obs_rc_cm, DIST_INVALID);
    return d;
  }

  function decodeEvent(bytes) {
    const [d, pos] = unpack(EVENT_HEADER, bytes);
    checkVersion(d);
    Object.assign(d, unpack(EVENT_BODY[d.type], bytes, pos)[0]);
    d.t_s = d.t_ds / 10;
    d.type_name = EVENT_TYPES[d.type];
    if (d.type === 0) {
      d.code_name = name(BARCODES, d.code);
      d.nav_name = name(NAV, d.nav);
    } else if (d.type === 2) {
      d.action_name = name(AVOID, d.action);
      d.d_cm = cm(d.d_cm, DIST_INVALID);
      d.w_cm = cm(d.w_cm, WIDTH_INVALID);
      d.lc_cm = cm(d.lc_cm, DIST_INVALID);
      d.rc_cm = cm(d.rc_cm, DIST_INVALID);
    } else if (d.type === 3) {
      d.err_text = errorText(d.err);
    }
    return d;
  }

  function decodeStatus(bytes) {
    if (bytes.length === 1) { // retained online byte / Last Will
      const d = { ver: bytes[0] >> 5, online: (bytes[0] >> 4) & 1 };
      checkVersion(d);
      return d;
    }
    const [d] = unpack(STATUS, bytes);
    checkVersion(d);
    d.link_name = name(LINK_STATES, d.link);
    d.ip_str = [0, 8, 16, 24].map((s) => Math.floor(d.ip / 2 ** s) % 256).join(".");
    d.telem_ms = d.telem_ds * 100;
    d.fault_names = faultNames(d.faults);
    d.t_s = d.t_ds / 10;
    return d;
  }

  function decodeAck(bytes) {
    const [d] = unpack(ACK, bytes);
    checkVersion(d);
    d.cmd_name = COMMAND_NAMES[d.cmd] || `?${d.cmd}`;
    d.result_text = RESULTS[d.result] || `?${d.result}`;
    return d;
  }

  const DECODERS = { telemetry: decodeTelemetry, event: decodeEvent,
    status: decodeStatus, "cmd/ack": decodeAck };

  function decode(kind, bytes) {
    const fn = DECODERS[kind];
    if (!fn) throw new Error(`unknown topic ${kind}`);
    return fn(bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes));
  }

  // "SPEED 40", "MODE LINE", "TELEM 500" (ms) -> 3 bytes (throws on bad input)
  function encodeCommand(text, seq) {
    const words = text.trim().toUpperCase().split(/\s+/);
    const verb = words[0];
    if (!(verb in COMMANDS)) throw new Error(`unknown command: ${text}`);
    const args = words.slice(1);
    let arg = 0;
    const oneInt = (lo, hi) => {
      if (args.length !== 1 || !/^\d+$/.test(args[0])) {
        throw new Error(`${verb} needs one number ${lo}..${hi}`);
      }
      const v = Number(args[0]);
      if (v < lo || v > hi) throw new Error(`${verb} ${v} is outside ${lo}..${hi}`);
      return v;
    };
    if (verb === "SPEED") arg = oneInt(0, 100);
    else if (verb === "MODE") {
      if (args.length !== 1 || !MODES.includes(args[0])) {
        throw new Error("MODE needs AUTO, LINE or TEST");
      }
      arg = MODES.indexOf(args[0]);
    } else if (verb === "TELEM") {
      const ms = oneInt(100, 10000);
      if (ms % 100) throw new Error("TELEM period must be a multiple of 100 ms");
      arg = ms / 100;
    } else if (args.length) throw new Error(`${verb} takes no argument`);
    return pack(COMMAND, { ver: WIRE_VERSION, cmd: COMMANDS[verb], arg,
      seq: (seq || 0) & 0xFF });
  }

  // Encoder totals arrive as the low 24 bits; rebuild the running total.
  class Unwrap24 {
    constructor() { this.total = null; }
    update(low24) {
      low24 = ((low24 % 2 ** 24) + 2 ** 24) % 2 ** 24;
      if (this.total === null) {
        this.total = low24 >= 2 ** 23 ? low24 - 2 ** 24 : low24;
      } else {
        let delta = (((low24 - this.total) % 2 ** 24) + 2 ** 24) % 2 ** 24;
        if (delta >= 2 ** 23) delta -= 2 ** 24;
        this.total += delta;
      }
      return this.total;
    }
  }

  return { WIRE_VERSION, ROBOT_STATES, LINK_STATES, MOTION_STATES, NAV, AVOID,
    BARCODES, MODES, COMMANDS, FAULT_BITS, TELEMETRY, EVENT_HEADER, EVENT_BODY,
    STATUS, ACK, COMMAND, unpack, pack, decode, encodeCommand, faultNames,
    errorText, Unwrap24 };
});
