// S1-H06: the dashboard's JavaScript decoder (tools/dashboard/s1_wire.js)
// must read every frame the C packer produces exactly as the C side
// printed it.  Input: the output of "./test_comm_pack vectors".
// Usage: node test_dashboard_decoder.js vectors.txt
"use strict";
const fs = require("fs");
const path = require("path");
const W = require(path.join(__dirname, "../../../tools/dashboard/s1_wire.js"));

const LAYOUT = { telemetry: () => W.TELEMETRY, status: () => W.STATUS,
  "cmd/ack": () => W.ACK };
let frames = 0;
let failures = 0;

for (const line of fs.readFileSync(process.argv[2], "utf8").split(/\r?\n/)) {
  if (!line.trim()) continue;
  const [kind, hex, ...pairs] = line.trim().split(/\s+/);
  const bytes = Uint8Array.from(hex.match(/../g).map((h) => parseInt(h, 16)));
  let raw;
  if (kind === "event") {
    const [head, pos] = W.unpack(W.EVENT_HEADER, bytes);
    raw = Object.assign(head, W.unpack(W.EVENT_BODY[head.type], bytes, pos)[0]);
  } else {
    raw = W.unpack(LAYOUT[kind](), bytes)[0];
  }
  W.decode(kind, bytes); // the readable decode must not throw either
  for (const pair of pairs) {
    const [key, value] = pair.split("=");
    if (!(key in raw)) continue; // printed-only helper fields
    if (raw[key] !== Number(value)) {
      failures++;
      console.log(`FAIL ${kind} ${key}: js ${raw[key]} c ${value}`);
    }
  }
  frames++;
}

// Commands: the dashboard's encoder vs known frames from MQTT.md.
const sent = W.encodeCommand("SPEED 40", 5);
const back = W.unpack(W.COMMAND, sent)[0];
if (sent.length !== 3 || back.cmd !== 7 || back.arg !== 40 || back.seq !== 5) {
  failures++;
  console.log("FAIL command encode SPEED 40");
}
const map = W.unpack(W.COMMAND, W.encodeCommand("MAP Z RIGHT", 6))[0];
if (map.cmd !== 12 || map.arg !== 103 || map.seq !== 6) {
  failures++;
  console.log("FAIL command encode MAP Z RIGHT");
}
for (const bad of ["SPEED 101", "MODE FAST", "STOP 1", "TELEM 150", "JUMP",
  "MAP AA LEFT", "MAP A BACKWARD", "MAP"]) {
  let threw = false;
  try { W.encodeCommand(bad, 0); } catch (e) { threw = true; }
  if (!threw) { failures++; console.log(`FAIL accepted bad command ${bad}`); }
}

console.log(`S1-H06 dashboard decoder: ${frames} frames, ${failures} failures`);
process.exit(failures === 0 && frames > 0 ? 0 : 1);
