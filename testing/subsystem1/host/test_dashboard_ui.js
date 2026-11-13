// Exercise the actual dashboard handlers with a minimal DOM, without a broker.
"use strict";
const assert = require("assert");
const fs = require("fs");
const path = require("path");
const vm = require("vm");
const W = require("../../../tools/dashboard/s1_wire.js");
class Element {
  constructor() { this.style = {}; this.dataset = {}; this.children = []; this.value = ""; }
  append(...items) { this.children.push(...items); }
  prepend(item) { this.children.unshift(item); }
  get childElementCount() { return this.children.length; }
  get lastElementChild() { return this.children.at(-1) || (this.child ||= new Element()); }
  querySelector() { return this.child ||= new Element(); }
}
const elements = new Map();
const element = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
const document = { getElementById: element, documentElement: new Element(),
  createElement: () => new Element(), createTextNode: text => ({ textContent: text }),
  querySelectorAll: () => [] };
const intervals = [];
let clock = 1000000;
class FakeDate extends Date {
  constructor(...a) { super(...(a.length ? a : [clock])); }
  static now() { return clock; }
}
const context = vm.createContext({ window: { S1Wire: W }, document,
  localStorage: { getItem: () => null, setItem() {} },
  matchMedia: () => ({ matches: false }), addEventListener() {},
  setInterval(fn) { intervals.push(fn); }, clearInterval() {},
  Date: FakeDate, getComputedStyle: () => ({ getPropertyValue: () => "black" }) });
const html = fs.readFileSync(path.join(__dirname, "../../../tools/dashboard/index.html"), "utf8");
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
function telemetry(distance, seq) {
  const values = { ver: 1, seq, state: 8, line: 2, obs_d_cm: distance,
    obs_w_cm: 127, obs_lc_cm: 511, obs_rc_cm: 511 };
  context.frame = W.decode("telemetry", W.pack(W.TELEMETRY, values));
  vm.runInContext("onTelemetry(frame)", context);
}
telemetry(50, 1);
assert.equal(element("o-d").textContent, "50 cm");
assert.equal(element("o-echo").textContent, "Measured return");
telemetry(510, 2);
assert.equal(element("o-d").textContent, "No target (assumed open)");
assert.equal(vm.runInContext("state.dist.at(-1).v", context), null);
assert.equal(element("o-w").textContent, "not measured");
telemetry(511, 3);
assert.equal(element("o-echo").textContent, "Unavailable / not measured");
telemetry(25, 4);
assert.equal(element("t-gap").textContent, 0);
assert.equal(element("t-state").textContent, "SAFE_STOP");
const paths = [...element("dist-chart").innerHTML.matchAll(/<path[^>]* d="([^"]+)"/g)];
assert(paths.some(p => (p[1].match(/M/g) || []).length === 2)); // no line across missing/assumed data
assert(!element("dist-chart").innerHTML.includes("NaN"));
context.event = { type: 3, seq: 1, type_name: "error", err: 161, detail: 3, err_text: "B5 no reliable ultrasonic echo" };
vm.runInContext("onEvent(event)", context);
assert(element("events").children[0].children[1].textContent.includes("no falling edge"));
assert.equal(W.decode("telemetry", W.pack(W.TELEMETRY, { ver: 1, obs_d_cm: 400 })).obs_open_space, false);

// Labels follow each sensor's new profile; flags still come from firmware.
for (const [left, centre, expectedLeft, expectedCentre] of [
  [185, 131, "WHITE band", "WHITE band"],
  [245, 777, "WHITE band", "BLACK band"],
  [346, 212, "BLACK band", "WHITE band"],
  [296, 495, "transition band", "transition band"],
  [0, 0, "invalid / zero", "invalid / zero"]
]) {
  context.frame = W.decode("telemetry", W.pack(W.TELEMETRY, {
    ver: 1, ir_l: left, ir_c: centre, line: 2
  }));
  vm.runInContext("updateDiagnostics(frame)", context);
  assert.equal(element("diag-left-class").textContent, expectedLeft);
  assert.equal(element("diag-centre-class").textContent, expectedCentre);
  assert.equal(element("diag-flags").textContent, "LOST");
}

// Every command button: publish one 3-byte frame on <prefix>/cmd, and the
// car's ack (or a missing ack) shows up in the command log.
const lastLog = () => element("acks").children[0].children[1].textContent;
const published = [];
element("prefix").value = "inf2004/g25/car";
context.fake = { connected: true, publish: (t, f, o) => published.push({ t, f, o }) };
vm.runInContext("state.client = fake", context);
const buttons = ["START", "PAUSE", "RESUME", "STOP", "ESTOP", "SPEED 40",
  "MODE LINE", "RESET_PEAK", "PING", "STATUS"];
buttons.forEach((text, i) => {
  context.text = text;
  vm.runInContext("send(text)", context);
  const sent = published.at(-1);
  assert.equal(sent.t, "inf2004/g25/car/cmd");
  assert.equal(sent.f.length, 3);
  assert.equal(sent.o.qos, 1);
  const [cmd] = W.unpack(W.COMMAND, sent.f);
  assert.equal(cmd.cmd, W.COMMANDS[text.split(" ")[0]], text);
  assert.equal(cmd.seq, i);
  context.ack = W.decode("cmd/ack", W.pack(W.ACK, { ver: 1, seq: i, cmd: cmd.cmd, ok: 1,
    result: text === "PING" ? 1 : text === "STATUS" ? 2 : 0 }));
  clock += 40;
  vm.runInContext("onAck(ack)", context);
  assert(lastLog().includes(`${text.split(" ")[0]} OK`), lastLog());
});
assert.equal(element("mode-now").textContent, "mode: LINE");
// No ack: reported after the timeout instead of silence.
vm.runInContext("send('PAUSE')", context);
clock += 3500; intervals.forEach(fn => fn());
assert(lastLog().includes("PAUSE: NO REPLY"), lastLog());
// Car known offline: warned at send time.
context.st = { online: 0 };
vm.runInContext("onStatus(st)", context);
vm.runInContext("send('STOP')", context);
assert(lastLog().includes("OFFLINE"), lastLog());
vm.runInContext("state.client = null", context);
vm.runInContext("send('STOP')", context);
assert(lastLog().includes("not connected"), lastLog());
console.log("PASS dashboard commands: every button publishes the right 3-byte frame, acks, no-reply and offline reported");
console.log("PASS dashboard handlers: measured/assumed/invalid echo, profile availability, chart gaps, faults/state and diagnostics");
