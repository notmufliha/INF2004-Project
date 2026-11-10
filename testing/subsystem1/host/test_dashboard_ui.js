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
const context = vm.createContext({ window: { S1Wire: W }, document,
  localStorage: { getItem: () => null, setItem() {} },
  matchMedia: () => ({ matches: false }), addEventListener() {},
  setInterval() {}, clearInterval() {}, getComputedStyle: () => ({ getPropertyValue: () => "black" }) });
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
console.log("PASS dashboard handlers: measured/assumed/invalid echo, profile availability, chart gaps, faults/state and diagnostics");
