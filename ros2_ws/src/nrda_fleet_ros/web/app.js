/* NRDA Fleet Console: live view of the decentralised fleet, served by fleet_gateway.
   No build step and no libraries. Data: /api/site once, then /api/stream (Server-Sent Events,
   4 Hz snapshots plus safety events) with /api/snapshot polling as a fallback. */
"use strict";

const SVGNS = "http://www.w3.org/2000/svg";
const $ = (id) => document.getElementById(id);

const MODE = {  // colour, words for people
  idle: ["#8D959C", "idle"], parked: ["#7D8FA3", "parked"], parking: ["#7D98B8", "parking"],
  parked_charger: ["#2E9A55", "at a charger"], charging: ["#2E9A55", "charging"],
  to_pickup: ["#2F74D0", "going to pick up"], loading: ["#8A55D6", "loading"],
  to_drop: ["#E58E2A", "delivering"], unloading: ["#D9579C", "unloading"],
  to_charger: ["#2FB8A8", "going to charge"], to_park: ["#6FA8DC", "going to park"],
  paused: ["#E0B000", "paused"], estop: ["#D3212F", "emergency stop"], fault: ["#8E1B16", "fault"],
};
const modeColour = (m) => (MODE[m] || ["#8D959C"])[0];
const modeText = (m) => (MODE[m] || [0, m || "unknown"])[1];
const BUSY = new Set(["to_pickup", "loading", "to_drop", "unloading"]);
const STATION_COLOUR = { charger: "#2E9A55", parking: "#8D959C", picker: "#8A55D6", drop: "#E58E2A",
  pack: "#D3212F", decant: "#2F74D0", overflow: "#B08A00" };
const PRIO = { 1: ["Low", "#8D959C"], 2: ["Normal", "#2F74D0"], 3: ["High", "#E58E2A"], 4: ["Critical", "#D3212F"] };
const WALLS = new Set(["wall", "column", "mezzanine_column", "guardrail", "partition", "maint_fence", "ops_glass"]);

const state = {
  site: null, snap: null, selected: null, placing: false, showPaths: true, showLabels: true,
  lvl: "", inc: "", lastCollisions: null, seenEvents: new Set(), view: null, sigs: {}, lastMsg: 0, poller: null,
  hb: {}, hbSort: "id", tick: 0,
};

/* ------------------------------------------------------------------ heartbeat
   `age` in the snapshot is how long ago that robot last reported, measured by the
   gateway against the robot's own stamp. Read it, never re-derive it: a robot that
   is simply standing still (deadlocked, parked, queued at a station) reports just
   as often as a moving one, and judging the beat by whether its numbers changed
   marked half a stationary fleet silent. HB_LEN beats of history make the strip. */
const HB_LEN = 40, HB_LATE = 1.5, HB_SILENT = 3.0;
function trackHeartbeat(snap) {
  state.tick++;
  for (const r of snap.robots) {
    let b = state.hb[r.id];
    if (!b) b = state.hb[r.id] = { beats: [], ms: [] };
    b.age = Math.max(r.age ?? 0, 0);
    b.stale = r.stale;
    b.beats.push(b.age < HB_LATE ? 1 : 0);
    if (b.beats.length > HB_LEN) b.beats.shift();
    b.ms.push(r.compute_ms || 0);
    if (b.ms.length > HB_LEN) b.ms.shift();
  }
  for (const id of Object.keys(state.hb)) {      // forget robots that left the fleet
    if (!snap.robots.some((r) => r.id === id)) delete state.hb[id];
  }
}
const hbLevel = (b) => (!b ? "silent" : b.stale || b.age >= HB_SILENT ? "silent" : b.age >= HB_LATE ? "late" : "live");

/* ------------------------------------------------------------------ helpers */
function svg(tag, attrs = {}, parent) {
  const n = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v);
  if (parent) parent.appendChild(n);
  return n;
}
function h(tag, attrs = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v === undefined || v === null || v === false) continue;
    if (k === "class") n.className = v; else if (k === "style") n.style.cssText = v; else n.setAttribute(k, v === true ? "" : v);
  }
  for (const c of kids) if (c !== undefined && c !== null && c !== false) n.append(c);
  return n;
}
const num = (v, d = 0) => (v === null || v === undefined || Number.isNaN(+v)) ? "–" : Number(v).toFixed(d);
const pct = (v) => (v === null || v === undefined) ? "–" : `${Math.round(v * 100)}%`;
function fmtTime(t) {
  t = Math.max(0, Math.floor(t || 0)); const m = Math.floor(t / 60), s = t % 60;
  return m >= 60 ? `${Math.floor(m / 60)}:${String(m % 60).padStart(2, "0")}:${String(s).padStart(2, "0")}`
    : `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}
const niceStation = (s) => (s || "").replace("charger_", "charger ").replace("packing_induct_", "pack ")
  .replace("overflow_induct_", "overflow ").replace("drop_", "drop ").replace("decant_pickup", "decant").replace(/_/g, " ");
function store(k, v) { try { if (v === undefined) return localStorage.getItem(k); localStorage.setItem(k, v); } catch (e) { return null; } return null; }
async function api(path, body) {
  const opt = body === undefined ? { cache: "no-store" }
    : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const r = await fetch(path, opt);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `The gateway answered ${r.status}.`);
  return j;
}
function toast(text, alarm = false) {
  const t = $("toast"); t.textContent = text; t.hidden = false; t.classList.toggle("alarm", alarm);
  clearTimeout(toast._t); toast._t = setTimeout(() => (t.hidden = true), alarm ? 9000 : 5000);
}
/* re-render a region only when its data changed: keeps open dropdowns and hovered buttons alive */
function changed(key, value) {
  const sig = JSON.stringify(value);
  if (state.sigs[key] === sig) return false;
  state.sigs[key] = sig; return true;
}

/* ------------------------------------------------------------------ robot health */
function problems(r) {
  const out = [];  // [text, severity] with severity 2 = act now, 1 = watch
  if (r.stale) out.push(["not reporting", 2]);
  if (r.mode === "fault" || (r.health && r.health !== "HEALTHY" && r.health !== "DEGRADED")) out.push(["motor fault (test): clear it in the details", 2]);
  if (r.mode === "estop") out.push(["emergency stop", 2]);
  if ((r.safety && r.safety.COLLISION) > 0) out.push([`${r.safety.COLLISION} collision${r.safety.COLLISION > 1 ? "s" : ""}`, 2]);
  if (r.battery < 0.2) out.push([`battery ${pct(r.battery)}`, 2]);
  else if (r.battery < 0.35) out.push([`battery ${pct(r.battery)}`, 1]);
  if (r.health === "DEGRADED") out.push(["weak radio link", 1]);
  if (r.mode === "paused") out.push(["paused", 1]);
  if (r.waiting_for && r.waiting_for.length) out.push([`waiting for ${r.waiting_for[0].replace("AMR_", "AMR ")}`, 0]);
  if (r.bypass) out.push(["driving around an obstacle", 0]);
  return out;
}
function blockedReason(r) {
  if (r.stale) return "not reporting";
  if (r.mode === "fault" || (r.health !== "HEALTHY" && r.health !== "DEGRADED")) return "fault";
  if (r.mode === "estop" || r.mode === "paused") return modeText(r.mode);
  if ((r.safety.COLLISION || 0) > 0) return "had a collision";
  if (r.battery < 0.35) return "low battery";
  return "";
}

/* ------------------------------------------------------------------ map */
const map = $("map");
const layers = {};
function buildMap(site) {
  map.innerHTML = "";
  const root = svg("g", { transform: "scale(1,-1)" }, map);  // warehouse y points north, SVG y points down
  for (const n of ["ground", "zones", "lanes", "static", "stations", "paths", "obstacles", "risk", "events", "robots", "labels"]) {
    layers[n] = svg("g", {}, root);
  }
  const [x0, y0, x1, y1] = site.bounds;
  svg("rect", { x: x0, y: y0, width: x1 - x0, height: y1 - y0, class: "m-ground" }, layers.ground);
  svg("rect", { x: -40, y: -25, width: 80, height: 50, class: "m-building" }, layers.ground);
  for (const [zid, b] of Object.entries(site.zones || {})) {
    if (b.x_min === undefined) continue;
    svg("rect", { x: b.x_min, y: b.y_min, width: b.x_max - b.x_min, height: b.y_max - b.y_min, class: "m-zone", "data-zone": zid }, layers.zones);
  }
  svg("path", { d: site.lanes.map(([ax, ay, bx, by]) => `M${ax} ${ay}L${bx} ${by}`).join(""), class: "m-lane" }, layers.lanes);
  for (const b of site.boxes) {
    const cls = b.kind === "rack" ? "m-rack" : WALLS.has(b.kind) ? "m-wall" : b.kind === "person" ? "m-person"
      : b.kind === "charger" ? "m-charger" : "m-thing";
    svg("polygon", { points: b.c.map((p) => p.join(",")).join(" "), class: cls }, layers.static);
  }
  for (const b of site.floor_boxes || []) {  // test boxes: in the world, not in the robots' map
    const g = svg("g", { transform: `translate(${b.x} ${b.y}) rotate(${(b.yaw * 180) / Math.PI})` }, layers.static);
    svg("rect", { x: -b.length / 2, y: -b.width / 2, width: b.length, height: b.width, class: "m-box" }, g);
    svg("path", { d: `M${-b.length / 2} ${-b.width / 2}L${b.length / 2} ${b.width / 2}M${-b.length / 2} ${b.width / 2}L${b.length / 2} ${-b.width / 2}`, class: "m-box-x" }, g);
    svg("title", {}, g).textContent = `${b.name}: a box left on the floor. It is not on the robots' map; they find it with their lidar.`;
  }
  for (const s of site.stations) {
    const g = svg("g", { transform: `translate(${s.x} ${s.y}) rotate(${(s.yaw * 180) / Math.PI})` }, layers.stations);
    svg("rect", { x: -0.62, y: -0.41, width: 1.24, height: 0.82, class: "m-station", stroke: STATION_COLOUR[s.kind] || "#888" }, g);
    svg("title", {}, g).textContent = niceStation(s.name);
    const t = svg("text", { x: s.x, y: -(s.y + 0.6), class: "m-station-label", "text-anchor": "middle", transform: "scale(1,-1)" }, layers.stations);
    t.textContent = niceStation(s.name);
  }
  fitView();
  const lg = $("legend"); lg.innerHTML = "";
  for (const m of ["to_pickup", "loading", "to_drop", "unloading", "to_park", "parked", "idle", "paused", "fault"]) {
    const i = h("i"); i.style.background = modeColour(m); lg.append(h("span", {}, i, modeText(m)));
  }
  const box = h("i"); box.style.background = "#B98A4E"; lg.append(h("span", {}, box, "box on the floor"));
}
function setView(v) { state.view = v; map.setAttribute("viewBox", `${v.x} ${v.y} ${v.w} ${v.h}`); }
function fitView() { const [x0, y0, x1, y1] = state.site.bounds; setView({ x: x0, y: -y1, w: x1 - x0, h: y1 - y0 }); }
function toWorld(evt) {
  const pt = map.createSVGPoint(); pt.x = evt.clientX; pt.y = evt.clientY;
  const p = pt.matrixTransform(map.getScreenCTM().inverse()); return { x: p.x, y: -p.y };
}
(function panZoom() {
  let drag = null, moved = false;
  map.addEventListener("wheel", (e) => {
    e.preventDefault();
    const v = state.view; if (!v) return;
    const k = e.deltaY > 0 ? 1.15 : 1 / 1.15;
    const pt = map.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
    const p = pt.matrixTransform(map.getScreenCTM().inverse());
    const w = Math.min(Math.max(v.w * k, 6), 120), hh = v.h * (w / v.w);
    setView({ x: p.x - (p.x - v.x) * (w / v.w), y: p.y - (p.y - v.y) * (hh / v.h), w, h: hh });
  }, { passive: false });
  map.addEventListener("pointerdown", (e) => {
    if (state.placing || !state.view) return;
    drag = { x: e.clientX, y: e.clientY, v: { ...state.view } }; moved = false;
  });
  map.addEventListener("pointermove", (e) => {
    if (!drag) return;
    if (Math.abs(e.clientX - drag.x) + Math.abs(e.clientY - drag.y) > 4) moved = true;
    if (!moved) return;
    const s = drag.v.w / map.clientWidth;
    setView({ ...drag.v, x: drag.v.x - (e.clientX - drag.x) * s, y: drag.v.y - (e.clientY - drag.y) * s });
  });
  window.addEventListener("pointerup", () => (drag = null));
  map.addEventListener("click", async (e) => {
    if (moved) return;
    if (state.placing) {
      const p = toWorld(e); setPlacing(false);
      try { const r = await api("/api/inject/obstacle", { x: +p.x.toFixed(2), y: +p.y.toFixed(2) });
        toast(`Pallet ${r.name} dropped at ${p.x.toFixed(1)}, ${p.y.toFixed(1)}. Robots will find it with their lidar.`); }
      catch (err) { toast(`Pallet not dropped: ${err.message}`); }
      return;
    }
    const robot = e.target.closest(".m-robot");
    if (robot) return select(robot.dataset.id);
    const obs = e.target.closest(".m-obstacle");
    if (obs) {
      if (obs.dataset.source === "lidar") {
        // not the operator's to remove: the robots found it themselves and drop it again when one of
        // them gets a clear view of the empty space
        toast(`${obs.dataset.name} was found by the robots' lidars, not placed. They clear it themselves once one of them can see the space is empty.`);
        return;
      }
      try { await api(`/api/obstacles/${obs.dataset.name}/remove`, {}); toast(`Pallet ${obs.dataset.name} removed.`); }
      catch (err) { toast(`Pallet not removed: ${err.message}`); }
    }
  });
})();
function setPlacing(on) {
  state.placing = on;
  $("btn-pallet").setAttribute("aria-pressed", on);
  map.classList.toggle("placing", on);
  $("map-hint").textContent = on ? "Click an aisle to drop a pallet there. Press Esc to cancel."
    : "Scroll to zoom, drag to pan, click a robot for details.";
}

function drawRobots(snap) {
  const R = layers.robots, P = layers.paths, T = layers.labels;
  const byId = Object.fromEntries(snap.robots.map((r) => [r.id, r]));
  for (const r of snap.robots) {
    let g = R.querySelector(`[data-id="${r.id}"]`);
    if (!g) {
      g = svg("g", { class: "m-robot", "data-id": r.id }, R);
      svg("circle", { r: 1.0, class: "ring" }, g);
      svg("rect", { x: -0.6, y: -0.39, width: 1.2, height: 0.78, rx: 0.1, class: "body" }, g);
      svg("polygon", { points: "0.56,0 0.28,0.2 0.28,-0.2", class: "nose" }, g);
      svg("title", {}, g);
      svg("text", { "text-anchor": "middle", class: "m-lbl", "data-for": r.id }, T);
      svg("polyline", { class: "m-path", "data-for": r.id }, P);
      svg("line", { class: "m-wait", "data-for": r.id }, P);
    }
    g.setAttribute("transform", `translate(${r.x} ${r.y}) rotate(${(r.th * 180) / Math.PI})`);
    g.querySelector(".body").setAttribute("fill", r.stale ? "#5B6570" : modeColour(r.mode));
    g.classList.toggle("selected", state.selected === r.id);
    g.classList.toggle("flag", problems(r).some((p) => p[1] === 2));
    g.querySelector("title").textContent = `${r.id}: ${modeText(r.mode)}, battery ${pct(r.battery)}`;
    const t = T.querySelector(`text[data-for="${r.id}"]`);
    t.setAttribute("x", r.x); t.setAttribute("y", -(r.y + 0.8)); t.setAttribute("transform", "scale(1,-1)");
    t.textContent = r.id.replace("AMR_", "");
    t.style.display = state.showLabels ? "" : "none";
    const pl = P.querySelector(`polyline[data-for="${r.id}"]`);
    const path = r.path || [];
    pl.setAttribute("points", [[r.x, r.y], ...path].map((p) => p.join(",")).join(" "));
    pl.setAttribute("stroke", modeColour(r.mode));
    pl.style.display = state.showPaths && path.length ? "" : "none";
    const w = P.querySelector(`line[data-for="${r.id}"]`);
    const b = (r.waiting_for || []).length ? byId[r.waiting_for[0]] : null;
    if (b) { w.setAttribute("x1", r.x); w.setAttribute("y1", r.y); w.setAttribute("x2", b.x); w.setAttribute("y2", b.y); w.style.display = ""; }
    else w.style.display = "none";
  }
  if (changed("obstacles", snap.obstacles)) {
    const O = layers.obstacles; O.innerHTML = "";
    for (const o of snap.obstacles) {
      const lidar = o.source === "lidar";
      const cls = lidar ? "m-obstacle found" : "m-obstacle";
      const r = svg("rect", { x: o.x - o.length / 2, y: o.y - o.width / 2, width: o.length, height: o.width, class: cls, "data-name": o.name, "data-source": o.source || "injected" }, O);
      svg("title", {}, r).textContent = lidar
        ? `${o.name}: something the robots found with their lidars, first seen by ${o.found_by} at t=${o.t}s. They are routing around it. It clears itself when a robot with a clear view sees the space empty.`
        : `Pallet ${o.name}. Click to remove it.`;
    }
  }
  if (changed("zones", snap.zone_rules)) {
    for (const z of layers.zones.querySelectorAll(".m-zone")) {
      const rule = snap.zone_rules[z.dataset.zone];
      z.classList.toggle("avoid", rule === "avoid"); z.classList.toggle("slow", rule === "slow");
    }
  }
  const empty = $("map-empty");
  empty.hidden = snap.robots.length > 0;
}
function drawEvents(events) {
  const E = layers.events; if (!E) return;
  for (const ev of events) {
    if (!ev || state.seenEvents.has(ev.event_id)) continue;
    state.seenEvents.add(ev.event_id);
    if (ev.x === null || ev.y === null) continue;
    const c = svg("circle", { cx: ev.x, cy: ev.y, r: ev.level === "COLLISION" ? 1.2 : 0.8, class: `m-event ${ev.level} fresh` }, E);
    svg("title", {}, c).textContent = `${ev.level.replace("_", " ").toLowerCase()}: ${(ev.robots || []).join(", ")} ${ev.other || ""}`;
    setTimeout(() => c.classList.remove("fresh"), 4500);
    setTimeout(() => c.remove(), ev.level === "COLLISION" ? 180000 : 30000);
  }
}

/* ------------------------------------------------------------------ top bar */
function renderTop(snap) {
  const k = snap.kpis;
  $("sim-time").textContent = fmtTime(snap.t);
  $("v-done").textContent = k.tasks_completed;
  $("v-busy").textContent = snap.robots.filter((r) => BUSY.has(r.mode)).length;
  $("robot-count").textContent = snap.robots.length || state.site.robots.length;
  const estop = snap.robots.length > 0 && snap.robots.every((r) => r.mode === "estop");
  const b = $("btn-estop"); b.setAttribute("aria-pressed", estop);
  b.textContent = estop ? "Release emergency stop" : "Emergency stop";
  const open = snap.tasks.filter((t) => !["completed", "cancelled"].includes(t.status)).length;
  $("t-open").textContent = open ? `(${open})` : "";
  const ts = $("t-safety"); ts.textContent = k.collisions ? `(${k.collisions})` : ""; ts.classList.toggle("hot", k.collisions > 0);
  const quiet = snap.robots.filter((r) => hbLevel(state.hb[r.id]) !== "live").length;
  const tf = $("t-fleet"); tf.textContent = quiet ? `(${quiet})` : ""; tf.classList.toggle("hot", quiet > 0);
}
const KIND = { deadlock: "Deadlock", deadlock_risk: "Deadlock risk", collision_risk: "Collision risk",
  collision: "Collision", near_miss: "Near miss" };
const robotsText = (rs) => (rs || []).map((r) => r.replace("AMR_", "AMR ")).join(", ");
/* The band says, in one sentence, the worst thing happening on the floor right now,
   and draws its probability as hazard tape. The ledger beside it is history: small,
   tabular, and deliberately quiet so it never competes with the live sentence. */
function renderBand(snap) {
  const k = snap.kpis, risk = snap.risk || [], incs = snap.incidents || [];
  const open = incs.filter((i) => i.status !== "solved");
  const escalated = open.filter((i) => i.status === "needs_you");
  const top = risk[0];
  const band = $("band");
  let cls, text, why, p;

  if (escalated.length) {
    const i = escalated[0];
    cls = "act"; p = 1;
    text = `${escalated.length === 1 ? "A deadlock needs you" : `${escalated.length} deadlocks need you`}: ${robotsText(i.robots)}`;
    why = `The robots tried 3 times and could not clear it. Pause ${i.priority ? i.priority.replace("_", " ") + "'s blocker" : "one of them"}, or release tasks elsewhere.`;
  } else if (top) {
    p = top.p;
    cls = top.p >= 0.7 ? "act" : "watch";
    const verb = top.kind === "deadlock" ? "Deadlock on" : top.kind === "deadlock_risk" ? "Deadlock building on" : "Closing course:";
    text = `${verb} ${robotsText(top.robots)}`;
    why = top.why || "";
  } else if (!snap.robots.length) {
    cls = ""; p = 0; text = "Waiting for the fleet";
    why = "The console shows the highest risk here as soon as the robots report in.";
  } else {
    cls = "clear"; p = 0; text = "Floor is clear";
    why = "No robot has waited long on another, and no pair is on a closing course.";
  }

  if (changed("band", [cls, text, why, Math.round(p * 100)])) {
    band.className = `band ${cls}`;
    $("band-text").textContent = text;
    $("band-why").textContent = why;
    $("band-p").textContent = p > 0 ? `${Math.round(p * 100)}%` : "clear";
    $("band-tape").style.width = `${Math.round(p * 100)}%`;
    $("band-say").title = top || escalated.length ? "Show it on the map" : "Nothing to show on the map";
  }

  // ledger: the shift's history, in units
  const solved = k.deadlocks_resolved ?? 0, plans = k.deadlocks ?? 0;
  const live = snap.robots.filter((r) => hbLevel(state.hb[r.id]) === "live").length;
  const set = (id, v, cls2 = "") => { const e = $(id); e.textContent = v; e.className = cls2; };
  set("led-open", snap.tasks.filter((t) => !["completed", "cancelled"].includes(t.status)).length);
  set("led-coll", k.collisions, k.collisions ? "hot" : "");
  set("led-near", k.near_misses, k.near_misses ? "warn" : "");
  set("led-dl", `${solved}/${plans}`, plans && solved < plans ? "warn" : "");
  set("led-hb", `${live}/${snap.robots.length}`, live < snap.robots.length ? "hot" : "");

  // totals, on the analysis tab
  $("c-rr").textContent = k.collisions_robot_robot; $("c-rs").textContent = k.collisions_robot_static;
  $("c-ro").textContent = k.collisions_obstacle; $("c-near").textContent = k.near_misses; $("c-avoid").textContent = k.avoided;
  $("c-dl").textContent = `${solved} of ${plans}`; $("c-dle").textContent = k.deadlocks_escalated ?? 0;
  $("c-dlt").textContent = k.deadlocks_timed_out ?? 0;
  if (state.lastEsc !== undefined && (k.deadlocks_escalated ?? 0) > state.lastEsc) {
    const ev = [...snap.log].reverse().find((l) => l[1].startsWith("DEADLOCK ESCALATED"));
    toast(`${ev ? ev[1] : "A deadlock needs you"}. Pause the robot named as priority's blocker, or release tasks elsewhere.`, true);
  }
  state.lastEsc = k.deadlocks_escalated ?? 0;
  if (state.lastCollisions !== null && k.collisions > state.lastCollisions) {
    const ev = [...snap.safety_recent].reverse().find((e) => e.level === "COLLISION");
    toast(ev ? `Collision: ${ev.robots.join(" and ")}${ev.other && !ev.robots.includes(ev.other) ? " hit " + ev.other : ""} at ${num(ev.x, 1)}, ${num(ev.y, 1)}. The robots back away and resume by themselves.`
      : "Collision detected. See Deadlocks and collisions.", true);
  }
  state.lastCollisions = k.collisions;
}
function drawRisk(snap) {
  const risk = snap.risk || [];
  if (!changed("riskmap", risk.map((r) => [r.kind, r.robots, Math.round(r.p * 20), Math.round(r.x), Math.round(r.y)]))) return;
  const G = layers.risk; G.innerHTML = "";
  const byId = Object.fromEntries(snap.robots.map((r) => [r.id, r]));
  for (const r of risk) {
    const ms = r.robots.map((id) => byId[id]).filter(Boolean); if (!ms.length) continue;
    const rad = Math.max(...ms.map((m) => Math.hypot(m.x - r.x, m.y - r.y))) + 1.3;
    const c = svg("circle", { cx: r.x, cy: r.y, r: rad, class: `m-risk ${r.kind}` }, G);
    c.style.opacity = 0.35 + 0.65 * r.p;
    svg("title", {}, c).textContent = `${KIND[r.kind]} ${Math.round(r.p * 100)}%: ${robotsText(r.robots)}`;
  }
}
function tick(status) {
  if (status === "solved") {
    const s = document.createElementNS(SVGNS, "svg"); s.setAttribute("viewBox", "0 0 24 24"); s.setAttribute("class", "tick solved"); s.setAttribute("aria-label", "solved");
    svg("circle", { cx: 12, cy: 12, r: 11 }, s); svg("path", { d: "M6.5 12.5l3.6 3.6 7.4-8" }, s); return s;
  }
  return h("span", { class: `tick ${status}`, "aria-label": status === "needs_you" ? "needs you" : "not solved yet" }, status === "needs_you" ? "!" : "");
}
function zoomTo(x, y, w = 18) { if (!state.view) return; const hh = w * (state.view.h / state.view.w); setView({ x: x - w / 2, y: -y - hh / 2, w, h: hh }); }
function incidentEnd(i) {
  const took = i.t_solved !== null && i.t_solved !== undefined ? Math.max(0, i.t_solved - i.t) : null;
  return i.status === "solved" ? (took > 0.5 ? `Solved in ${num(took, 0)} s: ${i.how}` : `Solved: ${i.how}`)
    : i.status === "needs_you" ? `Needs you: the robots could not clear it in 3 tries. Pause ${i.priority ? i.priority.replace("_", " ") + "'s blocker" : "one of them"} or release tasks elsewhere.`
      : i.kind === "deadlock" ? `Being solved${i.rung ? `, try ${i.rung} of 3` : ""}${i.priority ? `: ${i.priority.replace("_", " ")} goes first` : ""}`
        : "Robots paused, backing away";
}
function incidentItem(i) {
  const name = i.kind === "deadlock" ? `Deadlock${i.type ? " " + i.type : ""}` : KIND[i.kind];
  return h("li", { class: `inc ${i.status} ${i.kind}`, "data-x": i.x, "data-y": i.y, tabindex: 0, title: "Show on the map" }, tick(i.status),
    h("span", { class: "inc-body" }, h("b", {}, name), ` ${robotsText(i.robots)}${i.other ? " and " + i.other : ""}`,
      h("span", { class: "why" }, incidentEnd(i))),
    h("time", {}, fmtTime(i.t)));
}

/* "Needs you": the only things that want a decision now. Always on screen, never
   behind a tab, and it holds nothing cumulative. */
function renderNeeds(snap) {
  const risk = snap.risk || [];
  const open = (snap.incidents || []).filter((i) => i.status !== "solved").slice().reverse();
  if (!changed("needs", [risk.map((r) => [r.kind, r.robots, Math.round(r.p * 100), r.why]),
    open.map((i) => [i.id, i.status, i.rung])])) return;

  const n = risk.length + open.length;
  const c = $("needs-count");
  c.textContent = !n ? "nothing open" : `${n} open`;
  c.classList.toggle("hot", open.some((i) => i.status === "needs_you"));

  const ol = $("risk-list"); ol.innerHTML = "";
  if (!risk.length && !open.length) {
    ol.append(h("li", { class: "calm" }, tick("solved"),
      h("span", {}, "Nothing to decide. No robot has waited long on another, and no pair is on a closing course.")));
  }
  for (const r of risk) {
    const p = Math.round(r.p * 100);
    const bar = h("span", { class: "meter" }); const fill = h("i"); fill.style.width = `${p}%`; bar.append(fill);
    ol.append(h("li", { class: `risk ${r.kind}${r.p >= 0.7 ? " high" : r.p >= 0.3 ? " mid" : ""}`, "data-x": r.x, "data-y": r.y, tabindex: 0, title: "Show on the map" },
      h("span", { class: "risk-p" }, `${p}%`),
      h("span", { class: "risk-body" }, h("b", {}, KIND[r.kind]), ` ${robotsText(r.robots)}`, h("span", { class: "why" }, r.why)), bar));
  }
  const ul = $("open-list"); ul.innerHTML = "";
  for (const i of open) ul.append(incidentItem(i));
}

/* the full incident history lives on the Safety tab: it is a log, not a decision */
function renderIncidentLog(snap) {
  const incs = (snap.incidents || []).filter((i) => !state.inc || (state.inc === "solved") === (i.status === "solved")).slice().reverse();
  if (!changed("incs", [incs.map((i) => [i.id, i.status, i.rung, i.robots.length]), state.inc])) return;
  const ol = $("incident-list"); ol.innerHTML = "";
  if (!incs.length) ol.append(h("li", { class: "empty" }, state.inc === "open" ? "Nothing is waiting to be solved." : "No deadlocks or collisions yet. Each one is listed here with how it ended."));
  for (const i of incs) ol.append(incidentItem(i));
}
for (const id of ["risk-list", "open-list", "incident-list"]) {
  $(id).addEventListener("click", (e) => { const li = e.target.closest("li[data-x]"); if (li) zoomTo(+li.dataset.x, +li.dataset.y); });
  $(id).addEventListener("keydown", (e) => { const li = e.target.closest("li[data-x]"); if (li && (e.key === "Enter" || e.key === " ")) { e.preventDefault(); zoomTo(+li.dataset.x, +li.dataset.y); } });
}
$("band-say").addEventListener("click", () => {
  const top = (state.snap?.risk || [])[0] || (state.snap?.incidents || []).filter((i) => i.status !== "solved").pop();
  if (top) zoomTo(top.x, top.y);
});

/* ------------------------------------------------------- robots tab: heartbeat
   Liveness, separate from health: a robot can be perfectly healthy and still have
   stopped talking, which is the failure the fleet reacts to by withholding work. */
function renderHeartbeat(snap) {
  const rows = snap.robots.map((r) => {
    const b = state.hb[r.id] || {};
    return { r, b, lvl: hbLevel(b), age: b.age ?? 0, ms: r.compute_ms || 0 };
  });
  if (state.hbSort === "age") rows.sort((a, c) => c.age - a.age || a.r.id.localeCompare(c.r.id));

  if (!changed("hb", [rows.map((o) => [o.r.id, o.lvl, Math.round(o.age * 2), Math.round(o.ms * 2),
    (state.hb[o.r.id] || { beats: [] }).beats.join("")]), state.hbSort, state.selected])) return;

  const live = rows.filter((o) => o.lvl === "live").length;
  const late = rows.filter((o) => o.lvl === "late").length;
  const silent = rows.filter((o) => o.lvl === "silent").length;
  const slowest = rows.reduce((m, o) => (o.ms > m.ms ? o : m), { ms: -1, r: { id: "" } });
  const sum = $("hb-summary"); sum.innerHTML = "";
  if (!rows.length) sum.textContent = "Waiting for the first reports.";
  else {
    sum.append(h("b", {}, `${live} of ${rows.length}`), " reporting on time");
    if (late) sum.append(`, ${late} late`);
    if (silent) sum.append(", ", h("span", { class: "bad" }, `${silent} silent`));
    if (slowest.ms > 0) sum.append(`. Slowest on-board planning cycle ${num(slowest.ms, 1)} ms (${slowest.r.id.replace("AMR_", "AMR ")}).`);
    else sum.append(".");
  }

  const grid = $("hb-grid"); grid.innerHTML = "";
  for (const { r, b, lvl, ms } of rows) {
    const strip = h("span", { class: "hb-strip" });
    const beats = b.beats || [];
    for (let i = 0; i < HB_LEN; i++) {
      const v = beats[beats.length - HB_LEN + i];
      // height carries the cycle time, so one strip shows liveness and load at once
      const bar = h("i", { class: v === 1 ? "on" : v === 0 ? "miss" : "" });
      if (v === 1) bar.style.height = `${Math.min(100, 35 + (ms / 12) * 65)}%`;
      strip.append(bar);
    }
    const btn = h("button", { type: "button", class: `hb-row ${lvl}`, "data-id": r.id,
      "aria-label": `${r.id}, ${lvl === "live" ? "reporting" : lvl}, last report ${num(b.age, 1)} seconds ago, planning cycle ${num(ms, 1)} milliseconds` },
    h("span", { class: "hb-dot" }), h("span", { class: "hb-id" }, r.id.replace("AMR_", "")),
    strip, h("span", { class: "hb-age" }, lvl === "live" ? `${num(ms, 1)}ms` : `${num(b.age, 1)}s`));
    grid.append(btn);
  }
}
$("hb-grid").addEventListener("click", (e) => { const b = e.target.closest(".hb-row"); if (b) select(b.dataset.id); });

/* ------------------------------------------------------------------ robots tab: health board */
function renderFleet(snap) {
  const view = snap.robots.map((r) => [r.id, r.mode, Math.round(r.battery * 100), r.stale, r.health, r.task, r.goal,
    (r.waiting_for || [])[0] || "", r.bypass, r.safety]);
  if (changed("fleet", [view, state.selected])) {
    const counts = { act: 0, watch: 0 };
    const ul = $("robot-tags"); ul.innerHTML = "";
    for (const r of snap.robots) {
      const pr = problems(r);
      const worst = pr.reduce((m, p) => Math.max(m, p[1]), -1);
      if (worst === 2) counts.act++; else if (worst === 1) counts.watch++;
      const battCls = r.battery < 0.2 ? "batt low" : r.battery < 0.35 ? "batt mid" : "batt";
      const bar = h("span", { class: battCls }); const fill = h("i"); fill.style.width = `${Math.max(2, Math.round(r.battery * 100))}%`; bar.append(fill);
      const flag = pr.length ? pr.sort((a, b) => b[1] - a[1])[0] : null;
      const flagCls = !flag ? "tag-flag fine" : flag[1] === 2 ? "tag-flag" : flag[1] === 1 ? "tag-flag warn" : "tag-flag fine";
      const what = modeText(r.mode).replace("going to pick up", "to pick-up").replace("going to charge", "to charger").replace("going to park", "to parking");
      const btn = h("button", { type: "button", class: `tag${state.selected === r.id ? " sel" : ""}${r.stale ? " stale" : ""}${worst === 2 ? " problem" : ""}`,
        "data-id": r.id, "aria-label": `${r.id}, ${what}${r.task ? ", task " + r.task : ""}, battery ${pct(r.battery)}${flag ? ", " + flag[0] : ""}` },
      h("span", { class: "tag-num" }, r.id.replace("AMR_", ""), h("small", {}, pct(r.battery))),
      h("span", { class: "tag-mode" }, what),
      h("span", { class: flagCls }, flag ? flag[0] : r.task ? `task ${r.task}` : "no task"), bar);
      btn.style.setProperty("--mode", r.stale ? "#5B6570" : modeColour(r.mode));
      ul.append(h("li", {}, btn));
    }
    const n = snap.robots.length;
    const sum = $("health-summary"); sum.innerHTML = "";
    if (!n) sum.textContent = "Waiting for the robots to report in.";
    else {
      sum.append(h("b", {}, `${n - counts.act - counts.watch} of ${n}`), " robots healthy");
      if (counts.watch) sum.append(`, ${counts.watch} to watch`);
      if (counts.act) sum.append(", ", h("span", { class: "bad" }, `${counts.act} need attention`));
      sum.append(". Click a robot for details and controls.");
    }
  }
  renderDetail(snap);
}
function renderDetail(snap) {
  const box = $("robot-detail");
  const r = snap.robots.find((x) => x.id === state.selected);
  if (!r) { box.hidden = true; state.sigs.detail = ""; return; }
  box.hidden = false;
  if (!changed("detail", [r.mode, r.task, r.goal, r.station, Math.round(r.battery * 100), num(r.v, 1), num(r.x, 0), num(r.y, 0),
    r.health, r.waiting_for, r.bypass, num(r.compute_ms, 0), r.safety, r.stale])) return;
  box.innerHTML = ""; box.style.borderLeftColor = modeColour(r.mode);
  const sc = r.safety || {};
  const rows = [["Doing", modeText(r.mode)], ["Task", r.task || "none"], ["Heading to", niceStation(r.goal) || "nowhere"],
    ["Docked at", niceStation(r.station) || "not docked"], ["Battery", pct(r.battery)], ["Speed", `${num(r.v, 2)} m/s`],
    ["Position", `${num(r.x, 1)}, ${num(r.y, 1)}`], ["Health", r.stale ? "not reporting" : (r.health || "").toLowerCase()],
    ["Waiting for", (r.waiting_for || []).join(", ") || "nobody"], ["Local manoeuvre", r.bypass || "none"],
    ["On-board planning", `${num(r.compute_ms, 1)} ms per cycle`],
    ["Safety", `${sc.COLLISION || 0} collisions, ${sc.NEAR_MISS || 0} near misses, ${sc.AVOIDED || 0} avoided`]];
  const dl = h("dl"); for (const [k, v] of rows) dl.append(h("div", {}, h("dt", {}, k), h("dd", {}, v)));
  const act = h("div", { class: "actions" });
  const cmd = (mode, label, cls = "btn small") => h("button", { type: "button", class: cls, "data-cmd": mode, "data-label": label }, label);
  const faulted = r.mode === "fault" || (r.health && r.health !== "HEALTHY" && r.health !== "DEGRADED");
  act.append(r.mode === "paused" ? cmd("resume", "Resume") : cmd("pause", "Pause"),
    faulted ? cmd("clear_fault", "Clear the fault", "btn small primary") : "",
    r.mode === "estop" ? cmd("release", "Release emergency stop") : cmd("estop", "Emergency stop", "btn small danger"),
    h("button", { type: "button", class: "btn small", "data-close": "1" }, "Close"));
  // demo-only fault injection, kept apart from the real controls and confirmed before use
  const demo = h("div", { class: "demo-test" },
    h("span", {}, "Demo test: makes this robot stop as if its drive motor failed, so you can watch the fleet re-plan around it and re-auction its task. It stays stopped until you click Clear the fault."),
    faulted ? "" : cmd("fault", "Simulate a motor fault", "btn small danger"));
  box.append(h("h3", {}, r.id, h("small", {}, `AMR-01, ${modeText(r.mode)}`)), dl, act, demo);
}
$("robot-tags").addEventListener("click", (e) => { const b = e.target.closest(".tag"); if (b) select(b.dataset.id); });
$("robot-detail").addEventListener("click", async (e) => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.close) { state.selected = null; refresh(); return; }
  const rid = state.selected;
  if (b.dataset.cmd === "fault" && !confirm(`Demo test: stop ${rid} as if its motor failed? It stays stopped until you click "Clear the fault".`)) return;
  try { await api(`/api/robots/${rid}/mode`, { mode: b.dataset.cmd }); toast(`${rid}: ${b.dataset.label.toLowerCase()} sent.`); }
  catch (err) { toast(`${rid}: ${err.message}`); }
});
function select(id) {
  state.selected = state.selected === id ? null : id;
  if (state.selected) switchTab("fleet"); else refresh();
}

/* ------------------------------------------------------------------ tasks tab */
function fillForm(site) {
  const opt = (sel, items, first) => {
    sel.innerHTML = ""; sel.append(h("option", { value: "random" }, first));
    for (const i of items) sel.append(h("option", { value: i }, niceStation(i)));
  };
  opt($("f-pickup"), site.pickups, "Any pick station (random)");
  opt($("f-dropoff"), site.dropoffs, "Any drop-off (random)");
}
function refreshRobotChoices(snap) {
  const view = snap.robots.map((r) => [r.id, blockedReason(r), r.mode, Math.round(r.battery * 20)]);
  if (!changed("choices", view)) return;
  const sel = $("f-robot"); const cur = sel.value;
  sel.innerHTML = "";
  if (!snap.robots.length) sel.append(h("option", { value: "" }, "No robots reporting yet"));
  for (const r of snap.robots) {
    const why = blockedReason(r);
    sel.append(h("option", { value: r.id, disabled: !!why }, why ? `${r.id}: ${why}` : `${r.id}: ${modeText(r.mode)}, battery ${pct(r.battery)}`));
  }
  if (cur && [...sel.options].some((o) => o.value === cur && !o.disabled)) sel.value = cur;
}
function renderTasks(snap) {
  refreshRobotChoices(snap);
  const tasks = snap.tasks;
  if (!changed("tasks", tasks.map((t) => [t.task_id, t.status, t.robot, t.winner, t.winning_bid, Object.keys(t.bidders || {}).length]))) return;
  const groups = [
    ["Waiting for a robot", tasks.filter((t) => t.status === "queued" || t.status === "assigned"), "No tasks waiting. Release some with the yellow button."],
    ["Going to pick up", tasks.filter((t) => t.status === "in_progress"), "No robot is on its way to a pick-up."],
    ["Carrying to the drop-off", tasks.filter((t) => t.status === "picked"), "No robot is carrying a tote."],
    ["Delivered", tasks.filter((t) => t.status === "completed").slice(-15).reverse(), "Nothing delivered yet."],
  ];
  const board = $("board"); board.innerHTML = "";
  for (const [title, list, empty] of groups) {
    const total = title === "Delivered" ? tasks.filter((t) => t.status === "completed").length : list.length;
    board.append(h("h3", {}, title, h("span", {}, total ? String(total) : "")));
    const ol = h("ol");
    if (!list.length) ol.append(h("li", { class: "empty" }, empty));
    for (const t of list) {
      const [pname, pcol] = PRIO[t.priority] || ["", "#8D959C"];
      let who = "";
      if (t.status === "queued" || t.status === "assigned") {
        const n = Object.keys(t.bidders || {}).length;
        who = t.assign_to ? `Given directly to ${t.assign_to}` : t.winner ? `${t.winner} is winning the auction (${n} robot${n === 1 ? "" : "s"} bidding)` : "Auction open";
      } else if (t.status === "completed") who = `${t.robot}, delivered ${num(t.latency, 0)} s after release`;
      else who = t.robot;
      const side = h("span", { class: "side" }, h("span", { class: "prio" }, pname));
      if (t.status === "queued" || t.status === "assigned") side.append(h("button", { type: "button", class: "btn small", "data-cancel": t.task_id }, "Cancel"));
      const li = h("li", { class: "task" },
        h("span", { class: "route" }, h("b", {}, t.task_id), `${niceStation(t.pickup)} to ${niceStation(t.dropoff)}`),
        h("span", { class: "who" }, who), side);
      li.style.setProperty("--prio", pcol);
      ol.append(li);
    }
    board.append(ol);
  }
}
$("board").addEventListener("click", async (e) => {
  const b = e.target.closest("[data-cancel]"); if (!b) return;
  b.disabled = true;
  try { await api(`/api/tasks/${b.dataset.cancel}/cancel`, {}); toast(`Task ${b.dataset.cancel} cancelled.`); }
  catch (err) { toast(`Task not cancelled: ${err.message}`); b.disabled = false; }
});

/* ------------------------------------------------------------------ safety tab */
function inZone(ev, b) { return b && ev.x >= b.x_min && ev.x <= b.x_max && ev.y >= b.y_min && ev.y <= b.y_max; }
function renderSafety(snap) {
  const evs = snap.safety_recent.filter((e) => !state.lvl || e.level === state.lvl).slice().reverse();
  if (changed("events", [evs.map((e) => [e.event_id, e.acked]), state.lvl])) {
    const tb = $("event-table").querySelector("tbody"); tb.innerHTML = "";
    if (!evs.length) tb.append(h("tr", {}, h("td", { colspan: 5, class: "muted" },
      state.lvl ? "No events of this kind." : "No safety events. They appear here the moment the safety monitor sees one.")));
    for (const e of evs) {
      const ack = h("td");
      if (!e.acked && e.level !== "AVOIDED") ack.append(h("button", { type: "button", class: "btn small", "data-ack": e.event_id }, "Mark reviewed"));
      else if (e.acked) ack.append(h("span", { class: "muted" }, "reviewed"));
      const who = (e.robots || []).join(", ") + (e.other && !(e.robots || []).includes(e.other) ? ` and ${e.other}` : "");
      tb.append(h("tr", { "data-x": e.x, "data-y": e.y, title: "Show on the map" },
        h("td", {}, fmtTime(e.stamp)), h("td", {}, h("span", { class: `lvl ${e.level}` }, e.level === "NEAR_MISS" ? "near miss" : e.level.toLowerCase())),
        h("td", {}, who), h("td", { class: "muted" }, `${num(e.x, 1)}, ${num(e.y, 1)}`), ack));
    }
  }
  const zones = Object.entries(state.site.zones || {});
  const zview = zones.map(([zid, b]) => [zid, snap.safety_recent.filter((e) => e.level !== "AVOIDED" && inZone(e, b)).length, snap.zone_rules[zid] || ""]);
  if (changed("zonetable", zview)) {
    const zb = $("zone-table").querySelector("tbody"); zb.innerHTML = "";
    for (const [zid, n, rule] of zview) {
      const act = h("td");
      for (const [r, label] of [["avoid", "Keep out"], ["slow", "Slow"], ["clear", "Normal"]]) {
        if ((rule || "clear") === r) continue;
        act.append(h("button", { type: "button", class: "btn small", "data-zone": zid, "data-rule": r }, label), " ");
      }
      zb.append(h("tr", {}, h("td", {}, zid), h("td", { class: n ? "bad" : "muted" }, String(n)),
        h("td", {}, rule === "avoid" ? "keep out" : rule === "slow" ? "slow down" : "normal"), act));
    }
  }
}
$("event-table").addEventListener("click", async (e) => {
  const b = e.target.closest("[data-ack]");
  if (b) { e.stopPropagation(); b.disabled = true; try { await api(`/api/safety/events/${b.dataset.ack}/ack`, {}); } catch (err) { toast(err.message); } return; }
  const tr = e.target.closest("tr[data-x]");
  if (tr) zoomTo(+tr.dataset.x, +tr.dataset.y);
});
$("zone-table").addEventListener("click", async (e) => {
  const b = e.target.closest("[data-rule]"); if (!b) return;
  try { await api(`/api/zones/${b.dataset.zone}/rule`, { rule: b.dataset.rule }); toast(`Zone ${b.dataset.zone}: ${b.textContent.toLowerCase()}.`); }
  catch (err) { toast(`Zone rule not set: ${err.message}`); }
});

/* ------------------------------------------------------------------ analysis tab
   Every mark here is drawn from the live snapshot: kpis, timeline (sampled by the
   gateway every 5 s) and the task records. Nothing is precomputed or canned - when
   the stream stops, the charts say so rather than showing a stale picture. */

/* measure the SVG, set a 1:1 viewBox so text stays unstretched, and clear it */
function chartBox(id, height) {
  const sv = $(id);
  if (height) sv.style.height = `${height}px`;
  const w = Math.max(220, Math.round(sv.getBoundingClientRect().width) || 380);
  const hh = height || Math.round(sv.getBoundingClientRect().height) || 140;
  sv.setAttribute("viewBox", `0 0 ${w} ${hh}`);
  sv.innerHTML = "";
  return { sv, w, h: hh };
}
function chartEmpty(id, text, height) {
  const { sv, w, h: hh } = chartBox(id, height);
  const t = svg("text", { x: w / 2, y: hh / 2, "text-anchor": "middle", "dominant-baseline": "middle", class: "c-empty" }, sv);
  t.textContent = text;
}
function niceMax(v) {
  if (v <= 5) return Math.max(1, Math.ceil(v));
  const mag = Math.pow(10, Math.floor(Math.log10(v)));
  return Math.ceil(v / (mag / 2)) * (mag / 2);
}
const tipEl = $("tip");
function tipShow(evt, lines) {
  tipEl.innerHTML = lines.join("<br>");
  tipEl.hidden = false;
  const r = tipEl.getBoundingClientRect();
  let x = evt.clientX + 14, y = evt.clientY - r.height - 10;
  if (x + r.width > innerWidth - 8) x = evt.clientX - r.width - 14;
  if (y < 8) y = evt.clientY + 16;
  tipEl.style.left = `${x}px`; tipEl.style.top = `${y}px`;
}
const tipHide = () => { tipEl.hidden = true; };
function dataTable(id, cols, rows) {
  const box = $(id); if (!box) return;
  box.innerHTML = "";
  const t = h("table", { class: "grid" });
  const thead = h("thead"); const tr = h("tr");
  for (const c of cols) tr.append(h("th", { scope: "col" }, c));
  thead.append(tr); t.append(thead);
  const tb = h("tbody");
  for (const r of rows) tb.append(h("tr", {}, ...r.map((v) => h("td", {}, String(v)))));
  t.append(tb);
  box.append(h("div", { class: "scroll" }, t));
}

/* a shared crosshair for the two time charts, so they read as one timeline */
function timeHover(sv, geom, series, label) {
  const { x0, x1, plotT, pad } = geom;
  const hit = svg("rect", { x: pad.l, y: pad.t, width: Math.max(1, x1 - x0), height: Math.max(1, plotT), class: "c-hit" }, sv);
  const cross = svg("line", { y1: pad.t, y2: pad.t + plotT, class: "c-cross" }, sv);
  const dot = svg("circle", { r: 4.5, class: "c-dot" }, sv);
  cross.style.display = dot.style.display = "none";
  hit.addEventListener("pointermove", (e) => {
    const bb = sv.getBoundingClientRect();
    const px = ((e.clientX - bb.left) / bb.width) * (+sv.viewBox.baseVal.width || bb.width);
    let best = 0, bd = Infinity;
    series.forEach((p, i) => { const d = Math.abs(p.px - px); if (d < bd) { bd = d; best = i; } });
    const p = series[best];
    cross.setAttribute("x1", p.px); cross.setAttribute("x2", p.px);
    dot.setAttribute("cx", p.px); dot.setAttribute("cy", p.py);
    cross.style.display = dot.style.display = "";
    tipShow(e, [`<b>${fmtTime(p.t)}</b>`, `${p.v} ${label}`, ...(p.extra || [])]);
  });
  hit.addEventListener("pointerleave", () => { cross.style.display = dot.style.display = "none"; tipHide(); });
}

function drawDelivered(snap) {
  const tl = snap.timeline || [];
  if (tl.length < 2) return chartEmpty("ch-delivered", "The chart draws itself once the gateway has two samples (about 10 s).");
  const { sv, w, h: hh } = chartBox("ch-delivered");
  const pad = { l: 34, r: 10, t: 10, b: 20 };
  const x0 = pad.l, x1 = w - pad.r, plotT = hh - pad.t - pad.b;
  const t0 = tl[0][0], t1 = tl[tl.length - 1][0];
  const maxD = niceMax(Math.max(1, ...tl.map((r) => r[1])));
  const X = (t) => x0 + ((t - t0) / Math.max(t1 - t0, 1)) * (x1 - x0);
  const Y = (v) => pad.t + plotT - (v / maxD) * plotT;

  for (const g of [0, maxD / 2, maxD]) {                       // recessive grid, labelled in units
    svg("line", { x1: x0, x2: x1, y1: Y(g), y2: Y(g), class: g === 0 ? "c-axis" : "c-grid" }, sv);
    const lb = svg("text", { x: x0 - 6, y: Y(g), "text-anchor": "end", "dominant-baseline": "middle", class: "c-tick" }, sv);
    lb.textContent = Math.round(g);
  }
  for (const t of [t0, (t0 + t1) / 2, t1]) {
    const lb = svg("text", { x: X(t), y: hh - 6, "text-anchor": t === t0 ? "start" : t === t1 ? "end" : "middle", class: "c-tick" }, sv);
    lb.textContent = fmtTime(t);
  }

  const pts = tl.map((r) => `${X(r[0])},${Y(r[1])}`).join(" ");
  svg("polygon", { points: `${x0},${Y(0)} ${pts} ${x1},${Y(0)}`, class: "c-area" }, sv);
  svg("polyline", { points: pts, class: "c-line" }, sv);

  // hazards on the same timeline: a collision is a full-height rule, a near miss a
  // short tick at the baseline - the shapes differ, so colour is not doing the work
  let pc = tl[0][2], pn = tl[0][3];
  for (const r of tl) {
    if (r[2] > pc) svg("line", { x1: X(r[0]), x2: X(r[0]), y1: pad.t, y2: pad.t + plotT, class: "c-coll" }, sv);
    if (r[3] > pn) svg("line", { x1: X(r[0]), x2: X(r[0]), y1: pad.t + plotT - 7, y2: pad.t + plotT, class: "c-near" }, sv);
    pc = r[2]; pn = r[3];
  }

  const series = tl.map((r, i) => {
    const prev = i ? tl[i - 1] : r;
    const dt = Math.max(1e-6, r[0] - prev[0]);
    const rate = i ? ((r[1] - prev[1]) / dt) * 3600 : 0;
    return { t: r[0], v: r[1], px: X(r[0]), py: Y(r[1]),
      extra: [`${num(rate, 0)} per hour just then`, `${r[2]} collisions, ${r[3]} near misses so far`] };
  });
  timeHover(sv, { x0, x1, plotT, pad }, series, "delivered");
  dataTable("tb-delivered", ["Time", "Delivered", "Collisions", "Near misses"],
    tl.slice().reverse().map((r) => [fmtTime(r[0]), r[1], r[2], r[3]]));
}

function drawBusy(snap) {
  const tl = snap.timeline || [];
  if (tl.length < 2) return chartEmpty("ch-busy", "Waiting for the gateway's first two samples.");
  const { sv, w, h: hh } = chartBox("ch-busy");
  const pad = { l: 34, r: 10, t: 10, b: 20 };
  const x0 = pad.l, x1 = w - pad.r, plotT = hh - pad.t - pad.b;
  const t0 = tl[0][0], t1 = tl[tl.length - 1][0];
  const fleet = Math.max(snap.robots.length, ...tl.map((r) => r[5] || 0), 1);
  const maxB = niceMax(fleet);
  const X = (t) => x0 + ((t - t0) / Math.max(t1 - t0, 1)) * (x1 - x0);
  const Y = (v) => pad.t + plotT - (v / maxB) * plotT;

  for (const g of [0, maxB / 2, maxB]) {
    svg("line", { x1: x0, x2: x1, y1: Y(g), y2: Y(g), class: g === 0 ? "c-axis" : "c-grid" }, sv);
    const lb = svg("text", { x: x0 - 6, y: Y(g), "text-anchor": "end", "dominant-baseline": "middle", class: "c-tick" }, sv);
    lb.textContent = Math.round(g);
  }
  for (const t of [t0, (t0 + t1) / 2, t1]) {
    const lb = svg("text", { x: X(t), y: hh - 6, "text-anchor": t === t0 ? "start" : t === t1 ? "end" : "middle", class: "c-tick" }, sv);
    lb.textContent = fmtTime(t);
  }
  const pts = tl.map((r) => `${X(r[0])},${Y(r[5] || 0)}`).join(" ");
  svg("polygon", { points: `${x0},${Y(0)} ${pts} ${x1},${Y(0)}`, class: "c-area" }, sv);
  svg("polyline", { points: pts, class: "c-line" }, sv);
  svg("line", { x1: x0, x2: x1, y1: Y(snap.robots.length), y2: Y(snap.robots.length), class: "c-ref" }, sv);
  const rl = svg("text", { x: x1, y: Y(snap.robots.length) - 5, "text-anchor": "end", class: "c-lbl" }, sv);
  rl.textContent = `${snap.robots.length} robots in the fleet`;

  const series = tl.map((r) => ({ t: r[0], v: r[5] || 0, px: X(r[0]), py: Y(r[5] || 0),
    extra: [`${snap.robots.length - (r[5] || 0)} idle`] }));
  timeHover(sv, { x0, x1, plotT, pad }, series, "robots on a task");
  dataTable("tb-busy", ["Time", "On a task", "Idle"],
    tl.slice().reverse().map((r) => [fmtTime(r[0]), r[5] || 0, snap.robots.length - (r[5] || 0)]));
}

function drawLatency(snap) {
  const lats = snap.tasks.filter((t) => t.status === "completed" && t.latency != null).map((t) => t.latency);
  if (lats.length < 2) return chartEmpty("ch-latency", lats.length ? "One delivery so far. The shape appears from two." : "No deliveries yet.");
  const { sv, w, h: hh } = chartBox("ch-latency");
  const pad = { l: 28, r: 10, t: 12, b: 30 };
  const x0 = pad.l, x1 = w - pad.r, plotT = hh - pad.t - pad.b;
  const lo = Math.min(...lats), hi = Math.max(...lats);
  const nb = Math.min(14, Math.max(5, Math.ceil(Math.sqrt(lats.length))));
  const span = Math.max(hi - lo, 1e-6), bw = span / nb;
  const bins = new Array(nb).fill(0);
  for (const v of lats) bins[Math.min(nb - 1, Math.floor((v - lo) / bw))]++;
  const maxN = niceMax(Math.max(...bins));
  const Y = (v) => pad.t + plotT - (v / maxN) * plotT;
  const sorted = lats.slice().sort((a, b) => a - b);
  const q = (p) => sorted[Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * p))];

  for (const g of [0, maxN]) {
    svg("line", { x1: x0, x2: x1, y1: Y(g), y2: Y(g), class: g === 0 ? "c-axis" : "c-grid" }, sv);
    const lb = svg("text", { x: x0 - 6, y: Y(g), "text-anchor": "end", "dominant-baseline": "middle", class: "c-tick" }, sv);
    lb.textContent = Math.round(g);
  }
  const step = (x1 - x0) / nb;
  bins.forEach((n, i) => {
    const bx = x0 + i * step;
    const r = svg("rect", { x: bx + 1, y: Y(n), width: Math.max(1, step - 2), height: Math.max(0, Y(0) - Y(n)), rx: 2, class: n ? "c-bar" : "c-bar dim" }, sv);
    const a = lo + i * bw, b = a + bw;
    r.addEventListener("pointerenter", (e) => tipShow(e, [`<b>${num(a, 0)}–${num(b, 0)} s</b>`, `${n} task${n === 1 ? "" : "s"}`]));
    r.addEventListener("pointerleave", tipHide);
  });
  for (const [p, name] of [[0.5, "median"], [0.95, "slowest 5%"]]) {
    const v = q(p), px = x0 + Math.min(1, (v - lo) / span) * (x1 - x0);
    svg("line", { x1: px, x2: px, y1: pad.t, y2: pad.t + plotT, class: "c-pct" }, sv);
    const lb = svg("text", { x: Math.min(px + 4, x1 - 2), y: pad.t + 8, "text-anchor": px > x1 - 60 ? "end" : "start", class: "c-val" }, sv);
    lb.textContent = `${name} ${num(v, 0)} s`;
  }
  for (const [v, anchor] of [[lo, "start"], [hi, "end"]]) {
    const lb = svg("text", { x: v === lo ? x0 : x1, y: hh - 14, "text-anchor": anchor, class: "c-tick" }, sv);
    lb.textContent = `${num(v, 0)} s`;
  }
  const ax = svg("text", { x: (x0 + x1) / 2, y: hh - 2, "text-anchor": "middle", class: "c-lbl" }, sv);
  ax.textContent = `release to drop-off · ${lats.length} deliveries`;
  dataTable("tb-latency", ["Delivery time", "Tasks"],
    bins.map((n, i) => [`${num(lo + i * bw, 0)}–${num(lo + (i + 1) * bw, 0)} s`, n]));
}

/* horizontal bars: one hue, sorted, direct value labels - lengths on a shared
   baseline compare better than stacked segments, and no palette is needed */
function drawBars(id, rows, unit, emptyText) {
  if (!rows.length) return chartEmpty(id, emptyText, 90);
  const rowH = 15, pad = { l: 42, r: 34, t: 4, b: 4 };
  const height = rows.length * rowH + pad.t + pad.b;
  const { sv, w } = chartBox(id, height);
  const x0 = pad.l, x1 = w - pad.r;
  const maxV = Math.max(1, ...rows.map((r) => r[1]));
  rows.forEach(([label, v, title], i) => {
    const y = pad.t + i * rowH;
    const lb = svg("text", { x: x0 - 6, y: y + rowH / 2, "text-anchor": "end", "dominant-baseline": "middle", class: "c-tick mid" }, sv);
    lb.textContent = label;
    const bwid = (v / maxV) * (x1 - x0);
    const r = svg("rect", { x: x0, y: y + 2, width: Math.max(v ? 2 : 0, bwid), height: rowH - 4, rx: 2, class: v ? "c-bar" : "c-bar dim" }, sv);
    const vl = svg("text", { x: x0 + bwid + 5, y: y + rowH / 2, "dominant-baseline": "middle", class: "c-val" }, sv);
    vl.textContent = String(v);
    r.addEventListener("pointerenter", (e) => tipShow(e, [`<b>${label}</b>`, `${v} ${unit}`, ...(title ? [title] : [])]));
    r.addEventListener("pointerleave", tipHide);
  });
  svg("line", { x1: x0, x2: x0, y1: pad.t, y2: pad.t + rows.length * rowH, class: "c-axis" }, sv);
}

function drawPerRobot(snap) {
  const done = snap.tasks.filter((t) => t.status === "completed" && t.robot);
  const by = {};
  for (const r of snap.robots) by[r.id] = { n: 0, lat: [] };
  for (const t of done) { (by[t.robot] = by[t.robot] || { n: 0, lat: [] }).n++; if (t.latency != null) by[t.robot].lat.push(t.latency); }
  const rows = Object.entries(by)
    .map(([id, o]) => [id.replace("AMR_", ""), o.n,
      o.lat.length ? `mean ${num(o.lat.reduce((a, b) => a + b, 0) / o.lat.length, 0)} s per delivery` : "nothing delivered yet"])
    .sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]));
  drawBars("ch-robots", rows, "delivered", "No robots reporting yet.");
  dataTable("tb-robots", ["Robot", "Delivered", "Mean delivery time"], rows.map((r) => [`AMR ${r[0]}`, r[1], r[2]]));
}

function drawModes(snap) {
  const rows = Object.entries(snap.kpis.modes || {})
    .map(([m, n]) => [modeText(m), n, ""])
    .sort((a, b) => b[1] - a[1]);
  drawBars("ch-modes", rows, "robots", "No robots reporting yet.");
}

function renderAnalysis(snap) {
  const k = snap.kpis;
  const sig = [k, snap.timeline.length, snap.tasks.length, snap.robots.length, innerWidth];
  if (!changed("analysis", sig)) return;

  const u = (v, unit, d = 0) => (v === null || v === undefined ? h("span", { class: "kpi-v" }, "–")
    : h("span", {}, h("span", { class: "kpi-v" }, num(v, d)), h("span", { class: "kpi-u" }, unit)));
  const groups = [
    ["Throughput", [
      ["Delivered", u(k.tasks_completed, "tasks")],
      ["Rate", u(k.throughput_per_hour, "tasks/h", 1)],
      ["Carrying now", u(k.tasks_in_progress, "tasks")],
      ["Waiting for a robot", u(k.tasks_queued, "tasks")],
      ["Released in all", u(k.tasks_total, "tasks")],
      ["Finished", u(k.completion_rate_pct, "%", 1)],
    ]],
    ["Timeliness", [
      ["Median delivery", u(k.latency_p50_s, "s")],
      ["Mean delivery", u(k.latency_mean_s, "s")],
      ["Slowest 5%", u(k.latency_p95_s, "s")],
      ["Wait for a robot", u(k.assign_wait_mean_s, "s", 1)],
      ["Before deadline", u(k.deadline_compliance_pct, "%")],
      ["Shift length", u(k.makespan_s, "s")],
    ]],
    ["Fleet use", [
      ["Time on tasks", u(k.utilisation_pct, "%")],
      ["Time charging", u(k.charging_pct, "%")],
      ["Robots reporting", u(k.robots, "of " + (state.site.robots.length || k.robots))],
      ["Slowest planning cycle", u(k.compute_ms_p95, "ms", 1)],
    ]],
    ["Deadlock resolution", [
      ["Distinct deadlocks", u(k.deadlock_episodes, "")],
      ["Plans run", u(k.deadlocks, "")],
      ["Resolved by the fleet", u(k.deadlocks_resolved, "")],
      ["Sent to you", u(k.deadlocks_escalated, "")],
      ["Timed out", u(k.deadlocks_timed_out, "")],
      ["Unaccounted", u(k.deadlocks_unaccounted, "")],
    ]],
    ["In-aisle standoffs (L2b)", [
      ["Distinct standoffs", u(k.standoffs, "")],
      ["Chain reverses asked", u(k.standoff_plans, "")],
      ["Robots asked to back off", u(k.standoff_movers, "")],
    ]],
  ];
  const box = $("kpi-groups"); box.innerHTML = "";
  for (const [title, rows] of groups) {
    const dl = h("dl", { class: "kpi-rows" });
    for (const [label, val] of rows) dl.append(h("div", {}, h("dt", {}, label), h("dd", {}, val)));
    box.append(h("section", { class: "kpi-group" }, h("h3", {}, title), dl));
  }

  drawDelivered(snap); drawBusy(snap); drawLatency(snap); drawPerRobot(snap); drawModes(snap);
}

/* ------------------------------------------------------------------ activity strip */
function renderLog(snap) {
  if (!changed("log", snap.log.length ? snap.log[snap.log.length - 1] : null)) return;
  const ol = $("log"); ol.innerHTML = "";
  if (!snap.log.length) { ol.append(h("li", { class: "empty" }, "Fleet activity appears here: releases, pick-ups, deliveries and safety events.")); return; }
  for (const [t, text] of snap.log.slice().reverse()) {
    ol.append(h("li", { class: text.startsWith("COLLISION") ? "COLLISION" : "" }, h("span", { class: "t" }, fmtTime(t)), h("span", {}, text)));
  }
}

/* ------------------------------------------------------------------ render loop */
function currentTab() { return (document.querySelector('.tabs [aria-selected="true"]') || {}).dataset?.tab || "fleet"; }
function render(snap) {
  state.snap = snap;
  trackHeartbeat(snap);
  renderTop(snap); drawRobots(snap); drawEvents(snap.safety_recent); drawRisk(snap);
  renderBand(snap); renderNeeds(snap); renderLog(snap);
  const tab = currentTab();
  if (tab === "fleet") { renderHeartbeat(snap); renderFleet(snap); }
  else if (tab === "tasks") renderTasks(snap);
  else if (tab === "safety") { renderIncidentLog(snap); renderSafety(snap); }
  else if (tab === "analysis") renderAnalysis(snap);
}
function refresh() { state.sigs = {}; if (state.snap) render(state.snap); }
function switchTab(name) {
  for (const b of document.querySelectorAll(".tabs button")) b.setAttribute("aria-selected", b.dataset.tab === name);
  for (const p of document.querySelectorAll(".panel")) p.hidden = p.id !== `tab-${name}`;
  store("nrda.tab3", name);
  refresh();
}
// charts are measured in pixels, so a resize has to redraw them
addEventListener("resize", () => { clearTimeout(switchTab._r); switchTab._r = setTimeout(refresh, 200); });

/* ------------------------------------------------------------------ connection */
function setLink(text, cls) { const l = $("link-state"); l.textContent = text; l.className = `link ${cls}`; }
function onData(d) {
  state.lastMsg = Date.now();
  if (d.type === "snapshot") render(d);
  else if (d.type === "safety") drawEvents([d.event]);
}
function connect() {
  let es = null;
  const open = () => {
    try { es = new EventSource("/api/stream"); } catch (e) { es = null; return; }
    es.onopen = () => setLink("Live", "ok");
    es.onmessage = (m) => { try { onData(JSON.parse(m.data)); setLink("Live", "ok"); } catch (e) { console.error("bad message", e); } };
    es.onerror = () => setLink("Reconnecting", "bad");
  };
  open();
  // fallback: if the stream is silent (proxy buffering, gateway restart), poll snapshots until it recovers
  setInterval(async () => {
    if (Date.now() - state.lastMsg < 3000) return;
    try {
      onData(await api("/api/snapshot"));
      setLink("Live (polling)", "ok");
      if (!es || es.readyState === 2) open();
    } catch (e) { setLink("Gateway not reachable", "bad"); }
  }, 1000);
}

async function loadSite() {
  for (let attempt = 1; ; attempt++) {
    try { return await api("/api/site"); }
    catch (e) {
      setLink("Gateway not reachable", "bad");
      $("map-empty").hidden = false;
      $("empty-title").textContent = "Connecting to the fleet gateway";
      $("empty-text").textContent = `No answer from ${location.host} yet (attempt ${attempt}). The console connects by itself once the fleet launch is running: ./nrda.sh sim`;
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
}

/* ------------------------------------------------------------------ wiring */
function applyTheme(t) { if (t) document.documentElement.dataset.theme = t; else delete document.documentElement.dataset.theme; }
async function main() {
  applyTheme(store("nrda.theme"));
  for (const b of document.querySelectorAll(".tabs button")) b.addEventListener("click", () => switchTab(b.dataset.tab));
  state.site = await loadSite();
  $("empty-title").textContent = "Waiting for the robots";
  $("empty-text").textContent = "Gazebo is starting. Each AMR appears here as soon as its on-board agent reports its position.";
  buildMap(state.site); fillForm(state.site); setPlacing(false);
  switchTab(document.getElementById(`tab-${store("nrda.tab3")}`) ? store("nrda.tab3") : "fleet");

  $("btn-theme").addEventListener("click", () => {
    const dark = getComputedStyle(document.documentElement).colorScheme === "dark";
    const t = dark ? "light" : "dark"; applyTheme(t); store("nrda.theme", t);
  });
  $("btn-fit").addEventListener("click", fitView);
  $("btn-pallet").addEventListener("click", () => setPlacing(!state.placing));
  $("btn-legend").addEventListener("click", (e) => {       // the key is on demand, so it stops covering the floor
    const on = e.currentTarget.getAttribute("aria-pressed") !== "true";
    e.currentTarget.setAttribute("aria-pressed", on); $("legend").hidden = !on;
  });
  $("btn-ticker").addEventListener("click", (e) => {
    const on = e.currentTarget.getAttribute("aria-expanded") !== "true";
    e.currentTarget.setAttribute("aria-expanded", on);
    e.currentTarget.textContent = on ? "Less" : "More";
    $("log").classList.toggle("open", on);
  });
  $("btn-hb-sort").addEventListener("click", (e) => {
    state.hbSort = state.hbSort === "id" ? "age" : "id";
    e.currentTarget.textContent = state.hbSort === "id" ? "Sort by age" : "Sort by number";
    refresh();
  });
  $("btn-paths").addEventListener("click", (e) => { state.showPaths = !state.showPaths; e.currentTarget.setAttribute("aria-pressed", state.showPaths); refresh(); });
  $("btn-labels").addEventListener("click", (e) => { state.showLabels = !state.showLabels; e.currentTarget.setAttribute("aria-pressed", state.showLabels); refresh(); });

  const rel = $("btn-random");
  rel.addEventListener("click", async () => {
    if (rel.classList.contains("busy")) return;
    rel.classList.add("busy");
    try {
      const r = await api("/api/tasks", { random: 10 });
      toast(`Released ${r.created.length} tasks (${r.created[0]} to ${r.created[r.created.length - 1]}). The robots are bidding for them.`);
    } catch (e) { toast(`Tasks not released: ${e.message}`); }
    finally { rel.classList.remove("busy"); }
  });
  $("btn-pause-all").addEventListener("click", () => api("/api/fleet/pause", {}).then(() => toast("Fleet paused. Robots stop at their next safe point.")).catch((e) => toast(e.message)));
  $("btn-resume-all").addEventListener("click", () => api("/api/fleet/resume", {}).then(() => toast("Fleet resumed.")).catch((e) => toast(e.message)));
  $("btn-estop").addEventListener("click", async (e) => {
    const on = e.currentTarget.getAttribute("aria-pressed") === "true";
    if (!on && !confirm("Stop every robot now?")) return;
    try { await api(on ? "/api/fleet/release" : "/api/fleet/estop", {}); toast(on ? "Emergency stop released." : "Emergency stop sent. Every robot is braking.", !on); }
    catch (err) { toast(err.message); }
  });
  for (const r of document.querySelectorAll('input[name="mode"]')) {
    r.addEventListener("change", () => { $("f-robot").disabled = document.querySelector('input[name="mode"]:checked').value !== "direct"; });
  }
  for (const r of document.querySelectorAll('input[name="inc"]')) r.addEventListener("change", () => { state.inc = r.value; refresh(); });
  for (const r of document.querySelectorAll('input[name="lvl"]')) r.addEventListener("change", () => { state.lvl = r.value; refresh(); });
  $("task-form").addEventListener("submit", async (e) => {
    e.preventDefault();
    const f = new FormData(e.target);
    const body = { pickup: f.get("pickup"), dropoff: f.get("dropoff"), priority: +f.get("priority"), count: +f.get("count") || 1 };
    if (f.get("mode") === "direct") {
      body.assign_to = $("f-robot").value;
      if (!body.assign_to) { $("form-msg").className = "form-msg err"; $("form-msg").textContent = "Choose a robot first."; return; }
    }
    const msg = $("form-msg");
    try {
      const r = await api("/api/tasks", body);
      msg.className = "form-msg ok";
      msg.textContent = `Released ${r.created.join(", ")}${body.assign_to ? ` to ${body.assign_to}` : " to the fleet auction"}.`;
    } catch (err) { msg.className = "form-msg err"; msg.textContent = `Not released: ${err.message}`; }
  });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") { setPlacing(false); if (state.selected) { state.selected = null; refresh(); } } });
  connect();
}
main().catch((e) => { setLink("Console error", "bad"); console.error(e); toast(`The console failed to start: ${e.message}`, true); });
