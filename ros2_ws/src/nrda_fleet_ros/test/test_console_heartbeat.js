/* The console's heartbeat, run against the real app.js.   node test/test_console_heartbeat.js
 *
 * The console used to decide a robot was reporting by checking whether its x/y/th/mode/battery/
 * compute_ms had CHANGED since the last snapshot. A robot standing still - deadlocked, parked,
 * queued at a station - reports just as often as a moving one, so half a stationary fleet read as
 * silent ("Reporting 6/20" with every robot on the map). The gateway now sends the real age and the
 * console reads it. No test framework: the console has no build step and no libraries, nor does this.
 */
const fs = require("fs");
const path = require("path");
const src = fs.readFileSync(path.join(__dirname, "..", "web", "app.js"), "utf8");
const start = src.indexOf("/* ------------------------------------------------------------------ heartbeat");
const end = src.indexOf("\n", src.indexOf("const hbLevel ="));
if (start < 0 || end < 0) { console.error("could not locate the heartbeat block"); process.exit(1); }
const block = src.slice(start, end);

const state = { hb: {}, tick: 0 };
const { trackHeartbeat, hbLevel } = new Function("state", block + "\nreturn { trackHeartbeat, hbLevel };")(state);

let fail = 0;
const check = (name, got, want) => {
  const ok = JSON.stringify(got) === JSON.stringify(want);
  if (!ok) fail++;
  console.log(`${ok ? "PASS" : "FAIL"}  ${name}${ok ? "" : `  got ${JSON.stringify(got)} want ${JSON.stringify(want)}`}`);
};
const lvls = () => Object.fromEntries(Object.entries(state.hb).map(([k, b]) => [k, hbLevel(b)]));

// A deadlocked fleet: identical x/y/th/mode/battery/compute_ms every snapshot, reporting fine (age 0.05).
// This is the case the old signature-diff heartbeat got wrong.
const frozen = (id) => ({ id, x: -30.93, y: -11, th: 0, mode: "to_drop", battery: 1, compute_ms: 1.5, age: 0.05, stale: false });
for (let i = 0; i < 12; i++) trackHeartbeat({ robots: [frozen("AMR_4"), frozen("AMR_6"), frozen("AMR_19")] });
check("stationary robots that still report are live", lvls(), { AMR_4: "live", AMR_6: "live", AMR_19: "live" });
check("  and their strips are solid bars", state.hb.AMR_6.beats.every((b) => b === 1), true);

// A robot that really went quiet: it keeps the same values AND its age climbs.
trackHeartbeat({ robots: [{ ...frozen("AMR_6"), age: 2.0 }] });
check("age 2.0 s reads as late", hbLevel(state.hb.AMR_6), "late");
trackHeartbeat({ robots: [{ ...frozen("AMR_6"), age: 4.0 }] });
check("age 4.0 s reads as silent", hbLevel(state.hb.AMR_6), "silent");
check("  and those two snapshots are gaps", state.hb.AMR_6.beats.slice(-2), [0, 0]);

// The gateway's own stale flag still wins outright.
trackHeartbeat({ robots: [{ ...frozen("AMR_6"), age: 0.05, stale: true }] });
check("gateway stale flag overrides a fresh age", hbLevel(state.hb.AMR_6), "silent");

// A moving robot is live, exactly as before.
trackHeartbeat({ robots: [{ ...frozen("AMR_6"), x: -30.0, age: 0.05, stale: false }] });
check("a moving robot is live", hbLevel(state.hb.AMR_6), "live");

// An old gateway that does not send `age` must not mark the whole fleet silent.
state.hb = {};
trackHeartbeat({ robots: [{ id: "AMR_1", x: 0, y: 0, th: 0, mode: "idle", battery: 1, compute_ms: 0, stale: false }] });
check("missing age field degrades to live, not silent", hbLevel(state.hb.AMR_1), "live");

// Robots that leave the fleet are forgotten.
trackHeartbeat({ robots: [] });
check("departed robots are dropped", Object.keys(state.hb), []);

// The strip never grows past HB_LEN.
for (let i = 0; i < 100; i++) trackHeartbeat({ robots: [frozen("AMR_2")] });
check("strip is capped at HB_LEN", state.hb.AMR_2.beats.length, 40);

console.log(fail ? `\n${fail} FAILED` : "\nall heartbeat checks passed");
process.exit(fail ? 1 : 0);
