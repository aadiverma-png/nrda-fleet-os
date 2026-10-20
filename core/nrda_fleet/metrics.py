"""Fleet KPIs from the peer-bus traffic: one ledger for the web console, the 2-D benchmark and Gazebo runs.

:class:`FleetLedger` listens to the same messages every robot sees (tasks,
task events, robot states, bids, safety events) and derives:

* task table with timings (release -> assigned -> picked -> completed);
* throughput (tasks/hour), makespan, task latency mean / p50 / p95, waiting;
* utilisation (share of robots busy), charging share, messages;
* safety counters (collisions robot-robot / robot-static, near-misses, avoided).

Metric definitions are the standard warehouse-AMR fleet set (throughput,
makespan, completion rate) plus the NRDA safety and edge-compute metrics.
"""

from __future__ import annotations

import json
import math
import statistics
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, field

from .bus import AlertMsg, BidMsg, RobotStateMsg, SafetyEventMsg, TaskEventMsg, TaskMsg

BUSY = ("to_pickup", "loading", "to_drop", "unloading")
MOVING_MODES = ("to_pickup", "to_drop", "to_charger", "to_park")


def _rid_key(rid: str):
    tail = rid.rsplit("_", 1)[-1]
    return (0, int(tail), rid) if tail.isdigit() else (1, 0, rid)


def _has_cycle(g: dict) -> bool:
    state: dict = {}

    def visit(u) -> bool:
        state[u] = 1
        for v in g.get(u, ()):
            if state.get(v) == 1 or (state.get(v) is None and visit(v)):
                return True
        state[u] = 2
        return False
    return any(state.get(u) is None and visit(u) for u in g)


def _loop(live: dict, rs: list) -> list:
    """One wait loop among ``rs`` as a readable chain (a -> b -> ... -> a)."""
    comp = set(rs)
    for start in rs:
        path, r = [start], start
        for _ in range(len(rs) + 1):
            nxt = next((b for b in live[r].waiting_for if b in comp), None)
            if nxt is None:
                break
            if nxt in path:
                return path[path.index(nxt):] + [nxt]
            path.append(nxt)
            r = nxt
    return rs


@dataclass
class TaskRecord:
    task_id: str
    pickup: str
    dropoff: str
    priority: int
    release: float
    deadline: float = 0.0
    assign_to: str = ""
    status: str = "queued"
    robot: str = ""
    t_assigned: float = 0.0
    t_picked: float = 0.0
    t_done: float = 0.0
    bidders: dict = field(default_factory=dict)  # robot -> bid (latest)
    winner: str = ""
    winning_bid: float = 0.0

    def as_dict(self) -> dict:
        d = dict(self.__dict__)
        d["latency"] = round(self.t_done - self.release, 2) if self.t_done else None
        return d


class FleetLedger:
    def __init__(self):
        self.tasks: dict[str, TaskRecord] = {}
        self.robots: dict[str, RobotStateMsg] = {}
        self.safety: list[SafetyEventMsg] = []
        self.safety_counts: Counter = Counter()
        self.collision_kind: Counter = Counter()
        self.per_robot: dict[str, Counter] = defaultdict(Counter)
        self.acked: set[str] = set()
        self.messages = Counter()
        self.t = 0.0
        self.t0: float | None = None
        self.busy_time: dict[str, float] = defaultdict(float)
        self.charge_time: dict[str, float] = defaultdict(float)
        self._last_state_t: dict[str, float] = {}
        self.timeline: deque = deque(maxlen=720)  # (t, done, collisions, near, avoided, busy)
        self._next_sample = 0.0
        self.events_log: deque = deque(maxlen=300)
        self.drp: Counter = Counter()  # DRP: plans, episodes (distinct deadlocks), resolved, escalated, timeouts
        self.standoff: Counter = Counter()  # L2b in-aisle chain reverse: plans, robots asked to back off
        self.dead_chains: Counter = Counter()  # unsatisfiable wait chains given up on, by why they were dead
        self.drp_open: dict[str, str] = {}  # epoch -> kind
        self.standoff_open: dict[str, str] = {}  # L2b epoch -> the bypasser that asked
        # console "Deadlocks and collisions": every incident with its outcome, and the live risk picture
        self.incidents: list[dict] = []
        self._inc_seq = 0
        self._wait_since: dict[str, float] = {}  # robot -> start of its current wait

    # ------------------------------------------------------------------ ingest

    def on_message(self, msg, now: float | None = None) -> None:
        if now is not None:
            self.t = max(self.t, now)
        self.messages[type(msg).__name__] += 1
        if isinstance(msg, TaskMsg):
            self.t = max(self.t, msg.stamp)
            if msg.cancelled:
                if msg.task_id in self.tasks and self.tasks[msg.task_id].status in ("queued", "assigned"):
                    self.tasks[msg.task_id].status = "cancelled"
                return
            if msg.task_id not in self.tasks:
                self.tasks[msg.task_id] = TaskRecord(msg.task_id, msg.pickup, msg.dropoff, msg.priority, msg.stamp,
                                                     msg.deadline, msg.assign_to)
                self._log(msg.stamp, f"task {msg.task_id} released: {msg.pickup} -> {msg.dropoff}"
                          + (f" (direct: {msg.assign_to})" if msg.assign_to else ""))
        elif isinstance(msg, TaskEventMsg):
            self.t = max(self.t, msg.stamp)
            rec = self.tasks.get(msg.task_id)
            if rec is None:
                return
            if msg.status == "in_progress":
                rec.status, rec.robot, rec.t_assigned = "in_progress", msg.robot_id, msg.stamp
            elif msg.status == "picked":
                rec.status, rec.t_picked = "picked", msg.stamp
            elif msg.status == "completed":
                rec.status, rec.t_done = "completed", msg.stamp
            elif msg.status in ("failed", "released"):
                rec.status, rec.robot = "queued", ""
            self._log(msg.stamp, f"{msg.robot_id}: {msg.task_id} {msg.status}")
        elif isinstance(msg, RobotStateMsg):
            self.t = max(self.t, msg.stamp)
            if self.t0 is None:
                self.t0 = msg.stamp
            last = self._last_state_t.get(msg.robot_id)
            if last is not None:
                dt = min(max(msg.stamp - last, 0.0), 1.0)
                if msg.mode in BUSY:
                    self.busy_time[msg.robot_id] += dt
                elif msg.mode == "charging":
                    self.charge_time[msg.robot_id] += dt
            self._last_state_t[msg.robot_id] = msg.stamp
            self.robots[msg.robot_id] = msg
            if msg.waiting_for and abs(msg.v) < 0.05:
                self._wait_since.setdefault(msg.robot_id, msg.stamp)
            else:
                self._wait_since.pop(msg.robot_id, None)
            self._settle_incidents()
        elif isinstance(msg, BidMsg):
            for tid, winner in msg.winning_robots.items():
                rec = self.tasks.get(tid)
                if rec is None or rec.status not in ("queued", "assigned"):
                    continue
                if msg.robot_id in msg.bundle or winner == msg.robot_id:
                    rec.bidders[msg.robot_id] = round(msg.winning_bids.get(tid, 0.0), 2)
                rec.winner = winner
                rec.winning_bid = round(msg.winning_bids.get(tid, 0.0), 2)
                if winner:
                    rec.status = "assigned" if rec.status == "queued" else rec.status
        elif isinstance(msg, SafetyEventMsg):
            self.add_safety(msg)
        elif isinstance(msg, AlertMsg) and msg.kind.startswith("drp_"):
            self._on_drp(msg)
        elif isinstance(msg, AlertMsg) and msg.kind == "standoff_reverse":
            self._on_standoff(msg)
        elif isinstance(msg, AlertMsg) and msg.kind == "dead_chain":
            # a robot gave up on a wait chain that ended at a parked, idle or silent robot. Counted
            # because it is a recovery, not a success: a run full of these is one where the fleet keeps
            # queueing behind robots that have stopped for good.
            self.dead_chains["total"] += 1
            self.dead_chains[msg.detail or "unknown"] += 1
        self._sample()

    def add_safety(self, ev: SafetyEventMsg) -> None:
        if any(e.event_id == ev.event_id for e in self.safety[-50:]):
            return
        self.safety.append(ev)
        self.safety_counts[ev.level] += 1
        if ev.level == "COLLISION":
            kind = "robot-robot" if ev.other.startswith("AMR_") else (
                "obstacle" if ev.other.startswith(("obs", "box_")) else "robot-static")
            self.collision_kind[kind] += 1
        for r in ev.robots:
            self.per_robot[r][ev.level] += 1
        if ev.level in ("COLLISION", "NEAR_MISS"):
            at = {r: (self.robots[r].x, self.robots[r].y) for r in ev.robots if r in self.robots}
            self._incident("collision" if ev.level == "COLLISION" else "near_miss", ev.stamp, list(ev.robots), ev.x,
                           ev.y, ev.other if ev.other not in ev.robots else "", key=ev.event_id, at=at,
                           status="open" if ev.level == "COLLISION" else "solved",
                           how="" if ev.level == "COLLISION" else "the robots braked in time")
        self._log(ev.stamp, f"{ev.level}: {' / '.join(ev.robots)} {ev.other} {ev.detail}".strip())

    def _on_standoff(self, m: AlertMsg) -> None:
        """L2b chain reverse: one plan per broadcast, with the robots it asks to back off."""
        try:
            plan = json.loads(m.detail)
        except (ValueError, TypeError):
            return
        epoch = plan.get("epoch", "")
        if epoch in self.standoff_open:
            return  # gossip repeat of the same plan
        self.standoff_open[epoch] = m.sender
        self.standoff["plans"] += 1
        self.standoff["movers"] += len(plan.get("moves", {}))
        if int(plan.get("attempt", 1)) == 1:
            self.standoff["episodes"] += 1  # a new standoff, not another attempt at an open one
        self._log(m.stamp, f"STANDOFF {m.sender} asks {', '.join(sorted(plan.get('moves', {})))} to back off")

    def _on_drp(self, m: AlertMsg) -> None:
        if m.kind == "drp_plan":
            try:
                import json
                d = json.loads(m.detail)
                ep, kind, rung, members = d["epoch"], d["kind"], d["rung"], d["members"]
            except Exception:  # noqa: BLE001
                return
            if ep in self.drp_open:
                return
            self.drp_open[ep] = kind
            self.drp["plans"] += 1
            inc = next((i for i in self.incidents if i["kind"] == "deadlock" and i["status"] != "solved"
                        and set(i["robots"]) & set(members)), None)
            if inc is None:
                inc = self._incident("deadlock", m.stamp, list(members), m.x, m.y, key=ep)
                self.drp["episodes"] += 1  # a new physical deadlock, not another rung on an open one
            inc.update(type=kind, rung=rung, key=ep, priority=m.target,
                       robots=sorted(set(inc["robots"]) | set(members), key=_rid_key))
            self._log(m.stamp, f"DEADLOCK {kind} (rung {rung}) {', '.join(members)}: coordinated by {m.sender}, "
                               f"priority {m.target}")
        elif m.kind == "drp_done":
            kind = self.drp_open.pop(m.target, None)
            if kind is not None and m.detail == "timeout":
                self.drp["timeouts"] += 1  # the rung expired: the incident rightly stays open, the attempt counts
                self._log(m.stamp, f"deadlock {kind} rung timed out")
            elif kind is not None:
                self.drp["resolved"] += 1
                self._log(m.stamp, f"deadlock {kind} resolved")
                for inc in self.incidents:
                    if inc["kind"] == "deadlock" and inc["key"] == m.target and inc["status"] != "solved":
                        self._solve(inc, m.stamp, f"{inc.get('priority') or 'the priority robot'} got through "
                                                  f"(rung {inc.get('rung', 1)})")
        elif m.kind == "drp_escalated":
            self.drp["escalated"] += 1
            for inc in self.incidents:
                if inc["kind"] == "deadlock" and inc["status"] != "solved" and m.target in inc["robots"]:
                    inc["status"] = "needs_you"
            self._log(m.stamp, f"DEADLOCK ESCALATED to operator: {m.detail} near ({m.x:.1f}, {m.y:.1f}), "
                               f"priority {m.target}")

    # ------------------------------------------------------------------ incidents and live risk

    def _incident(self, kind: str, t: float, robots: list, x: float, y: float, other: str = "", key: str = "",
                  at: dict | None = None, status: str = "open", how: str = "") -> dict:
        self._inc_seq += 1
        inc = dict(id=f"I{self._inc_seq:04d}", kind=kind, type="", t=round(t, 2), robots=sorted(robots, key=_rid_key),
                   x=round(x, 2), y=round(y, 2), other=other, key=key, status=status,
                   t_solved=round(t, 2) if status == "solved" else None, how=how, rung=0, priority="")
        inc["_at"] = at or {}
        self.incidents.append(inc)
        if len(self.incidents) > 400:
            self.incidents = [i for i in self.incidents if i["status"] != "solved"][-200:] + \
                [i for i in self.incidents if i["status"] == "solved"][-150:]
            self.incidents.sort(key=lambda i: i["t"])
        return inc

    def _solve(self, inc: dict, t: float, how: str) -> None:
        inc["status"], inc["t_solved"], inc["how"] = "solved", round(t, 2), how

    def _settle_incidents(self) -> None:
        """Close incidents whose robots are free again (a collision pair driven apart, a jam that moved)."""
        for inc in self.incidents:
            if inc["status"] == "solved" or self.t - inc["t"] < 3.0:
                continue
            rs = [self.robots[r] for r in inc["robots"] if r in self.robots]
            if not rs:
                continue
            if inc["kind"] == "collision":
                moved = all(math.hypot(s.x - inc["_at"].get(s.robot_id, (s.x, s.y))[0],
                                       s.y - inc["_at"].get(s.robot_id, (s.x, s.y))[1]) > 0.5 for s in rs)
                apart = len(rs) >= 2 and math.hypot(rs[0].x - rs[1].x, rs[0].y - rs[1].y) > 1.6
                if (moved or apart) and all(s.mode not in ("paused", "estop", "fault") for s in rs):
                    self._solve(inc, self.t, "recovered: the robots separated and drove on")
            elif inc["kind"] == "deadlock":
                if all(not s.waiting_for and (abs(s.v) > 0.05 or s.mode not in MOVING_MODES) for s in rs):
                    self._solve(inc, self.t, "cleared: every robot in it is moving again")

    def risk(self) -> list[dict]:
        """Live hazards for the console, most likely first.

        Deadlock: robots joined by "waiting for" links. A closed loop is a deadlock (100 %); otherwise the
        probability grows with the longest standing wait T and the chain length n:
        P = (1 - exp(-T / 25 s)) * min(1, 0.5 + 0.25 (n - 1)).
        Collision: every pair within 6 m, both extrapolated at constant velocity for 3 s; closest approach
        d at time t*, margin m = d - 0.9 m (two bodies side by side), uncertainty s = 0.12 + 0.1 t* m:
        P = Phi(-m / s), the same risk model the robots use on board.
        """
        out = []
        live = {rid: s for rid, s in self.robots.items() if self.t - s.stamp < 3.0}
        # --- deadlocks: connected components of the wait-for graph
        adj = defaultdict(set)
        for rid, s in live.items():
            for b in s.waiting_for:
                if b in live:
                    adj[rid].add(b)
                    adj[b].add(rid)
        seen = set()
        for start in list(adj):
            if start in seen:
                continue
            comp, stack = set(), [start]
            while stack:
                r = stack.pop()
                if r in comp:
                    continue
                comp.add(r)
                stack.extend(adj[r] - comp)
            seen |= comp
            waits = [self.t - self._wait_since[r] for r in comp if r in self._wait_since]
            if not waits:
                continue
            T, n = max(waits), len(comp)
            cyc = _has_cycle({r: [b for b in live[r].waiting_for if b in comp] for r in comp})
            p = 1.0 if cyc and T > 2.0 else (1.0 - math.exp(-T / 25.0)) * min(1.0, 0.5 + 0.25 * (n - 1))
            if p < 0.1:
                continue
            rs = sorted(comp, key=_rid_key)
            longest = max((r for r in comp if r in self._wait_since), key=lambda r: self.t - self._wait_since[r])
            why = (f"closed wait loop: {' -> '.join(_loop(live, rs))}" if cyc else
                   f"{longest} has waited {T:.0f} s for {', '.join(live[longest].waiting_for) or 'a peer'}")
            out.append(dict(kind="deadlock" if cyc and T > 2.0 else "deadlock_risk", p=round(p, 3), robots=rs,
                            x=round(sum(live[r].x for r in rs) / n, 2), y=round(sum(live[r].y for r in rs) / n, 2),
                            why=why))
        # --- collisions: closest approach of every nearby pair
        ids = sorted(live, key=_rid_key)
        for i, a in enumerate(ids):
            A = live[a]
            for b in ids[i + 1:]:
                B = live[b]
                dx, dy = B.x - A.x, B.y - A.y
                if dx * dx + dy * dy > 36.0 or (abs(A.v) < 0.05 and abs(B.v) < 0.05):
                    continue
                vx = B.v * math.cos(B.th) - A.v * math.cos(A.th)
                vy = B.v * math.sin(B.th) - A.v * math.sin(A.th)
                vv = vx * vx + vy * vy
                ts = 0.0 if vv < 1e-9 else min(max(-(dx * vx + dy * vy) / vv, 0.0), 3.0)
                d = math.hypot(dx + vx * ts, dy + vy * ts)
                sig = 0.12 + 0.1 * ts
                p = 0.5 * math.erfc(((d - 0.9) / sig) / math.sqrt(2.0))
                if p < 0.05:
                    continue
                out.append(dict(kind="collision_risk", p=round(p, 3), robots=[a, b],
                                x=round((A.x + B.x) / 2, 2), y=round((A.y + B.y) / 2, 2),
                                why=f"{d:.1f} m apart in {ts:.1f} s at {math.hypot(vx, vy):.1f} m/s closing speed"))
        out.sort(key=lambda r: -r["p"])
        return out[:12]

    def incidents_view(self, n: int = 60) -> list[dict]:
        return [{k: v for k, v in i.items() if not k.startswith("_")} for i in self.incidents[-n:]]

    def _log(self, t: float, text: str) -> None:
        self.events_log.append((round(t, 2), text))

    def _sample(self) -> None:
        if self.t < self._next_sample:
            return
        self._next_sample = self.t + 5.0
        done = sum(1 for r in self.tasks.values() if r.status == "completed")
        busy = sum(1 for s in self.robots.values() if s.mode in BUSY)
        self.timeline.append((round(self.t, 1), done, self.safety_counts["COLLISION"],
                              self.safety_counts["NEAR_MISS"], self.safety_counts["AVOIDED"], busy))

    # ------------------------------------------------------------------ KPIs

    def kpis(self) -> dict:
        recs = list(self.tasks.values())
        done = [r for r in recs if r.status == "completed"]
        lat = sorted(r.t_done - r.release for r in done)
        waitq = sorted(r.t_assigned - r.release for r in recs if r.t_assigned)
        dur = max(self.t - (self.t0 or 0.0), 1e-6)
        n_rob = max(len(self.robots), 1)
        on_time = [r for r in done if r.deadline and r.t_done <= r.deadline]
        with_deadline = [r for r in done if r.deadline]
        return {
            "sim_time": round(self.t, 1),
            "robots": len(self.robots),
            "tasks_total": len(recs),
            "tasks_completed": len(done),
            "tasks_in_progress": sum(1 for r in recs if r.status in ("in_progress", "picked")),
            "tasks_queued": sum(1 for r in recs if r.status in ("queued", "assigned")),
            "completion_rate_pct": round(100.0 * len(done) / len(recs), 1) if recs else 0.0,
            "throughput_per_hour": round(len(done) / dur * 3600.0, 1),
            "makespan_s": round(max((r.t_done for r in done), default=0.0) - min((r.release for r in recs), default=0.0), 1)
            if done else None,
            "latency_mean_s": round(statistics.mean(lat), 1) if lat else None,
            "latency_p50_s": round(_pct(lat, 0.5), 1) if lat else None,
            "latency_p95_s": round(_pct(lat, 0.95), 1) if lat else None,
            "assign_wait_mean_s": round(statistics.mean(waitq), 1) if waitq else None,
            "deadline_compliance_pct": round(100.0 * len(on_time) / len(with_deadline), 1) if with_deadline else None,
            "utilisation_pct": round(100.0 * sum(self.busy_time.values()) / (dur * n_rob), 1),
            "charging_pct": round(100.0 * sum(self.charge_time.values()) / (dur * n_rob), 1),
            "collisions": self.safety_counts["COLLISION"],
            "collisions_robot_robot": self.collision_kind["robot-robot"],
            "collisions_robot_static": self.collision_kind["robot-static"],
            "collisions_obstacle": self.collision_kind["obstacle"],
            "near_misses": self.safety_counts["NEAR_MISS"],
            "deadlocks": self.drp["plans"],
            "deadlock_episodes": self.drp["episodes"],
            "deadlocks_resolved": self.drp["resolved"],
            "deadlocks_escalated": self.drp["escalated"],
            "deadlocks_timed_out": self.drp["timeouts"],
            "deadlocks_unaccounted": max(0, self.drp["plans"] - self.drp["resolved"] - self.drp["escalated"]
                                         - self.drp["timeouts"]),
            "standoffs": self.standoff["episodes"],
            "standoff_plans": self.standoff["plans"],
            "standoff_movers": self.standoff["movers"],
            "dead_chains": self.dead_chains["total"],
            "incidents_open": sum(1 for i in self.incidents if i["status"] != "solved"),
            "incidents_solved": sum(1 for i in self.incidents if i["status"] == "solved"),
            "avoided": self.safety_counts["AVOIDED"],
            "compute_ms_p95": round(max((s.compute_ms for s in self.robots.values()), default=0.0), 2),
            "messages": dict(self.messages),
            "modes": dict(Counter(s.mode for s in self.robots.values())),
        }


def _pct(xs: list[float], q: float) -> float:
    if not xs:
        return math.nan
    k = (len(xs) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)
