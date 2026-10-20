"""Deadlock Resolution Protocol (DRP): coordinated, rule-based recovery from fleet deadlocks.

Every robot runs the same code on the same gossiped data, so every replica reaches the same verdict:

1. **Detect**: a wait-for cycle that persists (:mod:`nrda_fleet.waitgraph`) or a *stall* (a robot in a
   driving mode that has not advanced 0.3 m in 20 s). The group is the cycle plus stalled robots within
   3 m, merged with union-find (``visison.standoff.conflict_groups``).
2. **Elect**: the coordinator is the lowest AMR number in the group; the next one takes over after 3 s of
   silence per rank. The robot asked to give way is elected from the wait-for *cycle* inside the group
   (:func:`cycle_members`), never from a robot merged in only by the 3 m proximity rule: a robot off the
   cycle cannot break it, so asking it to move just burns every rung.
3. **Classify** the deadlock and apply the rule of its type (D1 queue ... D6 general cycle, see
   :func:`classify`).
4. **Plan**: who backs off how far, in which order (chain reverse in the world frame:
   ``req_1 = F + g``, ``rev_i = max(0, req_i + h_i - u_i)``, ``req_{i+1} = u_i + rev_i + h_i + g``), which robot
   has priority, and the **frozen zone** everybody else must keep out of.
5. The plan is broadcast as ``AlertMsg(kind="drp_plan")``; members execute their own part through their own
   arbiter (no remote actuation), non-members treat the zone as blocked, the coordinator closes the
   incident (``drp_done``) and the priority robot keeps a 15 s precedence token on the zone.
6. **Escalate** after 20 s without progress: next rule, then re-route, then operator alert.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field

from visison.standoff import conflict_groups

from .geom import OrientedBox, boxes_overlap, wrap
from .ids import id_key

GAP = 0.3  # m, free gap left between robots after a back-off (min_gap + space margin)
MAX_REVERSE = 3.0  # m, the most any robot is asked to back off
STEP = 0.2  # m, back-off search resolution
CHAIN_DEPTH = 3  # robots pushed behind a yielder at most
CORRIDOR = 2.6  # m of the priority robot's path kept free (> the 2.4 m two-lane crossing band, so the
# priority robot is never granted a corridor too short to clear an intersection it is standing in)
FREEZE_S = 20.0  # s, frozen zone lifetime (renewed by a new rung)
RUNG_S = 20.0  # s without progress before escalating. The mechanism alone needs ~6.5 s (3.0 m back-off at
# 0.6 m/s = 5.0 s, plus decel, re-plan and gossip), but `_back_off` returns False and retries when it finds
# no clear space, and the robot must finish its current leg first. Measured 2026-10-18: at 12.0 s, 16 of 19
# plans timed out and fleet-side resolution fell from 54 % to 11 %.
TOKEN_S = 15.0  # s, anti-relapse precedence of the priority robot
STALL_S = 20.0  # s, no-progress watchdog. At 12.0 s legitimate scheduled waits were declared stalled and
# opened DRP episodes that were not needed (19 plans for 6 distinct deadlocks).
STALL_M = 0.3  # m


@dataclass
class Member:
    rid: str
    x: float
    y: float
    th: float
    v: float
    priority: int
    lane: str = ""  # "LINE/LANE" the robot drives in, "" off-lane
    in_core: bool = False  # body inside a crossing core
    in_station: bool = False  # docked / inside a station
    bypassing: bool = False
    waiting_for: tuple[str, ...] = ()
    path: tuple[tuple[float, float], ...] = ()  # upcoming route points
    single_file: bool = False

    @property
    def key(self) -> tuple:
        return (-self.priority, id_key(self.rid))


def is_single_file_member(m: Member) -> bool:
    if getattr(m, "single_file", False):
        return True
    line = m.lane.split("/")[0] if "/" in m.lane else m.lane
    if line == "VE":
        return True
    if line.startswith("V") and len(line) == 3 and line[1:].isdigit():
        return True
    return False


@dataclass
class Move:
    rid: str
    action: str  # backoff | hold | reroute | pass
    dist: float = 0.0  # m, backoff only (always reversing along the robot's heading)
    order: int = 0  # 0 executes first


@dataclass
class DRPPlan:
    epoch: str
    coordinator: str
    kind: str  # D1 ... D6
    priority: str
    moves: list[Move]
    zone: list[tuple[float, float, float, float, float]]  # frozen boxes (cx, cy, L, W, yaw)
    members: list[str]
    rung: int
    t_plan: float
    expires: float
    reason: str = ""

    def to_json(self) -> str:
        d = asdict(self)
        return json.dumps(d, separators=(",", ":"))

    @classmethod
    def from_json(cls, s: str) -> "DRPPlan":
        d = json.loads(s)
        d["moves"] = [Move(**m) for m in d["moves"]]
        d["zone"] = [tuple(z) for z in d["zone"]]
        return cls(**d)

    def move_of(self, rid: str) -> Move | None:
        return next((m for m in self.moves if m.rid == rid), None)


# ------------------------------------------------------------------ grouping and classification


HOLD_OTHERS = True  # non-acting members hold (drop bookings, stand) while the priority robot passes


def deadlock_group(cycles: list[list[str]], stalled: list[str], members: dict[str, Member]) -> list[set[str]]:
    """Cycle members plus stalled robots within 3 m of each other, merged (union-find)."""
    pairs: list[tuple[str, str]] = []
    for cyc in cycles:
        for a, b in zip(cyc, cyc[1:] + cyc[:1]):
            pairs.append((a, b))
    involved = {r for c in cycles for r in c} | set(stalled)
    for a in involved:
        for b in involved:
            if a < b and a in members and b in members:
                ma, mb = members[a], members[b]
                if math.hypot(ma.x - mb.x, ma.y - mb.y) < 3.0:
                    pairs.append((a, b))
    for r in stalled:
        pairs.append((r, r))
    return conflict_groups(pairs)


def classify(group: list[Member], static_blocked: bool) -> tuple[str, str]:
    """(type, why). The first matching rule wins, so the most constrained situation decides."""
    lanes = {m.lane for m in group}
    if any(m.in_station for m in group):
        return "D4", "a member is in a station or its doorway: the occupant exits first"
    if any(m.in_core for m in group):
        return "D3", "a member is inside a crossing: it has right of way, the others back off to the stop line"
    if static_blocked:
        return "D5", "an unmapped obstacle is part of the wait: the robot facing it re-routes"
    if any(m.bypassing for m in group):
        return "D2", "a member is borrowing the oncoming lane: the robots in that lane back off (chain reverse)"
    if len(lanes) == 1 and "" not in lanes:
        heads = {round(math.degrees(m.th) / 90.0) % 4 for m in group}
        if len(heads) == 1:
            return "D1", "one lane, one direction: the leader goes first, followers hold"
        return "D2", "head-on in one line: the robots facing the priority robot back off (chain reverse)"
    lines = {ln.split("/")[0] for ln in lanes if "/" in ln}
    opp_lines = [ln for ln in lines if f"{ln}/A" in lanes and f"{ln}/B" in lanes]
    if opp_lines:
        return "D2", "opposite lanes of one line: chain reverse"
    if any(m1.lane == m2.lane and m1.lane and abs(wrap(m1.th - m2.th)) > 2.0 for m1 in group for m2 in group):
        return "D2", "head-on in one line: the robots facing the priority robot back off (chain reverse)"
    if len({ln.split("/")[0] for ln in lanes if ln}) == 1 and len(lanes) == 2:
        return "D2", "opposite lanes of one line: chain reverse"
    return "D6", "general wait-for cycle: the movable lowest-priority robot and the robots behind it back off"


# ------------------------------------------------------------------ geometry helpers


def _body(x: float, y: float, th: float, L: float, W: float, pad: float = 0.0) -> OrientedBox:
    return OrientedBox(x, y, L + 2 * pad, W + 2 * pad, th)


def corridor(m: Member, L: float, W: float, length: float = CORRIDOR) -> list[OrientedBox]:
    """Boxes covering the next ``length`` m of a robot's path (its body sweep), from its current pose."""
    pts = [(m.x, m.y)] + [p for p in m.path]
    if len(pts) < 2:  # no path known: straight ahead
        pts = [(m.x, m.y), (m.x + length * math.cos(m.th), m.y + length * math.sin(m.th))]
    out, left = [_body(m.x, m.y, m.th, L, W, 0.1)], length
    r_turn = math.hypot(L, W) / 2 + 0.1
    d_turn = 2 * r_turn
    prev_ang = m.th
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-3:
            continue
        seg = min(d, left)
        ang = math.atan2(y1 - y0, x1 - x0)
        if abs(wrap(ang - prev_ang)) > 0.1:
            out.append(OrientedBox(x0, y0, d_turn, d_turn, 0.0))
        cx, cy = x0 + 0.5 * seg * math.cos(ang), y0 + 0.5 * seg * math.sin(ang)
        out.append(OrientedBox(cx, cy, seg + L, W + 0.2, ang))
        prev_ang = ang
        left -= seg
        if left <= 0:
            break
    return out


def _hits(box: OrientedBox, boxes: list[OrientedBox]) -> bool:
    return any(boxes_overlap(box, b) for b in boxes)


# ------------------------------------------------------------------ planning


def cycle_members(group: list[Member]) -> list[Member]:
    """Members that lie on a wait-for cycle inside the group.

    Only a robot on the cycle can break it; a robot merely merged in by the 3 m
    proximity rule in :func:`deadlock_group` cannot, however low its precedence.
    """
    inside = {m.rid for m in group}
    edges = {m.rid: tuple(w for w in m.waiting_for if w in inside) for m in group}
    out: list[Member] = []
    for m in group:  # tiny groups (2-8): a plain reachability test back to self is enough
        seen: set[str] = set()
        stack = list(edges.get(m.rid, ()))
        while stack:
            r = stack.pop()
            if r == m.rid:  # a cycle of length >= 2, or a self-loop
                out.append(m)
                break
            if r in seen:
                continue
            seen.add(r)
            stack.extend(edges.get(r, ()))
    return out


def _yielder(group: list[Member], movable: list[Member]) -> Member:
    """The robot asked to give way: lowest precedence ON THE CYCLE, else lowest in the group."""
    cyc = cycle_members(group)
    if not cyc:  # no cycle known (stall-only group): the old group-wide rule
        return max(movable or group, key=lambda m: m.key)
    can_move = {m.rid for m in movable}
    on_cycle = [m for m in cyc if m.rid in can_move]
    return max(on_cycle or cyc, key=lambda m: m.key)


def plan_backoffs(yielders: list[Member], keep_clear: list[OrientedBox], everyone: dict[str, Member],
                  static_hit, L: float, W: float, exclude: set[str]) -> tuple[list[Move], list[OrientedBox], str]:
    """Chain reverse in the world frame.

    Each yielder backs off (reversing along its heading) in 0.2 m steps until its body, grown by the gap,
    is clear of ``keep_clear``. A robot standing in that back-off path is pushed too, far enough to clear the
    yielder's new body (up to ``CHAIN_DEPTH`` robots, ``MAX_REVERSE`` m each). Returns the moves (deepest in the
    chain first), the swept boxes to freeze, and "" or the reason it is infeasible.
    """
    moves: list[Move] = []
    swept: list[OrientedBox] = []
    busy = set(exclude)

    def push(m: Member, clear_of: list[OrientedBox], depth: int) -> str:
        if m.in_station:
            return f"{m.rid} is in a station and cannot back off"
        c, s = math.cos(m.th), math.sin(m.th)
        d = 0.0
        while _hits(_body(m.x - d * c, m.y - d * s, m.th, L, W, GAP / 2), clear_of):
            d += STEP
            if d > MAX_REVERSE + 1e-9:
                return f"{m.rid} would have to back off more than {MAX_REVERSE:.1f} m"
        if d <= 1e-9:
            return ""
        path_boxes = []
        for k in range(1, int(math.ceil(d / STEP)) + 1):
            f = min(k * STEP, d)
            px, py = m.x - f * c, m.y - f * s
            if static_hit(px, py, m.th):
                return f"{m.rid} cannot back off: static obstacle behind it"
            path_boxes.append(_body(px, py, m.th, L, W, 0.05))
        target = _body(m.x - d * c, m.y - d * s, m.th, L, W, GAP / 2)
        # robots in the back-off path are pushed first (chain reverse)
        for other in sorted(everyone.values(), key=lambda o: math.hypot(o.x - m.x, o.y - m.y)):
            if other.rid in busy or other.rid == m.rid:
                continue
            ob = _body(other.x, other.y, other.th, L, W, 0.0)
            if _hits(ob, path_boxes + [target]):
                if depth >= CHAIN_DEPTH:
                    return f"chain too long behind {m.rid}"
                busy.add(other.rid)
                why = push(other, path_boxes + [target], depth + 1)
                if why:
                    return why
        moves.append(Move(m.rid, "backoff", round(d, 2), 0))
        swept.extend(path_boxes)
        return ""

    for y in yielders:
        busy.add(y.rid)
        why = push(y, keep_clear, 0)
        if why:
            return moves, swept, why
    # deepest first: moves appended by the recursion come before their yielder, so keep that order
    for i, mv in enumerate(moves):
        mv.order = i
    return moves, swept, ""


def make_plan(group: list[Member], everyone: dict[str, Member], coordinator: str, rung: int, t: float,
              static_blocked: bool, static_hit, L: float, W: float, facing_obstacle: str = "") -> DRPPlan:
    """The resolution for one deadlock group at escalation ``rung`` (1 = rule of its type)."""
    kind, why = classify(group, static_blocked)
    if rung == 2:
        kind, why = "D6", "rung 2: the rule of the type did not resolve it; general cycle rule"
    ids = [m.rid for m in group]
    by_key = sorted(group, key=lambda m: m.key)  # best precedence first
    movable = [m for m in group if not m.in_station]

    def leader() -> Member:
        # the member furthest along the common heading
        m0 = group[0]
        c, s = math.cos(m0.th), math.sin(m0.th)
        return max(group, key=lambda m: m.x * c + m.y * s)

    if kind == "D1":
        prio = leader()
    elif kind == "D3":
        prio = min((m for m in group if m.in_core), key=lambda m: m.key)
    elif kind == "D4":
        prio = min((m for m in group if m.in_station), key=lambda m: m.key)
    elif kind == "D2":
        prio = min((m for m in group if m.bypassing), key=lambda m: m.key, default=by_key[0])
    elif kind == "D5":
        prio = min((m for m in group if m.rid != facing_obstacle), key=lambda m: m.key, default=by_key[0])
    else:  # D6: the loser is the movable lowest-precedence member on the cycle; everyone else keeps going
        loser = _yielder(group, movable)
        prio = min((m for m in group if m.rid != loser.rid), key=lambda m: m.key, default=loser)

    moves: list[Move] = []
    zone: list[OrientedBox] = corridor(prio, L, W)
    reason = why
    if rung >= 3:
        # re-route the lowest-precedence movable member on the cycle, everyone else as usual
        loser = _yielder(group, movable)
        moves = [Move(loser.rid, "reroute", 0.0, 0)]
        reason = f"rung 3: {loser.rid} re-routes with the contested stretch forbidden"
    elif kind == "D1":
        moves = [Move(m.rid, "hold", 0.0, 0) for m in group if m.rid != prio.rid]
    elif kind == "D5" and facing_obstacle:
        moves = [Move(facing_obstacle, "reroute", 0.0, 0)]
    else:
        yielders = [m for m in group if m.rid != prio.rid]
        if kind == "D6":
            yielders = [_yielder(group, movable)]
        # In a single-file corridor or aisle, robots cannot pass each other by chain reverse along the line
        single_file_standoff = False
        prio_sf = is_single_file_member(prio)
        for y in yielders:
            if is_single_file_member(y):
                prio_line = prio.lane.split("/")[0] if "/" in prio.lane else ""
                y_line = y.lane.split("/")[0] if "/" in y.lane else ""
                if (prio_line and prio_line == y_line) or prio_sf:
                    single_file_standoff = True
                    break

        if single_file_standoff:
            candidates = [m for m in (movable or group) if m.rid != prio.rid]
            opposing = [m for m in candidates if (m.lane and prio.lane and m.lane.split("/")[0] == prio.lane.split("/")[0]) or is_single_file_member(m)]
            pool = opposing or candidates or [prio]
            on_cycle = {c.rid for c in cycle_members(group)}
            # same cycle preference, applied inside the single-file pool
            loser = max([c for c in pool if c.rid in on_cycle] or pool, key=lambda m: m.key)
            moves = [Move(loser.rid, "reroute", 0.0, 0)]
            reason = f"{why}; single-file line cannot pass: {loser.rid} re-routes"
        else:
            bk, swept, infeasible = plan_backoffs(yielders, zone, everyone, static_hit, L, W, {prio.rid})
            if infeasible:
                # cannot make room physically: re-route the lowest-precedence cycle member instead
                loser = _yielder(group, movable)
                moves = [Move(loser.rid, "reroute", 0.0, 0)]
                reason = f"{why}; chain reverse infeasible ({infeasible}): {loser.rid} re-routes"
            else:
                moves = bk
                zone += swept
                ids = sorted(set(ids) | {mv.rid for mv in bk}, key=id_key)
    # every other member holds: drops its bookings and stands, so nothing it booked blocks the priority robot
    acting = {mv.rid for mv in moves}
    for r in (sorted(ids, key=id_key) if HOLD_OTHERS else ()):
        if r != prio.rid and r not in acting:
            moves.append(Move(r, "hold", 0.0, len(moves)))
    moves.append(Move(prio.rid, "pass", 0.0, len(moves)))
    return DRPPlan(epoch=f"{coordinator}:{int(t * 10)}", coordinator=coordinator, kind=kind, priority=prio.rid,
                   moves=moves, zone=[(b.cx, b.cy, b.length, b.width, b.heading) for b in zone],
                   members=ids, rung=rung, t_plan=t, expires=t + FREEZE_S, reason=reason)


def zone_boxes(plan: DRPPlan) -> list[OrientedBox]:
    return [OrientedBox(*z) for z in plan.zone]


@dataclass
class ProgressWatch:
    """No-progress watchdog: a driving robot that has not moved ``STALL_M`` in ``STALL_S`` is stalled."""

    anchor: tuple[float, float] | None = None
    since: float = 0.0
    history: list = field(default_factory=list)

    def update(self, t: float, x: float, y: float, active: bool) -> bool:
        if not active:
            self.anchor, self.since = None, t
            return False
        if self.anchor is None or math.hypot(x - self.anchor[0], y - self.anchor[1]) >= STALL_M:
            self.anchor, self.since = (x, y), t
            return False
        return t - self.since >= STALL_S
