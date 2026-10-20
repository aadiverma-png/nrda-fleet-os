"""NRDA-FA layer L3: turn a route into a timed trajectory plus space-time claims.

Given the robot's pose and its route (a list of :class:`~nrda_fleet.trajectory.Leg`),
:class:`Scheduler.plan`:

1. times the route with the robot's limits;
2. sweeps the padded body along it and collects, per resource (lane cell,
   station footprint, free-space cell), the interval it is covered;
3. checks every interval, widened by the time margin ``tau``, against the
   peer claims that have precedence (:meth:`ClaimBook.blocking`). On a
   conflict it inserts a **wait at the last legal hold point** before that
   resource (in a lane, never inside an intersection: "don't block the box"),
   long enough to enter just after the blocking claim ends. This is visison's
   earliest-free-point rule (``visison.intervals``) applied cell by cell
   along the whole route;
4. keeps only the **rolling window** (``window`` seconds). The motion
   authority ends at the last hold point inside the window whose claims are
   all confirmed, and the trajectory brakes to a stop there unless the next
   replan extends it.

The result is a :class:`Plan`: the trajectory handed to the MotionArbiter
(PLANNER source), the claims to publish, and the waits with their causes
(for the wait-for graph and for the L2 bypass trigger).
"""

from __future__ import annotations

import math
import time
from collections import defaultdict
from dataclasses import dataclass, field

from visison.intervals import Interval, earliest_free_point, merge

from .claims import ClaimBook, Res
from .geom import wrap
from .lane_graph import LaneGraph
from .trajectory import Builder, Leg, PathTrajectory


@dataclass
class Wait:
    u: float  # path distance where the robot waits
    until: float  # earliest entry time it waits for
    res: Res  # contested resource
    blockers: tuple[str, ...]  # robots holding it
    t_start: float = 0.0


@dataclass
class Plan:
    traj: PathTrajectory
    claims: list[tuple[Res, float, float]]
    waits: list[Wait] = field(default_factory=list)
    u_authority: float = 0.0  # path distance the robot may drive this cycle
    full_length: float = 0.0
    compute_ms: float = 0.0
    forced: bool = False  # a conflict could not be avoided by waiting (inside braking distance)
    legs: list[Leg] = field(default_factory=list)  # legs actually executed (with waits, truncated)
    auth_blockers: tuple[str, ...] = ()

    @property
    def blocked_by(self) -> tuple[str, ...]:
        if self.waits:
            return self.waits[0].blockers
        return self.auth_blockers



@dataclass
class _Geo:
    """Straight-line geometry of the route legs (distance u -> pose), before timing."""

    kind: str
    leg: int
    u0: float
    u1: float
    x0: float
    y0: float
    x1: float
    y1: float
    th: float  # body heading during the leg (rotate: end heading)
    th0: float = 0.0


def _geometry(x: float, y: float, th: float, legs: list[Leg]) -> list[_Geo]:
    out, u = [], 0.0
    for i, lg in enumerate(legs):
        if lg.kind == "rotate":
            out.append(_Geo("rotate", i, u, u, x, y, x, y, lg.th1, th))
            th = lg.th1
        elif lg.kind == "wait":
            out.append(_Geo("wait", i, u, u, x, y, x, y, th, th))
        else:
            L = math.hypot(lg.x1 - x, lg.y1 - y)
            travel = math.atan2(lg.y1 - y, lg.x1 - x) if L > 1e-6 else th
            body = th if lg.kind == "shift" else wrap(travel + (math.pi if lg.reverse else 0.0))
            out.append(_Geo(lg.kind, i, u, u + L, x, y, lg.x1, lg.y1, body, th))
            u += L
            x, y = lg.x1, lg.y1
            if lg.kind != "shift":
                th = body
    return out


def _pose_at(geo: list[_Geo], u: float) -> tuple[float, float, float, int, float]:
    """(x, y, heading, leg index, fraction) at path distance u (at the start of rotations at that u)."""
    for g in geo:
        if g.kind in ("move", "shift") and g.u0 - 1e-9 <= u <= g.u1 + 1e-9:
            f = 0.0 if g.u1 - g.u0 < 1e-9 else (u - g.u0) / (g.u1 - g.u0)
            return g.x0 + (g.x1 - g.x0) * f, g.y0 + (g.y1 - g.y0) * f, g.th, g.leg, f
    g = geo[-1]
    return g.x1, g.y1, g.th, g.leg, 1.0


@dataclass
class Scheduler:
    graph: LaneGraph
    book: ClaimBook
    window: float = 12.0  # s of claims ahead
    tau: float = 0.6  # s time margin around every claim (applied both sides, so it costs 2*tau*v of lane)
    pad_long: float = 0.30  # m, safety margin ahead/behind the body. Floor: below 0.30 the padded box of a
    # robot at 1 m/s stops reaching into the 0.5 m cell it is about to enter, so it no longer reserves the
    # road ahead and a peer can claim that cell out from under it (test_nrda_fleet, leader/follower guard).
    pad_lat: float = 0.05
    pad_rear: float = 0.05  # m, behind a moving robot (the road it vacates). Measured: dropping it to 0 removes
    # a rare leader/follower stand-off but packs the lanes tight and costs about half the throughput (28 -> 15
    # tasks per 480 s with 20 robots), so the small margin stays.
    _docked: bool = False
    sample_dt: float = 0.15
    hold_step: float = 0.1
    max_iter: int = 25
    horizon_wait: float = 120.0

    def __post_init__(self):
        amr = self.graph.site.amr
        self.builder = Builder(amr.limits(), amr.w_turn, amr.alpha_turn)

    # --------------------------------------------------------------------------------------

    def plan(self, now: float, x: float, y: float, th: float, v: float, legs: list[Leg],
             hold_final: float = 0.0, w: float = 0.0, docked: bool = False) -> Plan:
        """Schedule ``legs`` from the current pose. ``hold_final``: seconds to claim at the end (service).
        ``docked``: the robot stands in a station bay; it books its exact body, with no safety padding (a
        padded hold of a slightly skewed parked robot reached into the corridor next to the bay for an hour)."""
        self._docked = docked
        t_cpu = time.perf_counter()
        legs = list(legs)
        waits: list[Wait] = []
        forced = False
        geo = _geometry(x, y, th, legs)
        full_len = geo[-1].u1 if geo else 0.0
        start_ln = self.graph.single_file_line(x, y)
        u_brake = v * v / (2 * self.graph.site.amr.e_decel) + 0.02  # what the robot can really stop in
        for _ in range(self.max_iter):
            traj = self.builder.build(now, x, y, th, v, legs, w)
            t_claim = self._claim_horizon(traj, _geometry(x, y, th, legs), now)
            occ = self._occupancy(traj, now, t_claim, hold_final)
            conflict = self._first_conflict(occ, now)
            if conflict is None:
                break
            res, t_in, t_out, u_in, until, blockers = conflict
            is_peer = any(b.startswith("AMR_") for b in blockers)
            geo = _geometry(x, y, th, legs)
            u_hold = self._hold_before(geo, res, u_in, u_min=u_brake if v > 0.05 else 0.0,
                                       start_ln=start_ln, is_peer=is_peer)
            if u_hold is None:
                forced = True
                waits.append(Wait(0.0, until, res, blockers))  # still tell L2 / WFG what blocks us
                break
            wait_for = min(max(until - t_in, 0.3), self.horizon_wait)
            legs = _insert_wait(legs, geo, u_hold, wait_for, tag=f"wait:{res[0]}:{res[1]}")
            waits.append(Wait(u_hold, until, res, blockers))
            if wait_for >= self.window:
                break  # the robot waits past the whole window: nothing beyond it needs scheduling now
        # rolling window + confirmation: claim everything planned inside the window, but move only
        # as far as the claims that are already confirmed (movement authority)
        geo = _geometry(x, y, th, legs)
        traj = self.builder.build(now, x, y, th, v, legs, w)
        t_claim = self._claim_horizon(traj, geo, now)
        u_auth, auth_blockers = self._authority(traj, geo, now, v, t_claim, start_ln=start_ln)
        if forced:  # cannot stop before the contested resource any more: stop as early as possible anyway
            u_auth = 0.0 if v < 0.1 else min(u_auth, u_brake)
        exec_legs = _truncate(legs, geo, u_auth)
        traj_exec = self.builder.build(now, x, y, th, v, exec_legs, w) if exec_legs else \
            self.builder.build(now, x, y, th, v, [Leg("wait", duration=0.5)])
        occ = self._occupancy(traj, now, t_claim, hold_final, rest_until=t_claim + 2.0)
        # where the robot stops if the authority is not extended: keep that spot claimed a little longer
        t_stop = traj_exec.t_end
        ex, ey, eth = traj_exec.end_pose()
        for res in self._cover_moving(ex, ey, eth, 0.0):  # standing there: road ahead only, never behind
            occ.setdefault(res, []).append((t_stop - self.sample_dt, t_stop + 1.5, u_auth))
        claims = []
        for res, ivs in occ.items():
            for t0, t1, _u in ivs:
                claims.append((res, t0 - self.tau, t1 + self.tau))
        for w in waits:
            w.t_start = self._time_at_u(traj, w.u, stopped=True)
        return Plan(traj_exec, claims, waits, u_auth, full_len, (time.perf_counter() - t_cpu) * 1e3, forced,
                    exec_legs, auth_blockers)


    # --------------------------------------------------------------------------------------

    def _cover_moving(self, x: float, y: float, th: float, v: float) -> set:
        """Resources of the body padded in its direction of travel only.

        A robot needs ``pad_long`` of free road ahead; behind it the road is being vacated, so it keeps
        only ``pad_rear`` (nothing at all at a standstill). Padding behind as much as ahead made a leader contend for the gap right behind it,
        which its follower had booked first: the follower then out-ranked the leader and both waited for ever.
        """
        if abs(v) <= 0.05:
            if self._docked:
                return self.graph.cover(x, y, th, 0.0, 0.0)
            # standing still: pad ahead only, for the same reason. A symmetric pad reached 0.1 m behind the
            # body, into the cell the robot queued behind physically occupies; its (older) claim then beat the
            # leader's, so the leader waited for its own follower and the queue never moved again.
            pad = 0.1
            return self.graph.cover(x + pad / 2 * math.cos(th), y + pad / 2 * math.sin(th), th,
                                    pad / 2, self.pad_lat)
        sgn = 1.0 if v > 0 else -1.0
        shift = sgn * (self.pad_long - self.pad_rear) / 2
        return self.graph.cover(x + shift * math.cos(th), y + shift * math.sin(th), th,
                                (self.pad_long + self.pad_rear) / 2, self.pad_lat)

    def _occupancy(self, traj: PathTrajectory, t0: float, t1: float, hold_final: float = 0.0,
                   rest_until: float | None = None) -> dict[Res, list[tuple[float, float, float]]]:
        """Per resource: merged (t_in, t_out, u_in) intervals the padded body covers between t0 and t1."""
        occ: dict[Res, list[list[float]]] = defaultdict(list)
        dt = self.sample_dt
        t_last = min(traj.t_end, t1)
        t = t0
        while True:
            x, y, th, v, u = traj.state(t)
            for res in self._cover_moving(x, y, th, v):
                ivs = occ[res]
                if ivs and t - ivs[-1][1] <= dt * 1.5:
                    ivs[-1][1] = t
                else:
                    ivs.append([t, t, u])
            if t >= t_last:
                break
            t = min(t + dt, t_last)
        # after the trajectory ends the robot stands at its last pose
        end = traj.t_end
        stay = max(rest_until or 0.0, end + hold_final) if (rest_until or hold_final) else None
        if stay is not None and end <= t1 + 1e-9:
            x, y, th = traj.end_pose()
            for res in self._cover_moving(x, y, th, 0.0):  # resting at the end of the route: same rule
                ivs = occ[res]
                if ivs and end - ivs[-1][1] <= dt * 1.5:
                    ivs[-1][1] = max(ivs[-1][1], stay)
                else:
                    ivs.append([end, stay, traj.length])
        return {r: [(a - dt, b + dt, u) for a, b, u in ivs] for r, ivs in occ.items()}

    def _first_conflict(self, occ, now):
        """Earliest (by entry time) covered interval that overlaps a claim with precedence."""
        best = None
        # sorted, and ties broken by the resource id: ``occ`` is built from sets of (key, index) tuples, whose
        # iteration order changes with Python's per-process string hash seed. Picking "the earliest conflict"
        # without a tie-break therefore made two runs of the same scenario diverge.
        for res, ivs in sorted(occ.items()):
            blk = self.book.blocking(res, now - 5.0, now)
            if not blk:
                continue
            for t_in, t_out, u_in in ivs:
                a, b = t_in - self.tau, t_out + self.tau
                hits = [(rid, s, e) for rid, s, e in blk if s < b and a < e]
                if not hits:
                    continue
                if (res in self.book.occupied or u_in <= 0.05) and t_in <= now + 0.2:
                    continue  # we are physically there / vacating: precedence is ours
                D = t_out - t_in
                forbidden = [Interval(s - D - self.tau, e + self.tau) for _r, s, e in blk]
                t_ok = earliest_free_point(merge(forbidden), t_in)
                until = t_ok if t_ok is not None else max(e for _r, _s, e in hits) + self.tau
                cand = (t_in, res, t_out, u_in, until, tuple(sorted({r for r, _s, _e in hits})))
                if best is None or cand[:2] < best[:2]:
                    best = cand
        if best is None:
            return None
        t_in, res, t_out, u_in, until, blockers = best
        return res, t_in, t_out, u_in, until, blockers

    def _is_legal_hold(self, x: float, y: float, th: float, u: float, total: float,
                       start_ln: str | None = None, is_peer: bool = True) -> bool:
        """May the robot hold at pose (x, y, th) at path distance u?

        A robot may NOT hold:
        1. inside an intersection core or station keep-clear zone (in_intersection);
        2. inside a narrow rack aisle or narrow corridor (in_single_file) when waiting on a peer robot, UNLESS:
           - it is at a station (_at_station);
           - this is its destination (u >= total - 0.2);
           - the robot already started inside this specific line (start_ln).
        """
        if self.graph.in_intersection(x, y, th):
            return False
        if is_peer and self.graph.in_single_file(x, y):
            if self.graph._at_station(x, y, 0.8) or u >= total - 0.2:
                return True
            s_ln = self.graph.single_file_line(x, y)
            if start_ln is not None and s_ln == start_ln:
                return True
            return False
        return True

    def _hold_before(self, geo: list[_Geo], res: Res, u_in: float, u_min: float,
                     start_ln: str | None = None, is_peer: bool = True) -> float | None:
        """Largest path distance <= u_in - margin where the robot may stand without covering ``res``."""
        if not geo:
            return 0.0  # no route (holding in place): where it stands is the only place it can wait
        u = min(u_in, geo[-1].u1)
        total = geo[-1].u1
        xg_map = self.graph.crossing_cell_map()
        xg_name = xg_map.get(res)
        # rotation points at exactly u are fine holds too (wait before rotating)
        while u >= u_min - 1e-9:
            x, y, th, _li, _f = _pose_at(geo, u)
            if self._is_legal_hold(x, y, th, u, total, start_ln, is_peer):
                cov = self._cover_moving(x, y, th, 0.0)  # the footprint it will claim while it holds
                if res not in cov:
                    if xg_name is None or not any(xg_map.get(c) == xg_name for c in cov):
                        return max(u, 0.0)
            if u <= 0:
                break
            u = max(u - self.hold_step, 0.0) if u > u_min else u_min - 1.0
        return None

    def _time_at_u(self, traj: PathTrajectory, u: float, stopped: bool = False) -> float:
        for p in traj.prims:
            if p.kind in ("move", "shift") and p.u0 <= u <= p.u0 + p.length + 1e-9:
                return p.profile.time_at(u - p.u0) if p.profile is not None else p.t0
            if p.kind in ("rotate", "wait") and abs(p.u0 - u) < 1e-6:
                return p.t0
        return traj.t_end

    def _claim_horizon(self, traj: PathTrajectory, geo: list[_Geo], now: float) -> float:
        """End of the claim window: ``now + window``, pushed on to the next legal hold point.

        A robot must never get authority that ends inside a crossing or narrow corridor, so the
        window is extended until the body is clear of every crossing core and narrow stretch.
        """
        horizon = now + self.window
        cap = horizon + 15.0  # never claim more than 15 s past the window
        if traj.t_end <= horizon:
            return traj.t_end
        u = traj.state(horizon)[4]
        total = geo[-1].u1 if geo else 0.0
        while u < total:
            px, py, pth, _li, _f = _pose_at(geo, u)
            if not self.graph.in_intersection(px, py, pth) and not self.graph.in_narrow_corridor(px, py):
                return min(max(self._time_at_u(traj, u), horizon), cap)
            u += self.hold_step
        return min(traj.t_end, cap)

    def _authority(self, traj: PathTrajectory, geo: list[_Geo], now: float, v: float,
                   t_claim: float | None = None, start_ln: str | None = None) -> tuple[float, tuple[str, ...]]:
        """Last legal hold point reached within the claim window whose covered claims are all confirmed."""
        horizon = t_claim if t_claim is not None else now + self.window
        u_brake = v * v / (2 * self.graph.site.amr.decel)
        total = geo[-1].u1 if geo else 0.0
        # the robot must not enter resources it does not hold a confirmed claim on
        ok_res = {res for res, lst in self.book.own.items() for c in lst if self.book.confirmed(c)}
        ok_res |= self.book.occupied
        if geo:
            x0, y0, th0, _li, _f = _pose_at(geo, 0.0)
            ok_res |= self._cover_moving(x0, y0, th0, max(v, 0.2))
        u_limit = total
        t = now
        dt = self.sample_dt
        auth_blockers: set[str] = set()
        while t <= min(traj.t_end, horizon):
            x, y, th, vv, u = traj.state(t)
            cov = self._cover_moving(x, y, th, vv)
            if not cov <= ok_res:
                u_limit = min(u_limit, u)
                for res in (cov - ok_res):
                    for rid, p in self.book.peers.items():
                        if res in p.occupied or res in p.by_res:
                            auth_blockers.add(rid)
                break
            t += dt
        if traj.t_end > horizon + 1e-6:
            u_limit = min(u_limit, traj.state(horizon)[4])
        if u_limit >= total - 1e-6:
            return total, ()
        # back off to a hold point
        u = u_limit
        while u > u_brake:
            x, y, th, _li, _f = _pose_at(geo, u)
            if self._is_legal_hold(x, y, th, u, total, start_ln, is_peer=bool(auth_blockers)):
                return u, tuple(sorted(auth_blockers))
            u -= self.hold_step
        return (max(u_brake, 0.0) if u_brake > 0 else 0.0), tuple(sorted(auth_blockers))




def _insert_wait(legs: list[Leg], geo: list[_Geo], u: float, duration: float, tag: str) -> list[Leg]:
    """Split the leg that passes ``u`` and wait there (before a rotation that starts at ``u``)."""
    for g in geo:
        if g.kind in ("move", "shift") and g.u0 + 1e-6 < u < g.u1 - 1e-6:
            f = (u - g.u0) / (g.u1 - g.u0)
            lg = legs[g.leg]
            if lg.kind == "shift":
                break  # never stop inside a docking shift: wait at its start instead
            hx, hy = g.x0 + (g.x1 - g.x0) * f, g.y0 + (g.y1 - g.y0) * f
            first = Leg("move", hx, hy, reverse=lg.reverse, v_max=lg.v_max, tag=lg.tag)
            return legs[:g.leg] + [first, Leg("wait", duration=duration, tag=tag)] + legs[g.leg:]
    # u at a leg boundary: wait before the first leg that starts at u
    for g in geo:
        if abs(g.u0 - u) < 1e-6 or (g.kind == "shift" and g.u0 <= u <= g.u1):
            idx = g.leg
            # merge with an existing wait at the same place
            if idx > 0 and legs[idx - 1].kind == "wait":
                legs = list(legs)
                legs[idx - 1] = Leg("wait", duration=legs[idx - 1].duration + duration, tag=tag)
                return legs
            return legs[:idx] + [Leg("wait", duration=duration, tag=tag)] + legs[idx:]
    return legs + [Leg("wait", duration=duration, tag=tag)]


def _truncate(legs: list[Leg], geo: list[_Geo], u: float) -> list[Leg]:
    """Legs up to path distance ``u``; the robot stops there (rotations at ``u`` wait for the next cycle)."""
    total = geo[-1].u1 if geo else 0.0
    if u >= total - 1e-6:
        return list(legs)
    out: list[Leg] = []
    for g in geo:
        lg = legs[g.leg]
        if g.kind in ("rotate", "wait"):
            if g.u0 < u - 1e-6:
                out.append(lg)
                continue
            break
        if g.u1 <= u + 1e-6:
            out.append(lg)
            continue
        if g.u0 < u - 1e-6 and lg.kind == "move":
            f = (u - g.u0) / (g.u1 - g.u0)
            out.append(Leg("move", g.x0 + (g.x1 - g.x0) * f, g.y0 + (g.y1 - g.y0) * f, reverse=lg.reverse,
                           v_max=lg.v_max, tag=lg.tag))
        break
    return out
