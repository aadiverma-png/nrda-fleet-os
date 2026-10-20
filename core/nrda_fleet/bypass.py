"""NRDA-FA layer L2: local coordination with the visison bypass planner.

When a robot's own lane is blocked, by a robot that has stopped (loading at a
picker stop, faulted, silent) or by an unmapped obstacle its lidar sees (a
dropped pallet), the blocker is turned into a :class:`visison.geometry.Obstacle`
in that aisle's frame and :class:`visison.controller.BypassController` makes
the one decision (FINAL_ALGORITHM 5.2):

* ``SQUEEZE``          pass inside the own lane (no claim needed);
* ``BYPASS_NOW``       borrow the oncoming lane now; its cells are claimed atomically
                       through the robot's :class:`~nrda_fleet.claims.ClaimBook`, so
                       oncoming robots schedule around the window;
* ``WAIT_THEN_BYPASS`` stop before the obstacle and go at the earliest safe time t*;
* ``REROUTE``          the lane-graph detour is cheaper (costed with the blocked
                       stretch removed);
* ``HOLD``             nothing is possible yet, retry.

While the manoeuvre runs the controller owns the robot's PLANNER intent (safe-
point rule: no re-allocation, no re-route), and it hands back to L3 once
the robot is back in its lane.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from visison.arbiter import Kind, MotionIntent, Source
from visison.controller import BypassController, ControllerState
from visison.geometry import Obstacle
from visison.planner import Action, BypassPlanner, PlannerConfig, RerouteEstimate, RobotState, SafetyMargins

from . import standoff
from .geom import OrientedBox, wrap
from .lane_graph import Line
from .trajectory import FramedTrajectory

MANOEUVRE = (Action.SQUEEZE, Action.BYPASS_NOW, Action.WAIT_THEN_BYPASS, Action.COOP_BYPASS)

# A live session suppresses re-planning and re-claiming in :meth:`~nrda_fleet.agent.EdgeAgent._l3`, so
# every phase it can sit in needs a way out. Only HOLDING had one, and a session that stopped making
# progress in CHANGE_OUT / PASSING / CHANGE_IN kept the robot where it stood - across both lanes of the
# aisle, holding no claims - for the rest of the run. These deadlines bound every phase instead.
SETTLING = (ControllerState.REJOINED, ControllerState.CRUISING)
SETTLE_MAX = 90.0  # s to finish rejoining once the controller's schedule says the pass is over
HOLD_MAX = 30.0  # s held with nothing possible before L3 is given the re-route (unchanged)
MANOEUVRE_MAX = 60.0  # s for the whole swing-out / pass / swing-in, whatever phase it is in
STALL_MAX = 12.0  # s inside a manoeuvre without moving STALL_EPS before the session is given up
STALL_EPS = 0.10  # m, net movement below this counts as not moving


class AisleRobot:
    """visison ``RobotInterface`` for one agent inside one aisle frame."""

    def __init__(self, agent, line: Line, lane: str):
        self.agent, self.line, self.lane = agent, line, lane
        self.robot_id = agent.rid
        self.direction = 1 if lane == "A" else -1

    def state(self) -> RobotState:
        x, y, th = self.agent.pose
        s, ya = self.line.to_frame(x, y)
        v = abs(self.agent.v)
        return RobotState(self.robot_id, s, v, self.line.lane_key(self.lane), self.direction, ya,
                          self.agent.v < -0.02, wrap(th - self.line.theta))

    def _now(self) -> float:
        return self.agent.t

    def follow(self, trajectory, source: Source = Source.PLANNER, kind: Kind = Kind.FOLLOW) -> None:
        t = self._now()
        world = FramedTrajectory(trajectory, self.line.origin, self.line.theta)
        self.agent.arbiter.request(MotionIntent(source, kind, world, stamp=t), t)

    def stop(self, source: Source = Source.PLANNER) -> None:
        t = self._now()
        self.agent.arbiter.request(MotionIntent(source, Kind.BRAKE if source is Source.PLANNER else Kind.BRAKE,
                                                None, stamp=t), t)

    def emergency_brake(self, source: Source = Source.EMERGENCY) -> None:
        t = self._now()
        self.agent.arbiter.request(MotionIntent(source, Kind.BRAKE, None, stamp=t, ttl=0.5), t)

    def release(self, source: Source, handover: bool = False) -> None:
        self.agent.arbiter.release(source, self._now(), handover)

    def reference_time(self, t: float) -> float:
        return self.agent.arbiter.reference_time(t)

    @property
    def lateness(self) -> float:
        return self.agent.arbiter.lateness


class LaneGraphRerouter:
    """visison ``ReroutePlanner`` backed by the lane graph: detour cost with the blocked stretch removed."""

    def __init__(self, agent, line: Line, lane: str):
        self.agent, self.line, self.lane = agent, line, lane

    def _forbid(self, ob: Obstacle) -> set[tuple[str, str]]:
        g, ln = self.agent.g, self.line
        lo, hi = ln.lo + ob.s_min - 1.0, ln.lo + ob.s_max + 1.0
        out = set()
        for es in g.edges.values():
            for e in es:
                if e.kind != "move" or e.line != ln.name or e.lane != self.lane:
                    continue
                p, q = g.points[e.src[0]], g.points[e.dst[0]]
                a, b = sorted((ln.along(p.x, p.y), ln.along(q.x, q.y)))
                if a <= hi and lo <= b:
                    out.add((e.src[0], e.dst[0]))
        return out

    def estimate(self, state: RobotState, obstacle: Obstacle) -> RerouteEstimate | None:
        a = self.agent
        if a.goal is None:
            return None
        node = a.g.stations[a.goal]
        pre, start = a._start_state()
        if start is None:
            return None
        # mid-lane the route keeps its legs up to the next graph point: if those already run into the
        # obstacle no lane-graph detour exists from here (a U-turn is not a lane-graph move)
        s0, _ = self.line.to_frame(*a.pose[:2])
        d = 1 if self.lane == "A" else -1
        for lg in pre:
            if lg.kind == "move":
                s1, _ = self.line.to_frame(lg.x1, lg.y1)
                if d * (s1 - s0) > 0 and d * (s1 - (obstacle.s_min if d > 0 else obstacle.s_max)) > 0:
                    return None
        goal_h = node.maneuver.root_heading if node.maneuver else node.heading
        base = a.g.route(start, node.root, goal_h)
        alt = a.g.route(start, node.root, goal_h, forbidden=self._forbid(obstacle))
        if base is None or alt is None:
            return None
        extra = max(a.g.route_cost(alt) - a.g.route_cost(base), 0.0)
        return RerouteEstimate(extra, extra * a.amr.v_cruise)

    def execute(self, state: RobotState, obstacle: Obstacle) -> None:
        a = self.agent
        for e in self._forbid(obstacle):
            a.forbid[e] = a.t + 60.0
        a._reroute(f"L2: detour around {obstacle.obstacle_id}")


@dataclass
class BypassSession:
    agent: object
    ctl: BypassController
    line: Line
    lane: str
    obstacle_id: str
    started: float
    obstacle: Obstacle | None = None  # lane frame; the session lasts until the robot is really past it
    stall_at: tuple[float, float] | None = None  # where the robot was when it last made progress
    stall_since: float | None = None  # when it stopped making progress
    gave_up: str = ""  # why this session ended, "" while it is still running

    def passed(self) -> bool:
        """The robot's rear bumper is beyond the obstacle along its travel direction."""
        if self.obstacle is None:
            return True
        x, y, _ = self.agent.pose
        s, _ = self.line.to_frame(x, y)
        half = self.agent.amr.length / 2
        if self.lane == "A":
            return s - half > self.obstacle.s_max + 0.05
        return s + half < self.obstacle.s_min - 0.05

    def active(self) -> bool:
        if self.ctl.state is ControllerState.REROUTING or self.gave_up:
            return False
        if self.ctl.state in SETTLING:
            # the controller's phases are timed; when the squeeze past the obstacle ran slow (protective field
            # creeping), its schedule says "rejoined" while the robot is still beside the obstacle in the
            # oncoming lane. Handing over to L3 there strands it (L3 routes back through the blocked cells, L2
            # sees no conflict from the other lane), so keep following the bypass path until really past it.
            return not self.passed() and self.agent.t - self.started < SETTLE_MAX
        return True

    def state_name(self) -> str:
        return self.ctl.state.value

    def expired(self, t: float) -> str:
        """Why this session has to end now, or ``""`` to carry on.

        Every phase is bounded. ``SETTLING`` keeps its own rule in :meth:`active`; ``HOLDING`` keeps its
        30 s. The manoeuvre phases get both a total deadline and a no-progress deadline, because the way
        this fails in practice is a robot frozen mid-pass that is not holding, not rerouting and not moving.
        """
        st = self.ctl.state
        if st is ControllerState.REROUTING or st in SETTLING:
            return ""
        if st is ControllerState.HOLDING:
            return f"hold timed out after {HOLD_MAX:.0f} s" if t - self.started > HOLD_MAX else ""
        if t - self.started > MANOEUVRE_MAX:
            return f"{st.value} did not finish in {MANOEUVRE_MAX:.0f} s"
        x, y, _ = self.agent.pose
        if self.stall_at is None or math.hypot(x - self.stall_at[0], y - self.stall_at[1]) > STALL_EPS:
            self.stall_at, self.stall_since = (x, y), t
            return ""
        if t - self.stall_since > STALL_MAX:
            return f"stalled {t - self.stall_since:.0f} s in {st.value}"
        return ""

    def tick(self, t: float) -> None:
        self.ctl.tick(t)
        a = self.agent
        if standoff.between_lanes(a, self):
            # L2b: stopped out of our own lane, so we need the lane we are sitting in cleared ahead to
            # swing back. Ask the robots in it to chain-reverse (standoff.py). The controller's phase is
            # not the test: the emergency layer can brake us between the lanes in any phase.
            standoff.request(a, self, t)
        why = self.expired(t)
        if why:
            # blocked for good: give L3 the reroute (FINAL_ALGORITHM: REROUTE is costed on the lane graph)
            for c in list(a.book.extra):
                a.book.extra.pop(c, None)
            self.gave_up = why
            self.ctl.state = ControllerState.REROUTING
            a._reroute(f"L2 {self.obstacle_id}: {why}")
        if not self.active():
            a._log(f"L2 {self.obstacle_id}: {self.ctl.state.value}")
            a.book.extra.clear()
            a._skip_passed_legs()


def obstacle_in_frame(line: Line, box: OrientedBox, pad: float = 0.05, name: str = "obstacle") -> Obstacle:
    ss, yy = zip(*(line.to_frame(x, y) for x, y in box.corners()))
    return Obstacle(min(ss) - pad, max(ss) + pad, min(yy) - pad, max(yy) + pad, obstacle_id=name)


def try_bypass(agent, t: float, box: OrientedBox | None = None, name: str = "") -> BypassSession | None:
    """Ask the visison planner what to do about a blocker in the robot's lane. None: L3 keeps control."""
    x, y, th = agent.pose
    where = agent.g.line_of(x, y, th)
    if where is None:
        return None
    line, lane = where
    if box is None:
        blk = _stopped_blocker(agent, line, lane)
        if blk is None:
            return None
        box, name = blk
    ob = obstacle_in_frame(line, box, 0.05, name or "obstacle")
    robot = AisleRobot(agent, line, lane)
    spec = agent.amr.robot_spec()
    cfg = PlannerConfig(clearance=0.12, margins=SafetyMargins(comm_latency=0.15, reservation_jitter=0.35,
                                                             other_brake_time=0.5), max_wait=45.0,
                        horizon=90.0, reroute_penalty=8.0)
    planner = BypassPlanner(line.aisle, spec, cfg)
    other = _other_lane_obstacles(agent, line, lane)
    ctl = BypassController(robot=robot, planner=planner, reservations=agent.book,
                           rerouter=LaneGraphRerouter(agent, line, lane), hold_retry=2.0)
    decision = ctl.on_obstacle(ob, t, other)
    agent._log(f"L2 {ob.obstacle_id}: {decision.action.value} ({decision.reason})")
    if decision.action in MANOEUVRE:
        return BypassSession(agent, ctl, line, lane, ob.obstacle_id, t, ob)
    if decision.action is Action.HOLD:
        return BypassSession(agent, ctl, line, lane, ob.obstacle_id, t, ob)
    return None  # CONTINUE / REROUTE (already executed)


def obstacle_ahead_l2(agent, t: float, plan) -> BypassSession | None:
    """Early L2: a known unmapped obstacle ahead in the robot's own lane that its route runs into.

    Asked while the robot is still approaching (not only once it has stopped in front of the
    obstacle), so the planner still has the room to start the lane change: stopped right before a
    box, the lateral move out no longer fits in front of it at any bypass speed.
    """
    if plan is None or not plan.waits:
        return None
    hit = lane_obstacle_ahead(agent, 9.0)
    if hit is None:
        return None
    box, name = hit
    if name not in plan.blocked_by:
        return None  # the route turns off the lane before the obstacle
    if agent.legs and agent.legs[0].kind == "rotate":
        return None  # still turning into the lane (the planner assumes a lane-aligned robot)
    return try_bypass(agent, t, box, name)


def _stopped_blocker(agent, line: Line, lane: str):
    """The stopped peer robot the scheduler is waiting for, if it stands in our lane ahead."""
    plan = agent.plan
    if plan is None or not plan.waits:
        return None
    x, y, _ = agent.pose
    s0, _ = line.to_frame(x, y)
    d = 1 if lane == "A" else -1
    for rid in plan.waits[0].blockers:
        p = agent.peers.get(rid)
        if p is None:
            continue
        stale = agent.t - agent.peer_heard.get(rid, -1e9) > agent.cfg.peer_timeout
        if abs(p.v) > 0.05 and not stale:
            continue
        if p.mode not in ("loading", "unloading", "fault", "paused", "estop", "idle") and not stale:
            continue
        if agent.goal and p.station == agent.goal and p.mode in ("loading", "unloading") and not stale:
            continue  # it is being served at the station we are going to: queue for our turn, do not overtake

        s, ya = line.to_frame(p.x, p.y)
        if not (0.0 < d * (s - s0) < 12.0):
            continue
        yc = line.lane_y(lane)
        if abs(ya - yc) > line.w * 0.6:
            continue
        return OrientedBox(p.x, p.y, agent.amr.length, agent.amr.width, p.th), rid
    return None


def _other_lane_obstacles(agent, line: Line, lane: str) -> tuple[Obstacle, ...]:
    other = "B" if lane == "A" else "A"
    yc = line.lane_y(other)
    out = []
    for rid, p in agent.peers.items():
        if abs(p.v) > 0.05:
            continue
        s, ya = line.to_frame(p.x, p.y)
        if abs(ya - yc) < line.w * 0.6 and 0 <= s <= line.hi - line.lo:
            out.append(obstacle_in_frame(line, OrientedBox(p.x, p.y, agent.amr.length, agent.amr.width, p.th),
                                         0.05, rid))
    for name, (ox, oy, L, W) in agent.known_obstacles.items():
        box = OrientedBox(ox, oy, L, W, 0.0)
        ob = obstacle_in_frame(line, box, 0.05, name)
        if ob.y_max > line.lane_y(other) - line.w / 2 and ob.y_min < line.lane_y(other) + line.w / 2:
            out.append(ob)
    return tuple(out)


def lane_obstacle_ahead(agent, max_range: float = 9.0):
    """A known unmapped obstacle (own lidar or reported by a peer) in the robot's lane ahead."""
    if not agent.known_obstacles:
        return None
    x, y, th = agent.pose
    where = agent.g.line_of(x, y, th)
    if where is None:
        return None
    line, lane = where
    s0, _ = line.to_frame(x, y)
    d = 1 if lane == "A" else -1
    yc = line.lane_y(lane)
    half = agent.amr.width / 2 + 0.1
    best = None
    for name, (ox, oy, L, W) in agent.known_obstacles.items():
        ob = obstacle_in_frame(line, OrientedBox(ox, oy, L, W, 0.0), 0.0, name)
        if not ob.overlaps_lateral(yc - half, yc + half):
            continue
        ds = d * ((ob.s_min if d > 0 else ob.s_max) - s0)
        if 0.0 < ds - agent.amr.length / 2 < max_range and (best is None or ds < best[0]):
            best = (ds, OrientedBox(ox, oy, L, W, 0.0), name)
    return (best[1], best[2]) if best else None
