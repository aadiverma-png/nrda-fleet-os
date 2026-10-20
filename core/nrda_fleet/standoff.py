"""NRDA-FA layer L2b: in-aisle standoff resolution by chain reverse.

L2 (:mod:`nrda_fleet.bypass`) borrows the oncoming lane to get past a blocker in its own.
Stopped robots in that lane are obstacles to the bypass planner
(:func:`~nrda_fleet.bypass._other_lane_obstacles`), so when one of them stands in the stretch
the bypasser needs, the planner can only answer ``HOLD``. Nothing then asks the robot in the
way to move: after 30 s L2 gives the problem to L3 as a re-route
(:meth:`~nrda_fleet.bypass.BypassSession.tick`), and the DRP declines the group outright
because an in-aisle standoff is L2's (:meth:`~nrda_fleet.agent.EdgeAgent._check_deadlock`).
The bypasser is left between the lanes until it times out.

This layer closes that gap with ``visison.standoff.plan_chain_reverse``, the recurrence that
pushes the robots in the borrowed lane back one after another, each only as far as it has to
and stopping at the first one that does not have to move at all::

    req_1     = F + g                     F = reach of the rejoin creep, g = min_gap + space
    rev_i     = max(0, req_i + h_i - u_i)
    req_{i+1} = u_i + rev_i + h_i + g

``visison.standoff.EmergencyCoordinator`` drives that recurrence centrally: it holds every
robot's state and commands peers through their ``RobotInterface``. An edge agent cannot do
that - it commands only itself (command authority, FINAL_ALGORITHM section 5) - so this layer
uses the planning function on its own and distributes the result the way the DRP does: the
bypasser computes the plan from gossiped peer poses, broadcasts it as one
``AlertMsg(kind="standoff_reverse")``, and every named robot reverses *its own* share through
its own arbiter. The plan is a pure function of gossiped state, so each replica that re-derives
it gets the same answer.

A robot we cannot command (faulted, e-stopped, docked in a station, or silent) is passed as a
*barrier* rather than a mover, so the plan comes out infeasible instead of silently assuming a
robot will move that never will. When it is infeasible the existing hold-then-re-route path is
left to run, unchanged.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field

from visison.emergency import EmergencyConfig
from visison.planner import RobotState
from visison.standoff import plan_chain_reverse

from .bus import AlertMsg
from .geom import wrap

RETRY_S = 6.0  # s between chain-reverse attempts for one obstacle (the movers need time to clear)
MAX_REVERSE = 6.0  # m, the most any robot is asked to back off
# Deliberately the visison default and not the DRP's 3.0 m: there the yielder only has to make a
# little room, while here the distance is set by the rejoin creep's reach (~4.8 m ahead of the
# bypasser), so a robot standing well inside that reach needs ~3.9 m. Clamping to 3 m would make
# the layer report infeasible exactly in the case it exists for. _back_off still verifies every
# metre against the static map and the fleet's claims, so a longer ask is not a less safe one.
MAX_ATTEMPTS = 3  # then let L2 hold and L3 re-route, as before
MOVER_RANGE = 14.0  # m ahead in the borrowed lane: beyond this a robot is not part of the standoff
STOPPED_V = 0.05  # m/s, below this a peer counts as standing (same test as _other_lane_obstacles)
PLAN_TTL = 20.0  # s a broadcast plan stays valid for its movers

# a peer in one of these cannot be asked to back off, so it is a barrier instead
UNCOMMANDABLE = ("fault", "estop", "paused", "loading", "unloading", "charging")


@dataclass
class StandoffPlan:
    """One chain reverse, broadcast to its members. Mirrors :class:`~nrda_fleet.deadlock.DRPPlan`."""

    epoch: str
    bypasser: str
    obstacle: str
    moves: dict[str, float]  # robot id -> metres it reverses along its own heading
    members: list[str]
    attempt: int
    t_plan: float
    expires: float
    reason: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"))

    @classmethod
    def from_json(cls, s: str) -> "StandoffPlan":
        d = json.loads(s)
        d["moves"] = {k: float(v) for k, v in d["moves"].items()}
        return cls(**d)


@dataclass
class StandoffState:
    """Per-agent L2b bookkeeping. One attempt at a time, keyed by the obstacle being passed."""

    obstacle: str = ""
    attempt: int = 0
    next_try: float = 0.0
    mine: StandoffPlan | None = None  # the plan this robot coordinates
    owed: dict[str, float] = field(default_factory=dict)  # epoch -> metres still owed by me

    def reset(self, obstacle: str) -> None:
        self.obstacle, self.attempt, self.next_try, self.mine = obstacle, 0, 0.0, None


def _lane_of(line, ya: float) -> str | None:
    """Which of the two lanes a lateral frame coordinate sits in, or None if it is neither."""
    for lane in ("A", "B"):
        if abs(ya - line.lane_y(lane)) < line.w * 0.6:
            return lane
    return None


def _peer_state(agent, line, rid: str, p) -> tuple[RobotState, str] | None:
    """A gossiped peer as a visison ``RobotState`` in this line's frame, plus its lane letter."""
    s, ya = line.to_frame(p.x, p.y)
    lane = _lane_of(line, ya)
    if lane is None:
        return None
    direction = 1 if lane == "A" else -1
    return RobotState(rid, s, abs(p.v), line.lane_key(lane), direction, ya,
                      False, wrap(p.th - line.theta)), lane


def _commandable(agent, rid: str, p) -> bool:
    """Can we ask this robot to back off? Silent, faulted or docked robots cannot be asked."""
    if agent.t - agent.peer_heard.get(rid, -1e9) > agent.cfg.peer_timeout:
        return False
    if p.mode in UNCOMMANDABLE or (p.health and p.health != "HEALTHY" and p.health != "DEGRADED"):
        return False
    return not agent._in_station(rid)


def between_lanes(agent, session) -> bool:
    """Is the robot stopped and displaced out of its own lane, with the borrowed lane to clear?

    This, not the controller's phase label, is the condition a chain reverse answers: the robot has
    swung out and stopped, so it needs the lane it is sitting in cleared ahead to creep forward and
    swing back. A robot still in its own lane has nothing to rejoin, and holds for L3 as before.
    """
    if session.obstacle is None or abs(agent.v) > STOPPED_V:
        return False
    s, ya = session.line.to_frame(agent.pose[0], agent.pose[1])
    return abs(ya - session.line.lane_y(session.lane)) > session.line.w * 0.5


def plan_for(agent, session, t: float) -> tuple[StandoffPlan | None, str]:
    """The chain reverse that would let ``agent`` rejoin its lane. Returns (plan or None, why not)."""
    line = session.line
    if session.obstacle is None:
        return None, "no obstacle in frame"
    spec = agent.amr.robot_spec()
    # the session's AisleRobot is the authority: its lane is the one the robot *owns* (the one it is
    # rejoining), while ``y`` shows it displaced into the lane it borrowed. Deriving the lane from the
    # pose instead would read the borrowed lane as its own and plan the chain in the wrong one.
    me = session.ctl.robot.state()
    s, direction = me.s, me.direction

    others: dict[str, tuple[RobotState, object]] = {}
    barriers: list[tuple[str, float, float]] = []
    for rid, p in agent.peers.items():
        got = _peer_state(agent, line, rid, p)
        if got is None:
            continue
        st, _ = got
        if not (0.0 < direction * (st.s - s) < MOVER_RANGE):
            continue  # behind us, or too far ahead to be part of this standoff
        half = agent.amr.length / 2
        if abs(p.v) > STOPPED_V or not _commandable(agent, rid, p):
            # cannot be moved: it bounds how far the robots behind it may be pushed
            barriers.append((st.lane, st.s - half, st.s + half))
            continue
        others[rid] = (st, spec)

    # static obstacles the fleet knows about bound the back-off too
    for name, (ox, oy, L, W) in agent.known_obstacles.items():
        os_, oya = line.to_frame(ox, oy)
        olane = _lane_of(line, oya)
        if olane is not None:
            barriers.append((line.lane_key(olane), os_ - L / 2, os_ + L / 2))

    cfg = EmergencyConfig(max_reverse=MAX_REVERSE)
    rp = plan_chain_reverse(session.ctl.planner, me, session.obstacle, others, t, cfg, tuple(barriers))
    if not rp.feasible:
        return None, rp.reason
    if not rp.moves:
        return None, "the other lane is already clear enough"

    attempt = agent.standoff.attempt + 1
    epoch = f"{agent.rid}:{session.obstacle_id}:{attempt}"
    members = sorted(rp.moves)
    return StandoffPlan(epoch, agent.rid, session.obstacle_id, dict(rp.moves), members,
                        attempt, t, t + PLAN_TTL, rp.reason), ""


def request(agent, session, t: float) -> bool:
    """L2 is holding on robots in the borrowed lane: ask them to chain-reverse. True if a plan went out."""
    st = agent.standoff
    if st.obstacle != session.obstacle_id:
        st.reset(session.obstacle_id)
    if st.attempt >= MAX_ATTEMPTS or t < st.next_try:
        return False

    plan, why = plan_for(agent, session, t)
    st.next_try = t + RETRY_S
    if plan is None:
        agent._log(f"L2b {session.obstacle_id}: no chain reverse ({why})")
        return False
    # never command a robot the DRP is already moving: one layer per robot (deadlock-scope rule)
    busy = {m for p in agent._drp_plans.values() if p.expires > t for m in p.members}
    if busy & set(plan.members):
        agent._log(f"L2b {session.obstacle_id}: DRP already owns {sorted(busy & set(plan.members))}")
        return False

    st.attempt, st.mine = plan.attempt, plan
    agent.stats.standoffs += 1
    agent.publish(AlertMsg(agent.rid, t, "standoff_reverse", "", 0.0, 0.0, float(plan.attempt), plan.to_json()))
    agent._log(f"L2b {session.obstacle_id} try {plan.attempt} of {MAX_ATTEMPTS}: {plan.reason}")
    return True


def on_plan(agent, plan: StandoffPlan, t: float) -> None:
    """A peer's chain reverse names us: reverse our own share, nothing else (no remote actuation)."""
    dist = plan.moves.get(agent.rid)
    if dist is None or plan.expires <= t:
        return
    owed = agent.standoff.owed
    if plan.epoch in owed:
        return  # already acted on this epoch
    owed[plan.epoch] = dist
    while len(owed) > 8:  # the epochs are only kept to stay idempotent under gossip repeats
        owed.pop(next(iter(owed)))
    if agent._back_off(t, dist):
        agent.stats.standoff_yields += 1
        agent._log(f"L2b {plan.epoch}: backing off {dist:.2f} m so {plan.bypasser} can rejoin its lane")
    else:
        agent._log(f"L2b {plan.epoch}: cannot back off {dist:.2f} m (no clear space behind)")
