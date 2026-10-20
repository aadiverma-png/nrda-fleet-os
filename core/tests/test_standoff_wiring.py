"""L2b in-aisle standoff: the chain reverse must actually reach the fleet.

`visison.standoff.plan_chain_reverse` was implemented and unit-tested (see `test_emergency.py`)
but nothing in the runtime called it: the only name the fleet imported from that module was
`conflict_groups`. A bypasser stuck between the lanes therefore held for 30 s and re-routed
(`bypass.BypassSession.tick`) while the DRP declined the group as "L2's"
(`agent.EdgeAgent._check_deadlock`), so the robot standing in the way was never asked to move.

These are the wiring guards: the plan is produced from gossiped peer state, goes out as one
`AlertMsg`, is executed only by the robot that receives its own share, and never fights the DRP
for the same robots. The recurrence itself is `test_emergency.py`'s job, not this file's.
"""

import pytest

from nrda_fleet import standoff as l2b
from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import AlertMsg, RobotStateMsg
from nrda_fleet.bypass import AisleRobot, BypassSession, obstacle_in_frame
from nrda_fleet.deadlock import DRPPlan, Move
from nrda_fleet.geom import OrientedBox
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site
from visison.controller import BypassController, ControllerState
from visison.planner import BypassPlanner, PlannerConfig, SafetyMargins

T0 = 100.0
LINE = "V02"  # a plain two-lane vertical aisle, no stations in the stretch used here


@pytest.fixture(scope="module")
def world():
    s = Site.load()
    return s, LaneGraph.load_or_build(s)


@pytest.fixture(scope="module")
def dist(world):
    return StationDistances(world[1])


def _line(world):
    return world[1].lines[LINE]


def _xy(world, lane: str, s: float) -> tuple[float, float]:
    """World point on a lane centre at frame position ``s``."""
    return _line(world).to_world(s, _line(world).lane_y(lane))


def _agent(world, dist, rid: str, sent: list | None = None) -> EdgeAgent:
    pub = (lambda m: sent.append(m)) if sent is not None else (lambda m: None)
    ag = EdgeAgent(rid, world[1], dist, pub, AgentConfig(), "REST_E1")
    ag.t = T0
    ag.station = None
    ag.v = 0.0
    return ag


def _add_peer(ag, world, rid: str, lane: str, s: float, mode: str = "idle", v: float = 0.0,
              health: str = "HEALTHY") -> None:
    """Gossip one peer standing on a lane centre, the way the real bus would."""
    x, y = _xy(world, lane, s)
    th = _line(world).lane_heading(lane)
    ag.on_message(RobotStateMsg(rid, T0, x, y, th, v, 0.0, mode=mode, health=health))
    ag.peer_heard[rid] = T0


def _session(ag, world, lane: str = "A", s_me: float = 24.0, s_box: float = 26.0) -> BypassSession:
    """A bypasser displaced into the other lane, beside a box blocking its own lane ahead.

    This is the state the old code could only hold in: the robot owns ``lane``, is laterally in the
    other one, and needs that other lane clear ahead to creep forward and swing back.
    """
    line = _line(world)
    other = "B" if lane == "A" else "A"
    bx, by = _xy(world, lane, s_box)
    ag.pose = (*_xy(world, other, s_me), line.lane_heading(lane))  # displaced into the borrowed lane
    box = OrientedBox(bx, by, 0.6, 1.0, 0.0)
    ob = obstacle_in_frame(line, box, 0.05, "pallet_1")
    spec = ag.amr.robot_spec()
    cfg = PlannerConfig(clearance=0.12, margins=SafetyMargins(comm_latency=0.15, reservation_jitter=0.35,
                                                              other_brake_time=0.5), max_wait=45.0,
                        horizon=90.0, reroute_penalty=8.0)
    ctl = BypassController(robot=AisleRobot(ag, line, lane), planner=BypassPlanner(line.aisle, spec, cfg))
    ctl.state, ctl.obstacle = ControllerState.HOLDING, ob
    return BypassSession(ag, ctl, line, lane, ob.obstacle_id, T0, ob)


# ------------------------------------------------------- A: the plan reaches the fleet


def test_a_stopped_robot_in_the_borrowed_lane_is_asked_to_back_off(world, dist):
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)  # standing in the lane AMR_3 needs

    assert l2b.request(ag, sess, T0) is True
    msgs = [m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse"]
    assert len(msgs) == 1, "the chain reverse must go out as exactly one broadcast"
    plan = l2b.StandoffPlan.from_json(msgs[0].detail)
    assert plan.bypasser == "AMR_3" and plan.obstacle == "pallet_1"
    assert "AMR_7" in plan.moves and plan.moves["AMR_7"] > 0.0
    assert ag.stats.standoffs == 1


def test_an_already_clear_lane_asks_nobody(world, dist):
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 40.0)  # far beyond the stretch needed

    assert l2b.request(ag, sess, T0) is False
    assert not [m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse"]
    assert ag.stats.standoffs == 0


# ------------------------------------------------- B: only the named robot acts, and only its share


def test_the_named_robot_reverses_its_own_share_and_others_ignore_the_plan(world, dist):
    mover = _agent(world, dist, "AMR_7")
    mover.pose = (*_xy(world, "B", 26.0), _line(world).lane_heading("B"))
    mover.mode = "to_drop"  # a moving mode: _back_off refuses to reverse a parked robot
    plan = l2b.StandoffPlan("AMR_3:pallet_1:1", "AMR_3", "pallet_1", {"AMR_7": 1.3}, ["AMR_7"],
                            1, T0, T0 + l2b.PLAN_TTL, "test")

    l2b.on_plan(mover, plan, T0)
    assert mover.stats.standoff_yields == 1
    assert mover.legs and mover.legs[0].tag == "backoff" and mover.legs[0].reverse is True

    bystander = _agent(world, dist, "AMR_9")
    bystander.mode = "to_drop"
    l2b.on_plan(bystander, plan, T0)
    assert bystander.stats.standoff_yields == 0 and not bystander.legs


def test_a_repeated_plan_is_executed_once(world, dist):
    mover = _agent(world, dist, "AMR_7")
    mover.pose = (*_xy(world, "B", 26.0), _line(world).lane_heading("B"))
    mover.mode = "to_drop"
    plan = l2b.StandoffPlan("AMR_3:pallet_1:1", "AMR_3", "pallet_1", {"AMR_7": 1.3}, ["AMR_7"],
                            1, T0, T0 + l2b.PLAN_TTL, "test")
    l2b.on_plan(mover, plan, T0)
    l2b.on_plan(mover, plan, T0 + 0.5)  # gossip repeat
    assert mover.stats.standoff_yields == 1


def test_an_expired_plan_is_ignored(world, dist):
    mover = _agent(world, dist, "AMR_7")
    mover.mode = "to_drop"
    plan = l2b.StandoffPlan("e", "AMR_3", "pallet_1", {"AMR_7": 1.3}, ["AMR_7"], 1, T0, T0 + 5.0, "test")
    l2b.on_plan(mover, plan, T0 + 6.0)
    assert mover.stats.standoff_yields == 0 and not mover.legs


# -------------------------------------- C: robots we cannot command bound the chain, not join it


@pytest.mark.parametrize("mode,health", [("fault", "HEALTHY"), ("estop", "HEALTHY"),
                                         ("loading", "HEALTHY"), ("idle", "LIDAR_FAIL")])
def test_a_robot_that_cannot_be_commanded_is_a_barrier_not_a_mover(world, dist, mode, health):
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0, mode=mode, health=health)

    plan, why = l2b.plan_for(ag, sess, T0)
    assert plan is None, f"{mode}/{health} must never be asked to back off"
    assert why, "an infeasible chain must say why, so the hold-then-reroute path stays explainable"


def test_a_moving_robot_is_not_asked_to_back_off(world, dist):
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0, mode="to_drop", v=0.8)

    plan, _ = l2b.plan_for(ag, sess, T0)
    assert plan is None or "AMR_7" not in plan.moves


# ------------------------------------------------ D: one layer per robot (the deadlock-scope rule)


def test_the_drp_keeps_the_robots_it_already_owns(world, dist):
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)
    ag._drp_plans["e1"] = DRPPlan("e1", "AMR_1", "D1", "AMR_1", [Move("AMR_7", "backoff", 1.0)],
                                  [], ["AMR_7", "AMR_1"], 1, T0, T0 + 30.0, "test")

    assert l2b.request(ag, sess, T0) is False
    assert not [m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse"]


def test_an_expired_drp_plan_no_longer_blocks_the_standoff(world, dist):
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)
    ag._drp_plans["e1"] = DRPPlan("e1", "AMR_1", "D1", "AMR_1", [Move("AMR_7", "backoff", 1.0)],
                                  [], ["AMR_7"], 1, T0, T0 - 1.0, "expired")

    assert l2b.request(ag, sess, T0) is True


# ---------------------------------------------------------------- E: retries are bounded


def test_attempts_are_capped_so_l2_can_still_reroute(world, dist):
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)

    t = T0
    for _ in range(l2b.MAX_ATTEMPTS):
        assert l2b.request(ag, sess, t) is True
        t += l2b.RETRY_S
    assert l2b.request(ag, sess, t) is False, "after the cap, the hold-then-reroute path must take over"
    assert ag.stats.standoffs == l2b.MAX_ATTEMPTS


def test_a_retry_waits_for_the_movers(world, dist):
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)

    assert l2b.request(ag, sess, T0) is True
    assert l2b.request(ag, sess, T0 + 0.5) is False  # no second plan before the movers have had time
    assert l2b.request(ag, sess, T0 + l2b.RETRY_S) is True


# ------------------------------------------- F: L2's hold actually reaches this layer


def test_the_l2_hold_triggers_the_chain_reverse(world, dist):
    """The seam that was missing: BypassSession.tick must ask L2b while it holds."""
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)

    sess.tick(T0 + 1.0)
    assert [m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse"], \
        "a holding bypass session must ask the borrowed lane to clear"


def test_the_alert_reaches_the_executor_through_on_message(world, dist):
    """End to end over the bus: the bypasser's broadcast makes the named peer reverse."""
    sent = []
    bypasser = _agent(world, dist, "AMR_3", sent)
    sess = _session(bypasser, world)
    _add_peer(bypasser, world, "AMR_7", "B", 26.0)
    assert l2b.request(bypasser, sess, T0) is True
    msg = next(m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse")

    mover = _agent(world, dist, "AMR_7")
    mover.pose = (*_xy(world, "B", 26.0), _line(world).lane_heading("B"))
    mover.mode = "to_drop"
    mover.on_message(msg)  # the real path: AlertMsg -> _on_alert -> l2b.on_plan
    assert mover.stats.standoff_yields == 1
    assert mover.legs and mover.legs[0].tag == "backoff"


def test_a_robot_still_in_its_own_lane_holds_for_l3_instead(world, dist):
    """The trigger is "stopped out of our lane", not the controller's phase: in-lane holds are L3's."""
    sent = []
    ag = _agent(world, dist, "AMR_3", sent)
    sess = _session(ag, world)
    line = _line(world)
    ag.pose = (*_xy(world, "A", 24.0), line.lane_heading("A"))  # back on its own lane centre
    _add_peer(ag, world, "AMR_7", "B", 26.0)

    assert l2b.between_lanes(ag, sess) is False
    sess.tick(T0 + 1.0)
    assert not [m for m in sent if isinstance(m, AlertMsg) and m.kind == "standoff_reverse"]


def test_a_moving_robot_does_not_open_a_standoff(world, dist):
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    ag.v = 0.7
    assert l2b.between_lanes(ag, sess) is False


def test_the_chain_reverse_frees_the_stretch_the_bypasser_needs(world, dist):
    """The point of the whole layer: every mover ends up further up the aisle, and still inside it."""
    ag = _agent(world, dist, "AMR_3")
    sess = _session(ag, world)
    _add_peer(ag, world, "AMR_7", "B", 26.0)
    _add_peer(ag, world, "AMR_9", "B", 27.4)  # a tight queue behind it: the chain must push both

    plan, _ = l2b.plan_for(ag, sess, T0)
    assert plan is not None and plan.moves
    me = sess.ctl.robot.state()
    half, length = ag.amr.length / 2, _line(world).aisle.length
    for rid, moved in plan.moves.items():
        assert 0.0 < moved <= l2b.MAX_REVERSE, f"{rid} asked for an unusable {moved} m"
        s_before, _ = _line(world).to_frame(ag.peers[rid].x, ag.peers[rid].y)
        s_after = s_before + me.direction * moved
        assert -1e-6 <= s_after - half and s_after + half <= length + 1e-6, f"{rid} pushed out of the aisle"
