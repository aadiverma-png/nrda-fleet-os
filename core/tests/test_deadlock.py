"""Deadlock Resolution Protocol: classification, plans and the chain reverse (nrda_fleet/deadlock.py)."""

from __future__ import annotations

import math

from nrda_fleet.deadlock import (DRPPlan, Member, ProgressWatch, classify, deadlock_group, make_plan,
                                 plan_backoffs, corridor)

L, W = 1.2, 0.78
NO_STATIC = lambda x, y, th: False  # noqa: E731


def m(rid, x, y, th=0.0, prio=2, lane="H2/A", core=False, station=False, bypass=False, path=()):
    return Member(rid, x, y, th, 0.0, prio, lane, core, station, bypass, (), path)


def test_classify_every_type():
    assert classify([m("AMR_1", 0, 0), m("AMR_2", -1.8, 0)], False)[0] == "D1"
    assert classify([m("AMR_1", 0, 0), m("AMR_2", 3, 0, th=math.pi)], False)[0] == "D2"
    assert classify([m("AMR_1", 0, 0, bypass=True), m("AMR_2", 3, 0.6, lane="H2/B")], False)[0] == "D2"
    assert classify([m("AMR_1", 0, 0, core=True), m("AMR_2", 0, 2, th=-math.pi / 2, lane="V01/B")], False)[0] == "D3"
    assert classify([m("AMR_1", 0, 0, station=True), m("AMR_2", 2, 0)], False)[0] == "D4"
    assert classify([m("AMR_1", 0, 0), m("AMR_2", 0, 2, lane="V01/B")], True)[0] == "D5"
    assert classify([m("AMR_1", 0, 0), m("AMR_2", 0, 2, lane="V01/B")], False)[0] == "D6"


def test_queue_leader_goes_first_followers_hold():
    grp = [m("AMR_3", 0.0, 0.0), m("AMR_7", 1.8, 0.0)]  # AMR_7 is ahead (heading +x)
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_3", 1, 10.0, False, NO_STATIC, L, W)
    assert plan.kind == "D1" and plan.priority == "AMR_7"
    assert [(mv.rid, mv.action) for mv in plan.moves] == [("AMR_3", "hold"), ("AMR_7", "pass")]


def test_crossing_occupant_has_priority_and_others_back_off():
    inside = m("AMR_5", 0.0, 0.0, core=True, path=((3.0, 0.0),))
    waiting = m("AMR_2", 1.6, 0.0, th=math.pi, lane="H2/B")  # facing it, in its way
    grp = [inside, waiting]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_2", 1, 0.0, False, NO_STATIC, L, W)
    assert plan.kind == "D3" and plan.priority == "AMR_5"
    mv = plan.move_of("AMR_2")
    assert mv.action == "backoff" and mv.dist > 0
    # after the back-off AMR_2 is clear of the priority robot's corridor
    from nrda_fleet.deadlock import _body, _hits
    after = _body(waiting.x - mv.dist * math.cos(waiting.th), waiting.y, waiting.th, L, W)
    assert not _hits(after, corridor(inside, L, W))


def test_chain_reverse_pushes_the_robot_behind_first():
    prio = m("AMR_1", 0.0, 0.0, path=((3.0, 0.0),))
    yielder = m("AMR_4", 1.8, 0.0, th=math.pi, lane="H2/B")  # facing AMR_1
    behind = m("AMR_9", 3.3, 0.0, th=math.pi, lane="H2/B")  # right behind the yielder
    everyone = {x.rid: x for x in (prio, yielder, behind)}
    moves, swept, why = plan_backoffs([yielder], corridor(prio, L, W), everyone, NO_STATIC, L, W, {"AMR_1"})
    assert why == ""
    assert [mv.rid for mv in moves] == ["AMR_9", "AMR_4"]  # deepest in the chain moves first
    assert all(mv.dist > 0 for mv in moves) and swept


def test_infeasible_chain_falls_back_to_reroute():
    prio = m("AMR_1", 0.0, 0.0, path=((3.0, 0.0),))
    yielder = m("AMR_4", 1.8, 0.0, th=math.pi, lane="H2/B")
    wall_behind = lambda x, y, th: x > 2.0  # noqa: E731  (no room to back off)
    grp = [prio, yielder]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_1", 1, 0.0, False, wall_behind, L, W)
    assert plan.move_of("AMR_4").action == "reroute" or plan.move_of("AMR_1").action == "reroute"


def test_obstacle_deadlock_reroutes_the_robot_facing_it():
    grp = [m("AMR_2", 0.0, 0.0), m("AMR_6", 0.0, 2.0, lane="V09/A")]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_2", 1, 0.0, True, NO_STATIC, L, W, facing_obstacle="AMR_6")
    assert plan.kind == "D5" and plan.move_of("AMR_6").action == "reroute" and plan.priority == "AMR_2"


def test_escalation_rung_three_reroutes_lowest_precedence():
    grp = [m("AMR_2", 0.0, 0.0, prio=3), m("AMR_6", 0.0, 2.0, lane="V09/A", prio=1)]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_2", 3, 0.0, False, NO_STATIC, L, W)
    assert plan.move_of("AMR_6").action == "reroute"


def test_plan_round_trips_through_the_peer_bus():
    grp = [m("AMR_3", 0.0, 0.0), m("AMR_7", 1.8, 0.0)]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_3", 1, 12.3, False, NO_STATIC, L, W)
    back = DRPPlan.from_json(plan.to_json())
    assert back.epoch == plan.epoch and back.moves == plan.moves and back.zone == plan.zone


def test_group_merges_cycle_and_nearby_stalled_robots():
    members = {r: m(r, x, 0.0) for r, x in (("AMR_1", 0.0), ("AMR_2", 1.8), ("AMR_3", 3.6), ("AMR_4", 20.0))}
    groups = deadlock_group([["AMR_1", "AMR_2"]], ["AMR_3", "AMR_4"], members)
    g = next(g for g in groups if "AMR_1" in g)
    assert g == {"AMR_1", "AMR_2", "AMR_3"}  # AMR_4 is 20 m away: its own group


def test_no_progress_watchdog():
    w = ProgressWatch()
    assert not w.update(0.0, 0.0, 0.0, True)
    assert not w.update(10.0, 0.1, 0.0, True)
    assert w.update(20.5, 0.1, 0.0, True)  # < 0.3 m in 20 s while driving
    assert not w.update(21.0, 1.0, 0.0, True)  # moved: reset
    assert not w.update(60.0, 1.0, 0.0, False)  # not driving (loading, parked): never stalled


# ------------------------------------------------------------------ agent integration (the protocol path)

import pytest  # noqa: E402

from nrda_fleet.agent import AgentConfig  # noqa: E402
from nrda_fleet.lane_graph import LaneGraph  # noqa: E402
from nrda_fleet.sim2d import Sim2D  # noqa: E402
from nrda_fleet.site import Site  # noqa: E402
from nrda_fleet.tasks import Task  # noqa: E402


@pytest.fixture(scope="module")
def world():
    s = Site.load()
    return s, LaneGraph.load_or_build(s)


def test_agents_coordinate_a_wait_for_cycle(world):
    site, graph = world
    sim = Sim2D(site, graph, AgentConfig(), robots=2, seed=1, lidar=False)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900), Task("T2", "P03", "drop_2", 2, 1.0, 900)])
    sim.run(20.0)
    a1, a2 = sim.robots["AMR_1"].agent, sim.robots["AMR_2"].agent
    # force a persistent 2-cycle as both replicas would gossip it
    a1.waiting_for = ["AMR_2"]
    a1.peers["AMR_2"].waiting_for = ["AMR_1"]
    for cyc in (("AMR_1", "AMR_2"), ("AMR_2", "AMR_1")):
        a1._cycle_since[cyc] = sim.t - 10.0
    a1._drp_tick(sim.t)
    assert a1._drp_mine is not None  # AMR_1 (lowest number) coordinates
    plan = a1._drp_mine
    assert set(plan.members) >= {"AMR_1", "AMR_2"} and plan.priority in plan.members
    # the plan reached AMR_2 over the bus and it adopted its part
    sim.run(0.5)
    assert plan.epoch in a2._drp_plans


def test_parking_bay_contention_reassignment(world):
    site, graph = world
    from nrda_fleet.agent import EdgeAgent
    from nrda_fleet.allocation import StationDistances
    from types import SimpleNamespace

    dist = StationDistances(graph)
    ag = EdgeAgent("AMR_20", graph, dist, lambda m: None, AgentConfig(), "REST_E1")
    ag.pose = (26.79, 4.95, 0.0)
    ag.station = None
    ag.goal = "REST_E1"
    ag.mode = "to_park"
    ag.t = 100.0

    # Peer AMR_19 is parked at REST_E1
    p19 = SimpleNamespace(
        robot_id="AMR_19", stamp=100.0, x=26.83, y=4.90, th=0.0, v=0.0, w=0.0,
        battery=1.0, mode="parked", station="REST_E1", goal="", legs=(), claims=(),
        waiting_for=(), bypass=False, health="HEALTHY",
    )
    ag.peers["AMR_19"] = p19
    ag.peer_heard["AMR_19"] = 100.0
    ag.waiting_for = ["AMR_19"]

    # Trigger mode handling
    ag._l5(ag.t)

    # Robot must have reallocated away from REST_E1 to a free parking bay
    assert ag.goal != "REST_E1"
    assert ag.goal.startswith("REST_")
    assert ag.mode == "to_park"
    assert len(ag.legs) > 0


def test_intersection_blocked_turn_reroute(world):
    site, graph = world
    from nrda_fleet.agent import EdgeAgent
    from nrda_fleet.allocation import StationDistances

    dist = StationDistances(graph)
    ag = EdgeAgent("AMR_17", graph, dist, lambda m: None, AgentConfig(), "REST_E1")
    ag.pose = (-11.3, 0.29, -1.57)
    ag.station = None
    ag.v = 0.0
    # Box 1 blocks H3/A near intersection X_V05_H3
    ag.known_obstacles["lidar:AMR_11:3"] = (-12.45, -2.0, 1.0, 1.0)
    ag._sync_static_blocks()
    ag._go("P01", "to_pickup")

    # Initial plan attempts turn at X_V05_H3:NW
    plan1 = ag.sched.plan(0.0, *ag.pose, ag.v, ag.legs)
    assert ("H3/A", 35) in [w.res for w in plan1.waits]

    # Handle wait and forbid resource
    ag._forbid_resource("H3/A", 35)
    ag._reroute("avoiding blocked intersection cell")

    # After reroute, the robot detours and plan executes without wait on cell 35
    plan2 = ag.sched.plan(0.0, *ag.pose, ag.v, ag.legs)
    assert ("H3/A", 35) not in [w.res for w in plan2.waits]


def test_classify_opposing_lanes_with_follower():
    # Opposing traffic on VE with a follower queued on H5
    amr7 = m("AMR_7", 23.57, 11.16, th=math.pi / 2, lane="VE/A")
    amr14 = m("AMR_14", 23.57, 14.11, th=-math.pi / 2, lane="VE/B")
    amr2 = m("AMR_2", 22.18, 14.60, th=0.0, lane="H5/A")
    kind, why = classify([amr7, amr14, amr2], False)
    assert kind == "D2"


def test_single_file_corridor_deadlock_reroutes_opposing():
    # Standoff in VE: AMR_14 must reroute, not back off
    amr7 = m("AMR_7", 23.57, 11.16, th=math.pi / 2, lane="VE/A", prio=2)
    amr14 = m("AMR_14", 23.57, 14.11, th=-math.pi / 2, lane="VE/B", prio=2)
    grp = [amr7, amr14]
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_7", 1, 0.0, False, NO_STATIC, L, W)
    assert plan.kind == "D2"
    assert plan.priority == "AMR_7"
    mv14 = plan.move_of("AMR_14")
    assert mv14 is not None and mv14.action == "reroute"

