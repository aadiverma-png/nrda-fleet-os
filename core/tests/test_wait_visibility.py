"""Wait-list observability: a robot that stands still in a driving mode must name who it waits for.

`metrics.risk()` and `deadlock.py` both build the fleet's wait-for graph out of `RobotState.waiting_for`,
so a blocked robot that publishes an empty list is invisible to deadlock detection. Measured in a 562 s
Gazebo run: AMR_17 moved 0.19 m in 137 s with `waiting_for=[]` throughout. These are the regression guards.
"""

import pytest

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import ClaimMsg
from nrda_fleet.deadlock import RUNG_S, DRPPlan, Move
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.sim2d import Sim2D
from nrda_fleet.site import Site
from nrda_fleet.tasks import Task

T0 = 100.0


@pytest.fixture(scope="module")
def world():
    s = Site.load()
    return s, LaneGraph.load_or_build(s)


@pytest.fixture(scope="module")
def dist(world):
    return StationDistances(world[1])


def _agent(world, dist, rid="AMR_3"):
    """A driving agent with a real route, no peers, standing still."""
    ag = EdgeAgent(rid, world[1], dist, lambda m: None, AgentConfig(), "REST_E1")
    ag.t = T0
    ag.station = None
    ag.v = 0.0
    return ag


def _hold_plan(held: str, priority: str = "AMR_4") -> DRPPlan:
    return DRPPlan("e1", priority, "D1", priority, [Move(held, "hold"), Move(priority, "pass")],
                   [], [held, priority], 1, T0, T0 + 30.0, "test")


# ------------------------------------------------------------------ A: the DRP hold edge


def test_drp_hold_publishes_the_priority_robot_as_blocker(world, dist):
    ag = _agent(world, dist)
    ag._go("P09", "to_pickup")
    ag._drp_on_plan(_hold_plan("AMR_3"))
    assert ag._drp_prio_rid == "AMR_4"  # the hold used to leave this empty, so the fallback was dead code
    # the held robot plans with no legs (the `_l3` hold branch): the scheduler names no blocker
    plan = ag.sched.plan(T0, *ag.pose, 0.0, [], hold_final=ag.cfg.window)
    assert not plan.blocked_by and not plan.auth_blockers
    ag._handle_waits(T0, plan)
    assert ag.waiting_for == ["AMR_4"]


def test_drp_hold_edge_goes_when_the_hold_lapses(world, dist):
    ag = _agent(world, dist)
    ag._go("P09", "to_pickup")
    ag._drp_on_plan(_hold_plan("AMR_3"))
    ag._drp_tick(ag._drp_hold_until + 1.0)
    assert ag._drp_prio_rid == ""
    plan = ag.sched.plan(T0, *ag.pose, 0.0, [], hold_final=ag.cfg.window)
    ag._handle_waits(ag._drp_hold_until + 1.0, plan)
    assert ag.waiting_for == []


def test_drp_done_clears_the_hold_edge(world, dist):
    ag = _agent(world, dist)
    ag._go("P09", "to_pickup")
    plan = _hold_plan("AMR_3")
    ag._drp_on_plan(plan)
    ag._drp_on_done(plan.epoch)
    assert ag._drp_hold_until == 0.0 and ag._drp_prio_rid == ""


# ------------------------------------------------------------------ B: the hold outlasts the rung


def test_drp_hold_covers_the_whole_rung(world, dist):
    ag = _agent(world, dist)
    ag._go("P09", "to_pickup")
    ag._drp_on_plan(_hold_plan("AMR_3"))
    # a 10 s hold lapsed half-way through the 20 s rung and the re-plan re-booked the cell with its old stamp
    assert ag._drp_hold_until - ag.t >= RUNG_S
    assert ag._drp_hold_until - ag.t <= RUNG_S + 1e-9  # and never past the plan's own expiry


# ------------------------------------------------------------------ C: the occupied station ahead


def test_route_cut_at_an_occupied_station_names_the_occupant(world, dist):
    ag = _agent(world, dist)
    ag._go("charger_E1", "to_charger")
    assert ag._legs_to_plan() == ag.legs and ag._station_blocker == ""  # free: full route, no edge
    ag.book.on_msg(ClaimMsg("AMR_4", T0, 1, 0, [], [("S:charger_E1", 0)]), T0)
    ag.book.now = T0
    assert ag._station_taken_rid("charger_E1") == "AMR_4" and ag._station_taken("charger_E1")
    cut = ag._legs_to_plan()
    assert len(cut) < len(ag.legs)  # held short of the doorway
    assert ag._station_blocker == "AMR_4"
    plan = ag.sched.plan(T0, *ag.pose, 0.0, cut, hold_final=5.0, w=ag.w)
    assert not plan.blocked_by and not plan.waits  # nothing in the schedule names the occupant
    ag._handle_waits(T0, plan)
    assert ag.waiting_for == ["AMR_4"]


def test_station_blocker_clears_when_the_occupant_goes(world, dist):
    ag = _agent(world, dist)
    ag._go("charger_E1", "to_charger")
    ag.book.on_msg(ClaimMsg("AMR_4", T0, 1, 0, [], [("S:charger_E1", 0)]), T0)
    ag.book.now = T0
    ag._legs_to_plan()
    assert ag._station_blocker == "AMR_4"
    ag.book.forget("AMR_4")
    assert ag._legs_to_plan() == ag.legs
    assert ag._station_blocker == ""
    plan = ag.sched.plan(T0, *ag.pose, 0.0, ag.legs, hold_final=5.0, w=ag.w)
    ag._handle_waits(T0, plan)
    assert ag.waiting_for == []


def test_station_helpers_never_name_the_robot_itself(world, dist):
    ag = _agent(world, dist)
    assert ag._station_taken_rid("charger_E1") == "" and ag._station_active_rid("charger_E1") == ""


# ------------------------------------------------------------------ no false edges


def test_a_free_moving_robot_publishes_no_blockers(world, dist):
    site, graph = world
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=False)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    ag = sim.robots["AMR_1"].agent
    moving = 0
    for _ in range(40):
        sim.run(1.0)
        if abs(ag.v) > 0.05:
            moving += 1
            assert ag.waiting_for == [], (sim.t, ag.mode, ag.waiting_for)
    assert moving > 10  # the run really did drive
    assert ag._station_blocker == ""
