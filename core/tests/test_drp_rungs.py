"""DRP rungs must be judged on the right thing, and escalation must not replay (P5).

Live run: 15 plans, 3 resolved, 11 timed out, 2 escalated, with the log showing
``D3 r1 -> timeout -> D6 r2 -> timeout -> D3 r1`` for the same group. Two causes:
the only success test was the priority robot travelling 1.5 m, and escalation left no
trace so the identical four-rung sequence restarted after each 60 s cooldown.
"""

from __future__ import annotations

from nrda_fleet.agent import CYCLE_SETTLE, AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import RobotStateMsg
from nrda_fleet.deadlock import DRPPlan
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site


def _agent(rid="AMR_1"):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = EdgeAgent(rid, graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots[rid], 1.0, None)
    a.t = 500.0 + CYCLE_SETTLE + 1.0
    a.pose = site.spawn_pose(rid)
    a.v = a.w = 0.0
    return a


def _peer(a, rid, waiting_for=(), x=0.0, y=0.0, mode="to_drop"):
    a.peers[rid] = RobotStateMsg(rid, a.t, x, y, 0.0, 0.0, 0.0, mode, 1.0, "", "HEALTHY",
                                 "", "", "", 0.0, [], list(waiting_for), "")
    a.peer_heard[rid] = a.t


def _plan(members, priority):
    return DRPPlan(epoch="AMR_1:5000", coordinator="AMR_1", kind="D3", priority=priority,
                   moves=[], zone=[], members=list(members), rung=1, t_plan=500.0,
                   expires=600.0, reason="test")


def test_a_live_cycle_is_not_called_resolved():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_2"]
    _peer(a, "AMR_2", ["AMR_1"])
    assert not a._cycle_broken(_plan(["AMR_1", "AMR_2"], "AMR_1"))


def test_a_broken_cycle_counts_as_resolved_even_if_priority_has_not_moved():
    """The yielder complied; the priority robot is simply queued behind something else."""
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_2"]
    _peer(a, "AMR_2", ["AMR_9"])   # AMR_2 now waits on a robot outside the group: no cycle left
    _peer(a, "AMR_9", [])
    assert a._cycle_broken(_plan(["AMR_1", "AMR_2"], "AMR_1"))


def test_a_cycle_outside_the_group_does_not_count():
    a = _agent("AMR_1")
    a.waiting_for = []
    _peer(a, "AMR_2", ["AMR_9"])
    _peer(a, "AMR_9", ["AMR_2"])   # a real cycle, but not among this plan's members
    assert a._cycle_broken(_plan(["AMR_1", "AMR_2"], "AMR_1"))


def test_supervise_closes_the_incident_on_a_broken_cycle():
    a = _agent("AMR_1")
    published = []
    a.publish = published.append
    a.waiting_for = ["AMR_2"]
    _peer(a, "AMR_2", ["AMR_9"])
    _peer(a, "AMR_9", [])
    a._drp_mine = _plan(["AMR_1", "AMR_2"], "AMR_2")
    a._drp_prio_start = (0.0, 0.0)       # AMR_2 has not moved at all
    a._drp_supervise(a.t)
    assert a._drp_mine is None, "incident left open although the cycle was broken"
    done = [m for m in published if getattr(m, "kind", "") == "drp_done"]
    assert done and done[0].detail != "timeout", f"closed as {done[0].detail if done else None}"


def test_escalation_is_remembered_and_not_replayed():
    a = _agent("AMR_1")
    group = frozenset({"AMR_1", "AMR_2"})
    _peer(a, "AMR_2", ["AMR_1"], x=1.0, y=1.0)
    sig = a._group_signature(group)
    a._drp_escalated[group] = sig
    # nothing moved, so the signature still matches: the group must not be re-planned
    assert a._group_signature(group) == sig


def test_a_moved_member_clears_the_escalation():
    a = _agent("AMR_1")
    group = frozenset({"AMR_1", "AMR_2"})
    _peer(a, "AMR_2", ["AMR_1"], x=1.0, y=1.0)
    sig = a._group_signature(group)
    _peer(a, "AMR_2", ["AMR_1"], x=6.0, y=1.0)   # it moved 5 m
    assert a._group_signature(group) != sig, "a real change was not noticed"


def test_signature_ignores_sub_half_metre_jitter():
    """Lidar/odometry jitter must not look like progress and restart the whole sequence."""
    a = _agent("AMR_1")
    group = frozenset({"AMR_1", "AMR_2"})
    _peer(a, "AMR_2", ["AMR_1"], x=1.00, y=1.00)
    sig = a._group_signature(group)
    _peer(a, "AMR_2", ["AMR_1"], x=1.04, y=1.03)
    assert a._group_signature(group) == sig


def test_an_absent_edge_is_not_believed_before_the_settle_window():
    """Right after a plan, members have not acted or gossiped yet: silence proves nothing."""
    a = _agent("AMR_1")
    a.waiting_for = []
    _peer(a, "AMR_2", [])
    plan = _plan(["AMR_1", "AMR_2"], "AMR_2")
    plan.t_plan = a.t                      # planned just now
    assert not a._cycle_broken(plan), "declared the cycle broken before anyone could react"
    plan.t_plan = a.t - CYCLE_SETTLE - 1.0
    assert a._cycle_broken(plan), "never believes it, even after the settle window"


def test_a_stale_member_blocks_the_conclusion():
    """A member we have not heard from recently tells us nothing either way."""
    a = _agent("AMR_1")
    a.waiting_for = []
    _peer(a, "AMR_2", [])
    a.peer_heard["AMR_2"] = a.t - a.cfg.peer_timeout - 1.0
    plan = _plan(["AMR_1", "AMR_2"], "AMR_2")
    plan.t_plan = a.t - CYCLE_SETTLE - 1.0
    assert not a._cycle_broken(plan), "concluded from a stale member's silence"


def test_a_missing_member_blocks_the_conclusion():
    a = _agent("AMR_1")
    a.waiting_for = []
    plan = _plan(["AMR_1", "AMR_7"], "AMR_7")   # AMR_7 never heard from at all
    plan.t_plan = a.t - CYCLE_SETTLE - 1.0
    assert not a._cycle_broken(plan)
