"""A robot under an L2 manoeuvre keeps claiming the cells it sits in (P2).

L3 stops scheduling while a bypass owns the robot, so its route claims age out. Observed in
Gazebo: a robot standing across both lanes of a corridor published ``claims=0``, leaving only
the momentary "covered but unclaimed" block, which an impatient peer discards after a second.
"""

from __future__ import annotations

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site


def _agent(rid="AMR_1"):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = EdgeAgent(rid, graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots[rid], 1.0, None)
    a.t = 100.0
    a.pose = site.spawn_pose(rid)
    a.v = a.w = 0.0
    return a


def test_route_claims_age_out_without_rescheduling():
    """The precondition: claims really do decay if nothing refreshes them."""
    a = _agent()
    a.book.set_route_claims([(("H4/A", 2), 100.0, 108.0)], 100.0)
    assert a.book.all_own(), "sanity: a claim was booked"
    a.book.prune(200.0)  # 92 s later, nothing refreshed it
    assert not a.book.all_own(), "claims should have aged out"


def test_body_claims_are_held_during_a_manoeuvre():
    a = _agent()
    a.book.occupied = {("H4/A", 2), ("H4/A", 3), ("H4/B", 2), ("VW/A", 59)}
    a._hold_body_claims(a.t)
    claimed = {(c.key, c.idx) for c in a.book.all_own()}
    assert claimed == a.book.occupied, f"body cells not claimed: {claimed}"
    for c in a.book.all_own():
        assert c.end > a.t, "a claim that has already expired is no claim"


def test_body_claims_survive_repeated_refresh_and_pruning():
    """The frozen-robot case: 200 s standing still, still holding its cells."""
    a = _agent()
    a.book.occupied = {("H4/A", 2), ("H4/B", 2)}
    t = 100.0
    while t < 300.0:
        a.book.prune(t)
        a._hold_body_claims(t)
        assert a.book.all_own(), f"lost its claims at t={t}"
        t += 0.25
    claimed = {(c.key, c.idx) for c in a.book.all_own()}
    assert claimed == a.book.occupied


def test_precedence_is_preserved_across_refreshes():
    """Refreshing must not reset the claim stamp, or the robot loses its place in the queue."""
    a = _agent()
    a.book.occupied = {("H4/A", 2)}
    a._hold_body_claims(100.0)
    first = [c.stamp for c in a.book.all_own()][0]
    t = 100.0
    while t < 300.0:  # 200 s frozen, refreshed at the real planning cadence
        t += a.cfg.plan_period
        a.book.prune(t)
        a._hold_body_claims(t)
    later = [c.stamp for c in a.book.all_own()][0]
    assert later == first, f"claim stamp moved {first} -> {later}, precedence lost"


def test_refresh_is_faster_than_the_claim_window():
    """Precedence survives only while the refresh beats expiry; a bypassing robot never goes idle.

    ``_l3`` runs every ``plan_period`` whenever ``bypass`` is set (the 1 Hz idle budget in
    ``tick`` requires ``bypass is None``), and the booked window is ``cfg.window``. Keep the
    margin large, or a long manoeuvre silently loses its place in the queue.
    """
    cfg = AgentConfig()
    assert cfg.window > 4 * cfg.plan_period, (
        f"window {cfg.window}s is too close to the planning period {cfg.plan_period}s: "
        "a dropped tick would reset claim precedence")


def test_borrowed_cells_are_not_disturbed():
    """``extra`` holds the oncoming-lane cells the bypass reserved; refreshing ``own`` must not touch them."""
    from visison.reservation import Reservation
    a = _agent()
    a.book.now = a.t
    assert a.book.try_reserve([Reservation(lane="H4/B", cell=5, start=100.0, end=120.0, robot_id="AMR_1")])
    before = {(c.key, c.idx) for lst in a.book.extra.values() for c in lst}
    a.book.occupied = {("H4/A", 2)}
    a._hold_body_claims(a.t)
    after = {(c.key, c.idx) for lst in a.book.extra.values() for c in lst}
    assert after == before, "the borrowed reservation was clobbered"
