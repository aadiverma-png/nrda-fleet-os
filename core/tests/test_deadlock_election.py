"""DRP yield election: the robot asked to give way must be on the wait-for cycle.

Regression cover for a live incident where ``deadlock_group`` merged a robot in by the 3 m
proximity rule and ``make_plan`` then elected it on precedence alone. It had no wait-for edge,
so moving it could not break the cycle: the real blocker was held on every rung and the
incident escalated to the operator unresolved.
"""

from __future__ import annotations

import math

from nrda_fleet.deadlock import Member, _yielder, cycle_members, make_plan

L, W = 1.2, 0.78
NO_STATIC = lambda x, y, th: False  # noqa: E731


def m(rid, x, y, th=0.0, prio=2, lane="H2/A", core=False, station=False, bypass=False,
      waiting=(), path=()):
    return Member(rid, x, y, th, 0.0, prio, lane, core, station, bypass, tuple(waiting), path)


def outage_group():
    """The measured group {AMR_2, AMR_5, AMR_7, AMR_17}: a 3-cycle plus AMR_17 merged in at < 3 m.

    AMR_17 has the worst precedence of the four at equal priority, so the old group-wide
    ``max(key)`` elected it.
    """
    amr2 = m("AMR_2", 0.0, 0.0, waiting=("AMR_7",), path=((3.0, 0.0),))
    amr7 = m("AMR_7", 2.5, 0.3, th=math.pi / 2, lane="", waiting=("AMR_2", "AMR_5"))
    amr5 = m("AMR_5", 1.0, 2.0, lane="V01/B", waiting=("AMR_2",))
    amr17 = m("AMR_17", 0.5, -1.5, lane="V01/B", waiting=())  # within 3 m, no wait-for edge
    return [amr2, amr7, amr5, amr17]


def test_yielder_is_never_a_proximity_merged_non_cycle_robot():
    grp = outage_group()
    movable = [x for x in grp if not x.in_station]
    # AMR_17 still has the worst precedence of the four: the old rule would have picked it
    assert max(movable, key=lambda x: x.key).rid == "AMR_17"
    cyc = {x.rid for x in cycle_members(grp)}
    chosen = _yielder(grp, movable)
    assert chosen.rid != "AMR_17"
    assert chosen.rid in cyc

    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_2", 1, 0.0, False, NO_STATIC, L, W)
    acting = [mv for mv in plan.moves if mv.action in ("backoff", "reroute")]
    assert acting, "the plan must ask somebody to move"
    assert "AMR_17" not in [mv.rid for mv in acting]
    assert plan.move_of("AMR_17").action == "hold"
    assert any(mv.rid in cyc for mv in acting)


def test_cycle_members_excludes_the_proximity_merged_robot():
    grp = outage_group()
    assert {x.rid for x in cycle_members(grp)} == {"AMR_2", "AMR_7", "AMR_5"}


def test_no_cycle_falls_back_to_the_old_group_wide_election():
    grp = [m("AMR_3", 0.0, 0.0), m("AMR_11", 1.8, 0.0), m("AMR_6", 3.6, 0.0, station=True)]
    movable = [x for x in grp if not x.in_station]
    assert cycle_members(grp) == []
    assert _yielder(grp, movable) is max(movable or grp, key=lambda x: x.key)


def test_all_cycle_group_elects_the_same_robot_as_before():
    grp = [m("AMR_2", 0.0, 0.0, waiting=("AMR_7",)),
           m("AMR_7", 1.8, 0.0, waiting=("AMR_5",)),
           m("AMR_5", 3.6, 0.0, waiting=("AMR_2",))]
    movable = [x for x in grp if not x.in_station]
    assert len(cycle_members(grp)) == 3
    assert _yielder(grp, movable) is max(movable or grp, key=lambda x: x.key)


def test_two_robot_mutual_wait_elects_one_of_the_pair():
    grp = [m("AMR_4", 0.0, 0.0, waiting=("AMR_19",)),
           m("AMR_19", 1.8, 0.0, waiting=("AMR_4",))]
    assert {x.rid for x in cycle_members(grp)} == {"AMR_4", "AMR_19"}
    assert _yielder(grp, grp).rid in {"AMR_4", "AMR_19"}


def test_rung_three_reroute_targets_a_cycle_member():
    grp = outage_group()
    cyc = {x.rid for x in cycle_members(grp)}
    plan = make_plan(grp, {x.rid: x for x in grp}, "AMR_2", 3, 0.0, False, NO_STATIC, L, W)
    reroutes = [mv for mv in plan.moves if mv.action == "reroute"]
    assert len(reroutes) == 1
    assert reroutes[0].rid != "AMR_17"
    assert reroutes[0].rid in cyc
