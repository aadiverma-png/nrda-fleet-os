"""DRP accounting in the fleet ledger: every plan must end up resolved, escalated or timed out."""

from __future__ import annotations

import json

from nrda_fleet.bus import AlertMsg, RobotStateMsg
from nrda_fleet.metrics import FleetLedger


def plan(ledger, epoch, members, kind="D6", rung=1, t=10.0, priority=""):
    detail = json.dumps({"epoch": epoch, "kind": kind, "rung": rung, "members": list(members)})
    ledger.on_message(AlertMsg("AMR_1", t, "drp_plan", priority or members[0], 1.0, 0.0, 1.0, detail), t)


def done(ledger, epoch, detail, t=14.0):
    ledger.on_message(AlertMsg("AMR_1", t, "drp_done", epoch, 0.0, 0.0, 1.0, detail), t)


def test_a_resolved_deadlock_is_accounted_once():
    L = FleetLedger()
    plan(L, "AMR_1:100", ["AMR_1", "AMR_2"])
    done(L, "AMR_1:100", "D6")
    k = L.kpis()
    assert (k["deadlocks"], k["deadlock_episodes"]) == (1, 1)
    assert (k["deadlocks_resolved"], k["deadlocks_timed_out"], k["deadlocks_escalated"]) == (1, 0, 0)
    assert k["deadlocks_unaccounted"] == 0
    assert L.incidents_view()[-1]["status"] == "solved"


def test_a_timed_out_rung_is_counted_and_left_open():
    """The rung expired, so the deadlock is genuinely unresolved - but the attempt must still be on the books."""
    L = FleetLedger()
    plan(L, "AMR_1:100", ["AMR_1", "AMR_2"])
    done(L, "AMR_1:100", "timeout")
    k = L.kpis()
    assert (k["deadlocks"], k["deadlock_episodes"]) == (1, 1)
    assert (k["deadlocks_resolved"], k["deadlocks_timed_out"], k["deadlocks_escalated"]) == (0, 1, 0)
    assert k["deadlocks_unaccounted"] == 0
    assert L.incidents_view()[-1]["status"] != "solved"


def test_three_rungs_then_escalation_are_one_episode():
    """The live-run failure: each rung re-planned with a fresh epoch, inflating plans and losing the timeouts."""
    L = FleetLedger()
    for i, rung in enumerate((1, 2, 3)):
        plan(L, f"AMR_1:10{i}", ["AMR_1", "AMR_2"], rung=rung, t=10.0 + 10 * i, priority="AMR_1")
        done(L, f"AMR_1:10{i}", "timeout", t=14.0 + 10 * i)
    L.on_message(AlertMsg("AMR_1", 44.0, "drp_escalated", "AMR_1", 1.0, 0.0, 1.0, "D6"), 44.0)
    k = L.kpis()
    assert k["deadlocks"] == 3 and k["deadlock_episodes"] == 1  # three attempts, one physical deadlock
    assert (k["deadlocks_timed_out"], k["deadlocks_escalated"], k["deadlocks_resolved"]) == (3, 1, 0)
    assert k["deadlocks_unaccounted"] == 0
    incs = [i for i in L.incidents_view() if i["kind"] == "deadlock"]
    assert len(incs) == 1 and incs[0]["status"] == "needs_you"


def test_nothing_is_ever_unaccounted():
    """Regression guard: no plan ever falls through the three outcomes, whatever the mix."""
    L = FleetLedger()
    assert L.kpis()["deadlocks_unaccounted"] == 0
    plan(L, "AMR_1:100", ["AMR_1", "AMR_2"], t=10.0)
    done(L, "AMR_1:100", "timeout", t=14.0)
    assert L.kpis()["deadlocks_unaccounted"] == 0
    plan(L, "AMR_1:101", ["AMR_1", "AMR_2"], rung=2, t=20.0)
    done(L, "AMR_1:101", "D6", t=24.0)
    assert L.kpis()["deadlocks_unaccounted"] == 0
    plan(L, "AMR_5:200", ["AMR_5", "AMR_6"], t=30.0, priority="AMR_5")
    L.on_message(AlertMsg("AMR_5", 35.0, "drp_escalated", "AMR_5", 9.0, 0.0, 1.0, "D4"), 35.0)
    k = L.kpis()
    assert k["deadlocks"] == 3 and k["deadlock_episodes"] == 2
    assert k["deadlocks_unaccounted"] == 0
    assert (k["deadlocks_resolved"], k["deadlocks_timed_out"], k["deadlocks_escalated"]) == (1, 1, 1)


# ------------------------------------------------------------- a running plan is not a lost deadlock


def test_a_plan_still_running_is_in_flight_not_unaccounted():
    """Measured in a 20-robot run: the KPI read unaccounted=1 mid-rung, then 0 once the plan finished."""
    L = FleetLedger()
    plan(L, "AMR_1:100", ["AMR_1", "AMR_2"])
    k = L.kpis()
    assert (k["deadlocks"], k["deadlock_episodes"]) == (1, 1)
    assert k["deadlocks_in_flight"] == 1
    assert k["deadlocks_unaccounted"] == 0  # it is running, not lost
    done(L, "AMR_1:100", "D6")
    k = L.kpis()
    assert k["deadlocks_in_flight"] == 0
    assert (k["deadlocks_resolved"], k["deadlocks_unaccounted"]) == (1, 0)


def test_several_rungs_in_flight_are_all_counted():
    L = FleetLedger()
    plan(L, "AMR_1:100", ["AMR_1", "AMR_2"])
    plan(L, "AMR_3:200", ["AMR_3", "AMR_4"], t=11.0)
    k = L.kpis()
    assert (k["deadlocks"], k["deadlocks_in_flight"], k["deadlocks_unaccounted"]) == (2, 2, 0)
    done(L, "AMR_1:100", "timeout")
    k = L.kpis()
    assert (k["deadlocks_in_flight"], k["deadlocks_timed_out"], k["deadlocks_unaccounted"]) == (1, 1, 0)


# ------------------------------------------------------- the cycle only closes if every robot names one


def _stand(L, rid, x, y, waiting, t):
    """One stationary robot in a driving mode, parked in the wait-for graph."""
    L.on_message(RobotStateMsg(rid, t, x, y, 0.0, 0.0, 0.0, mode="to_drop", waiting_for=list(waiting)), t)


def _chain(L, head_waits_for, t):
    """AMR_1 -> AMR_17 -> AMR_19 -> (head_waits_for). The real geometry from the Gazebo run."""
    _stand(L, "AMR_1", -27.89, 4.34, ["AMR_17"], t)
    _stand(L, "AMR_17", -29.08, 7.98, ["AMR_19"], t)
    _stand(L, "AMR_19", -29.02, 9.84, head_waits_for, t)


def test_a_closed_wait_loop_is_a_deadlock_not_a_risk():
    """With the head naming the tail the loop closes, so the DRP sees a deadlock to resolve."""
    L = FleetLedger()
    _chain(L, ["AMR_1"], 10.0)
    _chain(L, ["AMR_1"], 20.0)  # T > 2 s of standing wait
    hits = [r for r in L.risk() if r["robots"] == ["AMR_1", "AMR_17", "AMR_19"]]
    assert len(hits) == 1
    assert hits[0]["kind"] == "deadlock" and hits[0]["p"] == 1.0
    assert "closed wait loop" in hits[0]["why"]


def test_a_silent_head_leaves_the_same_jam_as_a_mere_risk():
    """The pre-fix shape: AMR_19 stopped but naming nobody. No cycle, so the DRP declines it."""
    L = FleetLedger()
    _chain(L, [], 10.0)
    _chain(L, [], 20.0)
    hits = [r for r in L.risk() if set(r["robots"]) == {"AMR_1", "AMR_17", "AMR_19"}]
    assert len(hits) == 1
    assert hits[0]["kind"] == "deadlock_risk" and hits[0]["p"] < 1.0
    assert "has waited" in hits[0]["why"]
