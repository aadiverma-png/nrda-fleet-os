"""A wait chain that cannot resolve itself must not be honoured for ever (P4).

The DRP acts only on cycles. A chain is normally a queue - the robot in front is moving and we
follow it - but when the chain ends at a robot that is parked, idle or silent, nobody in it is
going anywhere. Observed in Gazebo: AMR_1 -> AMR_3 -> AMR_19 -> AMR_17(parked, 56 m away);
three robots held station for 264 s while the console reported p=0.999 and nothing acted.
"""

from __future__ import annotations

import pytest

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import RobotStateMsg
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site


def _agent(rid="AMR_1"):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = EdgeAgent(rid, graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots[rid], 1.0, None)
    a.t = 1000.0
    a.pose = site.spawn_pose(rid)
    a.v = a.w = 0.0
    a.goal = "drop_1"
    a.mode = "to_drop"
    return a


def _peer(a, rid, mode, waiting_for=(), heard=None):
    a.peers[rid] = RobotStateMsg(rid, a.t, 0.0, 0.0, 0.0, 0.0, 0.0, mode, 1.0, "", "HEALTHY",
                                 "", "", "", 0.0, [], list(waiting_for), "")
    a.peer_heard[rid] = a.t if heard is None else heard


def test_chain_ending_at_a_parked_robot_is_dead():
    """The observed topology, exactly."""
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "to_pickup", ["AMR_19"])
    _peer(a, "AMR_19", "to_drop", ["AMR_17"])
    _peer(a, "AMR_17", "parked", [])
    head, why = a._chain_head()
    assert head == "AMR_17", f"followed the chain to {head}"
    assert why == "parked"


def test_a_queue_behind_a_moving_robot_is_not_dead():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "to_pickup", ["AMR_19"])
    _peer(a, "AMR_19", "to_drop", [])        # working its way forward, no edge
    head, why = a._chain_head()
    assert why == "driving", f"a live queue was called {why}"


def test_a_cycle_is_left_to_the_drp():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "to_pickup", ["AMR_1"])   # closes back on us
    head, why = a._chain_head()
    assert why == "cycle"


def test_a_silent_chain_head_is_dead():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "to_pickup", ["AMR_9"])
    _peer(a, "AMR_9", "to_drop", [], heard=0.0)  # last heard 1000 s ago
    head, why = a._chain_head()
    assert (head, why) == ("AMR_9", "silent")


@pytest.mark.parametrize("mode", ["parked", "idle", "charging", "fault"])
def test_every_non_driving_head_counts_as_dead(mode):
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", mode, [])
    _head, why = a._chain_head()
    assert why == mode


def test_the_robot_reroutes_only_after_chain_timeout():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "parked", [])
    rerouted = []
    a._reroute = lambda why: rerouted.append(why)

    a._check_dead_chain(a.t)                      # first sight: start the clock, do nothing
    assert not rerouted
    a._check_dead_chain(a.t + a.cfg.chain_timeout - 1.0)
    assert not rerouted, "gave up before chain_timeout"
    a._check_dead_chain(a.t + a.cfg.chain_timeout + 1.0)
    assert rerouted, "never gave up on a dead chain"
    assert "AMR_3" in rerouted[0]
    assert a.waiting_for == [], "kept advertising a wait it has abandoned"
    assert a.stats.dead_chains == 1


def test_a_chain_that_starts_moving_again_resets_the_clock():
    a = _agent("AMR_1")
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "parked", [])
    rerouted = []
    a._reroute = lambda why: rerouted.append(why)
    a._check_dead_chain(a.t)
    _peer(a, "AMR_3", "to_pickup", [])            # it woke up and is driving
    a._check_dead_chain(a.t + 5.0)
    assert a._chain_since is None, "clock not reset when the chain came alive"
    a._check_dead_chain(a.t + a.cfg.chain_timeout + 10.0)
    assert not rerouted, "gave up on a chain that had recovered"


def test_timeout_is_above_the_drp_persist_window():
    """A real cycle must get resolved by the DRP before this rule fires."""
    cfg = AgentConfig()
    assert cfg.chain_timeout > cfg.deadlock_persist * 2


# ------------------------------------------------------------------ it has to be observable

def test_giving_up_is_announced_on_the_bus():
    """A silent recovery is one you cannot tell happened; the ledger counts these."""
    a = _agent("AMR_1")
    published = []
    a.publish = published.append
    a._reroute = lambda why: None
    a.waiting_for = ["AMR_3"]
    _peer(a, "AMR_3", "parked", [])
    a._check_dead_chain(a.t)
    a._check_dead_chain(a.t + a.cfg.chain_timeout + 1.0)
    alerts = [m for m in published if getattr(m, "kind", "") == "dead_chain"]
    assert alerts, "gave up on a chain without telling the fleet"
    assert alerts[0].target == "AMR_3"
    assert alerts[0].detail == "parked", f"reason not carried: {alerts[0].detail}"


def test_the_ledger_counts_dead_chains():
    from nrda_fleet.bus import AlertMsg
    from nrda_fleet.metrics import FleetLedger
    led = FleetLedger()
    led.on_message(AlertMsg("AMR_1", 10.0, "dead_chain", "AMR_3", 0.0, 0.0, 0.0, "parked"), 10.0)
    led.on_message(AlertMsg("AMR_2", 11.0, "dead_chain", "AMR_9", 0.0, 0.0, 0.0, "silent"), 11.0)
    k = led.kpis()
    assert k["dead_chains"] == 2, f"kpi reported {k.get('dead_chains')}"
    assert led.dead_chains["parked"] == 1 and led.dead_chains["silent"] == 1
