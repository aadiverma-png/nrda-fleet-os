"""An L2 manoeuvre must not gossip a stale wait edge (P3).

Observed in Gazebo: AMR_19 froze in bypass phase ``passing`` and advertised
``waiting_for=['AMR_17']`` for 264 s. AMR_17 had since parked 56 m away, so the fleet's
wait-for graph was a chain ending at an idle robot - no cycle, and the DRP declines
non-cyclic groups by construction, so nothing ever acted on it.
"""

from __future__ import annotations

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
    a.t = 100.0
    a.pose = site.spawn_pose(rid)
    a.v = a.w = 0.0
    return a


class FakeSession:
    def __init__(self, obstacle_id):
        self.obstacle_id = obstacle_id


def _hear(a, rid, t=100.0):
    a.peers[rid] = RobotStateMsg(rid, t, 0.0, 0.0, 0.0, 0.0, 0.0, "parked", 1.0, "", "HEALTHY",
                                 "", "", "", 0.0, [], [], "")
    a.peer_heard[rid] = t


def test_a_stale_edge_is_not_kept_while_manoeuvring():
    """The exact failure: edge set before the bypass, robot now moving in the manoeuvre."""
    a = _agent()
    a.waiting_for = ["AMR_17"]
    a.bypass = FakeSession("lidar:AMR_5:1")
    a.v = 0.4  # making progress
    a._bypass_waits(a.t)
    assert a.waiting_for == [], "a moving robot still advertised a wait"


def test_a_stopped_manoeuvre_names_its_own_blocker_not_the_old_one():
    a = _agent()
    a.waiting_for = ["AMR_17"]          # the stale value
    _hear(a, "AMR_9")
    a.bypass = FakeSession("AMR_9")     # the manoeuvre is about AMR_9
    a.v = 0.0
    a._bypass_waits(a.t)
    assert a.waiting_for == ["AMR_9"], f"expected the real blocker, got {a.waiting_for}"
    assert "AMR_17" not in a.waiting_for


def test_an_obstacle_blocker_yields_no_robot_edge():
    """You cannot wait for a pallet: no edge, so the liveness watchdog owns this case."""
    a = _agent()
    a.waiting_for = ["AMR_17"]
    a.bypass = FakeSession("lidar:AMR_5:1")
    a.v = 0.0
    a._bypass_waits(a.t)
    assert a.waiting_for == [], f"an obstacle name leaked into the wait graph: {a.waiting_for}"


def test_a_silent_blocker_is_dropped():
    a = _agent()
    _hear(a, "AMR_9", t=1.0)            # last heard 99 s ago
    a.bypass = FakeSession("AMR_9")
    a.v = 0.0
    a._bypass_waits(a.t)
    assert a.waiting_for == [], "waited on a peer that has gone silent"


def test_starting_a_bypass_clears_the_queueing_edge():
    """Both L2 entry paths must clear the edge; only the early-L2 one used to."""
    import inspect
    from nrda_fleet import agent as ag
    src = inspect.getsource(ag.EdgeAgent._handle_waits)
    # the try_bypass success path must clear the edge before returning
    tail = src.split("from .bypass import try_bypass", 1)[1]
    success = tail.split("if ctl is not None:", 1)[1].split("self._l2_retry", 1)[0]
    assert "self.waiting_for = []" in success, "try_bypass path does not clear waiting_for"


def test_no_bypass_means_no_edge_from_this_path():
    a = _agent()
    a.waiting_for = ["AMR_17"]
    a.bypass = None
    a._bypass_waits(a.t)
    assert a.waiting_for == []
