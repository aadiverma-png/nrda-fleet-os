"""An obstacle may only be cleared by a robot that can actually see the spot (P7).

A clear is a fleet-wide assertion that a space is empty and it overrides everyone, including a
robot parked beside the thing looking at it. Live Gazebo run: real pallets were erased and
re-discovered at the same coordinates four times over, and *every* clear was issued by a robot
other than the one that reported it.
"""

from __future__ import annotations

import math

import numpy as np

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import AlertMsg, RobotStateMsg
from nrda_fleet.geom import OrientedBox, segment_hits_box
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.perception import StaticMap
from nrda_fleet.site import Site


def _agent(rid="AMR_1", pose=(-8.0, -2.0, 0.0), with_map=True):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    smap = StaticMap(site) if with_map else None
    a = EdgeAgent(rid, graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots[rid], 1.0, smap)
    a.t, a.v, a.w = 100.0, 0.0, 0.0
    a.pose = pose
    return a


def _peer(a, rid, x, y, th=0.0):
    a.peers[rid] = RobotStateMsg(rid, a.t, x, y, th, 0.0, 0.0, "to_drop", 1.0, "", "HEALTHY",
                                 "", "", "", 0.0, [], [], "")
    a.peer_heard[rid] = a.t


# ---------------------------------------------------------------- the geometry helper

def test_segment_box_hit_and_miss():
    b = OrientedBox(5.0, 0.0, 1.2, 0.78, 0.0)
    assert segment_hits_box(0.0, 0.0, 10.0, 0.0, b), "a segment straight through the box"
    assert not segment_hits_box(0.0, 3.0, 10.0, 3.0, b), "a segment well clear of it"
    assert not segment_hits_box(0.0, 0.0, 3.0, 0.0, b), "a segment that stops short"
    assert segment_hits_box(5.0, 0.0, 9.0, 0.0, b), "a segment starting inside"


def test_segment_box_respects_rotation():
    b = OrientedBox(5.0, 0.0, 2.0, 0.2, math.pi / 2)  # long axis across the path
    assert segment_hits_box(0.0, 0.0, 10.0, 0.0, b)
    b2 = OrientedBox(5.0, 0.0, 2.0, 0.2, 0.0)  # long axis along the path, thin
    assert segment_hits_box(0.0, 0.0, 10.0, 0.0, b2)
    assert not segment_hits_box(0.0, 0.5, 10.0, 0.5, b2), "0.5 m off a 0.2 m wide box"


# ---------------------------------------------------------------- the visibility gate

def test_a_clear_view_is_visible():
    a = _agent(pose=(-8.0, -2.0, 0.0))
    assert a._can_see(-8.95, -2.0), "an open lane spot should be visible"


def test_a_peer_body_blocks_the_view():
    a = _agent(pose=(-8.0, -2.0, 0.0))
    _peer(a, "AMR_9", -8.5, -2.0)  # standing between us and the spot
    assert not a._can_see(-8.95, -2.0)


def test_a_peer_off_to_the_side_does_not_block():
    a = _agent(pose=(-8.0, -2.0, 0.0))
    _peer(a, "AMR_9", -8.5, 1.5)
    assert a._can_see(-8.95, -2.0)


def test_a_silent_peer_does_not_block_the_view():
    """Its gossiped pose is too old to trust as an occluder."""
    a = _agent(pose=(-8.0, -2.0, 0.0))
    _peer(a, "AMR_9", -8.5, -2.0)
    a.peer_heard["AMR_9"] = a.t - 60.0
    assert a._can_see(-8.95, -2.0)


def test_mapped_structure_blocks_the_view():
    """Somewhere across the racks must not be claimed as seen."""
    site = Site.load()
    a = _agent(pose=(-8.0, -2.0, 0.0))
    smap = a.static_map
    # find a mapped cell and look at a point directly beyond it
    blocked = None
    for nb in site.boxes:
        b = nb.box
        d = math.hypot(b.cx + 8.0, b.cy + 2.0)
        if 2.0 < d < 9.0:
            f = (d + 1.5) / d
            blocked = (-8.0 + f * (b.cx + 8.0), -2.0 + f * (b.cy + 2.0))
            break
    assert blocked is not None, "no mapped box found to hide behind"
    assert not a._can_see(*blocked), "claimed to see through a rack"


# ---------------------------------------------------------------- the behaviour

def test_an_occluded_robot_cannot_clear_a_peer_report():
    """The observed failure: a different robot erases a report it cannot verify."""
    a = _agent("AMR_1", pose=(-8.0, -2.0, 0.0))
    published = []
    a.publish = published.append
    a._on_alert(AlertMsg("AMR_9", 100.0, "obstacle", "lidar:AMR_9:1", -8.95, -2.0, 0.0, "1.10x0.90"))
    a.tracks = []
    a.t = 104.0                             # > 3 s with nothing seen there
    _peer(a, "AMR_5", -8.5, -2.0)          # another robot blocks the view of the spot (heard just now)
    a._register_obstacles(a.t, np.zeros((0, 2)))
    assert "lidar:AMR_9:1" in a.known_obstacles, "erased an obstacle it could not see"
    assert not [m for m in published if m.kind == "obstacle_cleared"]


def test_a_robot_with_a_clear_view_may_still_clear_it():
    """The feature must keep working, or stale obstacles would block lanes for ever."""
    a = _agent("AMR_1", pose=(-8.0, -2.0, 0.0))
    published = []
    a.publish = published.append
    a._on_alert(AlertMsg("AMR_9", 100.0, "obstacle", "lidar:AMR_9:1", -8.95, -2.0, 0.0, "1.10x0.90"))
    a.tracks = []
    a.t = 104.0
    a._register_obstacles(a.t, np.zeros((0, 2)))
    assert "lidar:AMR_9:1" not in a.known_obstacles, "a verified-empty spot was not cleared"
    assert [m for m in published if m.kind == "obstacle_cleared"]


def test_returns_at_the_spot_still_keep_it():
    a = _agent("AMR_1", pose=(-8.0, -2.0, 0.0))
    a._on_alert(AlertMsg("AMR_9", 100.0, "obstacle", "lidar:AMR_9:1", -8.95, -2.0, 0.0, "1.10x0.90"))
    a.tracks = []
    a.t = 104.0
    pts = np.array([[-8.95 + 0.05 * i, -2.0] for i in range(5)])
    a._register_obstacles(a.t, pts)
    assert "lidar:AMR_9:1" in a.known_obstacles
