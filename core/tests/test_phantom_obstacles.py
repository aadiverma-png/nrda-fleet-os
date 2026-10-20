"""A peer robot must never be published to the fleet as a dropped pallet (P6).

Live Gazebo run, 480 s, nothing on the floor: six "unmapped obstacles" were invented, four of
them at one lane hot-spot where robots queue. Cause: ``_identify`` compared the tracker's
*near-face* centroid with the peer's *centre* under a flat 0.9 m radius. A body seen at a corner
puts that centroid 0.72 m from the centre, leaving 0.18 m for pose age and extrapolation.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from nrda_fleet.agent import PEER_ID_MARGIN, AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.bus import RobotStateMsg
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site
from visison.emergency import TrackedObject


def _agent(rid="AMR_1"):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = EdgeAgent(rid, graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots[rid], 1.0, None)
    a.t = 100.0
    a.pose = (0.0, 0.0, 0.0)
    a.v = a.w = 0.0
    return a


def _peer(a, rid, x, y, th=0.0, v=0.0, stamp=None, mode="to_drop"):
    a.peers[rid] = RobotStateMsg(rid, stamp if stamp is not None else a.t, x, y, th, v, 0.0, mode,
                                 1.0, "", "HEALTHY", "", "", "", 0.0, [], [], "")
    a.peer_heard[rid] = a.t


def _face_track(px, py, pth, face):
    """Centroid of the near face of a peer at (px, py, pth), as the tracker would report it."""
    L, W = 1.20, 0.78
    off = {"front": (L / 2, 0.0), "rear": (-L / 2, 0.0), "left": (0.0, W / 2),
           "right": (0.0, -W / 2), "corner": (L / 2, W / 2)}[face]
    c, s = math.cos(pth), math.sin(pth)
    return TrackedObject(px + c * off[0] - s * off[1], py + s * off[0] + c * off[1],
                         0.0, 0.0, 0.4, 0.4, "T1")


@pytest.mark.parametrize("face", ["front", "rear", "left", "right", "corner"])
@pytest.mark.parametrize("pth", [0.0, 0.7854, 1.5708, 2.3562, 3.0])
def test_a_stopped_peer_is_identified_from_any_face_and_angle(face, pth):
    """The failure mode: a queueing robot seen from any side must be recognised as a robot."""
    a = _agent()
    _peer(a, "AMR_9", 4.0, 1.0, th=pth)
    tr = _face_track(4.0, 1.0, pth, face)
    assert a._identify(tr) == "AMR_9", f"face={face} th={pth:.2f} not identified"


def test_the_old_flat_radius_would_have_missed_the_corner_case():
    """Documents why this changed: the previous rule had only 0.18 m of budget."""
    a = _agent()
    _peer(a, "AMR_9", 4.0, 1.0, th=0.0)
    tr = _face_track(4.0, 1.0, 0.0, "corner")
    old_d = math.hypot(tr.s - 4.0, tr.y - 1.0)
    assert old_d > 0.7, "corner offset should be large"
    assert a._identify(tr) == "AMR_9", "footprint test must cover what the radius barely did"


def test_a_real_pallet_is_still_not_a_robot():
    a = _agent()
    _peer(a, "AMR_9", 4.0, 1.0)
    pallet = TrackedObject(9.0, 5.0, 0.0, 0.0, 1.1, 0.9, "T2")  # nowhere near any peer
    assert a._identify(pallet) is None
    assert not a._near_any_peer(pallet)


def test_a_stale_peer_is_not_used_for_identification():
    a = _agent()
    _peer(a, "AMR_9", 4.0, 1.0)
    a.peer_heard["AMR_9"] = a.t - 10.0
    assert a._identify(_face_track(4.0, 1.0, 0.0, "front")) is None


def test_a_track_near_a_peer_is_never_published_as_an_obstacle():
    """Belt and braces: even an unidentified track near a robot must not become a pallet."""
    a = _agent()
    published = []
    a.publish = published.append
    _peer(a, "AMR_9", 4.0, 1.0)
    # a track just outside the footprint test but clearly still the robot
    tr = TrackedObject(4.0 + PEER_ID_MARGIN + 0.9, 1.0, 0.0, 0.0, 0.4, 0.4, "T3")
    assert a._near_any_peer(tr), "the loose peer test should cover this"
    a.tracks = [tr]
    for _ in range(12):  # well past the 5-scan confirmation
        a._register_obstacles(a.t, np.array([[tr.s, tr.y]]))
    assert not a.known_obstacles, f"invented {list(a.known_obstacles)} for a robot"
    assert not [m for m in published if m.kind == "obstacle"]


def test_a_genuine_pallet_in_a_lane_is_still_reported():
    """The feature must keep working: a real unmapped object is still announced."""
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = EdgeAgent("AMR_1", graph, StationDistances(graph), lambda m: None, AgentConfig(),
                  site.robots["AMR_1"], 1.0, None)
    a.t, a.v, a.w = 100.0, 0.0, 0.0
    a.pose = (-8.0, -2.0, 0.0)
    published = []
    a.publish = published.append
    tr = TrackedObject(-8.95, -2.0, 0.0, 0.0, 1.1, 0.9, "T4")  # a pallet in a lane, no peers at all
    a.tracks = [tr]
    for _ in range(8):
        a._register_obstacles(a.t, np.array([[tr.s, tr.y]]))
    assert a.known_obstacles, "a real pallet was not reported"
    assert [m for m in published if m.kind == "obstacle"]
