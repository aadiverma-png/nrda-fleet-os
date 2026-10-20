"""Lidar-reported obstacles reach the console (P10).

The gateway only tracked operator-injected obstacles, so a lane blocked fleet-wide by a lidar
report rendered nothing: the robots routed around something the operator could not see. Live
run: two obstacles still believed at the end, console ``obstacles: []``.

Pure logic test - it exercises the gateway's bookkeeping without starting a ROS node.
"""

from __future__ import annotations

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "core"))

from nrda_fleet.bus import AlertMsg  # noqa: E402


class Gateway:
    """The gateway's found-obstacle bookkeeping, lifted out so it needs no rclpy."""

    FOUND_TTL = 120.0

    def __init__(self):
        self.found = {}

    _on_found_obstacle = None  # bound below from the real implementation


def _load_impl():
    """Pull `_on_found_obstacle` out of gateway_node.py without importing rclpy."""
    path = os.path.join(os.path.dirname(__file__), "..", "nrda_fleet_ros", "gateway_node.py")
    src = open(path).read()
    start = src.index("    def _on_found_obstacle(self, msg) -> None:")
    end = src.index("\n    def ", start + 10)
    body = "\n".join(line[4:] if line.startswith("    ") else line
                     for line in src[start:end].splitlines())
    ns: dict = {}
    exec(compile(body, "gateway_node.py", "exec"), ns)
    return ns["_on_found_obstacle"]


Gateway._on_found_obstacle = _load_impl()


def _alert(kind, name, x=3.9, y=10.35, sender="AMR_2", stamp=100.0, detail="1.00x1.00"):
    return AlertMsg(sender, stamp, kind, name, x, y, 0.0, detail)


def test_a_reported_obstacle_is_recorded():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle", "lidar:AMR_2:20"))
    assert "lidar:AMR_2:20" in g.found
    o = g.found["lidar:AMR_2:20"]
    assert (o["x"], o["y"]) == (3.9, 10.35)
    assert o["found_by"] == "AMR_2"
    assert o["length"] == 1.0 and o["width"] == 1.0


def test_a_cleared_obstacle_disappears():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle", "lidar:AMR_2:20"))
    g._on_found_obstacle(_alert("obstacle_cleared", "lidar:AMR_2:20", sender="AMR_4"))
    assert g.found == {}


def test_the_size_in_the_detail_field_is_parsed():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle", "o1", detail="1.35x0.90"))
    assert g.found["o1"]["length"] == 1.35
    assert g.found["o1"]["width"] == 0.90


def test_a_malformed_size_falls_back_without_raising():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle", "o1", detail="widexnarrow"))
    assert g.found["o1"]["length"] == 1.0


def test_the_first_sighting_time_is_kept_when_a_report_repeats():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle", "o1", stamp=100.0))
    g._on_found_obstacle(_alert("obstacle", "o1", stamp=160.0))
    o = g.found["o1"]
    assert o["t"] == 100.0, "first-seen time was overwritten"
    assert o["seen"] == 160.0, "latest sighting not updated"


def test_clearing_an_unknown_obstacle_is_harmless():
    g = Gateway()
    g._on_found_obstacle(_alert("obstacle_cleared", "never-seen"))
    assert g.found == {}


def test_a_stale_belief_is_dropped_eventually():
    """A report whose clear never arrives must not sit on the map for the rest of the run."""
    g = Gateway()
    g.FOUND_TTL = 120.0
    g._on_found_obstacle(_alert("obstacle", "old", stamp=100.0))
    g._on_found_obstacle(_alert("obstacle", "fresh", stamp=300.0))
    assert "old" not in g.found, "a belief nobody has mentioned for 200 s survived"
    assert "fresh" in g.found


def test_a_maintained_belief_is_kept():
    """A real obstacle is re-reported as robots pass it, so it must not be pruned."""
    g = Gateway()
    g.FOUND_TTL = 120.0
    for t in range(100, 500, 30):
        g._on_found_obstacle(_alert("obstacle", "real", stamp=float(t)))
    assert "real" in g.found
    assert g.found["real"]["t"] == 100.0, "first-seen time lost during pruning"
