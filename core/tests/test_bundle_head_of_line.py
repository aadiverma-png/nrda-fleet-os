"""A robot must not idle while holding runnable work (P13).

Station admission control caps how many robots may head for one station. ``next_task`` returned
only the *first* bundle task, so when that one was capped the robot started nothing at all, even
with perfectly runnable tasks behind it in its own bundle. Observed live: 30 tasks released at
once, dropoffs clustered on two capped stations, four robots parked and 13 tasks unstarted, with
one robot (AMR_20) holding both a blocked task and a runnable one.
"""

from __future__ import annotations

from types import SimpleNamespace

from nrda_fleet.allocation import Allocator, StationDistances
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.site import Site


class FakeAgent:
    def __init__(self, bundle):
        self.rid = "AMR_20"
        self.state = SimpleNamespace(bundle=list(bundle))


def _alloc(bundle, tasks, stable_s=0.0):
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    a = Allocator("AMR_20", StationDistances(graph), lambda m: None)
    a.agent = FakeAgent(bundle)
    a.tasks = {t: dict(task_id=t) for t in tasks}
    a.stable_s = stable_s
    a._winner_since = {t: ("AMR_20", 0.0) for t in tasks}
    return a


def test_without_a_filter_the_first_task_is_returned():
    a = _alloc(["T1", "T2"], ["T1", "T2"])
    assert a.next_task(100.0) == "T1"


def test_a_blocked_first_task_no_longer_hides_the_rest():
    """The bug: T1 is capped, T2 is runnable, and the robot used to start neither."""
    a = _alloc(["T1", "T2"], ["T1", "T2"])
    assert a.next_task(100.0, lambda t: t != "T1") == "T2"


def test_bundle_order_is_still_preferred():
    a = _alloc(["T1", "T2"], ["T1", "T2"])
    assert a.next_task(100.0, lambda t: True) == "T1"


def test_everything_blocked_still_yields_nothing():
    a = _alloc(["T1", "T2"], ["T1", "T2"])
    assert a.next_task(100.0, lambda t: False) is None


def test_an_unsettled_first_task_still_stops_the_scan():
    """Consensus has not settled, so the tasks behind it are not ours to start either."""
    a = _alloc(["T1", "T2"], ["T1", "T2"], stable_s=5.0)
    a._winner_since["T1"] = ("AMR_20", 99.0)   # won only 1 s ago, needs 5 s
    assert a.next_task(100.0, lambda t: True) is None


def test_a_task_won_by_someone_else_stops_the_scan():
    a = _alloc(["T1", "T2"], ["T1", "T2"])
    a._winner_since["T1"] = ("AMR_7", 0.0)
    assert a.next_task(100.0, lambda t: True) is None


def test_tasks_missing_from_the_auction_are_skipped():
    a = _alloc(["TX", "T2"], ["T2"])           # TX is not in self.tasks
    assert a.next_task(100.0, lambda t: True) == "T2"


def test_the_filter_sees_every_candidate_in_order():
    seen = []
    a = _alloc(["T1", "T2", "T3"], ["T1", "T2", "T3"])
    a.next_task(100.0, lambda t: seen.append(t) or t == "T3")
    assert seen == ["T1", "T2", "T3"], f"scan order was {seen}"
