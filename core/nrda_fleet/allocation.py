"""NRDA-FA layer L4: decentralised task allocation with CBBA.

The CBBA agent itself is :mod:`nrda_fleet.cbba` (Choi, Brunet & How 2009):
bundle construction, marginal utility, the consensus decision table, cascade
drop, failed-peer purge, reconnection reconcile. NRDA-FA changes only its
inputs:

* distances are **lane-graph travel distances** between stations (a
  precomputed station-to-station table), not straight lines through racks;
* weights are set for an 80 x 50 m warehouse so bids do not collapse to the
  0.01 floor (the textbook 10 per metre against 100 per priority level does
  at this scale);
* the robot position is its *anchor*: the station it is at or will finish
  its current task at.

Consensus runs over the peer bus: every agent broadcasts its winning bids and
winners (:class:`~nrda_fleet.bus.BidMsg`) and merges its peers' with
``resolve_conflicts``. A task starts only after its winner has been stable
for ``stable_s`` seconds; from then on it is *locked* (cannot be outbid).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from .bus import BidMsg
from .cbba import CBBAAgent, CBBAConfig
from .lane_graph import LaneGraph

FA_CBBA = CBBAConfig(max_bundle_size=2, weight_priority=150.0, weight_distance=1.0, weight_late=0.5,
                     discount_factor=0.95, nominal_speed=0.8)


class StationDistances:
    """Lane-graph travel distance (metres at cruise speed) between every pair of stations."""

    def __init__(self, graph: LaneGraph):
        self.graph = graph
        self.coords: dict[tuple[float, float], str] = {}
        for name, node in graph.stations.items():
            st = node.station
            self.coords[(round(st.x, 3), round(st.y, 3))] = name
        self._cache: dict[tuple[str, str], float] = {}

    def station_at(self, p) -> str | None:
        return self.coords.get((round(float(p[0]), 3), round(float(p[1]), 3)))

    def between(self, a: str, b: str) -> float:
        if a == b:
            return 0.0
        key = (a, b)
        if key not in self._cache:
            g = self.graph
            na, nb = g.stations[a], g.stations[b]
            start = (na.root, na.maneuver.exit_heading if na.maneuver else na.heading)
            path = g.route(start, nb.root, nb.heading if nb.maneuver is None else None)
            if path is None:
                self._cache[key] = math.inf
            else:
                cost = g.route_cost(path)
                if na.maneuver:
                    cost += na.maneuver.cost_out
                if nb.maneuver:
                    cost += nb.maneuver.cost_in
                self._cache[key] = cost * g.site.amr.v_cruise
        return self._cache[key]

    def points(self, p1, p2) -> float:
        a, b = self.station_at(p1), self.station_at(p2)
        if a is None or b is None:
            return 1.3 * (abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))  # Manhattan through the aisles
        return self.between(a, b)


class LaneCBBAAgent(CBBAAgent):
    """CBBA with lane-graph travel distance instead of the straight-line metric."""

    def __init__(self, robot_id: str, dist: StationDistances, config: CBBAConfig | None = None):
        super().__init__(robot_id, config or FA_CBBA)
        self.dist = dist

    def _euclidean_dist(self, p1, p2) -> float:  # noqa: D401  (overrides the straight-line metric)
        return self.dist.points(p1, p2)


@dataclass
class Allocator:
    """Wraps one robot's CBBA agent: task pool, message handling, stability and locking."""

    robot_id: str
    dist: StationDistances
    stable_s: float = 1.5
    config: CBBAConfig = field(default_factory=lambda: FA_CBBA)

    def __post_init__(self):
        self.agent = LaneCBBAAgent(self.robot_id, self.dist, self.config)
        self.tasks: dict[str, dict] = {}  # open tasks (CBBA dicts)
        self.done: set[str] = set()
        self.locked: set[str] = set()
        self.eligible = True
        self._winner_since: dict[str, tuple[str, float]] = {}
        self.iteration = 0
        self.dirty = True

    # task pool -------------------------------------------------------------------------

    def add_task(self, task_id: str, pickup: str, dropoff: str, priority: int, deadline: float,
                 assign_to: str = "") -> None:
        if task_id in self.done or task_id in self.tasks:
            return
        g = self.dist.graph
        if pickup not in g.stations or dropoff not in g.stations:
            return
        p, d = g.stations[pickup].station, g.stations[dropoff].station
        self.tasks[task_id] = {"task_id": task_id, "pickup": (p.x, p.y), "dropoff": (d.x, d.y),
                               "priority": priority, "deadline": deadline or None,
                               "requested_robot": assign_to or "AUTO",
                               "pickup_station": pickup, "dropoff_station": dropoff}
        self.dirty = True

    def finish(self, task_id: str) -> None:
        """A task was picked up or finished by someone: it leaves the auction."""
        self.tasks.pop(task_id, None)
        self.done.add(task_id)
        self.locked.discard(task_id)
        st = self.agent.state
        if task_id in st.bundle:
            st.bundle.remove(task_id)
        st.winning_bids.pop(task_id, None)
        st.winning_robots.pop(task_id, None)
        st.timestamps.pop(task_id, None)
        self.dirty = True

    def cancel(self, task_id: str) -> None:
        self.finish(task_id)

    # consensus -------------------------------------------------------------------------

    def set_anchor(self, station: str) -> None:
        st = self.dist.graph.stations[station].station
        self.agent.update_position((st.x, st.y))

    def step(self, now: float) -> BidMsg:
        """Rebuild the bundle and produce this round's broadcast."""
        st = self.agent.state
        if not self.eligible:
            # an ineligible robot (charging, fault) releases every task it has not started
            for t in [t for t in st.bundle if t not in self.locked]:
                st.bundle.remove(t)
                st.winning_robots[t] = ""
                st.winning_bids[t] = 0.0
                st.timestamps[t] = now
        else:
            open_tasks = {k: v for k, v in self.tasks.items()}
            self.agent.build_bundle(open_tasks, now, self.locked, self.tasks)
        self.iteration += 1
        for t in list(st.winning_robots):
            w = st.winning_robots[t]
            prev = self._winner_since.get(t)
            if prev is None or prev[0] != w:
                self._winner_since[t] = (w, now)
        return BidMsg(self.robot_id, now, self.iteration, dict(st.winning_bids), dict(st.winning_robots),
                      dict(st.timestamps), list(st.bundle))

    def on_bid(self, msg: BidMsg, now: float) -> None:
        if msg.robot_id == self.robot_id:
            return
        # tasks the peer knows are done must not come back
        wb = {k: v for k, v in msg.winning_bids.items() if k not in self.done}
        wr = {k: v for k, v in msg.winning_robots.items() if k not in self.done}
        ts = {k: v for k, v in msg.timestamps.items() if k not in self.done}
        self.agent.resolve_conflicts(msg.robot_id, msg.iteration, wb, wr, ts, self.tasks, now, self.locked)

    def purge_peer(self, rid: str, now: float) -> list[str]:
        return self.agent.purge_failed_peer_tasks(rid, now)

    def next_task(self, now: float, ready: Callable[[str], bool] | None = None) -> str | None:
        """First bundle task whose winner (us) has been stable long enough.

        ``ready`` is an optional extra test a task must pass to be startable now (the caller's station
        admission control). A task that is ours and settled but fails that test does not end the scan:
        bundle order is a preference for *routing*, not a barrier to working at all. Without this, a
        robot whose first bundle task was held back by the per-station WIP cap sat in its bay with
        perfectly runnable tasks queued behind it - observed live with 30 tasks released at once, four
        robots idle and 13 tasks unstarted, because the dropoffs clustered on two capped stations.

        A task that is not yet stably ours still stops the scan, as before: the auction has not settled,
        so the ones behind it in the bundle are not ours to start either.
        """
        st = self.agent.state
        for t in st.bundle:
            if t not in self.tasks:
                continue
            w, since = self._winner_since.get(t, ("", now))
            if not (w == self.robot_id and now - since >= self.stable_s):
                return None  # keep bundle order while consensus settles
            if ready is None or ready(t):
                return t
        return None

    def lock(self, task_id: str) -> None:
        self.locked.add(task_id)

    def task(self, task_id: str) -> dict | None:
        return self.tasks.get(task_id)
