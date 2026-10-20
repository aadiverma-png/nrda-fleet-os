"""Fleet-level ground-truth collision detection, shared by the 2-D simulator and Gazebo.

Independent of the agents, so a robot cannot hide its own collisions:

* **COLLISION**: two robot footprints overlap, or a footprint overlaps a site
  box or an injected obstacle (``visison.geometry.boxes_overlap``). In Gazebo
  the chassis contact sensor reports physical contacts as well
  (:meth:`SafetyTruth.contact`);
* **NEAR_MISS**: edge-to-edge gap below ``near_gap`` while closing;
* **AVOIDED**: an agent's emergency layer braked for a predicted collision
  (from its risk alerts).

Events are de-duplicated per pair (``dedup`` seconds) and counted.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

from .bus import SafetyEventMsg
from .geom import BoxIndex, OrientedBox, boxes_overlap, point_box_distance


@dataclass
class Body:
    rid: str
    x: float
    y: float
    th: float
    v: float = 0.0
    docked_at: str = ""  # charger/station name: its own tower does not count


@dataclass
class SafetyTruth:
    index: BoxIndex
    length: float = 1.2
    width: float = 0.78
    near_gap: float = 0.10
    dedup: float = 2.0
    events: list[SafetyEventMsg] = field(default_factory=list)
    counts: Counter = field(default_factory=Counter)
    min_gap: float = math.inf
    _last: dict = field(default_factory=dict)

    def check(self, t: float, bodies: list[Body], obstacles: list[tuple[str, OrientedBox]] = ()) -> list[SafetyEventMsg]:
        new = []
        boxes = {b.rid: OrientedBox(b.x, b.y, self.length, self.width, b.th) for b in bodies}
        for i, a in enumerate(bodies):
            ba = boxes[a.rid]
            for b in bodies[i + 1:]:
                if abs(a.x - b.x) > 2.0 or abs(a.y - b.y) > 2.0:
                    continue
                bb = boxes[b.rid]
                rel = math.hypot(a.v * math.cos(a.th) - b.v * math.cos(b.th), a.v * math.sin(a.th) - b.v * math.sin(b.th))
                if boxes_overlap(ba, bb):
                    new += self._emit(t, (a.rid, b.rid), "COLLISION", [a.rid, b.rid], b.rid, (a.x + b.x) / 2,
                                      (a.y + b.y) / 2, rel, "overlap")
                else:
                    gap = box_gap(ba, bb)
                    self.min_gap = min(self.min_gap, gap)
                    if gap < self.near_gap and rel > 0.05:
                        new += self._emit(t, (a.rid, b.rid), "NEAR_MISS", [a.rid, b.rid], b.rid, (a.x + b.x) / 2,
                                          (a.y + b.y) / 2, rel, "gap", f"gap {gap:.2f} m")
            for nb in self.index.box_hits(ba, -0.01):
                if nb.name.startswith("charger_") and a.docked_at:
                    continue
                new += self._emit(t, (a.rid, nb.name), "COLLISION", [a.rid], nb.name, a.x, a.y, abs(a.v), "overlap")
            for name, ob in obstacles:
                if boxes_overlap(ba, ob):
                    new += self._emit(t, (a.rid, name), "COLLISION", [a.rid], name, a.x, a.y, abs(a.v), "overlap")
        return new

    def contact(self, t: float, rid: str, other: str, x: float, y: float,
                rel: float = 0.0) -> list[SafetyEventMsg]:
        """A physical contact reported by the chassis contact sensor (Gazebo).

        ``rel`` is the impact-relevant relative speed in m/s (0.0 when the caller cannot measure it).
        """
        key = tuple(sorted((rid, other)))
        return self._emit(t, key, "COLLISION", [rid] + ([other] if other.startswith("AMR_") else []), other, x, y,
                          rel, "contact")

    def avoided(self, t: float, rid: str, other: str, x: float, y: float, detail: str,
                rel: float = 0.0) -> list[SafetyEventMsg]:
        return self._emit(t, ("avoid", rid, other), "AVOIDED", [rid], other, x, y, rel, "risk", detail)

    def _emit(self, t, key, level, robots, other, x, y, rel, source, detail=""):
        k = (level,) + tuple(key)
        last = self._last.get(k)
        self._last[k] = t
        if last is not None and t - last < self.dedup:
            return []
        ev = SafetyEventMsg(f"E{len(self.events) + 1:05d}", round(t, 3), level, list(robots), other, round(x, 2),
                            round(y, 2), round(rel, 2), source, detail)
        self.events.append(ev)
        self.counts[level] += 1
        return [ev]


def box_gap(a: OrientedBox, b: OrientedBox) -> float:
    """Edge-to-edge distance of two non-overlapping rectangles (corner-to-rectangle minimum)."""
    return min(min(point_box_distance(x, y, b) for x, y in a.corners()),
               min(point_box_distance(x, y, a) for x, y in b.corners()))
