"""2-D geometry helpers: oriented boxes, a spatial index, circle and sweep tests.

Everything is in the world (``map``) frame: x east, y north, heading 0 = +x.
Rectangles reuse :class:`visison.geometry.OrientedBox` and its
separating-axis test so the fleet layer and the bypass planner agree
exactly on what "overlap" means.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Iterable, Sequence

from visison.geometry import OrientedBox, boxes_overlap

__all__ = [
    "OrientedBox", "boxes_overlap", "NamedBox", "BoxIndex", "wrap", "inflate",
    "point_box_distance", "circle_hits_box", "aabb",
]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class NamedBox:
    name: str
    box: OrientedBox
    height: float = 1.0

    @property
    def kind(self) -> str:
        """``rack``, ``wall``, ``person``, ``charger`` ... (prefix of the SDF collision name)."""
        n = self.name
        for k in ("person", "rack", "wall", "charger", "column", "mezzanine_column", "picker_zone"):
            if n.startswith(k):
                return k
        return n.rsplit("_", 1)[0]


def inflate(b: OrientedBox, margin: float) -> OrientedBox:
    return OrientedBox(b.cx, b.cy, b.length + 2 * margin, b.width + 2 * margin, b.heading)


def aabb(b: OrientedBox) -> tuple[float, float, float, float]:
    xs, ys = zip(*b.corners())
    return min(xs), min(ys), max(xs), max(ys)


def point_box_distance(px: float, py: float, b: OrientedBox) -> float:
    """Euclidean distance from a point to a rectangle (0 inside)."""
    c, s = math.cos(b.heading), math.sin(b.heading)
    dx, dy = px - b.cx, py - b.cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    ex = max(abs(lx) - b.length / 2, 0.0)
    ey = max(abs(ly) - b.width / 2, 0.0)
    return math.hypot(ex, ey)


def segment_hits_box(x0: float, y0: float, x1: float, y1: float, b: OrientedBox) -> bool:
    """Does the segment (x0, y0) -> (x1, y1) cross the oriented box?

    The usual slab test, done in the box's own frame. Used for line of sight: whether a robot's view
    of a point is cut off by another body.
    """
    c, s = math.cos(b.heading), math.sin(b.heading)
    px, py = x0 - b.cx, y0 - b.cy
    qx, qy = x1 - b.cx, y1 - b.cy
    ax, ay = c * px + s * py, -s * px + c * py  # start, in box frame
    bx, by = c * qx + s * qy, -s * qx + c * qy  # end, in box frame
    dx, dy = bx - ax, by - ay
    hl, hw = b.length / 2, b.width / 2
    t0, t1 = 0.0, 1.0
    for p, d, h in ((ax, dx, hl), (ay, dy, hw)):
        if abs(d) < 1e-12:
            if abs(p) > h:
                return False  # parallel to this slab and outside it
            continue
        lo, hi = (-h - p) / d, (h - p) / d
        if lo > hi:
            lo, hi = hi, lo
        t0, t1 = max(t0, lo), min(t1, hi)
        if t0 > t1:
            return False
    return True


def circle_hits_box(cx: float, cy: float, r: float, b: OrientedBox) -> bool:
    return point_box_distance(cx, cy, b) < r


class BoxIndex:
    """Uniform-grid spatial hash over named boxes."""

    def __init__(self, boxes: Iterable[NamedBox], cell: float = 2.0):
        self.cell = cell
        self.boxes: list[NamedBox] = list(boxes)
        self._grid: dict[tuple[int, int], list[int]] = defaultdict(list)
        for i, nb in enumerate(self.boxes):
            for key in self._keys(*aabb(nb.box)):
                self._grid[key].append(i)

    def _keys(self, x0, y0, x1, y1):
        c = self.cell
        for i in range(math.floor(x0 / c), math.floor(x1 / c) + 1):
            for j in range(math.floor(y0 / c), math.floor(y1 / c) + 1):
                yield i, j

    def near(self, x0: float, y0: float, x1: float, y1: float) -> list[NamedBox]:
        seen: set[int] = set()
        out = []
        for key in self._keys(x0, y0, x1, y1):
            for i in self._grid.get(key, ()):
                if i not in seen:
                    seen.add(i)
                    out.append(self.boxes[i])
        return out

    def box_hits(self, b: OrientedBox, margin: float = 0.0, ignore: Sequence[str] = ()) -> list[NamedBox]:
        q = inflate(b, margin) if margin else b
        x0, y0, x1, y1 = aabb(q)
        return [nb for nb in self.near(x0, y0, x1, y1)
                if nb.name not in ignore and boxes_overlap(q, nb.box)]

    def circle_hits(self, cx: float, cy: float, r: float, ignore: Sequence[str] = ()) -> list[NamedBox]:
        return [nb for nb in self.near(cx - r, cy - r, cx + r, cy + r)
                if nb.name not in ignore and circle_hits_box(cx, cy, r, nb.box)]

    def clearance(self, x: float, y: float, radius: float = 3.0) -> float:
        """Distance from a point to the nearest box within ``radius`` (``radius`` if none)."""
        best = radius
        for nb in self.near(x - radius, y - radius, x + radius, y + radius):
            best = min(best, point_box_distance(x, y, nb.box))
        return best
