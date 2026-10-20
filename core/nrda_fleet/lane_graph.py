"""The shared road network: two-lane directed aisles, intersections, stations.

Derived automatically from the site's collision boxes
(:func:`LaneGraph.build`):

* **rack columns**: vertical rack boxes grouped by x;
* **vertical lines**: every aisle between two rack columns, plus the west and
  east corridors (the widest free band beside the outer columns);
* **horizontal lines**: every cross-aisle (y band free in *all* columns),
  plus the top and bottom corridors;
* **intersections** where a vertical and a horizontal line cross.

Every line is a two-lane road with right-hand traffic, and each one is a
:class:`visison.geometry.Aisle` (lanes ``<line>/A`` in +t direction and
``<line>/B`` against it, 0.5 m cells). So the visison bypass planner works on
a whole aisle, across its intersections.

Validation against the real boxes (AMR-01 footprint, 5 cm margin):
* a lane stretch between two intersections is usable only if the body swept
  along its centre line is clear;
* a turn (rotation in place) at an intersection lane point is allowed only
  if the rotation circle is clear.

Stations are attached either on a lane (picker stops, overflow inducts,
decant) or through a validated **manoeuvre** from a lane point: rotate,
straight moves, or a quintic shift into an alcove.

Space-time resources (what robots reserve) are lane cells of every line the
body overlaps, a footprint resource per station, and a 0.25 m free-space
grid for body parts outside every lane.
"""

from __future__ import annotations

import heapq
import math
from collections import defaultdict
from dataclasses import dataclass, field, replace
from typing import Iterable

from visison.geometry import Aisle

from .geom import OrientedBox, aabb, boxes_overlap, wrap
from .site import Site, Station
from .trajectory import Builder, Leg

CELL = 0.5
FREE_CELL = 0.25
LANE_MARGIN = 0.05  # body clearance required along lanes
TURN_MARGIN = 0.03  # clearance required for rotations in place
MANEUVER_MARGIN = 0.03
HEADINGS = {"E": 0.0, "N": math.pi / 2, "W": math.pi, "S": -math.pi / 2}


def heading_name(th: float) -> str:
    return min(HEADINGS, key=lambda k: abs(wrap(HEADINGS[k] - th)))


# ------------------------------------------------------------------------------ lines


# Half-size of a crossing's no-stop zone (``in_intersection``) beyond the crossing lanes' centre band: robot half width + this pad. A robot held at
# the stop line must be clear of the whole cross lane (band + the 0.1 m claim pad), not just of the bodies driving
# in it: with 0.05 its nose stood in the cross lane's cells, the robot turning inside the crossing needed those
# cells, and the two waited for each other for ever (20-robot runs, 2026-09-30).
CORE_PAD = 0.10


@dataclass
class Line:
    name: str
    orient: str  # "V" (along +y) or "H" (along +x)
    center: float  # x of a V line, y of an H line
    lo: float  # extent along the line
    hi: float
    w: float  # lane width
    kind: str = "aisle"  # aisle | cross | corridor

    def __post_init__(self):
        self.theta = math.pi / 2 if self.orient == "V" else 0.0
        if self.orient == "V":
            self.origin = (self.center + self.w, self.lo)
        else:
            self.origin = (self.lo, self.center - self.w)
        self.aisle = Aisle(lane_width=self.w, length=self.hi - self.lo, cell_size=CELL,
                           lanes=(f"{self.name}/A", f"{self.name}/B"), aisle_id=self.name)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self._t, self._n = (c, s), (-s, c)

    # frames
    def to_frame(self, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - self.origin[0], y - self.origin[1]
        return dx * self._t[0] + dy * self._t[1], dx * self._n[0] + dy * self._n[1]

    def to_world(self, s: float, ya: float) -> tuple[float, float]:
        return (self.origin[0] + s * self._t[0] + ya * self._n[0],
                self.origin[1] + s * self._t[1] + ya * self._n[1])

    def lane_y(self, lane: str) -> float:
        """Lateral centre (frame y) of lane 'A' or 'B'."""
        return self.w / 2 if lane == "A" else 1.5 * self.w

    def contains(self, x: float, y: float) -> bool:
        """Is (x, y) on this line's two-lane band?"""
        if self.orient == "V":
            return abs(x - self.center) <= self.w and self.lo <= y <= self.hi
        return abs(y - self.center) <= self.w and self.lo <= x <= self.hi

    def lane_heading(self, lane: str) -> float:
        return wrap(self.theta + (0.0 if lane == "A" else math.pi))

    def lane_key(self, lane: str) -> str:
        return f"{self.name}/{lane}"

    def lane_point(self, lane: str, along: float) -> tuple[float, float]:
        """World point on the lane centre at world coordinate ``along`` (y for V, x for H)."""
        s = along - self.lo
        return self.to_world(s, self.lane_y(lane))

    def along(self, x: float, y: float) -> float:
        return y if self.orient == "V" else x

    def band(self) -> OrientedBox:
        L = self.hi - self.lo
        cx, cy = self.to_world(L / 2, self.w)
        return OrientedBox(cx, cy, L, 2 * self.w, self.theta)


@dataclass
class Intersection:
    name: str
    v: Line
    h: Line

    @property
    def center(self) -> tuple[float, float]:
        return self.v.center, self.h.center

    def corners(self) -> dict[str, tuple[float, float]]:
        cx, cy = self.center
        dv, dh = self.v.w / 2, self.h.w / 2
        return {"SE": (cx + dv, cy - dh), "NE": (cx + dv, cy + dh),
                "NW": (cx - dv, cy + dh), "SW": (cx - dv, cy - dh)}

    def square(self, shrink: float = 0.0) -> OrientedBox:
        cx, cy = self.center
        return OrientedBox(cx, cy, 2 * self.v.w - 2 * shrink, 2 * self.h.w - 2 * shrink, 0.0)


# corner -> (lane on the V line, lane on the H line). V: A=north, B=south. H: A=east, B=west.
CORNER_LANES = {"SE": ("A", "A"), "NE": ("A", "B"), "NW": ("B", "B"), "SW": ("B", "A")}


# ------------------------------------------------------------------------------ graph


@dataclass(frozen=True)
class Point:
    pid: str
    x: float
    y: float
    lanes: tuple[tuple[str, str], ...]  # (line name, lane) pairs this point lies on
    xing: str = ""  # intersection name if it is an intersection corner


@dataclass
class Edge:
    src: tuple[str, str]  # (point id, heading name)
    dst: tuple[str, str]
    kind: str  # move | rotate | enter | exit
    cost: float
    line: str = ""
    lane: str = ""


@dataclass
class Maneuver:
    station: str
    root: str  # point id
    root_heading: str  # heading the robot has at the root when it starts the entry
    entry: list[Leg]
    exit_heading: str  # heading at the root after the exit
    exit: list[Leg]
    cost_in: float
    cost_out: float
    depends: tuple[str, ...] = ()  # stations that must be empty while manoeuvring
    samples: list[tuple[float, float, float]] = field(default_factory=list)


@dataclass
class StationNode:
    station: Station
    root: str  # point id on a lane
    heading: str  # lane heading at the root (lane stops) or root heading (manoeuvres)
    maneuver: Maneuver | None = None

    @property
    def on_lane(self) -> bool:
        return self.maneuver is None


class LaneGraph:
    def __init__(self, site: Site):
        self.site = site
        self.lines: dict[str, Line] = {}
        self.xings: dict[str, Intersection] = {}
        self.points: dict[str, Point] = {}
        self.edges: dict[tuple[str, str], list[Edge]] = defaultdict(list)
        self.blocked_lanes: list[tuple[str, str, float, float]] = []  # line, lane, from, to (along)
        self.turns: dict[str, set[frozenset]] = defaultdict(set)
        self.stations: dict[str, StationNode] = {}
        self.report: list[str] = []
        self.builder = Builder(site.amr.limits(), site.amr.w_turn, site.amr.alpha_turn)
        self._lane_points: dict[tuple[str, str], list[tuple[float, str]]] = defaultdict(list)

    # ============================================================== construction

    @classmethod
    def load_or_build(cls, site: Site, cache_dir: str | None = None) -> "LaneGraph":
        """Build once per site version and reuse it (every robot process loads the same graph)."""
        import hashlib
        import os
        import pickle
        h = hashlib.sha256()
        for nb in site.boxes:
            h.update(f"{nb.name}{nb.box}".encode())
        for st in sorted(site.stations.values(), key=lambda s: s.name):
            h.update(repr(st).encode())
        h.update(repr(site.amr).encode())  # edge costs, turn radius and the station manoeuvres all use the spec
        h.update(open(__file__, "rb").read())
        plan = os.path.join(site.root, "config", "lanes.yaml")
        if os.path.exists(plan):
            h.update(open(plan, "rb").read())
        cache_dir = cache_dir or os.path.join(os.path.expanduser("~"), ".cache", "nrda")
        path = os.path.join(cache_dir, f"lane_graph_{h.hexdigest()[:16]}.pkl")
        if os.path.exists(path):
            try:
                with open(path, "rb") as f:
                    g = pickle.load(f)
                g.site = site
                return g
            except Exception:  # noqa: BLE001  (stale or partial cache: rebuild)
                pass
        g = cls.build(site)
        os.makedirs(cache_dir, exist_ok=True)
        tmp = path + f".{os.getpid()}.tmp"
        site_ref, g.site = g.site, None
        for k in ("_cover_cache", "_sbox_cache", "_xgrid", "_sorted_cache", "_xcells", "_keep_clear", "_station_xy", "_ccells", "_ccmap"):
            g.__dict__.pop(k, None)
        try:
            with open(tmp, "wb") as f:
                pickle.dump(g, f)
            os.replace(tmp, path)
        finally:
            g.site = site_ref
        return g

    @classmethod
    def build(cls, site: Site) -> "LaneGraph":
        g = cls(site)
        g._detect_lines()
        g._apply_traffic_plan()
        g._make_intersections()
        g._validate_and_link()
        g._attach_stations()
        return g

    def _detect_lines(self) -> None:
        idx = self.site.index
        racks = [nb for nb in self.site.boxes if nb.name.startswith("rack_")
                 and nb.box.length < 1.3 and nb.box.width > 1.4 and abs(nb.box.heading) < 1e-3]
        cols: dict[float, list] = defaultdict(list)
        for nb in racks:
            cols[round(nb.box.cx, 2)].append(nb)
        cols = {x: v for x, v in cols.items() if len(v) >= 2}
        xs = sorted(cols)
        if len(xs) < 2:
            raise RuntimeError("no rack columns found in the site model")
        depth = max(nb.box.length for nb in racks)
        y_lo = min(nb.box.cy - nb.box.width / 2 for nb in racks)
        y_hi = max(nb.box.cy + nb.box.width / 2 for nb in racks)
        x_lo, x_hi = xs[0] - depth / 2, xs[-1] + depth / 2

        # cross aisles: y bands free in every column
        free = [(y_lo, y_hi)]
        for x in xs:
            occ = sorted((nb.box.cy - nb.box.width / 2, nb.box.cy + nb.box.width / 2) for nb in cols[x])
            free = _subtract(free, occ)
        cross = [(a, b) for a, b in free if b - a >= 2.0]
        # top / bottom corridors: up to the nearest obstacle across the rack field
        top_limit = min((aabb(nb.box)[1] for nb in idx.near(x_lo, y_hi, x_hi, y_hi + 6)
                         if aabb(nb.box)[1] >= y_hi - 1e-6 and nb not in racks), default=y_hi + 2.4)
        bot_limit = max((aabb(nb.box)[3] for nb in idx.near(x_lo, y_lo - 6, x_hi, y_lo)
                         if aabb(nb.box)[3] <= y_lo + 1e-6 and nb not in racks), default=y_lo - 2.4)
        h_bands = [("HB", bot_limit, y_lo, "corridor")]
        h_bands += [(f"H{i + 1}", a, b, "cross") for i, (a, b) in enumerate(cross)]
        h_bands += [("HT", y_hi, top_limit, "corridor")]
        y_first = (h_bands[0][1] + h_bands[0][2]) / 2
        y_last = (h_bands[-1][1] + h_bands[-1][2]) / 2
        # west / east corridors: nearest obstacle beside the outer columns over the whole y span
        west_limit = max((aabb(nb.box)[2] for nb in idx.near(x_lo - 6, y_first - 1.2, x_lo, y_last + 1.2)
                          if aabb(nb.box)[2] <= x_lo + 1e-6 and nb not in racks), default=x_lo - 2.4)
        east_limit = min((aabb(nb.box)[0] for nb in idx.near(x_hi, y_first - 1.2, x_hi + 6, y_last + 1.2)
                          if aabb(nb.box)[0] >= x_hi - 1e-6 and nb not in racks), default=x_hi + 2.4)
        v_bands = [("VW", west_limit, x_lo, "corridor")]
        v_bands += [(f"V{i + 1:02d}", xs[i] + depth / 2, xs[i + 1] - depth / 2, "aisle") for i in range(len(xs) - 1)]
        v_bands += [("VE", x_hi, east_limit, "corridor")]
        x_first = (v_bands[0][1] + v_bands[0][2]) / 2
        x_last = (v_bands[-1][1] + v_bands[-1][2]) / 2
        for name, a, b, kind in v_bands:
            c, w = (a + b) / 2, min(1.2, (b - a) / 2)
            self.lines[name] = Line(name, "V", c, 0.0, 0.0, w, kind)
        for name, a, b, kind in h_bands:
            c, w = (a + b) / 2, min(1.2, (b - a) / 2)
            self.lines[name] = Line(name, "H", c, 0.0, 0.0, w, kind)
        hs = [ln for ln in self.lines.values() if ln.orient == "H"]
        vs = [ln for ln in self.lines.values() if ln.orient == "V"]
        for ln in vs:  # V lines run from the bottom corridor to the top corridor (through the far side)
            lo, hi = hs[0].center - hs[0].w, hs[-1].center + hs[-1].w
            self.lines[ln.name] = Line(ln.name, "V", ln.center, lo, hi, ln.w, ln.kind)
        for ln in hs:
            lo, hi = vs[0].center - vs[0].w, vs[-1].center + vs[-1].w
            self.lines[ln.name] = Line(ln.name, "H", ln.center, lo, hi, ln.w, ln.kind)
        self.report.append(f"{len(xs)} rack columns, {len(vs)} vertical lines, {len(hs)} horizontal lines")
        self.report += [f"  {ln.name:4s} {ln.orient} center={ln.center:7.3f} lane_w={ln.w:.3f} "
                        f"extent=[{ln.lo:.2f}, {ln.hi:.2f}] ({ln.kind})" for ln in self.lines.values()]
        _ = (x_first, x_last)

    def _apply_traffic_plan(self) -> None:
        """Extra lines / extensions from ``config/lanes.yaml`` (site-specific traffic design)."""
        import os
        import yaml
        path = os.path.join(self.site.root, "config", "lanes.yaml")
        if not os.path.exists(path):
            return
        with open(path) as f:
            plan = yaml.safe_load(f) or {}
        for name, d in (plan.get("extend") or {}).items():
            ln = self.lines.get(name)
            if ln is not None:
                self.lines[name] = Line(name, ln.orient, ln.center, d.get("lo", ln.lo), d.get("hi", ln.hi), ln.w, ln.kind)
        for name, d in (plan.get("extra_lines") or {}).items():
            self.lines[name] = Line(name, d["orient"], d["center"], d["lo"], d["hi"], d.get("w", 1.2),
                                    d.get("kind", "corridor"))
        self.report.append(f"traffic plan: +{len(plan.get('extra_lines') or {})} lines, "
                           f"{len(plan.get('extend') or {})} extended")

    def _make_intersections(self) -> None:
        vs = [ln for ln in self.lines.values() if ln.orient == "V"]
        hs = [ln for ln in self.lines.values() if ln.orient == "H"]
        for v in vs:
            for h in hs:
                # both lines must actually reach the crossing
                if not (v.lo - 1e-6 <= h.center - h.w and h.center + h.w <= v.hi + 1e-6
                        and h.lo - 1e-6 <= v.center - v.w and v.center + v.w <= h.hi + 1e-6):
                    continue
                x = Intersection(f"X_{v.name}_{h.name}", v, h)
                hits = self.site.index.box_hits(x.square(shrink=0.1))
                if hits:
                    self.report.append(f"  intersection {x.name} blocked by {hits[0].name}")
                    continue
                self.xings[x.name] = x
                for corner, (vl, hl) in CORNER_LANES.items():
                    px, py = x.corners()[corner]
                    pid = f"{x.name}:{corner}"
                    self.points[pid] = Point(pid, px, py, ((v.name, vl), (h.name, hl)), x.name)
                    self._lane_points[(v.name, vl)].append((v.along(px, py), pid))
                    self._lane_points[(h.name, hl)].append((h.along(px, py), pid))

    def is_lane_enabled(self, line_name: str, lane: str) -> bool:
        """Narrow corridors (w < 1.05 m) operate one-way (lane A only) to eliminate head-on standoffs."""
        ln = self.lines.get(line_name)
        if ln is not None and ln.kind == "corridor" and ln.w < 1.05 and lane == "B":
            return False
        return True

    def _validate_and_link(self) -> None:
        amr = self.site.amr
        r_turn = amr.rotation_radius + TURN_MARGIN
        # turns at intersection corners
        for pid, p in self.points.items():
            if not p.xing:
                continue
            (vn, vl), (hn, hl) = p.lanes
            if not self.is_lane_enabled(vn, vl) or not self.is_lane_enabled(hn, hl):
                continue
            hv, hh = self.lines[vn].lane_heading(vl), self.lines[hn].lane_heading(hl)
            if not self.site.index.circle_hits(p.x, p.y, r_turn):
                self._add_turn(pid, hv, hh)
        # lane stretches between consecutive points
        for (ln_name, lane), pts in self._lane_points.items():
            if not self.is_lane_enabled(ln_name, lane):
                continue
            ln = self.lines[ln_name]
            direction = 1 if lane == "A" else -1
            pts.sort(key=lambda t: direction * t[0])
            for (a0, p0), (a1, p1) in zip(pts, pts[1:]):
                if self._stretch_clear(ln, lane, a0, a1):
                    self._add_move(p0, p1, ln, lane)
                else:
                    self.blocked_lanes.append((ln_name, lane, a0, a1))
        n_turn = sum(1 for e in self._all_edges() if e.kind == "rotate")
        n_move = sum(1 for e in self._all_edges() if e.kind == "move")
        self.report.append(f"{len(self.xings)} intersections, {n_move} lane edges, {n_turn} turn edges, "
                           f"{len(self.blocked_lanes)} blocked lane stretches")
        for b in self.blocked_lanes:
            self.report.append(f"  blocked: {b[0]}/{b[1]} {b[2]:.2f} -> {b[3]:.2f}")

    def _stretch_clear(self, ln: Line, lane: str, a0: float, a1: float, step: float = 0.25) -> bool:
        h = ln.lane_heading(lane)
        n = max(int(abs(a1 - a0) / step), 1)
        for i in range(n + 1):
            a = a0 + (a1 - a0) * i / n
            x, y = ln.lane_point(lane, a)
            if self.site.index.box_hits(self.site.body(x, y, h), LANE_MARGIN):
                return False
        return True

    def _add_turn(self, pid: str, h1: float, h2: float) -> None:
        n1, n2 = heading_name(h1), heading_name(h2)
        cost = abs(wrap(h1 - h2)) / self.site.amr.w_turn + 1.5
        self.edges[(pid, n1)].append(Edge((pid, n1), (pid, n2), "rotate", cost))
        self.edges[(pid, n2)].append(Edge((pid, n2), (pid, n1), "rotate", cost))
        self.turns[pid].add(frozenset((n1, n2)))

    def _add_move(self, p0: str, p1: str, ln: Line, lane: str) -> None:
        a, b = self.points[p0], self.points[p1]
        hn = heading_name(ln.lane_heading(lane))
        cost = math.hypot(b.x - a.x, b.y - a.y) / self.site.amr.v_cruise
        self.edges[(p0, hn)].append(Edge((p0, hn), (p1, hn), "move", cost, ln.name, lane))

    def _all_edges(self) -> Iterable[Edge]:
        for es in self.edges.values():
            yield from es

    # ============================================================== stations

    def _attach_stations(self) -> None:
        for name, st in sorted(self.site.stations.items()):
            node = self._attach_lane(st) if st.approach == "lane" else self._attach_maneuver_for(st)
            if node is None:
                self.report.append(f"  station {name}: NOT REACHABLE")
                continue
            self.stations[name] = node
        ok = len(self.stations)
        self.report.append(f"{ok}/{len(self.site.stations)} stations attached")

    def _lane_candidates(self, x: float, y: float, radius: float):
        """(line, lane, along, distance) for lane centre lines passing within ``radius``."""
        for ln in self.lines.values():
            for lane in ("A", "B"):
                if not self.is_lane_enabled(ln.name, lane):
                    continue
                s, ya = ln.to_frame(x, y)
                if not (0 <= s <= ln.hi - ln.lo):
                    continue
                d = abs(ya - ln.lane_y(lane))
                if d <= radius:
                    yield ln, lane, ln.along(*ln.to_world(s, ln.lane_y(lane))), d

    def _insert_lane_point(self, ln: Line, lane: str, along: float, pid: str) -> str | None:
        """Split the lane edge that passes ``along`` with a new point. Returns the point id."""
        if pid in self.points:
            return pid
        pts = self._lane_points[(ln.name, lane)]
        direction = 1 if lane == "A" else -1
        pts.sort(key=lambda t: direction * t[0])
        prev = nxt = None
        for a, p in pts:
            if direction * a <= direction * along:
                prev = (a, p)
            elif nxt is None:
                nxt = (a, p)
        x, y = ln.lane_point(lane, along)
        hn = heading_name(ln.lane_heading(lane))
        # must lie on a usable stretch
        if prev is None or nxt is None:
            return None
        edge = next((e for e in self.edges[(prev[1], hn)] if e.kind == "move" and e.dst[0] == nxt[1]), None)
        if edge is None:
            return None
        self.points[pid] = Point(pid, x, y, ((ln.name, lane),))
        pts.append((along, pid))
        self.edges[(prev[1], hn)].remove(edge)
        self._add_move(prev[1], pid, ln, lane)
        self._add_move(pid, nxt[1], ln, lane)
        return pid

    def _attach_lane(self, st: Station) -> StationNode | None:
        best = None
        for ln, lane, along, d in self._lane_candidates(st.x, st.y, 0.8):
            if best is None or d < best[3]:
                best = (ln, lane, along, d)
        if best is None:
            return None
        ln, lane, along, _ = best
        pid = self._insert_lane_point(ln, lane, along, f"stop:{st.name}")
        if pid is None:
            return None
        p = self.points[pid]
        snapped = replace(st, x=round(p.x, 3), y=round(p.y, 3), yaw=ln.lane_heading(lane))
        if math.hypot(p.x - st.x, p.y - st.y) > 0.01:
            self.report.append(f"  station {st.name}: served on lane {ln.name}/{lane} at ({p.x:.2f}, {p.y:.2f})")
        return StationNode(snapped, pid, heading_name(ln.lane_heading(lane)))

    def _attach_maneuver(self, st: Station) -> StationNode | None:
        amr = self.site.amr
        travel_end = st.yaw if st.approach == "forward" else wrap(st.yaw + math.pi)
        rev_end = st.approach == "reverse"
        radius = 12.0
        cands = []  # (distance, line, lane, along, point id or None)
        for pid, p in self.points.items():
            d = math.hypot(p.x - st.x, p.y - st.y)
            if p.xing and d <= radius:
                for ln_name, lane in p.lanes:
                    ln = self.lines[ln_name]
                    cands.append((d, ln, lane, ln.along(p.x, p.y), pid))
        for ln in self.lines.values():
            for lane in ("A", "B"):
                for k in range(int((ln.hi - ln.lo) / 0.5) + 1):
                    a = ln.lo + 0.5 * k
                    x, y = ln.lane_point(lane, a)
                    d = math.hypot(x - st.x, y - st.y)
                    if d <= radius:
                        cands.append((d, ln, lane, a, None))
        cands.sort(key=lambda c: c[0])
        best: tuple[float, dict] | None = None
        for d, ln, lane, along, pid in cands:
            if best is not None and d / amr.v_station > best[0]:
                break  # every remaining root is further away than the best manoeuvre takes
            if not self._on_usable_stretch(ln, lane, along):
                continue
            rx, ry = ln.lane_point(lane, along)
            h_lane = ln.lane_heading(lane)
            for legs, cost in self._maneuver_options(rx, ry, h_lane, st, travel_end, rev_end):
                cost += 0.0 if pid else 0.3  # slight preference for existing corners (no lane split)
                if best is not None and cost >= best[0]:
                    continue
                samples = self._sweep(rx, ry, h_lane, legs)
                if samples is None:
                    continue
                best = (cost, dict(ln=ln, lane=lane, along=along, pid=pid, legs=legs, samples=samples,
                                   h_lane=h_lane))
        if best is None:
            return None
        b = best[1]
        ln, lane = b["ln"], b["lane"]
        pid = b["pid"] or self._insert_lane_point(ln, lane, b["along"], f"root:{st.name}")
        if pid is None:
            return None
        h_name = heading_name(b["h_lane"])
        exit_legs = _reverse_legs(b["legs"], b["h_lane"], self.points[pid])
        depends = tuple(sorted({
            o.name for o in self.site.stations.values()
            if o.name != st.name and o.approach != "lane"
            and any(boxes_overlap(self.site.body(x, y, th), self.site.body(o.x, o.y, o.yaw, 0.02))
                    for x, y, th in b["samples"])}))
        m = Maneuver(st.name, pid, h_name, b["legs"], h_name, exit_legs, best[0], best[0], depends, b["samples"])
        return StationNode(st, pid, h_name, m)

    def _on_usable_stretch(self, ln: Line, lane: str, along: float) -> bool:
        key = (ln.name, lane)
        cache = self.__dict__.setdefault("_sorted_cache", {})
        pts = cache.get(key)
        if pts is None or len(pts) != len(self._lane_points[key]):
            pts = cache[key] = sorted(self._lane_points[key])
        for (a0, p0), (a1, p1) in zip(pts, pts[1:]):
            if a0 - 1e-6 <= along <= a1 + 1e-6:
                hn = heading_name(ln.lane_heading(lane))
                src, dst = (p0, p1) if lane == "A" else (p1, p0)
                return any(e.kind == "move" and e.dst[0] == dst for e in self.edges[(src, hn)])
        return False

    def _maneuver_options(self, rx, ry, h_lane, st: Station, travel_end: float, rev_end: bool):
        amr = self.site.amr
        vs = amr.v_station
        ex, ey = math.cos(travel_end), math.sin(travel_end)
        # 1) straight onto the station axis from the root
        dx, dy = st.x - rx, st.y - ry
        along = dx * ex + dy * ey
        lateral = -dx * ey + dy * ex
        if along > 0.2 and abs(lateral) < 0.03:
            legs = [Leg("rotate", th1=st.yaw), Leg("move", st.x, st.y, reverse=rev_end, v_max=vs, tag="dock")]
            yield legs, along / vs + abs(wrap(st.yaw - h_lane)) / amr.w_turn
        # 2) shift (lateral quintic) when the station is parallel to the lane
        if abs(wrap(st.yaw - h_lane)) < math.radians(2) and 0.3 < abs(lateral) <= 1.4 and 2.0 <= along <= 4.5:
            legs = [Leg("shift", st.x, st.y, th1=travel_end, reverse=rev_end, v_max=0.3, tag="dock")]
            yield legs, along / 0.3 + 2.0
        # 3) two legs via a pre-station point on the axis
        for k in range(4, 16):
            d = 0.2 * k
            px, py = st.x - ex * d, st.y - ey * d
            L1 = math.hypot(px - rx, py - ry)
            if L1 < 0.05:
                continue
            phi = math.atan2(py - ry, px - rx)
            for rev1 in (False, True):
                body1 = wrap(phi + (math.pi if rev1 else 0.0))
                legs = [Leg("rotate", th1=body1), Leg("move", px, py, reverse=rev1, v_max=vs),
                        Leg("rotate", th1=st.yaw), Leg("move", st.x, st.y, reverse=rev_end, v_max=vs, tag="dock")]
                cost = ((L1 + d) / vs + (abs(wrap(body1 - h_lane)) + abs(wrap(st.yaw - body1))) / amr.w_turn + 2.0)
                yield legs, cost

    def _sweep(self, x: float, y: float, th: float, legs: list[Leg]) -> list[tuple[float, float, float]] | None:
        """Body poses along a manoeuvre, or None if any part collides with the site.

        Straight moves are checked as one swept box (exact for a translating
        rectangle), rotations every 8 degrees, shifts every 5 cm.
        """
        amr, idx, ign = self.site.amr, self.site.index, self._ignore
        cx, cy, cth = x, y, th
        for lg in legs:
            if lg.kind == "rotate":
                ang = wrap(lg.th1 - cth)
                n = max(int(abs(ang) / math.radians(8)), 1)
                for i in range(1, n + 1):
                    a = cth + ang * i / n
                    if idx.box_hits(self.site.body(cx, cy, a), MANEUVER_MARGIN, ign):
                        return None
                cth = lg.th1
            elif lg.kind == "move":
                L = math.hypot(lg.x1 - cx, lg.y1 - cy)
                travel = math.atan2(lg.y1 - cy, lg.x1 - cx)
                swept = OrientedBox((cx + lg.x1) / 2, (cy + lg.y1) / 2, L + amr.length, amr.width, travel)
                if idx.box_hits(swept, MANEUVER_MARGIN, ign):
                    return None
                cx, cy = lg.x1, lg.y1
                cth = wrap(travel + (math.pi if lg.reverse else 0.0))
            elif lg.kind == "shift":
                traj = self.builder.build(0.0, cx, cy, cth, 0.0, [lg])
                for _t, px, py, pth, _v in traj.samples(0.1):
                    if idx.box_hits(self.site.body(px, py, pth), MANEUVER_MARGIN, ign):
                        return None
                cx, cy, cth = traj.end_pose()
        traj = self.builder.build(0.0, x, y, th, 0.0, legs)
        return [(px, py, pth) for _t, px, py, pth, _v in traj.samples(0.1)]

    _ignore: tuple[str, ...] = ()

    def _attach_maneuver_for(self, st: Station):
        self._ignore = st.ignore
        try:
            return self._attach_maneuver(st)
        finally:
            self._ignore = ()

    # ============================================================== routing

    def neighbours(self, node: tuple[str, str]) -> list[Edge]:
        return self.edges.get(node, [])

    def route(self, start: tuple[str, str], goal_pid: str, goal_heading: str | None = None,
              cost_fn=None, forbidden: set[tuple[str, str]] | None = None) -> list[tuple[str, str]] | None:
        """A* over (point, heading) states. ``cost_fn(edge) -> extra seconds`` adds congestion costs."""
        gp = self.points[goal_pid]
        vmax = self.site.amr.v_cruise

        def h(node):
            p = self.points[node[0]]
            return abs(p.x - gp.x) / vmax + abs(p.y - gp.y) / vmax * 0.999

        open_ = [(h(start), 0.0, start)]
        best = {start: 0.0}
        came: dict = {}
        while open_:
            _, g, node = heapq.heappop(open_)
            if g > best.get(node, math.inf) + 1e-9:
                continue
            if node[0] == goal_pid and (goal_heading is None or node[1] == goal_heading):
                path = [node]
                while node in came:
                    node = came[node]
                    path.append(node)
                return path[::-1]
            for e in self.neighbours(node):
                if forbidden and (e.src[0], e.dst[0]) in forbidden:
                    continue
                c = e.cost + (cost_fn(e) if cost_fn else 0.0)
                ng = g + c
                if ng < best.get(e.dst, math.inf) - 1e-9:
                    best[e.dst] = ng
                    came[e.dst] = node
                    heapq.heappush(open_, (ng + h(e.dst), ng, e.dst))
        return None

    def route_cost(self, path: list[tuple[str, str]], cost_fn=None) -> float:
        total = 0.0
        for a, b in zip(path, path[1:]):
            e = next(e for e in self.edges[a] if e.dst == b)
            total += e.cost + (cost_fn(e) if cost_fn else 0.0)
        return total

    def legs_for(self, path: list[tuple[str, str]]) -> list[Leg]:
        legs: list[Leg] = []
        for a, b in zip(path, path[1:]):
            if a[0] == b[0]:
                legs.append(Leg("rotate", th1=HEADINGS[b[1]]))
            else:
                p = self.points[b[0]]
                legs.append(Leg("move", p.x, p.y, tag=b[0]))
        return legs

    def nearest_node(self, x: float, y: float, th: float) -> tuple[str, str] | None:
        """The closest graph state ahead of a pose (for robots that start off the graph or re-plan mid-lane).

        In a one-way lane it is the next lane point in the lane's direction of travel: a point behind the
        robot would need a U-turn inside the lane (after a back-off the nearest point is often just behind).
        """
        where = self.line_of(x, y, th)
        if where is not None:
            ln, lane = where
            s = ln.along(x, y)  # same coordinate as the lane points (NOT the line-frame distance from its origin)
            h = ln.lane_heading(lane)
            sign = 1.0 if (math.cos(h) + math.sin(h)) > 0 else -1.0  # the lane runs towards +along or -along
            hn_lane = heading_name(h)
            ahead = sorted(((a - s) * sign, pid) for a, pid in self._lane_points[(ln.name, lane)]
                           if (a - s) * sign > 0.05 and (pid, hn_lane) in self.edges)
            if ahead:
                return ahead[0][1], hn_lane
        best = None
        hn = heading_name(th)
        c, sn = math.cos(th), math.sin(th)
        for pid, p in self.points.items():
            d = math.hypot(p.x - x, p.y - y)
            behind = (p.x - x) * c + (p.y - y) * sn < -0.05
            for node in ((pid, k) for k in HEADINGS):
                if node not in self.edges:
                    continue
                pen = (0.0 if node[1] == hn else 1.0) + (5.0 if behind else 0.0)
                if best is None or d + pen < best[0]:
                    best = (d + pen, node)
        return best[1] if best else None

    # ============================================================== resources

    def lines_near(self, box: OrientedBox) -> list[Line]:
        x0, y0, x1, y1 = aabb(box)
        out = []
        for ln in self.lines.values():
            if ln.orient == "V":
                if x1 > ln.center - ln.w and x0 < ln.center + ln.w and y1 > ln.lo and y0 < ln.hi:
                    out.append(ln)
            elif y1 > ln.center - ln.w and y0 < ln.center + ln.w and x1 > ln.lo and x0 < ln.hi:
                out.append(ln)
        return out

    def cover(self, x: float, y: float, th: float, pad_long: float = 0.0, pad_lat: float = 0.0) -> set[tuple[str, int]]:
        """Resources ``(key, index)`` covered by the body at a pose (padded). Cached per 1 cm / 0.5 degree."""
        key = (round(x, 2), round(y, 2), round(th * 115.0), round(pad_long, 3), round(pad_lat, 3))
        cache = self.__dict__.get("_cover_cache")
        if cache is None:
            cache = self.__dict__["_cover_cache"] = {}
        hit = cache.get(key)
        if hit is not None:
            return set(hit)
        res = self._cover(x, y, th, pad_long, pad_lat)
        if len(cache) > 200_000:
            cache.clear()
        cache[key] = frozenset(res)
        return res

    def _cover(self, x: float, y: float, th: float, pad_long: float = 0.0, pad_lat: float = 0.0) -> set[tuple[str, int]]:
        amr = self.site.amr
        return self.cover_box(OrientedBox(x, y, amr.length + 2 * pad_long, amr.width + 2 * pad_lat, th))

    def cover_box(self, body: OrientedBox) -> set[tuple[str, int]]:
        """Resources an arbitrary rectangle (a robot body, a dropped pallet) covers."""
        amr = self.site.amr
        x, y = body.cx, body.cy
        res: set[tuple[str, int]] = set()
        corners = body.corners()
        inside_lanes = False
        for ln in self.lines_near(body):
            ss, yy = zip(*(ln.to_frame(cx, cy) for cx, cy in corners))
            s0, s1 = max(min(ss), 0.0), min(max(ss), ln.hi - ln.lo)
            y0, y1 = min(yy), max(yy)
            if s1 <= s0:
                continue
            # a lane is used only where a robot driving in it actually sweeps: centre +- (half width + 0.05).
            # Together with CORE_PAD this keeps a robot held at a crossing's stop line out of the cross lane's
            # cells (with +0.1 its nose grazed them by 5 cm and blocked the robot turning inside the crossing).
            narrow = ln.w < 1.05
            half = ln.w if narrow else (amr.width / 2 + 0.05)
            for lane in ("A", "B"):
                yc = ln.lane_y(lane)
                lo, hi = yc - half, yc + half
                if y1 > lo + 0.015 and y0 < hi - 0.015:
                    for k in range(int(s0 // CELL), int(math.ceil(s1 / CELL))):
                        c_lo = k * CELL
                        c_hi = (k + 1) * CELL
                        ov = min(s1, c_hi) - max(s0, c_lo)
                        if ov > 0.08:
                            res.add((f"{ln.name}/{lane}", k))
            s_c, y_c = ln.to_frame(x, y)
            if 0 <= s_c <= ln.hi - ln.lo and 0 <= y_c <= 2 * ln.w:
                inside_lanes = True
        bx0, by0, bx1, by1 = aabb(body)
        for name, sbox, (sx0, sy0, sx1, sy1) in self._station_boxes():
            if bx1 < sx0 or sx1 < bx0 or by1 < sy0 or sy1 < by0:
                continue
            if boxes_overlap(body, sbox):
                res.add((f"S:{name}", 0))
        if not inside_lanes:  # free-space cells the (rotated) body really overlaps, not its bounding box
            x0, y0, x1, y1 = aabb(body)
            c, s_ = math.cos(body.heading), math.sin(body.heading)
            hl, hw = body.length / 2 + FREE_CELL * 0.7072, body.width / 2 + FREE_CELL * 0.7072
            for i in range(math.floor(x0 / FREE_CELL), math.floor(x1 / FREE_CELL) + 1):
                cx = (i + 0.5) * FREE_CELL - x
                for j in range(math.floor(y0 / FREE_CELL), math.floor(y1 / FREE_CELL) + 1):
                    cy = (j + 0.5) * FREE_CELL - y
                    # cell centre within the body grown by the cell half-diagonal (conservative, tight)
                    if abs(c * cx + s_ * cy) <= hl and abs(-s_ * cx + c * cy) <= hw:
                        res.add((f"F:{i}", j))
        return res

    def _station_boxes(self):
        cache = self.__dict__.get("_sbox_cache")
        if cache is None:
            cache = []
            for name, node in self.stations.items():
                if node.on_lane:
                    continue
                st = node.station
                b = self.site.body(st.x, st.y, st.yaw, 0.02)
                cache.append((name, b, aabb(b)))
            self.__dict__["_sbox_cache"] = cache
        return cache

    def core_cells(self) -> frozenset:
        """Lane cells inside the exact crossing cores (where crossing traffic sweeps)."""
        cache = self.__dict__.get("_ccells")
        if cache is None:
            cells = set()
            W = self.site.amr.width / 2 + 0.05
            for xg in self.xings.values():
                cx, cy = xg.center
                core = OrientedBox(cx, cy, 2 * (xg.v.w / 2 + W), 2 * (xg.h.w / 2 + W), 0.0)
                cells |= {res for res in self.cover_box(core) if "/" in res[0]}
            cache = self.__dict__["_ccells"] = frozenset(cells)
        return cache

    def core_cell_map(self) -> dict:
        """Lane cell -> name of the crossing whose core it lies in (a cell shared by two cores keeps the first)."""
        cache = self.__dict__.get("_ccmap")
        if cache is None:
            cache = {}
            W = self.site.amr.width / 2 + 0.05
            for name, xg in self.xings.items():
                cx, cy = xg.center
                core = OrientedBox(cx, cy, 2 * (xg.v.w / 2 + W), 2 * (xg.h.w / 2 + W), 0.0)
                for res in self.cover_box(core):
                    if "/" in res[0]:
                        cache.setdefault(res, name)
            self.__dict__["_ccmap"] = cache
        return cache

    def crossing_cells(self) -> frozenset:
        """Lane cells inside any crossing core: always reserved, whatever the traffic direction."""
        cache = self.__dict__.get("_xcells")
        if cache is None:
            cells = set()
            W = self.site.amr.width / 2 + 0.05
            for xg in self.xings.values():
                cx, cy = xg.center
                core = OrientedBox(cx, cy, 2 * (xg.v.w / 2 + W) + 0.6, 2 * (xg.h.w / 2 + W) + 0.6, 0.0)
                for res in self.cover_box(core):
                    if "/" in res[0]:
                        cells.add(res)
            cache = self.__dict__["_xcells"] = frozenset(cells)
        return cache

    def crossing_cell_map(self) -> dict:
        """Lane cell -> name of the crossing whose crossing cells it belongs to."""
        cache = self.__dict__.get("_xcmap")
        if cache is None:
            cache = {}
            W = self.site.amr.width / 2 + 0.05
            for name, xg in self.xings.items():
                cx, cy = xg.center
                core = OrientedBox(cx, cy, 2 * (xg.v.w / 2 + W) + 0.6, 2 * (xg.h.w / 2 + W) + 0.6, 0.0)
                for res in self.cover_box(core):
                    if "/" in res[0]:
                        cache.setdefault(res, name)
            self.__dict__["_xcmap"] = cache
        return cache

    def in_intersection(self, x: float, y: float, th: float) -> bool:
        """May the robot NOT stop here? True if the body overlaps the *core* of a crossing (where crossing
        traffic sweeps) or a station's keep-clear zone (the corridor stretch a docked robot sweeps when it
        leaves: a robot queued there and the robot inside the station would each wait for the other)."""
        if self._at_station(x, y):  # serving / docked at a station is always a legal place to be
            return False
        body = self.site.body(x, y, th)
        bx0, by0, bx1, by1 = aabb(body)
        for (zx0, zy0, zx1, zy1), zone in self._keep_clear_near(x, y):
            if zx0 < bx1 and bx0 < zx1 and zy0 < by1 and by0 < zy1 and boxes_overlap(body, zone):
                return True
        W = self.site.amr.width / 2 + CORE_PAD - 0.01
        for xg in self._xings_near(x, y):
            cx, cy = xg.center
            core = OrientedBox(cx, cy, 2 * (xg.v.w / 2 + W), 2 * (xg.h.w / 2 + W), 0.0)
            if boxes_overlap(body, core):
                return True
        return False

    def in_aisle(self, x: float, y: float) -> bool:
        """Is (x, y) inside a narrow rack aisle (not in a cross-aisle or outer corridor)?"""
        lines_a = self.__dict__.get("_aisle_lines")
        if lines_a is None:
            self._aisle_lines = [ln for ln in self.lines.values() if ln.kind == "aisle"]
            self._cross_lines = [ln for ln in self.lines.values() if ln.kind in ("cross", "corridor")]
            lines_a = self._aisle_lines
        if not any(ln.contains(x, y) for ln in lines_a):
            return False
        return not any(ln.contains(x, y) for ln in self._cross_lines)

    def in_narrow_corridor(self, x: float, y: float) -> bool:
        """Is (x, y) inside a corridor whose width requires single-file traffic (not in a crossing)?"""
        narrow = self.__dict__.get("_narrow_lines")
        if narrow is None:
            self._narrow_lines = [ln for ln in self.lines.values() if ln.kind == "corridor" and ln.w < 1.05]
            self._cross_lines_h = [ln for ln in self.lines.values() if ln.orient == "H"]
            narrow = self._narrow_lines
        if not any(ln.contains(x, y) for ln in narrow):
            return False
        return not any(cl.contains(x, y) for cl in self._cross_lines_h)

    def in_single_file(self, x: float, y: float) -> bool:
        """Is (x, y) inside a single-file rack aisle or narrow corridor?"""
        return self.in_aisle(x, y) or self.in_narrow_corridor(x, y)

    def single_file_line(self, x: float, y: float) -> str | None:
        """Name of the aisle or narrow corridor containing (x, y), if currently in single file."""
        if not self.in_single_file(x, y):
            return None
        return next((ln.name for ln in self.lines.values()
                     if (ln.kind == "aisle" or (ln.kind == "corridor" and ln.w < 1.05))
                     and ln.contains(x, y)), None)


    def _keep_clear_near(self, x: float, y: float):
        grid = self.__dict__.get("_keep_clear")
        if grid is None:
            grid = defaultdict(list)
            for node in self.stations.values():
                m = node.maneuver
                if m is None:
                    continue
                for sx, sy, sth in m.samples:
                    # only the part of the manoeuvre that lies in a corridor / aisle lane band
                    if not any(ln.contains(sx, sy) for ln in self.lines.values()):
                        continue
                    box = self.site.body(sx, sy, sth)
                    bb = aabb(box)
                    r = math.hypot(self.site.amr.length, self.site.amr.width) / 2  # reach of a querying body
                    x0, y0, x1, y1 = bb[0] - r, bb[1] - r, bb[2] + r, bb[3] + r
                    for gx in range(int(math.floor(x0 / 3.0)), int(math.floor(x1 / 3.0)) + 1):
                        for gy in range(int(math.floor(y0 / 3.0)), int(math.floor(y1 / 3.0)) + 1):
                            grid[(gx, gy)].append((bb, box))
            self.__dict__["_keep_clear"] = grid
        return grid.get((int(math.floor(x / 3.0)), int(math.floor(y / 3.0))), ())

    def _at_station(self, x: float, y: float, tol: float = 0.3) -> bool:
        pts = self.__dict__.get("_station_xy")
        if pts is None:
            pts = self.__dict__["_station_xy"] = [(n.station.x, n.station.y) for n in self.stations.values()]
        return any(abs(x - sx) < tol and abs(y - sy) < tol for sx, sy in pts)

    def _xings_near(self, x: float, y: float, r: float = 3.2):
        grid = self.__dict__.get("_xgrid")
        if grid is None:
            grid = defaultdict(list)
            for xg in self.xings.values():
                cx, cy = xg.center
                grid[(int(math.floor(cx / 4)), int(math.floor(cy / 4)))].append(xg)
            self.__dict__["_xgrid"] = grid
        i, j = int(math.floor(x / 4)), int(math.floor(y / 4))
        out = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for xg in grid.get((i + di, j + dj), ()):
                    if abs(xg.center[0] - x) < r and abs(xg.center[1] - y) < r:
                        out.append(xg)
        return out

    def line_of(self, x: float, y: float, th: float) -> tuple[Line, str] | None:
        """The line and lane a robot is driving in (heading aligned with a lane direction)."""
        best = None
        for ln in self.lines.values():
            s, ya = ln.to_frame(x, y)
            if not (-0.5 <= s <= ln.hi - ln.lo + 0.5) or not (-0.3 <= ya <= 2 * ln.w + 0.3):
                continue
            for lane in ("A", "B"):
                if not self.is_lane_enabled(ln.name, lane):
                    continue
                if abs(wrap(th - ln.lane_heading(lane))) < math.radians(20):
                    d = abs(ya - ln.lane_y(lane))
                    if best is None or d < best[2]:
                        best = (ln, lane, d)
        return (best[0], best[1]) if best else None

    # ============================================================== summary / plot

    def summary(self) -> str:
        return "\n".join(self.site.adjustments + self.report)

    def plot(self, path: str, routes: Iterable[list[tuple[float, float]]] = ()) -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon

        fig, ax = plt.subplots(figsize=(26, 17))
        for nb in self.site.boxes:
            col = "0.35" if nb.kind in ("wall", "rack") else "tab:red"
            ax.add_patch(Polygon(nb.box.corners(), closed=True, fc=col, ec=col, alpha=0.45))
        for ln in self.lines.values():
            b = ln.band()
            ax.add_patch(Polygon(b.corners(), closed=True, fc="tab:blue", alpha=0.06, ec="none"))
            for lane in ("A", "B"):
                p0, p1 = ln.lane_point(lane, ln.lo), ln.lane_point(lane, ln.hi)
                ax.plot([p0[0], p1[0]], [p0[1], p1[1]], lw=0.4, color="tab:blue", alpha=0.5)
        for (a, b) in [(e.src, e.dst) for e in self._all_edges() if e.kind == "move"]:
            pa, pb = self.points[a[0]], self.points[b[0]]
            ax.annotate("", xy=(pb.x, pb.y), xytext=(pa.x, pa.y),
                        arrowprops=dict(arrowstyle="->", color="tab:green", lw=0.8))
        for ln_name, lane, a0, a1 in self.blocked_lanes:
            ln = self.lines[ln_name]
            p0, p1 = ln.lane_point(lane, a0), ln.lane_point(lane, a1)
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], lw=2.5, color="tab:red")
        for pid, p in self.points.items():
            if p.xing:
                ok = bool(self.turns.get(pid))
                ax.add_patch(Circle((p.x, p.y), 0.12, color="tab:green" if ok else "tab:orange"))
        for name, node in self.stations.items():
            st = node.station
            ax.add_patch(Polygon(self.site.body(st.x, st.y, st.yaw).corners(), closed=True, fc="none",
                                 ec="tab:purple", lw=1.2))
            ax.text(st.x, st.y, name, fontsize=6, color="tab:purple")
            if node.maneuver:
                xs = [s[0] for s in node.maneuver.samples]
                ys = [s[1] for s in node.maneuver.samples]
                ax.plot(xs, ys, color="tab:purple", lw=0.8)
        for name, st in self.site.stations.items():
            if name not in self.stations:
                ax.add_patch(Polygon(self.site.body(st.x, st.y, st.yaw).corners(), closed=True, fc="red",
                                     ec="red", alpha=0.6))
                ax.text(st.x, st.y, name + " (unreachable)", fontsize=7, color="red")
        for r in routes:
            ax.plot([p[0] for p in r], [p[1] for p in r], lw=2, alpha=0.7)
        ax.set_xlim(-41, 41)
        ax.set_ylim(-26, 26)
        ax.set_aspect("equal")
        ax.set_title("NRDA lane graph (green arrows: lane edges, red: blocked stretches, purple: stations)")
        fig.savefig(path, dpi=80, bbox_inches="tight")
        plt.close(fig)


def _subtract(intervals: list[tuple[float, float]], occ: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out = []
    for a, b in intervals:
        cur = a
        for o0, o1 in occ:
            if o1 <= cur or o0 >= b:
                continue
            if o0 > cur:
                out.append((cur, o0))
            cur = max(cur, o1)
        if cur < b:
            out.append((cur, b))
    return out


def _reverse_legs(legs: list[Leg], h_lane: float, root: Point) -> list[Leg]:
    """Exit manoeuvre: drive the entry backwards to the root, then face the lane direction."""
    pts = [(root.x, root.y)]
    moving = [lg for lg in legs if lg.kind in ("move", "shift")]
    for lg in moving:
        pts.append((lg.x1, lg.y1))
    out: list[Leg] = []
    for lg, target in zip(reversed(moving), reversed(pts[:-1])):
        if lg.kind == "shift":
            out.append(Leg("shift", target[0], target[1], th1=wrap(lg.th1 + math.pi), reverse=not lg.reverse,
                           v_max=lg.v_max, tag="undock"))
        else:
            out.append(Leg("move", target[0], target[1], reverse=not lg.reverse, v_max=lg.v_max, tag="undock"))
    out.append(Leg("rotate", th1=h_lane))
    return out
