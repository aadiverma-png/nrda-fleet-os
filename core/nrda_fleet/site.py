"""Site model: the warehouse geometry and its named stations.

Everything comes from the ``nrds_warehouse_sim`` package, so the fleet sees
exactly the world Gazebo simulates:

* ``models/nrds_warehouse/model.sdf``: the 335 collision boxes. The ones at
  AMR height (0.02-0.45 m above the floor) inside the building are the static
  obstacles.
* ``config/locations.yaml``: picker zones, chargers, drops, stations.
* ``config/fleet.yaml``: parking slots and the robot roster (spawn stations).

Station poses are validated against the obstacles with the AMR-01
footprint. A pose that touches something (for example ``drop_3`` against
building column 41) is nudged backwards along its approach axis, or sideways,
until it is clear. Every nudge is recorded in :attr:`Site.adjustments`.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field, replace
from typing import Iterable

import yaml

from .amr import AMR01, AMR01Spec
from .geom import BoxIndex, NamedBox, OrientedBox
from .ids import id_key

ROBOT_BAND = (0.02, 0.45)  # z range of the AMR-01 body and lidars
INTERIOR = (-40.0, -25.0, 40.0, 25.0)  # inner faces of the outer walls


@dataclass(frozen=True)
class Station:
    name: str
    kind: str  # picker | drop | pack | decant | overflow | charger | parking
    x: float
    y: float
    yaw: float  # robot heading while serviced
    approach: str  # lane | forward | reverse
    dwell: float = 10.0  # s, nominal service time
    zone: str = ""  # picker zone id, charger bank ...
    ignore: tuple[str, ...] = ()  # collision boxes that belong to the station itself (charger tower)

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


@dataclass
class Site:
    root: str
    boxes: list[NamedBox]
    stations: dict[str, Station]
    robots: dict[str, str]  # robot id -> spawn station name
    amr: AMR01Spec = AMR01
    adjustments: list[str] = field(default_factory=list)
    zones: dict[str, dict] = field(default_factory=dict)  # picker zone bounds
    battery: dict[str, float] = field(default_factory=dict)  # initial state of charge per robot
    floor_boxes: dict[str, tuple[float, float, float, float, float]] = field(default_factory=dict)
    # ^ test obstacles on the floor (x, y, yaw, length, width): world truth for simulators and the safety
    #   monitor only. They are deliberately NOT part of ``boxes``: robots must find them with their lidar.

    def __post_init__(self):
        self.index = BoxIndex(self.boxes)

    # loading ---------------------------------------------------------------------

    @classmethod
    def load(cls, root: str | None = None, fleet_file: str | None = None) -> "Site":
        root = root or default_root()
        boxes = list(load_boxes(os.path.join(root, "models", "nrds_warehouse", "model.sdf")))
        with open(os.path.join(root, "config", "locations.yaml")) as f:
            loc = yaml.safe_load(f)
        fleet_path = fleet_file or os.path.join(root, "config", "fleet.yaml")
        fleet = {}
        if os.path.exists(fleet_path):
            with open(fleet_path) as f:
                fleet = yaml.safe_load(f) or {}
        stations: dict[str, Station] = {}
        zones = {}
        for zid, z in (loc.get("picker_zones") or {}).items():
            s = z["amr_stop"]
            stations[zid] = Station(zid, "picker", s["x"], s["y"], s.get("yaw", 0.0), "lane", 6.0, zid)
            zones[zid] = z.get("bounds", {})
        boxes += list(load_world_towers(os.path.join(root, "worlds", "nrds_warehouse.sdf")))
        disabled = []
        for cid, c in (loc.get("chargers") or {}).items():
            if c.get("disabled"):
                disabled.append(f"charger_{cid}: disabled ({c['disabled']})")
                continue
            tower = [b.name for b in boxes if b.name.startswith(f"charger_{cid}_tower")]
            stations[f"charger_{cid}"] = Station(f"charger_{cid}", "charger", c["x"], c["y"], c.get("yaw", 0.0),
                                                 "reverse", 60.0, cid[0], tuple(tower))
        for did, d in (loc.get("drop_points") or {}).items():
            # drive-through: a stop in the corridor lane beside the conveyor (no dead-end dock)
            stations[did] = Station(did, "drop", d["x"], d["y"], d.get("yaw", 0.0), d.get("approach", "lane"), 5.0)
        for sid, s in (loc.get("stations") or {}).items():
            if sid.startswith("packing"):
                kind, app, dwell = "pack", "lane", 5.0  # drive-through stop in the packing lane VP
            elif sid.startswith("overflow"):
                kind, app, dwell = "overflow", "lane", 5.0
            elif sid.startswith("decant"):
                kind, app, dwell = "decant", "lane", 6.0
            else:
                kind, app, dwell = "station", "forward", 5.0
            stations[sid] = Station(sid, kind, s["x"], s["y"], s.get("yaw", 0.0), app, dwell)
        for pid, p in (fleet.get("parking") or {}).items():
            stations[pid] = Station(pid, "parking", p["x"], p["y"], p.get("yaw", 0.0),
                                    p.get("approach", "reverse"), 0.0)
        robots = {rid: r["spawn"] for rid, r in (fleet.get("robots") or {}).items()}
        site = cls(root, boxes, stations, dict(sorted(robots.items(), key=lambda kv: id_key(kv[0]))), zones=zones)
        site.adjustments += disabled
        fb_path = os.path.join(root, "config", "floor_boxes.yaml")
        if os.path.exists(fb_path):
            with open(fb_path) as f:
                for name, b in ((yaml.safe_load(f) or {}).get("boxes") or {}).items():
                    site.floor_boxes[name] = (b["x"], b["y"], b.get("yaw", 0.0), b["length"], b["width"])
        site.battery = {rid: float(r.get("battery", 1.0)) for rid, r in (fleet.get("robots") or {}).items()}
        site._validate_stations()
        return site

    # queries ---------------------------------------------------------------------

    def body(self, x: float, y: float, yaw: float, pad: float = 0.0) -> OrientedBox:
        return OrientedBox(x, y, self.amr.length + 2 * pad, self.amr.width + 2 * pad, yaw)

    def body_hits(self, x: float, y: float, yaw: float, margin: float = 0.0,
                  ignore: Iterable[str] = ()) -> list[NamedBox]:
        return self.index.box_hits(self.body(x, y, yaw), margin, tuple(ignore))

    def stations_of(self, kind: str) -> list[Station]:
        return [s for s in self.stations.values() if s.kind == kind]

    def spawn_pose(self, rid: str) -> tuple[float, float, float]:
        return self.stations[self.robots[rid]].pose

    # validation -------------------------------------------------------------------

    def _validate_stations(self, margin: float = 0.02) -> None:
        for name, st in list(self.stations.items()):
            if st.approach == "lane":
                continue  # snapped onto its lane by the lane graph
            if not self.body_hits(st.x, st.y, st.yaw, margin, st.ignore):
                continue
            back = st.yaw + (math.pi if st.approach == "forward" else 0.0)  # towards the entry side
            fixed = None
            for step in range(1, 26):  # smallest displacement first: back, sideways, then both
                d = 0.02 * step
                for dd, ll in ((d, 0.0), (0.0, d), (0.0, -d), (d, d), (d, -d)):
                    xx = st.x + dd * math.cos(back) - ll * math.sin(st.yaw)
                    yy = st.y + dd * math.sin(back) + ll * math.cos(st.yaw)
                    if not self.body_hits(xx, yy, st.yaw, margin, st.ignore):
                        fixed = (xx, yy)
                        break
                if fixed:
                    break
            if fixed is None:
                hits = ", ".join(b.name for b in self.body_hits(st.x, st.y, st.yaw, margin, st.ignore))
                self.adjustments.append(f"{name}: collides with {hits}; could not fix, station disabled")
                del self.stations[name]
                continue
            self.adjustments.append(
                f"{name}: moved ({st.x:.2f}, {st.y:.2f}) -> ({fixed[0]:.2f}, {fixed[1]:.2f}) to clear "
                + ", ".join(b.name for b in self.body_hits(st.x, st.y, st.yaw, margin, st.ignore)))
            self.stations[name] = replace(st, x=round(fixed[0], 3), y=round(fixed[1], 3))


_BOX = re.compile(r'<collision name="([^"]+)"><pose>([^<]+)</pose><geometry><box><size>([^<]+)</size>')


def load_boxes(sdf_path: str) -> Iterable[NamedBox]:
    """Floor-level collision boxes inside the building from the warehouse model."""
    with open(sdf_path) as f:
        src = f.read()
    x0, y0, x1, y1 = INTERIOR
    for name, pose, size in _BOX.findall(src):
        x, y, z, _r, _p, yaw = map(float, pose.split())
        sx, sy, sz = map(float, size.split())
        if z + sz / 2 <= ROBOT_BAND[0] or z - sz / 2 >= ROBOT_BAND[1]:
            continue
        if not (x0 - 1 <= x <= x1 + 1 and y0 - 1 <= y <= y1 + 1):
            continue  # truck yard, car park
        yield NamedBox(name, OrientedBox(x, y, sx, sy, yaw), sz)


_INCLUDE = re.compile(r"<include><uri>model://nrda_charger</uri><name>charger_([A-Z0-9]+)</name><pose>([^<]+)</pose>")


def load_world_towers(world_path: str) -> Iterable[NamedBox]:
    """Charger towers added to the world as ``nrda_charger`` models (same 0.13 x 0.52 m tower)."""
    if not os.path.exists(world_path):
        return
    with open(world_path) as f:
        src = f.read()
    for cid, pose in _INCLUDE.findall(src):
        x, y, _z, _r, _p, yaw = map(float, pose.split())
        yield NamedBox(f"charger_{cid}_tower", OrientedBox(x, y, 0.13, 0.52, yaw), 0.44)


def default_root() -> str:
    """The ``nrds_warehouse_sim`` package: installed share dir, env override, or the source tree."""
    env = os.environ.get("NRDA_SITE_ROOT")
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
        return get_package_share_directory("nrds_warehouse_sim")
    except Exception:  # noqa: BLE001  (ROS not sourced / package not built)
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, "..", "..", "ros2_ws", "src", "nrds_warehouse_sim"))
