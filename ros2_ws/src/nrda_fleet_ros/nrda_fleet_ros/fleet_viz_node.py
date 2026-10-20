"""RViz view of the decentralised fleet.

* ``/nrda/robots``  (5 Hz) footprint coloured by mode, ``AMR_i`` label with mode / battery / task,
  upcoming route, wait-for arrows;
* ``/nrda/claims``  (2 Hz) each robot's space-time claims drawn as lane cells in its colour;
* ``/nrda/site``    (latched) lane network, intersections, stations with names;
* ``/nrda/safety``  collision (red) / near-miss (amber) / avoided (blue) markers for 20 s.
"""

from __future__ import annotations

import colorsys
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from nrda_fleet.ids import parse_robot_id
from nrda_msgs.msg import ClaimSet, RobotState, SafetyEvent

from .common import QOS_RELIABLE, QOS_STATE, load_site

MODE_COLOURS = {
    "idle": (0.6, 0.6, 0.6), "parked": (0.45, 0.55, 0.65), "charging": (0.1, 0.75, 0.3),
    "to_pickup": (0.15, 0.5, 0.95), "loading": (0.55, 0.3, 0.9), "to_drop": (0.95, 0.6, 0.1),
    "unloading": (0.9, 0.4, 0.7), "to_charger": (0.2, 0.85, 0.75), "to_park": (0.5, 0.65, 0.8),
    "paused": (1.0, 0.85, 0.1), "estop": (1.0, 0.1, 0.1), "fault": (0.8, 0.0, 0.0),
}
LEVEL_COLOURS = {"COLLISION": (1.0, 0.1, 0.1), "NEAR_MISS": (1.0, 0.65, 0.0), "AVOIDED": (0.2, 0.5, 1.0)}
STATION_COLOURS = {"charger": (0.1, 0.8, 0.3), "parking": (0.7, 0.7, 0.75), "picker": (0.55, 0.3, 0.9),
                   "drop": (0.95, 0.55, 0.1), "pack": (0.95, 0.35, 0.3), "decant": (0.3, 0.7, 0.9),
                   "overflow": (0.8, 0.8, 0.2)}


def rgba(c, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


def robot_colour(rid: str):
    try:
        i = parse_robot_id(rid)
    except ValueError:
        i = 0
    return colorsys.hsv_to_rgb((i * 0.137) % 1.0, 0.75, 0.95)


def yaw_q(m: Marker, yaw: float) -> None:
    m.pose.orientation.z, m.pose.orientation.w = math.sin(float(yaw) / 2), math.cos(float(yaw) / 2)


class FleetViz(Node):
    def __init__(self):
        super().__init__("fleet_viz", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.site, self.graph = load_site()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_robots = self.create_publisher(MarkerArray, "/nrda/robots", 10)
        self.pub_claims = self.create_publisher(MarkerArray, "/nrda/claims", 10)
        self.pub_site = self.create_publisher(MarkerArray, "/nrda/site", latched)
        self.pub_safety = self.create_publisher(MarkerArray, "/nrda/safety", 10)
        self.states: dict[str, RobotState] = {}
        self.claims: dict[str, ClaimSet] = {}
        self.create_subscription(RobotState, "/fleet/state", lambda m: self.states.__setitem__(m.robot_id, m), QOS_STATE)
        self.create_subscription(ClaimSet, "/fleet/claims", lambda m: self.claims.__setitem__(m.robot_id, m), QOS_RELIABLE)
        self.create_subscription(SafetyEvent, "/fleet/safety_events", self._on_event, QOS_RELIABLE)
        self._ev_id = 0
        self.pub_site.publish(self._site_markers())
        self.create_timer(0.2, self._robots)
        self.create_timer(0.5, self._claims)

    # ------------------------------------------------------------------ static site

    def _site_markers(self) -> MarkerArray:
        arr = MarkerArray()
        lanes = Marker(type=Marker.LINE_LIST, ns="lanes", id=0, action=Marker.ADD)
        lanes.header.frame_id = "map"
        lanes.scale.x = 0.04
        lanes.color = rgba((0.2, 0.6, 1.0), 0.35)
        lanes.pose.orientation.w = 1.0
        for es in self.graph.edges.values():
            for e in es:
                if e.kind != "move":
                    continue
                a, b = self.graph.points[e.src[0]], self.graph.points[e.dst[0]]
                lanes.points += [Point(x=float(a.x), y=float(a.y), z=float(0.02)), Point(x=float(b.x), y=float(b.y), z=float(0.02))]
        arr.markers.append(lanes)
        for i, (name, node) in enumerate(sorted(self.graph.stations.items())):
            st = node.station
            col = STATION_COLOURS.get(st.kind, (0.8, 0.8, 0.8))
            m = Marker(type=Marker.CUBE, ns="stations", id=i, action=Marker.ADD)
            m.header.frame_id = "map"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(st.x), float(st.y), 0.01
            yaw_q(m, st.yaw)
            m.scale.x, m.scale.y, m.scale.z = 1.25, 0.84, 0.02
            m.color = rgba(col, 0.45)
            arr.markers.append(m)
            t = Marker(type=Marker.TEXT_VIEW_FACING, ns="station_names", id=i, action=Marker.ADD)
            t.header.frame_id = "map"
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(st.x), float(st.y) - 0.6, 0.3
            t.pose.orientation.w = 1.0
            t.scale.z = 0.35
            t.color = rgba(col, 0.9)
            t.text = name.replace("charger_", "CHG ").replace("packing_induct_", "PACK ").replace("overflow_induct_", "OVF ")
            arr.markers.append(t)
        return arr

    # ------------------------------------------------------------------ robots

    def _robots(self) -> None:
        arr = MarkerArray()
        now = self.get_clock().now().nanoseconds * 1e-9
        for rid, s in sorted(self.states.items()):
            k = parse_robot_id(rid) if rid.startswith("AMR_") else 0
            stale = now - s.stamp > 3.0
            col = (0.3, 0.3, 0.3) if stale else MODE_COLOURS.get(s.mode, (0.8, 0.8, 0.8))
            body = Marker(type=Marker.CUBE, ns="body", id=k, action=Marker.ADD)
            body.header.frame_id = "map"
            body.pose.position.x, body.pose.position.y, body.pose.position.z = s.x, s.y, 0.45
            yaw_q(body, s.th)
            body.scale.x, body.scale.y, body.scale.z = 1.22, 0.8, 0.12
            body.color = rgba(col, 0.9)
            arr.markers.append(body)
            nose = Marker(type=Marker.ARROW, ns="heading", id=k, action=Marker.ADD)
            nose.header.frame_id = "map"
            nose.pose.position.x, nose.pose.position.y, nose.pose.position.z = s.x, s.y, 0.53
            yaw_q(nose, s.th)
            nose.scale.x, nose.scale.y, nose.scale.z = 0.7, 0.12, 0.12
            nose.color = rgba((1, 1, 1), 0.9)
            arr.markers.append(nose)
            label = Marker(type=Marker.TEXT_VIEW_FACING, ns="label", id=k, action=Marker.ADD)
            label.header.frame_id = "map"
            label.pose.position.x, label.pose.position.y, label.pose.position.z = s.x, s.y, 1.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color = rgba((1, 1, 1), 1.0)
            extra = f" {s.task_id}" if s.task_id else ""
            label.text = f"{rid}\n{s.mode} {int(s.battery * 100)}%{extra}"
            arr.markers.append(label)
            path = Marker(type=Marker.LINE_STRIP, ns="path", id=k, action=Marker.ADD)
            path.header.frame_id = "map"
            path.pose.orientation.w = 1.0
            path.scale.x = 0.08
            path.color = rgba(robot_colour(rid), 0.8)
            pts = [(s.x, s.y)] + list(zip(s.path_x, s.path_y))
            path.points = [Point(x=float(float(x)), y=float(float(y)), z=float(0.05)) for x, y in pts]
            if len(path.points) < 2:
                path.action = Marker.DELETE
            arr.markers.append(path)
            wait = Marker(type=Marker.ARROW, ns="waits", id=k, action=Marker.ADD)
            wait.header.frame_id = "map"
            wait.scale.x, wait.scale.y, wait.scale.z = 0.06, 0.15, 0.2
            wait.color = rgba((1.0, 0.8, 0.0), 0.9)
            blk = [self.states[b] for b in s.waiting_for if b in self.states]
            if blk:
                wait.points = [Point(x=float(s.x), y=float(s.y), z=float(0.9)), Point(x=float(blk[0].x), y=float(blk[0].y), z=float(0.9))]
            else:
                wait.action = Marker.DELETE
            arr.markers.append(wait)
        self.pub_robots.publish(arr)

    def _claims(self) -> None:
        arr = MarkerArray()
        now = self.get_clock().now().nanoseconds * 1e-9
        for rid, cs in self.claims.items():
            k = parse_robot_id(rid) if rid.startswith("AMR_") else 0
            m = Marker(type=Marker.CUBE_LIST, ns="claims", id=k, action=Marker.ADD)
            m.header.frame_id = "map"
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.42
            m.scale.z = 0.02
            m.color = rgba(robot_colour(rid), 0.35)
            seen = set()
            for c in cs.claims:
                if c.end < now or c.start > now + 12.0 or (c.key, c.idx) in seen:
                    continue
                seen.add((c.key, c.idx))
                p = self._cell_centre(c.key, c.idx)
                if p is not None:
                    m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(0.03)))
            arr.markers.append(m)
        self.pub_claims.publish(arr)

    def _cell_centre(self, key: str, idx: int):
        if "/" in key:
            line, lane = key.split("/")
            ln = self.graph.lines.get(line)
            if ln is None:
                return None
            return ln.to_world((idx + 0.5) * 0.5, ln.lane_y(lane))
        if key.startswith("F:"):
            return (int(key[2:]) + 0.5) * 0.25, (idx + 0.5) * 0.25
        return None

    def _on_event(self, ev: SafetyEvent) -> None:
        arr = MarkerArray()
        self._ev_id += 1
        col = LEVEL_COLOURS.get(ev.level, (1, 1, 1))
        m = Marker(type=Marker.SPHERE, ns=f"safety_{ev.level.lower()}", id=self._ev_id, action=Marker.ADD)
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = ev.x, ev.y, 0.6
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.8 if ev.level == "COLLISION" else 1.1
        m.color = rgba(col, 0.55)
        m.lifetime.sec = 20
        t = Marker(type=Marker.TEXT_VIEW_FACING, ns="safety_text", id=self._ev_id, action=Marker.ADD)
        t.header.frame_id = "map"
        t.pose.position.x, t.pose.position.y, t.pose.position.z = ev.x, ev.y, 2.2
        t.pose.orientation.w = 1.0
        t.scale.z = 0.5
        t.color = rgba(col, 1.0)
        t.text = f"{ev.level} {' / '.join(ev.robots)}"
        t.lifetime.sec = 20
        arr.markers += [m, t]
        self.pub_safety.publish(arr)


def main():
    rclpy.init()
    node = FleetViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
