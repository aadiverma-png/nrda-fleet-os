"""Independent fleet safety monitor: the source of truth for the console's collision counters.

Inputs (none of them come from the agents' own collision logic):
* ``/nrda/ground_truth``   Gazebo world poses of every robot -> footprint overlap / near-miss
* ``/AMR_i/contacts``      chassis contact sensors -> physical contacts
* ``/fleet/alerts``        agents' risk brakes -> AVOIDED events
* ``/fleet/obstacles``     injected obstacles

Outputs: ``/fleet/safety_events`` (SafetyEvent) and ``~/.nrda/events.db`` (SQLite).
Policy ``on_collision``: ``stop_pair`` (pause the robots involved, default), ``stop_fleet`` or ``log_only``.
"""

from __future__ import annotations

import math
import os
import sqlite3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage

from nrda_fleet.bus import ControlMsg
from nrda_fleet.geom import OrientedBox
from nrda_fleet.safety_monitor import Body, SafetyTruth
from nrda_msgs.msg import Alert, ObstacleList, RobotState, SafetyEvent

from .common import QOS_LATCHED, QOS_RELIABLE, QOS_STATE, ROS_TYPE, TOPIC, load_site, sim_now, to_ros, yaw_of

IGNORE_CONTACT = ("ground_plane", "floor_slab", "truck_yard", "apron", "parking_lot")


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.policy = self.declare_parameter("on_collision", "stop_pair").value
        db = os.path.expanduser(self.declare_parameter("db", "~/.nrda/events.db").value)
        self.run_id = self.declare_parameter("run_id", "").value
        self.site, _ = load_site()
        count = int(self.declare_parameter("robots", len(self.site.robots)).value)
        self.robots = list(self.site.robots)[:count]
        self.truth = SafetyTruth(self.site.index, self.site.amr.length, self.site.amr.width)
        self.poses: dict[str, tuple[float, float, float]] = {}
        self.states: dict[str, RobotState] = {}
        self.obstacles: list[tuple[str, OrientedBox]] = []
        self.pub = self.create_publisher(SafetyEvent, "/fleet/safety_events", QOS_RELIABLE)
        self.ctl = self.create_publisher(ROS_TYPE[ControlMsg], TOPIC[ControlMsg], QOS_LATCHED)
        self.create_subscription(TFMessage, "/nrda/ground_truth", self._on_truth, 10)
        self.create_subscription(RobotState, "/fleet/state", lambda m: self.states.__setitem__(m.robot_id, m), QOS_STATE)
        self.create_subscription(Alert, "/fleet/alerts", self._on_alert, QOS_RELIABLE)
        self.create_subscription(ObstacleList, "/fleet/obstacles", self._on_obstacles, QOS_LATCHED)
        for rid in self.robots:
            self.create_subscription(Contacts, f"/{rid}/contacts", lambda m, r=rid: self._on_contacts(r, m), 10)
        os.makedirs(os.path.dirname(db), exist_ok=True)
        self.db = sqlite3.connect(db, check_same_thread=False)
        self.db.execute("CREATE TABLE IF NOT EXISTS events (run TEXT, id TEXT, stamp REAL, level TEXT, robots TEXT, "
                        "other TEXT, x REAL, y REAL, rel_speed REAL, source TEXT, detail TEXT, acked INTEGER DEFAULT 0)")
        self.db.commit()
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"safety monitor: {len(self.robots)} robots, on_collision={self.policy}, db={db}")

    def _on_truth(self, m: TFMessage) -> None:
        for tf in m.transforms:
            name = tf.child_frame_id
            if name in self.site.robots:
                t = tf.transform
                self.poses[name] = (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def _on_obstacles(self, m: ObstacleList) -> None:
        self.obstacles = [(o.name, OrientedBox(o.x, o.y, o.length, o.width, o.yaw)) for o in m.obstacles if o.active]

    def _on_alert(self, m: Alert) -> None:
        if m.kind == "risk":
            self._publish(self.truth.avoided(sim_now(self), m.sender, m.target, m.x, m.y, m.detail,
                                             self._rel_speed(m.sender, m.target)))

    def _on_contacts(self, rid: str, m: Contacts) -> None:
        for c in m.contacts:
            names = [c.collision1.name, c.collision2.name]
            other = next((n for n in names if not n.startswith(f"{rid}::")), None)
            if other is None or any(k in other for k in IGNORE_CONTACT):
                continue
            model = other.split("::")[0]
            # robots, floor boxes, pallets and chargers are models of their own; the warehouse is one model
            # whose collision names say what was hit (rack_..., wall_...)
            label = model if (model in self.site.robots or model.startswith(("box_", "obs_", "charger_"))) \
                else other.split("::")[-1]
            st = self.states.get(rid)
            if label.startswith("charger_") and st is not None and st.mode in ("charging", "parked", "to_charger"):
                continue  # docking contacts with its own charger
            x, y, _ = self.poses.get(rid, (0.0, 0.0, 0.0))
            self._publish(self.truth.contact(sim_now(self), rid, label, x, y, self._rel_speed(rid, label)))

    def _rel_speed(self, rid: str, label: str) -> float:
        """Impact speed of a contact: the closing speed of the pair for robot-robot, else the ego speed."""
        st = self.states.get(rid)
        ego = abs(st.v) if st is not None else 0.0
        if label in self.site.robots:
            other, pa, pb = self.states.get(label), self.poses.get(rid), self.poses.get(label)
            if st is not None and other is not None and pa is not None and pb is not None:
                tha, thb = pa[2], pb[2]
                return math.hypot(st.v * math.cos(tha) - other.v * math.cos(thb),
                                  st.v * math.sin(tha) - other.v * math.sin(thb))
        return ego

    def _tick(self) -> None:
        if not self.poses:
            return
        t = sim_now(self)
        bodies = []
        for rid, (x, y, th) in self.poses.items():
            st = self.states.get(rid)
            docked = st.station if st is not None and st.mode in ("charging", "parked") else ""
            bodies.append(Body(rid, x, y, th, st.v if st else 0.0, docked))
        boxes = [(n, OrientedBox(x, y, L, W, yaw)) for n, (x, y, yaw, L, W) in self.site.floor_boxes.items()]
        self._publish(self.truth.check(t, bodies, self.obstacles + boxes))

    def _publish(self, events) -> None:
        for ev in events:
            self.pub.publish(to_ros(ev))
            self.db.execute("INSERT INTO events VALUES (?,?,?,?,?,?,?,?,?,?,?,0)",
                            (self.run_id, ev.event_id, ev.stamp, ev.level, ",".join(ev.robots), ev.other, ev.x, ev.y,
                             ev.rel_speed, ev.source, ev.detail))
            self.db.commit()
            if ev.level == "COLLISION":
                self.get_logger().error(f"COLLISION {ev.event_id}: {' <-> '.join(ev.robots)} / {ev.other} at "
                                        f"({ev.x:.1f}, {ev.y:.1f}) [{ev.source}]")
                if self.policy == "stop_pair":
                    for rid in ev.robots:
                        self.ctl.publish(to_ros(ControlMsg(ev.stamp, "pause", rid, f"collision {ev.event_id}")))
                elif self.policy == "stop_fleet":
                    self.ctl.publish(to_ros(ControlMsg(ev.stamp, "pause", "", f"collision {ev.event_id}")))
            elif ev.level == "NEAR_MISS":
                self.get_logger().warning(f"near miss {ev.event_id}: {' <-> '.join(ev.robots)} {ev.detail}")


def main():
    rclpy.init()
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
