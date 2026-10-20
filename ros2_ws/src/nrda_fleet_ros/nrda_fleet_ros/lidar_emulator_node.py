"""CPU lidar emulator for the ``light`` sensor profile.

A 4 GB laptop GPU cannot render 40 gpu_lidars over the 24 MB warehouse mesh at
10 Hz (measured: real-time factor 0.25). This node publishes the same two
270-degree scans per robot (``/AMR_i/scan_front``, ``/AMR_i/scan_rear``,
frames ``AMR_i/base_link/lidar_*``) by ray casting the site's collision boxes,
every robot's *true* footprint (Gazebo ground-truth poses) and the obstacles
injected from the console. The agents cannot tell the difference: same
topics, same message type, same mount geometry as ``models/amr_01``.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from rclpy.event_handler import PublisherEventCallbacks

from nrda_fleet.geom import OrientedBox
from nrda_fleet.raycast import BoxSet, scan
from nrda_msgs.msg import ObstacleList

from .common import QOS_LATCHED, load_site, yaw_of

# No default QoS-event handlers on the 40 scan publishers, as in nrda_fleet_ros/edge_agent_node: each one
# is an extra waitable the Python executor scans on every wake-up. It also silences a burst of 40
# "requesting incompatible QoS. No messages will be sent to it" warnings at every launch, one per scan
# topic, which RViz provokes while building its displays and then resolves by resubscribing with the
# Best Effort QoS its config actually specifies. The warnings named a fault that was not there.
NO_PUB_EVENTS = PublisherEventCallbacks(use_default_callbacks=False)


class LidarEmulator(Node):
    def __init__(self):
        super().__init__("lidar_emulator", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.rate = float(self.declare_parameter("rate", 10.0).value)
        self.beams = int(self.declare_parameter("beams", 271).value)
        self.noise = float(self.declare_parameter("noise", 0.01).value)
        self.site, _ = load_site()
        self.robots = list(self.site.robots)
        count = int(self.declare_parameter("robots", len(self.robots)).value)
        self.robots = self.robots[:count]
        self.truth: dict[str, tuple[float, float, float]] = {}
        self.truth_stamp: dict = {}  # robot -> stamp of its latest true pose (the scan is cast from that pose)
        self.obstacles: list[tuple[str, OrientedBox]] = []
        self.pubs = {rid: (self.create_publisher(LaserScan, f"/{rid}/scan_front", qos_profile_sensor_data,
                                                 event_callbacks=NO_PUB_EVENTS),
                           self.create_publisher(LaserScan, f"/{rid}/scan_rear", qos_profile_sensor_data,
                                                 event_callbacks=NO_PUB_EVENTS))
                     for rid in self.robots}
        self.create_subscription(TFMessage, "/nrda/ground_truth", self._on_truth, 10)
        self.create_subscription(ObstacleList, "/fleet/obstacles", self._on_obstacles, QOS_LATCHED)
        self._static = [(nb.name, nb.box) for nb in self.site.boxes]
        # boxes left on the floor exist in the world (so the lidar sees them) but not in the robots' map
        self._static += [(n, OrientedBox(x, y, L, W, yaw)) for n, (x, y, yaw, L, W) in self.site.floor_boxes.items()]
        self._rng = np.random.default_rng(7)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f"emulating 2 x {self.beams}-beam lidars at {self.rate:.0f} Hz for {len(self.robots)} robots")

    def _on_truth(self, m: TFMessage) -> None:
        for tf in m.transforms:
            name = tf.child_frame_id.split("/")[0].split("::")[0]
            if name in self.pubs:
                t = tf.transform
                self.truth[name] = (t.translation.x, t.translation.y, yaw_of(t.rotation))
                self.truth_stamp[name] = tf.header.stamp

    def _on_obstacles(self, m: ObstacleList) -> None:
        self.obstacles = [(o.name, OrientedBox(o.x, o.y, o.length, o.width, o.yaw)) for o in m.obstacles if o.active]

    def _tick(self) -> None:
        if not self.truth:
            return
        amr = self.site.amr
        items = list(self._static) + [(rid, OrientedBox(x, y, amr.length, amr.width, th))
                                      for rid, (x, y, th) in self.truth.items()] + self.obstacles
        boxes = BoxSet([b for _, b in items], [n for n, _ in items])
        stamp = self.get_clock().now().to_msg()
        for rid, pose in self.truth.items():
            for mount, pub, frame in ((amr.lidars[0], self.pubs[rid][0], "lidar_front_right"),
                                      (amr.lidars[1], self.pubs[rid][1], "lidar_rear_left")):
                _o, _ang, rel, rng = scan(boxes, pose, mount, self.beams, rid, 0.0, None, 12.0)
                if self.noise > 0:
                    fin = np.isfinite(rng)
                    rng[fin] += self._rng.normal(0.0, self.noise, int(fin.sum()))
                msg = LaserScan()
                # the pose's own time: the agent projects the scan with its pose *at this stamp*, so a turning
                # robot does not see the racks smeared sideways (phantom "moving" obstacles -> false brakes)
                ts = self.truth_stamp.get(rid)
                msg.header.stamp = ts if ts is not None and (ts.sec or ts.nanosec) else stamp
                msg.header.frame_id = f"{rid}/base_link/{frame}"
                msg.angle_min, msg.angle_max = float(rel[0]), float(rel[-1])
                msg.angle_increment = float(rel[1] - rel[0])
                msg.scan_time = 1.0 / self.rate
                msg.range_min, msg.range_max = 0.05, 12.0
                msg.ranges = [float(r) if math.isfinite(r) else math.inf for r in rng]
                pub.publish(msg)


def main():
    rclpy.init()
    node = LidarEmulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        # On SIGINT the context can be torn down while spin is rebuilding its wait set, which surfaces as
        # "failed to initialize wait set: the given context is not valid". It is the shutdown we asked for,
        # not a fault, but uncaught it made every Ctrl-C end in a traceback and a non-zero exit.
        pass


if __name__ == "__main__":
    main()
