"""One NRDA edge agent per AMR (``ros2 run nrda_fleet_ros edge_agent --ros-args -p robot_id:=AMR_7``).

Hardware abstraction: pose = spawn pose composed with the DiffDrive odometry,
speed from the odometry twist, lidar from ``/AMR_i/scan_front`` and
``/AMR_i/scan_rear`` (Gazebo GPU lidars or the CPU lidar emulator).
The only output to the robot is ``/AMR_i/cmd_vel``, written from the agent's
MotionArbiter (FINAL_ALGORITHM section 5). Everything the robot tells the fleet
goes out on the ``/fleet/*`` peer topics, and everything it knows about the
fleet comes in on them.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.allocation import StationDistances
from nrda_fleet.perception import StaticMap, scan_points

from .common import ROS_TYPE, TOPIC, from_ros, load_site, qos_for, sim_now, to_ros, yaw_of


_SHARED: dict = {}
# no default QoS-event handlers: each one is an extra waitable the Python executor scans on every wake-up
NO_SUB_EVENTS = SubscriptionEventCallbacks(use_default_callbacks=False)
NO_PUB_EVENTS = PublisherEventCallbacks(use_default_callbacks=False)
# s without a fresh pose before the agent stops reporting. Matches ClaimBook.peer_timeout and the
# gateway's `stale` threshold, so the peers drop this robot's claims and the console greys it out in
# the same breath. Below it the arbiter's odom watchdog (0.5 s) is already braking.
POSE_LOST = 3.0


def pose_feed_age(t: float, odom_t: float, localized, localized_t: float) -> float:
    """Age of the pose an agent would act on at time ``t``.

    With ``localization:=truth`` that is the localiser feed (/nrda/ground_truth), which stops dead if
    the robot's Gazebo model or its bridge goes away; otherwise it is the odometry the pose is dead
    reckoned from. ``odom_t`` is 0.0 until the first odometry message, which reads as very old on
    purpose: the arbiter's own watchdog then brakes until the feed starts.
    """
    odom_age = max(t - odom_t, 0.0)
    return (t - localized_t) if localized is not None else odom_age


class PeerBus:
    """The /fleet/* peer topics for every agent hosted in this process.

    Each topic is subscribed and deserialised **once per process** and fanned out to the local agents
    (never back to the sender). On a real robot the process hosts one agent, so this is exactly its radio
    link; in simulation it removes the per-agent copies of 20 robots' gossip that dominated CPU time.
    """

    def __init__(self, node: Node, localization: str = "truth"):
        self.agents: dict[str, "EdgeAgentNode"] = {}
        if localization == "truth":
            # simulated localisation: the simulator's true pose stands in for the robot's localiser (AMCL /
            # lidar scan matching on a real AMR). Dead-reckoned wheel odometry drifts with slip, and a pose
            # error of a few cm shifts every lidar return: rack faces then look like unmapped obstacles in the
            # lanes and robots stop for boxes that are not there.
            node.create_subscription(TFMessage, "/nrda/ground_truth", self._on_truth, 20,
                                     event_callbacks=NO_SUB_EVENTS)
        self._pubs = {cls: node.create_publisher(ROS_TYPE[cls], TOPIC[cls], qos_for(cls), event_callbacks=NO_PUB_EVENTS)
                      for cls in TOPIC}
        for cls, topic in TOPIC.items():
            node.create_subscription(ROS_TYPE[cls], topic, self._on_msg, qos_for(cls), event_callbacks=NO_SUB_EVENTS)

    def publish(self, msg) -> None:
        self._pubs[type(msg)].publish(to_ros(msg))

    def _on_truth(self, m: TFMessage) -> None:
        for tf in m.transforms:
            a = self.agents.get(tf.child_frame_id.split("/")[0].split("::")[0])
            if a is not None:
                tr = tf.transform
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                a.localized = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                a.localized_t = stamp
                a.pose_hist.append((stamp, a.localized))

    def _on_msg(self, m) -> None:
        msg = from_ros(m)
        sender = getattr(msg, "robot_id", None) or getattr(msg, "sender", None)
        for rid, a in self.agents.items():
            if rid != sender:
                a.agent.on_message(msg)


def shared_world():
    """Read-only map data (site, lane graph, static lidar map), loaded once per process."""
    if not _SHARED:
        site, graph = load_site()
        _SHARED.update(site=site, graph=graph, static=StaticMap(site))
    return _SHARED["site"], _SHARED["graph"], _SHARED["static"]


class EdgeAgentNode(Node):
    def __init__(self, robot_id: str | None = None, comm_loss: float | None = None,
                 bus: PeerBus | None = None, localization: str | None = None):
        kw = {"namespace": f"/{robot_id}"} if robot_id else {}
        super().__init__("edge_agent", parameter_overrides=[Parameter("use_sim_time", value=True)], **kw)
        self.rid = robot_id or self.declare_parameter("robot_id", "AMR_1").value
        rate = float(self.declare_parameter("control_rate", 20.0).value)
        drop = comm_loss if comm_loss is not None else float(self.declare_parameter("comm_loss", 0.0).value)
        self._drop = drop
        self._rng = np.random.default_rng(abs(hash(self.rid)) % (2**32))
        t0 = time.time()
        self.site, self.graph, static = shared_world()
        # command-to-wheel + feedback delay through ros_gz_bridge / Gazebo physics (latency compensation)
        latency = float(self.declare_parameter("actuation_latency", 0.1).value)
        cfg = AgentConfig(control_dt=1.0 / rate, latency=latency)
        self.spawn = self.site.spawn_pose(self.rid)
        self.agent = EdgeAgent(self.rid, self.graph, StationDistances(self.graph), self._publish, cfg,
                               self.site.robots[self.rid], self.site.battery.get(self.rid, 1.0), static)
        self.get_logger().info(f"{self.rid}: edge agent ready in {time.time() - t0:.1f} s "
                               f"(spawn {self.site.robots[self.rid]} at {self.spawn[0]:.2f}, {self.spawn[1]:.2f})")
        # peer bus (shared by the agents of this process)
        loc = localization or self.declare_parameter("localization", "truth").value
        self.localized = None  # pose from the localisation source (None: dead reckoning from odometry)
        self.localized_t = 0.0  # sim time of that pose: the feed has to be watched, not just read
        self._pose_lost = False
        self.pose_hist: deque = deque(maxlen=40)  # (stamp, pose): ~2 s at 20 Hz, to project scans in time
        self.bus = bus or PeerBus(self, loc)
        self.bus.agents[self.rid] = self
        # hardware
        self.cmd_pub = self.create_publisher(Twist, f"/{self.rid}/cmd_vel", 10, event_callbacks=NO_PUB_EVENTS)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10, event_callbacks=NO_PUB_EVENTS)
        self.create_subscription(Odometry, f"/{self.rid}/odom", self._on_odom, 20, event_callbacks=NO_SUB_EVENTS)
        self.scans: dict[str, LaserScan] = {}
        for name in ("scan_front", "scan_rear"):
            self.create_subscription(LaserScan, f"/{self.rid}/{name}",
                                     lambda m, n=name: self.scans.__setitem__(n, m), qos_profile_sensor_data,
                                     event_callbacks=NO_SUB_EVENTS)
        self.pose = None
        self.v = self.w = 0.0
        self.odom_t = 0.0
        self._last_scan_stamp = None
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._diagnostics)
        self.create_timer(3.0, self._check_single_writer)
        self._checked = False

    # ------------------------------------------------------------------ peer bus

    def _publish(self, msg) -> None:
        if self._drop > 0 and self._rng.random() < self._drop:
            return  # emulated radio loss (comm_loss parameter)
        self.bus.publish(msg)  # the bus subscription delivers it to the other agents of this process too

    # ------------------------------------------------------------------ hardware

    def _on_odom(self, m: Odometry) -> None:
        x0, y0, th0 = self.spawn
        p = m.pose.pose.position
        c, s = math.cos(th0), math.sin(th0)
        self.pose = (x0 + c * p.x - s * p.y, y0 + s * p.x + c * p.y,
                     math.atan2(math.sin(th0 + yaw_of(m.pose.pose.orientation)),
                                math.cos(th0 + yaw_of(m.pose.pose.orientation))))
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z
        self.odom_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _points(self):
        front, rear = self.scans.get("scan_front"), self.scans.get("scan_rear")
        if front is None or rear is None:
            return None
        stamp = (front.header.stamp.sec, front.header.stamp.nanosec)
        if stamp == self._last_scan_stamp:
            return None
        self._last_scan_stamp = stamp
        pts = []
        for scan, mount in ((front, self.site.amr.lidars[0]), (rear, self.site.amr.lidars[1])):
            pose = self._pose_at(scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9)
            pts.append(scan_points(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                                   (mount.x, mount.y, mount.yaw), pose))
        return np.vstack(pts)

    def _pose_at(self, stamp: float):
        """Pose when the scan was taken (interpolated from the pose history); current pose if unknown."""
        h = self.pose_hist
        if not h or stamp <= 0.0:
            return self.pose
        if stamp >= h[-1][0]:
            return h[-1][1]
        for (t0, p0), (t1, p1) in zip(reversed(list(h)[:-1]), reversed(list(h))):
            if t0 <= stamp <= t1:
                f = 0.0 if t1 <= t0 else (stamp - t0) / (t1 - t0)
                dth = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
                return (p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1]), p0[2] + f * dth)
        return h[0][1]

    def _tick(self) -> None:
        if self.localized is not None:
            self.pose = self.localized  # localiser pose; odometry still gives speed and the watchdog stamp
        if self.pose is None:
            return
        t = sim_now(self)
        if t <= 0.0:
            return
        # Age of the pose the agent is about to act on, not just of the odometry: with localization:=truth
        # the pose comes from /nrda/ground_truth, and that feed stops dead if the robot's Gazebo model or
        # its bridge goes away. Unwatched, the agent then drives on a frozen pose and keeps publishing a
        # confident, non-stale RobotState, so the console draws a robot where the simulator has none.
        odom_age = max(t - self.odom_t, 0.0)  # odom_t is 0.0 until the first message: reads as very old
        pose_age = pose_feed_age(t, self.odom_t, self.localized, self.localized_t)
        if pose_age > POSE_LOST:
            self.cmd_pub.publish(Twist())  # hold the wheels, then go silent: peers drop our claims
            if not self._pose_lost:
                self._pose_lost = True
                self.get_logger().error(
                    f"{self.rid}: no pose for {pose_age:.1f} s (localisation feed stale). Holding and "
                    "leaving the fleet until it returns.")
            return
        if self._pose_lost:
            self._pose_lost = False
            self.get_logger().info(f"{self.rid}: pose feed back, rejoining the fleet")
        cmd = self.agent.tick(t, self.pose, self.v, self.w, self._points(), odom_age)
        tw = Twist()
        tw.linear.x, tw.angular.z = float(cmd.v), float(cmd.w)
        self.cmd_pub.publish(tw)

    def _diagnostics(self) -> None:
        a = self.agent
        st = DiagnosticStatus(name=f"nrda/{self.rid}", hardware_id=self.rid)
        st.level = DiagnosticStatus.OK if a.health.value == "HEALTHY" and not a.estop else DiagnosticStatus.WARN
        st.message = a.mode
        pm = sorted(a.stats.plan_ms)
        tm = sorted(a.stats.tick_ms)
        st.values = [KeyValue(key=k, value=str(v)) for k, v in (
            ("battery", round(a.battery, 3)), ("task", a.job["task_id"] if a.job else ""),
            ("plan_ms_p95", round(pm[int(0.95 * (len(pm) - 1))], 2) if pm else 0),
            ("tick_ms_p95", round(tm[int(0.95 * (len(tm) - 1))], 2) if tm else 0),
            ("safety_brakes", a.stats.safety_brakes), ("risk_brakes", a.stats.risk_brakes),
            ("tasks_done", a.stats.tasks_done))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def _check_single_writer(self) -> None:
        """FINAL_ALGORITHM 5.6: nobody else may command this robot."""
        if self._checked:
            return
        self._checked = True
        infos = self.get_publishers_info_by_topic(f"/{self.rid}/cmd_vel")
        others = [i for i in infos if not (i.node_name == self.get_name() and i.node_namespace == self.get_namespace())]
        if others:
            names = ", ".join(f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in others)
            self.get_logger().error(f"COMMAND CONFLICT: other publishers on /{self.rid}/cmd_vel: {names}. "
                                    "NRDA-FA requires a single motor-command writer; stopping this agent.")
            self.agent.estop = True


def main_group():
    """Several robots' agents in one process (``robot_ids:=AMR_1,AMR_2,...``).

    Each agent is still its own node (/AMR_i/edge_agent) with its own peer-bus
    subscriptions, arbiter and cmd_vel; they share only read-only map data. On real
    robots every agent runs on its own computer; in simulation grouping them keeps
    20 robots inside a 12 GB RAM budget.
    """
    rclpy.init()
    # Unique per process: several of these run side by side (one per group of robots), and two nodes with
    # the same fully-qualified name make the graph ambiguous for `ros2 node info` and the single-writer check.
    tag = str(os.getpid())
    boot = rclpy.create_node(f"edge_agents_boot_{tag}",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
    ids = [r for r in boot.declare_parameter("robot_ids", "AMR_1").value.split(",") if r]
    loss = float(boot.declare_parameter("comm_loss", 0.0).value)
    executor = boot.declare_parameter("executor", "events").value
    localization = boot.declare_parameter("localization", "truth").value
    boot.destroy_node()
    hub = rclpy.create_node(f"edge_agents_bus_{ids[0].lower() if ids else tag}",
                            parameter_overrides=[Parameter("use_sim_time", value=True)])
    bus = PeerBus(hub, localization)
    nodes = [EdgeAgentNode(rid, loss, bus, localization) for rid in ids]
    if executor == "events":
        from rclpy.experimental import EventsExecutor  # event-driven: no wait-set rebuild per message
        ex = EventsExecutor()
    else:
        ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(hub)
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            try:
                n.cmd_pub.publish(Twist())
            except Exception:  # noqa: BLE001
                pass


def main():
    rclpy.init()
    node = EdgeAgentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
