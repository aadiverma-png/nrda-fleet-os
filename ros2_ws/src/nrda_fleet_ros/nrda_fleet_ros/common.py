"""Shared ROS plumbing: topics, QoS, message conversion, site loading."""

from __future__ import annotations

import math

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nrda_fleet import bus as B
from nrda_msgs.msg import (Alert, Bid, Claim, ClaimSet, Control, RobotState, SafetyEvent, Task, TaskEvent)

TOPIC = {
    B.RobotStateMsg: "/fleet/state",
    B.ClaimMsg: "/fleet/claims",
    B.BidMsg: "/fleet/bids",
    B.TaskMsg: "/fleet/tasks",
    B.TaskEventMsg: "/fleet/task_events",
    B.AlertMsg: "/fleet/alerts",
    B.ControlMsg: "/fleet/control",
    B.SafetyEventMsg: "/fleet/safety_events",
}
ROS_TYPE = {
    B.RobotStateMsg: RobotState, B.ClaimMsg: ClaimSet, B.BidMsg: Bid, B.TaskMsg: Task,
    B.TaskEventMsg: TaskEvent, B.AlertMsg: Alert, B.ControlMsg: Control, B.SafetyEventMsg: SafetyEvent,
}

QOS_STATE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
QOS_RELIABLE = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
QOS_LATCHED = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def qos_for(cls):
    if cls is B.RobotStateMsg:
        return QOS_STATE
    if cls in (B.TaskMsg, B.ControlMsg):
        return QOS_LATCHED
    return QOS_RELIABLE


def _fin(v: float) -> float:
    return float(v) if math.isfinite(v) else 1e9


def to_ros(msg):
    """nrda_fleet dataclass -> nrda_msgs message."""
    if isinstance(msg, B.RobotStateMsg):
        m = RobotState(robot_id=msg.robot_id, stamp=float(msg.stamp), x=float(msg.x), y=float(msg.y),
                       th=float(msg.th), v=float(msg.v), w=float(msg.w), mode=msg.mode, battery=float(msg.battery),
                       task_id=msg.task_id, health=msg.health, lane=msg.lane, goal=msg.goal,
                       bypass=msg.bypass, compute_ms=float(msg.compute_ms), waiting_for=list(msg.waiting_for),
                       station=msg.station)
        m.path_x = [float(p[0]) for p in msg.path]
        m.path_y = [float(p[1]) for p in msg.path]
        return m
    if isinstance(msg, B.ClaimMsg):
        m = ClaimSet(robot_id=msg.robot_id, stamp=float(msg.stamp), seq=int(msg.seq) & 0xFFFFFFFF,
                     priority=int(msg.priority))
        m.claims = [Claim(key=k, idx=int(i), start=_fin(s), end=_fin(e), stamp=float(st))
                    for k, i, s, e, st in msg.claims]
        m.occupied_keys = [k for k, _ in msg.occupied]
        m.occupied_idx = [int(i) for _, i in msg.occupied]
        return m
    if isinstance(msg, B.BidMsg):
        tids = sorted(set(msg.winning_bids) | set(msg.winning_robots) | set(msg.timestamps))
        return Bid(robot_id=msg.robot_id, stamp=float(msg.stamp), iteration=int(msg.iteration), task_ids=tids,
                   bids=[float(msg.winning_bids.get(t, 0.0)) for t in tids],
                   winners=[msg.winning_robots.get(t, "") for t in tids],
                   timestamps=[float(msg.timestamps.get(t, 0.0)) for t in tids], bundle=list(msg.bundle))
    if isinstance(msg, B.TaskMsg):
        return Task(task_id=msg.task_id, stamp=float(msg.stamp), pickup=msg.pickup, dropoff=msg.dropoff,
                    priority=int(msg.priority), deadline=float(msg.deadline), assign_to=msg.assign_to,
                    kind=msg.kind, cancelled=bool(msg.cancelled))
    if isinstance(msg, B.TaskEventMsg):
        return TaskEvent(task_id=msg.task_id, robot_id=msg.robot_id, stamp=float(msg.stamp), status=msg.status,
                         detail=msg.detail)
    if isinstance(msg, B.AlertMsg):
        return Alert(sender=msg.sender, stamp=float(msg.stamp), kind=msg.kind, target=msg.target, x=float(msg.x),
                     y=float(msg.y), value=float(msg.value), detail=msg.detail)
    if isinstance(msg, B.ControlMsg):
        return Control(stamp=float(msg.stamp), command=msg.command, target=msg.target, value=msg.value)
    if isinstance(msg, B.SafetyEventMsg):
        return SafetyEvent(event_id=msg.event_id, stamp=float(msg.stamp), level=msg.level, robots=list(msg.robots),
                           other=msg.other, x=float(msg.x), y=float(msg.y), rel_speed=float(msg.rel_speed),
                           source=msg.source, detail=msg.detail)
    raise TypeError(type(msg))


def from_ros(m):
    """nrda_msgs message -> nrda_fleet dataclass."""
    if isinstance(m, RobotState):
        return B.RobotStateMsg(m.robot_id, m.stamp, m.x, m.y, m.th, m.v, m.w, m.mode, m.battery, m.task_id, m.health,
                               m.lane, m.goal, m.bypass, m.compute_ms,
                               list(zip(m.path_x, m.path_y)), list(m.waiting_for), m.station)
    if isinstance(m, ClaimSet):
        return B.ClaimMsg(m.robot_id, m.stamp, m.seq, m.priority,
                          [(c.key, c.idx, c.start, c.end if c.end < 1e8 else math.inf, c.stamp) for c in m.claims],
                          list(zip(m.occupied_keys, m.occupied_idx)))
    if isinstance(m, Bid):
        return B.BidMsg(m.robot_id, m.stamp, m.iteration, dict(zip(m.task_ids, m.bids)),
                        dict(zip(m.task_ids, m.winners)), dict(zip(m.task_ids, m.timestamps)), list(m.bundle))
    if isinstance(m, Task):
        return B.TaskMsg(m.task_id, m.stamp, m.pickup, m.dropoff, m.priority, m.deadline, m.assign_to, m.kind,
                         m.cancelled)
    if isinstance(m, TaskEvent):
        return B.TaskEventMsg(m.task_id, m.robot_id, m.stamp, m.status, m.detail)
    if isinstance(m, Alert):
        return B.AlertMsg(m.sender, m.stamp, m.kind, m.target, m.x, m.y, m.value, m.detail)
    if isinstance(m, Control):
        return B.ControlMsg(m.stamp, m.command, m.target, m.value)
    if isinstance(m, SafetyEvent):
        return B.SafetyEventMsg(m.event_id, m.stamp, m.level, list(m.robots), m.other, m.x, m.y, m.rel_speed,
                                m.source, m.detail)
    raise TypeError(type(m))


def sim_now(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_site():
    """Site + lane graph (cached on disk: every process loads the same graph in ~10 ms)."""
    from nrda_fleet.lane_graph import LaneGraph
    from nrda_fleet.site import Site
    site = Site.load()
    return site, LaneGraph.load_or_build(site)
