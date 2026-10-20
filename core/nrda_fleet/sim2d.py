"""Headless 2-D warehouse simulator: the same edge agents as in Gazebo, faster than real time.

* AMR-01 dynamics like the gz-sim DiffDrive plugin (1.5 m/s, 1.5 rad/s,
  1.0 m/s^2, 2.0 rad/s^2), integrated at the 20 Hz control rate;
* two 270-degree lidars per robot at the SDF mounts, ray-cast against the
  site's collision boxes, the other robots and dynamic obstacles;
* all inter-robot information over :class:`~nrda_fleet.bus.SimBus` (latency,
  burst loss, partitions): no shared state between agents;
* ground-truth collision detection with the same rotated-box test the Gazebo
  safety monitor uses (``visison.geometry.boxes_overlap``), plus near-misses.

Everything is seeded, so a run is reproducible.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .agent import AgentConfig, EdgeAgent
from .allocation import StationDistances
from .bus import AlertMsg, CommProfile, ControlMsg, SimBus, TaskEventMsg
from .geom import OrientedBox, wrap
from .lane_graph import LaneGraph
from .perception import StaticMap
from .raycast import BoxSet, scan
from .safety_monitor import Body, SafetyTruth
from .site import Site
from .tasks import Task


@dataclass
class DynObstacle:
    """A pallet dropped in a lane, or a person walking between two points."""

    name: str
    x: float
    y: float
    length: float = 1.2
    width: float = 1.0
    t_on: float = 0.0
    t_off: float = math.inf
    waypoints: list[tuple[float, float]] = field(default_factory=list)  # walking person
    speed: float = 0.0
    _k: int = 0

    def active(self, t: float) -> bool:
        return self.t_on <= t < self.t_off

    def step(self, dt: float) -> None:
        if not self.waypoints or self.speed <= 0:
            return
        tx, ty = self.waypoints[self._k]
        d = math.hypot(tx - self.x, ty - self.y)
        if d < 0.05:
            self._k = (self._k + 1) % len(self.waypoints)
            return
        s = min(self.speed * dt, d)
        self.x += (tx - self.x) / d * s
        self.y += (ty - self.y) / d * s

    def box(self) -> OrientedBox:
        return OrientedBox(self.x, self.y, self.length, self.width, 0.0)


@dataclass
class SimRobot:
    rid: str
    x: float
    y: float
    th: float
    v: float = 0.0
    w: float = 0.0
    agent: EdgeAgent | None = None
    failed: bool = False  # hardware failure: stops, stops talking

    def box(self, L: float, W: float) -> OrientedBox:
        return OrientedBox(self.x, self.y, L, W, self.th)


class Sim2D:
    def __init__(self, site: Site, graph: LaneGraph, cfg: AgentConfig | None = None, robots: int | None = None,
                 seed: int = 0, comm: CommProfile | None = None, dt: float = 0.05, lidar: bool = True,
                 beams: int = 120, lidar_rate: float = 10.0,
                 floor_boxes: bool = False, latency: float = 0.0):
        self.site, self.g = site, graph
        self.cfg = cfg or AgentConfig()
        self.dt = dt
        # actuation / feedback latency like Gazebo's (ROS bridge + physics + 20-50 Hz state): a command takes
        # effect `latency` s after the agent issued it. 0 = ideal (unit tests); ~0.1 s = Gazebo-like.
        self.latency = latency
        self._cmd_queue: dict = {}
        self.t = 0.0
        self.rng = random.Random(seed)
        self.bus = SimBus(comm or CommProfile(), seed)
        self.dist = StationDistances(graph)
        self.static_map = StaticMap(site)
        self.lidar = lidar
        self.beams = beams
        self.lidar_period = 1.0 / lidar_rate
        self._next_scan = 0.0
        self.obstacles: list[DynObstacle] = []
        if floor_boxes:  # the test boxes of config/floor_boxes.yaml (unknown to the robots)
            for name, (bx, by, yaw, L, W) in site.floor_boxes.items():
                if abs(math.sin(yaw)) > 0.7:
                    L, W = W, L
                self.obstacles.append(DynObstacle(name, bx, by, L, W))
        self.tasks: dict[str, Task] = {}
        self._pending: list[Task] = []
        self.truth = SafetyTruth(site.index, site.amr.length, site.amr.width)
        self.faults: list[tuple[float, str, str]] = []  # (t, robot, kind)
        self.task_events: list[TaskEventMsg] = []
        self.alerts: list[AlertMsg] = []
        self.bus.observers.append(self._observe)
        ids = list(site.robots)[: robots or len(site.robots)]
        self.robots: dict[str, SimRobot] = {}
        L, W = site.amr.length, site.amr.width
        self._static_boxes = site.boxes
        for rid in ids:
            x, y, th = site.spawn_pose(rid)
            r = SimRobot(rid, x, y, th)
            r.agent = EdgeAgent(rid, graph, self.dist, self._publisher(rid), self.cfg,
                                site.robots[rid], site.battery.get(rid, 1.0), self.static_map)
            self.bus.subscribe(rid, r.agent.on_message)
            self.robots[rid] = r
        self._L, self._W = L, W

    # ----------------------------------------------------------------------- inputs

    def _publisher(self, rid: str):
        def pub(msg):
            if not self.robots.get(rid) or not self.robots[rid].failed:
                self.bus.publish(rid, msg)
        return pub

    def add_tasks(self, tasks: list[Task]) -> None:
        for t in tasks:
            self.tasks[t.task_id] = t
            self._pending.append(t)
        self._pending.sort(key=lambda t: t.release)

    def control(self, msg: ControlMsg) -> None:
        self.bus.inject(msg)

    def fail_robot(self, rid: str, at: float) -> None:
        self.faults.append((at, rid, "fail"))

    def _observe(self, msg) -> None:
        if isinstance(msg, TaskEventMsg):
            self.task_events.append(msg)
            t = self.tasks.get(msg.task_id)
            if t is not None:
                t.history.append((msg.stamp, msg.status, msg.robot_id))
                if msg.status == "in_progress":
                    t.status, t.robot, t.t_assigned = "in_progress", msg.robot_id, msg.stamp
                elif msg.status == "picked":
                    t.status, t.t_picked = "picked", msg.stamp
                elif msg.status == "completed":
                    t.status, t.t_done = "completed", msg.stamp
        elif isinstance(msg, AlertMsg):
            self.alerts.append(msg)
            if msg.kind == "risk":
                self.truth.avoided(self.t, msg.sender, msg.target, msg.x, msg.y, msg.detail,
                                   self._closing_speed(msg.sender, msg.target))

    def _closing_speed(self, rid: str, other: str) -> float:
        """Relative speed of the pair, so an AVOIDED event records what the brake was worth."""
        a = self.robots.get(rid)
        if a is None:
            return 0.0
        b = self.robots.get(other)
        if b is None:  # braked for an obstacle or a person: the robot's own speed is the impact speed
            return abs(a.v)
        return math.hypot(a.v * math.cos(a.th) - b.v * math.cos(b.th),
                          a.v * math.sin(a.th) - b.v * math.sin(b.th))

    # ----------------------------------------------------------------------- stepping

    def step(self) -> None:
        self.t += self.dt
        t = self.t
        while self._pending and self._pending[0].release <= t:
            task = self._pending.pop(0)
            self.bus.inject(task.msg(t))
        for at, rid, kind in list(self.faults):
            if at <= t:
                self.faults.remove((at, rid, kind))
                r = self.robots[rid]
                r.failed = True
                self.bus.unsubscribe(rid)
        for ob in self.obstacles:
            if ob.active(t):
                ob.step(self.dt)
        self.bus.positions = {rid: (r.x, r.y) for rid, r in self.robots.items()}
        self.bus.advance(t)
        scan = self.lidar and t >= self._next_scan
        if scan:
            self._next_scan = t + self.lidar_period
        boxes = self._world_boxes() if scan else None
        for rid, r in self.robots.items():
            if r.failed:
                r.v = r.w = 0.0
                continue
            pts = self._scan(r, boxes) if scan else None
            cmd = r.agent.tick(t, (r.x, r.y, r.th), r.v, r.w, pts, 0.0)
            if self.latency > 0.0:
                q = self._cmd_queue.setdefault(rid, [])
                q.append((t + self.latency, cmd.v, cmd.w))
                while len(q) > 1 and q[1][0] <= t + 1e-9:
                    q.pop(0)
                _due, cv, cw = q[0] if q[0][0] <= t + 1e-9 else (0.0, r.v, r.w)
                self._integrate(r, cv, cw)
            else:
                self._integrate(r, cmd.v, cmd.w)
        self._check_contacts()

    def run(self, duration: float, until_done: bool = False, callback: Callable | None = None) -> None:
        end = self.t + duration
        while self.t < end:
            self.step()
            if callback is not None:
                callback(self)
            if until_done and self.tasks and not self._pending and all(
                    t.status == "completed" for t in self.tasks.values()):
                break

    def _integrate(self, r: SimRobot, v_cmd: float, w_cmd: float) -> None:
        amr, dt = self.site.amr, self.dt
        v_cmd = max(-amr.v_hw, min(amr.v_hw, v_cmd))
        w_cmd = max(-amr.w_hw, min(amr.w_hw, w_cmd))
        dv = max(-amr.a_hw * dt, min(amr.a_hw * dt, v_cmd - r.v))
        dw = max(-amr.alpha_hw * dt, min(amr.alpha_hw * dt, w_cmd - r.w))
        r.v += dv
        r.w += dw
        r.x += r.v * math.cos(r.th + 0.5 * r.w * dt) * dt
        r.y += r.v * math.sin(r.th + 0.5 * r.w * dt) * dt
        r.th = wrap(r.th + r.w * dt)

    # ----------------------------------------------------------------------- lidar

    def _world_boxes(self) -> BoxSet:
        boxes = [nb.box for nb in self._static_boxes]
        names = [nb.name for nb in self._static_boxes]
        for rid, r in self.robots.items():
            boxes.append(r.box(self._L, self._W))
            names.append(rid)
        for ob in self.obstacles:
            if ob.active(self.t):
                boxes.append(ob.box())
                names.append(ob.name)
        return BoxSet(boxes, names)

    def _scan(self, r: SimRobot, boxes: BoxSet) -> np.ndarray:
        pts = []
        for m in self.site.amr.lidars:
            (ox, oy), ang, _rel, rng = scan(boxes, (r.x, r.y, r.th), m, self.beams, r.rid, 0.01, self.rng)
            ok = np.isfinite(rng)
            pts.append(np.column_stack((ox + rng[ok] * np.cos(ang[ok]), oy + rng[ok] * np.sin(ang[ok]))))
        return np.vstack(pts) if pts else np.zeros((0, 2))

    # ----------------------------------------------------------------------- safety truth

    def _check_contacts(self) -> None:
        bodies = [Body(rid, r.x, r.y, r.th, r.v,
                       r.agent.station or "" if r.agent and r.agent.mode in ("charging", "parked") else "")
                  for rid, r in self.robots.items()]
        obs = [(ob.name, ob.box()) for ob in self.obstacles if ob.active(self.t)]
        self.truth.check(self.t, bodies, obs)

    @property
    def events(self):
        return self.truth.events

    @property
    def min_gap(self):
        return self.truth.min_gap
