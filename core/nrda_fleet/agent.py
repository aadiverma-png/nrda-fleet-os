"""The edge agent: everything one AMR runs on board (NRDA-FA layers L1-L5).

One :class:`EdgeAgent` per robot. It talks to the rest of the fleet only
through peer messages (:mod:`nrda_fleet.bus`) and to its own hardware only
through ``tick()``: pose, speed and lidar points in, one velocity command out.

    L5  task + health     state machine, battery, faults, operator commands
    L4  allocation        CBBA over the peer bus                   (allocation.py)
    L3  route + reserve   lane-graph route -> space-time schedule  (scheduler.py, claims.py)
    L2  local coord.      blocked lane -> visison bypass decision  (bypass.py)
    L1  motion + safety   lidar tracking + collision risk, MotionArbiter
                          (tracker, ORCA-DD speed-only, limit gate, watchdog), safety supervisor

Command authority (docs/FINAL_ALGORITHM.md, section 5): the MotionArbiter
is the only thing that produces the command. Every layer submits intents with
a priority (OPERATOR > EMERGENCY > FLEET > PLANNER). The safety supervisor
filters the arbiter's output and can only lower it.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from visison.arbiter import Kind, MotionArbiter, MotionIntent, Source
from visison.emergency import EmergencyConfig, EmergencyMonitor, ObjectTracker, RiskLevel, TrackedObject
from visison.motion_models import Command
from visison.orca import OrcaAgent, OrcaConfig, OrcaMode
from visison.tracking import UnicycleTracker

from .allocation import Allocator, StationDistances
from .bus import AlertMsg, BidMsg, ClaimMsg, ControlMsg, RobotStateMsg, TaskEventMsg, TaskMsg
from .claims import ClaimBook, precedence_key
from .geom import OrientedBox, segment_hits_box, wrap
from .health import FaultState
from .lane_graph import LaneGraph, heading_name
from .perception import StaticMap, clusters, drop_self, to_body
from .safety import SafetySupervisor
from .ids import id_key
from .deadlock import DRPPlan, Member, ProgressWatch, RUNG_S, TOKEN_S, deadlock_group, make_plan, zone_boxes
from . import standoff as l2b
from .scheduler import Plan, Scheduler
from .trajectory import Leg
from .waitgraph import WaitForGraph

MOVING = ("to_pickup", "to_drop", "to_charger", "to_park")
# s after a DRP plan before the coordinator will believe an absent wait edge means the cycle is broken:
# the members have to act and gossip the result first, and a plan is judged on evidence, not on silence.
CYCLE_SETTLE = 3.0
# Peer identification for lidar tracks (_identify). A track is that peer if its centroid falls inside
# the peer's footprint grown by this margin; the margin covers pose age, not body size.
PEER_ID_MARGIN = 0.35
PEER_ID_MAX_AGE = 2.0  # s; beyond this a peer's gossiped pose is too old to identify anything with
SERVICE = {"to_pickup": "loading", "to_drop": "unloading", "to_charger": "charging", "to_park": "parked"}


@dataclass
class AgentConfig:
    control_dt: float = 0.05
    plan_period: float = 0.33
    bid_period: float = 0.5
    state_period: float = 0.15
    window: float = 8.0
    tau: float = 0.6
    battery_model: bool = False  # off: batteries stay at 100 % and nobody leaves work to charge (chargers are
    # docking spots only). On: drain / charge model below, low-battery robots go charging.
    battery_low: float = 0.30
    battery_resume: float = 0.92
    battery_min_task: float = 0.40
    drain_move: float = 0.00030  # per second while moving
    drain_idle: float = 0.00003
    charge_rate: float = 0.004
    reroute_wait: float = 12.0  # s of scheduled waiting on a stopped/failed robot before re-routing
    bypass_trigger: float = 2.5  # s of waiting on a stopped robot in the same lane before L2 is asked
    # deliberately below deadlock_persist (4.0): L2's local bypass gets a head start on the DRP, which is
    # far more expensive. Equal values had the two layers racing for the same robots.
    patience: float = 15.0  # s of waiting on bookings before the robot drives on peer bodies + lidar only
    # Measured 2026-10-18: at 8.0 the impatient window (8 s, ignores the bookings of *standing* peers) ran
    # half the time a robot spent waiting, and robots drove into occupied REST bays and wedged the row.
    latency: float = 0.0  # s from a command to its effect on the wheels + age of the pose / speed feedback
    # (Gazebo through ros_gz_bridge: ~0.1 s). Tracking and the safety layer act on the pose predicted this far
    # ahead, so a turn next to a rack is judged where the robot will really be when the command lands.
    peer_timeout: float = 3.0
    orca: bool = True
    safety: bool = True
    bypass: bool = True
    wfg: bool = True
    congestion_learning: bool = True
    arrive_tol: float = 0.15  # 0.20 left too much of the body in the lane at a narrow REST bay
    deadlock_persist: float = 4.0
    # s a robot keeps honouring a wait chain that ends at a parked, idle or silent robot before it gives
    # up and re-routes (_check_dead_chain). Well above deadlock_persist so a real cycle is resolved by the
    # DRP first, and above the time a station occupant normally needs to finish and pull out.
    chain_timeout: float = 20.0
    station_wip: int = 4  # robots allowed to work towards one station at a time (one served + three queued);
    # a robot that won a task for a busier station waits in its bay, off the lanes, until a slot frees up


@dataclass
class AgentStats:
    distance: float = 0.0
    safety_brakes: int = 0
    risk_brakes: int = 0
    waits: int = 0
    wait_time: float = 0.0
    replans: int = 0
    reroutes: int = 0
    bypasses: int = 0
    deadlocks: int = 0
    dead_chains: int = 0  # unsatisfiable wait chains this robot gave up on (_check_dead_chain)
    standoffs: int = 0  # in-aisle chain reverses this robot asked for (L2b)
    standoff_yields: int = 0  # times it backed off for someone else's chain reverse
    tasks_done: int = 0
    energy: float = 0.0
    plan_ms: deque = field(default_factory=lambda: deque(maxlen=400))
    tick_ms: deque = field(default_factory=lambda: deque(maxlen=400))
    messages: int = 0


class EdgeAgent:
    def __init__(self, robot_id: str, graph: LaneGraph, dist: StationDistances, publish, cfg: AgentConfig,
                 start_station: str, battery: float = 1.0, static_map: StaticMap | None = None):
        self.rid, self.g, self.cfg = robot_id, graph, cfg
        self.site, self.amr = graph.site, graph.site.amr
        self._publish = publish
        self.book = ClaimBook(robot_id, priority=self._priority, peer_timeout=cfg.peer_timeout)
        self.book.waiting_on_me = lambda rid: rid in self.peers and self.rid in self.peers[rid].waiting_for
        self.book.my_blockers = lambda: self.waiting_for
        self.book.follow_ok = self._follow_ok
        self.book.core_cells = self.g.core_cells()
        self.book.core_map = self.g.core_cell_map()
        self.book.peer_moving = lambda rid: (rid in self.peers and abs(self.peers[rid].v) > 0.05
                                             and self.t - self.peer_heard.get(rid, -1e9) <= 1.0)
        self.sched = Scheduler(graph, self.book, window=cfg.window, tau=cfg.tau)
        self.alloc = Allocator(robot_id, dist)
        self.dist = dist
        model = self.amr.motion_model()
        self.arbiter = MotionArbiter(
            robot_id, model, cfg.control_dt,
            tracker=UnicycleTracker(v_max=self.amr.v_hw * 0.9, w_max=self.amr.w_hw, k_x=1.5, k_y=4.0, k_th=2.5,
                                    k_0=1.2, v_reverse=self.amr.v_reverse + 0.05),
            neighbours=self._orca_inputs, orca_cfg=OrcaConfig(tau=2.0, neighbour_range=6.0, headway=0.6),
            orca_enabled=cfg.orca, odom_timeout=0.5)
        self.safety = SafetySupervisor(self.amr, enabled=cfg.safety)
        self.safety.reaction += cfg.latency
        self.safety.scan_age += cfg.latency
        self.static_map = static_map
        self.tracker = ObjectTracker(prefix=f"{robot_id}.T")
        self.monitor = EmergencyMonitor(robot_id, self.amr.e_decel,
                                        EmergencyConfig(sigma_pos=0.08, sigma_vel=0.15, min_gap=0.15,
                                                        compute_latency=0.1))
        # state
        st = graph.stations[start_station].station
        self.pose = st.pose
        self.v = self.w = 0.0
        self.t = 0.0
        self.station: str | None = start_station
        self.mode = "charging" if st.kind == "charger" else "parked"
        self.battery = battery
        self.health = FaultState.HEALTHY
        self.legs: list[Leg] = []
        self._leg_start: tuple[float, float] = (st.x, st.y)
        self.goal: str | None = None
        self.job: dict | None = None
        self.dwell_until = 0.0
        self._impatient_until = 0.0
        self._backed_off = False
        self._collision_pause: float | None = None
        # Deadlock Resolution Protocol (deadlock.py)
        self._drp_watch = ProgressWatch()
        self._chain_since: float | None = None  # since when my wait chain has looked unsatisfiable
        self._drp_plans: dict[str, DRPPlan] = {}  # epoch -> plan (active frozen zones)
        self._drp_mine: DRPPlan | None = None  # plan I coordinate
        self.standoff = l2b.StandoffState()  # L2b in-aisle chain reverse (standoff.py)
        self._drp_rung: dict[frozenset, int] = {}
        self._drp_detect: dict[frozenset, float] = {}
        self._drp_cooldown: dict[frozenset, float] = {}
        self._drp_escalated: dict[frozenset, tuple] = {}  # group -> its pose signature when we gave up
        self._drp_hold_until = 0.0
        self._drp_prio_start: tuple[float, float] | None = None
        self._drp_prio_rid = ""
        self._station_blocker = ""  # rid of the robot whose station cut our route short ("" if none)
        self.plan: Plan | None = None
        self.peers: dict[str, RobotStateMsg] = {}
        self.peer_heard: dict[str, float] = {}
        self.estop = False
        self.paused = False
        self.forbid: dict[tuple[str, str], float] = {}  # lane edges avoided after a block -> expiry time
        self.zone_rules: dict[str, str] = {}
        self.points_body: np.ndarray | None = None
        self.tracks: list[TrackedObject] = []
        self.risk_active = False
        self.waiting_for: list[str] = []
        self._wait_since: float | None = None
        self._cycle_since: dict[tuple, float] = {}
        self._next = {"plan": 0.0, "bid": 0.0, "state": 0.0, "claims": 0.0}
        self._claim_sig = None
        self._last_t: float | None = None
        self.edge_extra: dict[tuple[str, str], float] = {}  # learned congestion (s) per lane edge
        self._edge_enter: tuple[tuple[str, str], float] | None = None
        self.stats = AgentStats()
        self.events: deque = deque(maxlen=200)
        self.bypass = None  # L2 controller (bypass.py) while a local manoeuvre runs
        self.known_obstacles: dict[str, tuple[float, float, float, float]] = {}  # unmapped blockers (x, y, L, W)
        # pick-up / drop-off of every open task, for the per-station work-in-progress cap. Entries are dropped
        # when the task ends, so a fleet running for days does not accumulate one per task ever released.
        self._task_stations: dict[str, tuple[str, str]] = {}
        self._replan_empty = -1e9  # last time the route was re-planned after running out short of the goal
        self._obstacle_seen: dict[str, float] = {}
        self._obs_candidates: dict[str, int] = {}
        self.task_log: list[tuple[float, str, str]] = []

    # ============================================================ messaging

    def publish(self, msg) -> None:
        self.stats.messages += 1
        self._publish(msg)

    def on_message(self, msg) -> None:
        t = self.t
        if isinstance(msg, RobotStateMsg):
            if msg.robot_id != self.rid:
                self.peers[msg.robot_id] = msg
                self.peer_heard[msg.robot_id] = t
        elif isinstance(msg, ClaimMsg):
            self.book.on_msg(msg, t)
            self.peer_heard[msg.robot_id] = t
        elif isinstance(msg, BidMsg):
            self.alloc.on_bid(msg, t)
        elif isinstance(msg, TaskMsg):
            if msg.cancelled:
                self.alloc.cancel(msg.task_id)
                if self.job and self.job["task_id"] == msg.task_id and self.mode == "to_pickup":
                    self._log(f"task {msg.task_id} cancelled by operator")
                    self.job = None
                    self.mode = "idle"
            else:
                self.alloc.add_task(msg.task_id, msg.pickup, msg.dropoff, msg.priority, msg.deadline, msg.assign_to)
                self._task_stations[msg.task_id] = (msg.pickup, msg.dropoff)
        elif isinstance(msg, TaskEventMsg):
            if msg.robot_id != self.rid and msg.status in ("in_progress", "picked", "completed"):
                self.alloc.finish(msg.task_id)
            if msg.status in ("completed", "cancelled", "failed"):
                self._task_stations.pop(msg.task_id, None)  # the station cap no longer needs it
        elif isinstance(msg, ControlMsg):
            self._on_control(msg)
        elif isinstance(msg, AlertMsg):
            self._on_alert(msg)

    def _on_control(self, msg: ControlMsg) -> None:
        if msg.target and msg.target != self.rid:
            if msg.command == "zone_rule":
                pass
            else:
                return
        c = msg.command
        if c == "estop":
            self.estop = True
            self.arbiter.request(MotionIntent(Source.OPERATOR, Kind.BRAKE, None, stamp=self.t), self.t)
            self._log("E-STOP")
        elif c == "release_estop":
            self.estop = False
            self.arbiter.release(Source.OPERATOR, self.t)
            self._log("e-stop released")
        elif c == "pause":
            self.paused = True
            self.arbiter.request(MotionIntent(Source.OPERATOR, Kind.BRAKE, None, stamp=self.t), self.t)
            if (msg.value or "").startswith("collision"):
                self._collision_pause = self.t  # the safety monitor stopped us: recover by ourselves
                self._log(f"paused after {msg.value}; recovering once the lidar shows a clear side")
            else:
                self._collision_pause = None  # an operator pause is only ever released by the operator
                self._log("paused by operator")
        elif c == "resume":
            self.paused = False
            self._collision_pause = None
            if not self.estop:
                self.arbiter.release(Source.OPERATOR, self.t)
            self._log("resumed by operator")
        elif c == "charge" and self.mode in ("parked", "idle"):
            self._go_charge()
        elif c == "fault":
            self.health = FaultState.from_str(msg.value or "MOTION_FAILURE")
            self._log(f"fault injected: {self.health.value}")
        elif c == "clear_fault":
            self.health = FaultState.HEALTHY
            self._log("fault cleared")
        elif c == "zone_rule":
            zone, _, rule = msg.value.partition("=")
            if rule in ("", "clear"):
                self.zone_rules.pop(zone, None)
            else:
                self.zone_rules[zone] = rule

    def _on_alert(self, msg: AlertMsg) -> None:
        if msg.kind == "obstacle":
            L, _, W = msg.detail.partition("x")
            self.known_obstacles[msg.target] = (msg.x, msg.y, float(L or 1.0), float(W or 1.0))
            self._obstacle_seen[msg.target] = self.t
            return
        if msg.kind == "obstacle_cleared":
            self.known_obstacles.pop(msg.target, None)
            return
        if msg.kind == "drp_plan":
            self._drp_on_plan(DRPPlan.from_json(msg.detail))
            return
        if msg.kind == "drp_done":
            self._drp_on_done(msg.target)
            return
        if msg.kind == "standoff_reverse":
            l2b.on_plan(self, l2b.StandoffPlan.from_json(msg.detail), self.t)
            return
        if msg.target and msg.target != self.rid:
            return
        if msg.kind == "wfg_yield" and msg.target == self.rid:
            self._reroute("deadlock yield requested")

    # ============================================================ control tick (20 Hz)

    def tick(self, t: float, pose: tuple[float, float, float], v: float, w: float,
             points_world: np.ndarray | None = None, odom_age: float = 0.0) -> Command:
        c0 = time.perf_counter()
        dt = 0.0 if self._last_t is None else max(t - self._last_t, 0.0)
        self._last_t = t
        if self.pose is not None:
            self.stats.distance += math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1])
        self.t, self.pose, self.v, self.w = t, pose, v, w
        self.book.prune(t)
        self.book.occupied = self.g.cover(pose[0], pose[1], pose[2], 0.02, 0.02)
        rr = self.book.race_radius
        self.book.contended = any(
            t - self.peer_heard.get(rid, -1e9) <= self.cfg.peer_timeout and abs(p.x - pose[0]) < rr
            and abs(p.y - pose[1]) < rr and math.hypot(p.x - pose[0], p.y - pose[1]) < rr
            for rid, p in self.peers.items())
        self._battery(dt)
        if points_world is not None:
            self._perceive(t, points_world)
        if self.paused and self._collision_pause is not None and t - self._collision_pause >= 3.0:
            self._collision_recover(t)
        if t >= self._next["plan"]:
            # edge compute budget: 4 Hz while driving, 1 Hz while parked / charging / serving
            idle = self._stationary() and self.bypass is None and not self.waiting_for
            self._next["plan"] = t + (1.0 if idle else self.cfg.plan_period)
            self._l5(t)
            self._l3(t)
            if self.cfg.wfg:
                self._drp_tick(t)
        if t >= self._next["bid"]:
            self._next["bid"] = t + self.cfg.bid_period
            self._l4(t)
        if t >= self._next["state"]:
            self._next["state"] = t + self.cfg.state_period
            self.publish(self.state_msg(t))
        lat = self.cfg.latency
        if lat > 0.0:
            # latency compensation: track (and later check safety) from where the robot will be when this
            # command takes effect, against the reference at that same time
            thm = pose[2] + 0.5 * w * lat
            pose_c = (pose[0] + v * math.cos(thm) * lat, pose[1] + v * math.sin(thm) * lat, wrap(pose[2] + w * lat))
            cmd = self.arbiter.step(t + lat, pose_c, Command(v, w), odom_age)
        else:
            cmd = self.arbiter.step(t, pose, Command(v, w), odom_age)
        if self.health.can_move is False:
            cmd = self.arbiter.model.brake_command(Command(v, w), self.cfg.control_dt)
        docking = False
        active = self.arbiter.trajectory(t)
        if active is not None and hasattr(active, "primitive_at"):
            docking = active.primitive_at(t).tag in ("dock", "undock")
        pts_b = self.points_body
        if lat > 0.0 and pts_b is not None and len(pts_b):
            # lidar returns seen from the predicted body pose (moved v*lat ahead, turned w*lat)
            dth = w * lat
            c_, s_ = math.cos(-dth), math.sin(-dth)
            q = pts_b - np.array([v * lat, 0.0])
            pts_b = np.column_stack((c_ * q[:, 0] - s_ * q[:, 1], s_ * q[:, 0] + c_ * q[:, 1]))
        out = self.safety.filter(cmd, pts_b, docking)
        if out.brake:
            self.stats.safety_brakes += 1
            self.arbiter.request(MotionIntent(Source.EMERGENCY, Kind.BRAKE, None, stamp=t, ttl=0.3), t)
            self._log(f"safety: {out.reason}")
        self.stats.tick_ms.append((time.perf_counter() - c0) * 1e3)
        return out.cmd

    # ============================================================ L5 task + health

    def _battery(self, dt: float) -> None:
        if not self.cfg.battery_model:
            self.battery = 1.0
            return
        if self.mode == "charging":
            self.battery = min(1.0, self.battery + self.cfg.charge_rate * dt)
        else:
            moving = abs(self.v) > 0.05 or abs(self.w) > 0.05
            d = (self.cfg.drain_move if moving else self.cfg.drain_idle) * dt
            self.battery = max(0.0, self.battery - d)
            self.stats.energy += d

    def _l5(self, t: float) -> None:
        if not self.health.can_move:
            self.mode = "fault"
            return
        if self.mode == "fault" and self.health.can_move:
            self.mode = "idle"
        if self.estop or self.paused:
            return
        m = self.mode
        if m in MOVING:
            if not self.legs and self._near_goal():
                self._arrive(t)
                return
            if not self.legs and self._backed_off:
                self._backed_off = False  # back-off done: plan the route again from here
                self._go(self.goal, m)
                return
            if not self.legs and self.goal and self.bypass is None:
                # the route ran out short of the goal (a manoeuvre, a skipped leg or a stop just outside the
                # arrival tolerance). Nothing else notices: the stall watchdog only watches robots that still
                # have legs, so the robot stood at its last leg for the rest of the run. Plan again from here.
                if t - self._replan_empty > 5.0:
                    self._replan_empty = t
                    gap = self._goal_gap()
                    st = self.g.stations[self.goal].station
                    if gap <= 1.0 and not self.site.body_hits(st.x, st.y, self.pose[2], 0.02, (self.goal,)):
                        if st.approach == "lane":
                            self._arrive(t)
                            return
                        # a stop just outside the arrival tolerance: creep the last few centimetres. A full
                        # re-route from here would send the robot once around the lane loop instead.
                        self.legs = [Leg("move", st.x, st.y, v_max=self.amr.v_station, tag="close_gap")]
                        self._leg_start = (self.pose[0], self.pose[1])
                    else:
                        self._log(f"route ended {gap:.2f} m short of {self.goal}: planning again from here")
                        self._go(self.goal, m)
            if m in ("to_park", "to_charger") and self.goal:
                peer_at_goal = any(p.station == self.goal for rid, p in self.peers.items()
                                   if t - self.peer_heard.get(rid, -1e9) <= self.cfg.peer_timeout * 3)
                blocked_by_parked = any(b in self.peers and (self.peers[b].station == self.goal or
                                                            self.peers[b].mode in ("parked", "charging", "parked_charger"))
                                        for b in self.waiting_for)
                if peer_at_goal or blocked_by_parked:
                    free = self._free_station("parking" if m == "to_park" else "charger") or \
                           self._free_station("charger" if m == "to_park" else "parking")
                    if free and free != self.goal:
                        self._log(f"destination station {self.goal} occupied; reassigning to {free}")
                        self._go(free, m)
                        return
            if m == "to_park" and self._at_safe_point() and self.battery >= self.cfg.battery_min_task:
                task_id = self.alloc.next_task(t, self._station_has_room)
                if task_id is not None:
                    self._start_task(t, task_id)
            return
        if m in ("loading", "unloading"):
            if t >= self.dwell_until:
                self._service_done(t)
            return
        if m == "charging":
            ready = self.alloc.next_task(t) is not None and self.battery >= self.cfg.battery_min_task + 0.15
            if self.battery >= self.cfg.battery_resume or ready:
                self.mode = "parked_charger"
            else:
                return
        # idle / parked / parked_charger: pick work
        if self.battery < self.cfg.battery_low:
            if self.mode not in ("to_charger", "charging"):
                if self.station and self.site.stations[self.station].kind == "charger":
                    self.mode = "charging"
                else:
                    self._go_charge()
            return
        task_id = self.alloc.next_task(t, self._station_has_room) \
            if self.battery >= self.cfg.battery_min_task else None
        if task_id is not None:
            self._start_task(t, task_id)
            return
        if self.mode == "idle":
            # standing at a work station or in a lane: free it
            st = self.site.stations.get(self.station) if self.station else None
            if st is None or st.kind not in ("charger", "parking"):
                if self.battery < 0.7:
                    self._go_charge()
                else:
                    self._go_park()
            else:
                self.mode = "charging" if st.kind == "charger" and self.battery < 0.99 else "parked"
        elif self.mode == "parked_charger":
            self.mode = "charging" if self.battery < 0.99 and self.alloc.next_task(t) is None else "parked_charger"

    def _station_has_room(self, task_id: str) -> bool:
        """Work-in-progress cap per station (admission control): start a task only while fewer than
        ``station_wip`` robots are already working towards its pick-up or its drop-off.

        Every station is a dead-end pocket that serves one robot at a time. Robots sent there faster than it
        serves them queue in the lanes, the queues grow back through the crossings and lock the block (gridlock).
        Holding the surplus robots in their bays keeps the queue at one robot per station; the task stays ours.
        """
        cap = self.cfg.station_wip
        task = self.alloc.task(task_id)
        if cap <= 0 or task is None:
            return True
        n_pick = n_drop = 0
        for rid, p in self.peers.items():
            if not p.task_id or self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout:
                continue
            other = self._task_stations.get(p.task_id)  # a peer's running task has left the auction
            if other is None:
                continue
            if p.mode in ("to_pickup", "loading") and other[0] == task["pickup_station"]:
                n_pick += 1
            if p.mode in ("to_pickup", "loading", "to_drop", "unloading") and other[1] == task["dropoff_station"]:
                n_drop += 1
        ok = n_pick < cap and n_drop < cap
        if not ok and self.t - self.__dict__.get("_wip_logged", -1e9) > 30.0:
            self._wip_logged = self.t
            self._log(f"task {task_id} waits in the bay: {task['dropoff_station'] if n_drop >= cap else task['pickup_station']} "
                      f"already has {max(n_pick, n_drop)} robots on the way")
        return ok

    def _start_task(self, t: float, task_id: str) -> None:
        task = self.alloc.task(task_id)
        self.alloc.lock(task_id)
        self.job = dict(task)
        self.job["t_start"] = t
        self.publish(TaskEventMsg(task_id, self.rid, t, "in_progress", f"{task['pickup_station']}"))
        self.task_log.append((t, task_id, "in_progress"))
        self._log(f"task {task_id}: {task['pickup_station']} -> {task['dropoff_station']}")
        self._go(task["pickup_station"], "to_pickup")

    def _goal_gap(self) -> float:
        if self.goal is None:
            return 0.0
        st = self.g.stations[self.goal].station
        return math.hypot(self.pose[0] - st.x, self.pose[1] - st.y)

    def _near_goal(self) -> bool:
        if self.goal is None:
            return True
        st = self.g.stations[self.goal].station
        tol = self.cfg.arrive_tol + (0.55 if st.approach == "lane" else 0.1)
        return self._goal_gap() < tol

    def _arrive(self, t: float) -> None:
        self.forbid.clear()
        st = self.g.stations[self.goal].station
        self.station = self.goal
        mode = SERVICE[self.mode]
        self._log(f"arrived at {self.goal}")
        if mode in ("loading", "unloading"):
            self.dwell_until = t + st.dwell
        if mode == "loading" and self.job:
            pass
        self.mode = mode
        self.goal = None

    def _service_done(self, t: float) -> None:
        if self.mode == "loading" and self.job:
            self.publish(TaskEventMsg(self.job["task_id"], self.rid, t, "picked", self.station or ""))
            self.task_log.append((t, self.job["task_id"], "picked"))
            self._go(self.job["dropoff_station"], "to_drop")
        elif self.mode == "unloading" and self.job:
            tid = self.job["task_id"]
            self.publish(TaskEventMsg(tid, self.rid, t, "completed", self.station or ""))
            self.task_log.append((t, tid, "completed"))
            self.alloc.finish(tid)
            self.stats.tasks_done += 1
            self._log(f"task {tid} completed")
            self.job = None
            self.mode = "idle"
        else:
            self.mode = "idle"

    def _free_station(self, kind: str) -> str | None:
        """Nearest station of ``kind`` nobody is heading to, booked or docked at."""
        taken = set()
        for rid, p in self.peers.items():
            if self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout * 3:
                continue
            if p.goal:
                taken.add(p.goal)
            if p.station:
                taken.add(p.station)
        best = None
        anchor = self.station or self._anchor_station()
        for name, node in self.g.stations.items():
            if node.station.kind != kind or name in taken:
                continue
            if self.book.blocking((f"S:{name}", 0), self.t, ignore_losers=False):
                continue
            if node.maneuver and any(dep in taken for dep in node.maneuver.depends):
                continue
            d = self.dist.between(anchor, name) if anchor else 0.0
            if best is None or d < best[0]:
                best = (d, name)
        return best[1] if best else None

    def _go_charge(self) -> None:
        name = self._free_station("charger")
        if name:
            self._go(name, "to_charger")
        else:
            self._go_park()

    def _go_park(self) -> None:
        name = self._free_station("parking") or self._free_station("charger")
        if name:
            self._go(name, "to_park")
        else:
            self.mode = "idle"

    def _anchor_station(self) -> str | None:
        if self.station:
            return self.station
        if self.goal:
            return self.goal
        # nearest station
        best = min(self.g.stations.values(),
                   key=lambda n: math.hypot(n.station.x - self.pose[0], n.station.y - self.pose[1]))
        return best.station.name

    # ============================================================ routing

    def _cost_fn(self):
        extra = self.edge_extra if self.cfg.congestion_learning else {}
        zones = self.zone_rules

        def fn(e):
            c = extra.get((e.src[0], e.dst[0]), 0.0)
            if zones and e.kind == "move":
                for zone, rule in zones.items():
                    b = self.site.zones.get(zone)
                    if not b:
                        continue
                    p = self.g.points[e.dst[0]]
                    if b.get("x_min", 1e9) <= p.x <= b.get("x_max", -1e9) and b.get("y_min", 1e9) <= p.y <= b.get("y_max", -1e9):
                        c += 60.0 if rule == "avoid" else 8.0
            return c
        return fn

    def _start_state(self) -> tuple[list[Leg], tuple[str, str] | None]:
        """Legs to reach the first graph state of a new route, and that state."""
        if self.station and not self.legs:
            node = self.g.stations[self.station]
            if node.maneuver:
                return list(node.maneuver.exit), (node.maneuver.root, node.maneuver.exit_heading)
            return [], (node.root, node.heading)
        # mid-route: keep the legs up to the next graph point
        kept: list[Leg] = []
        for lg in self.legs:
            kept.append(lg)
            if lg.kind == "move" and lg.tag in self.g.points:
                th = math.atan2(lg.y1 - self._leg_start[1], lg.x1 - self._leg_start[0])
                last = kept[-2] if len(kept) >= 2 and kept[-2].kind == "move" else None
                if last is not None:
                    th = math.atan2(lg.y1 - last.y1, lg.x1 - last.x1)
                return kept, (lg.tag, heading_name(th))
        node = self.g.nearest_node(*self.pose)
        if node is None:
            return [], None
        p = self.g.points[node[0]]
        return [Leg("move", p.x, p.y, tag=node[0])], node

    def _go(self, station: str, mode: str) -> bool:
        node = self.g.stations[station]
        pre, start = self._start_state()
        if start is None:
            self._log(f"no graph state near the robot; cannot go to {station}")
            return False
        goal_heading = node.maneuver.root_heading if node.maneuver else node.heading
        self.forbid = {e: until for e, until in self.forbid.items() if until > self.t}
        path = self.g.route(start, node.root, goal_heading, self._cost_fn(), set(self.forbid) or None)
        if path is None:
            path = self.g.route(start, node.root, goal_heading, self._cost_fn())
        if path is None:
            # the state kept from the old legs can be unusable (a heading taken from a zero-length leg):
            # start again from the next lane point ahead of the robot
            near = self.g.nearest_node(*self.pose)
            if near is not None and near != start and not self.station:
                p_ = self.g.points[near[0]]
                alt = self.g.route(near, node.root, goal_heading, self._cost_fn())
                if alt is not None:
                    pre, start, path = [Leg("move", p_.x, p_.y, tag=near[0])], near, alt
        if path is None:
            self._log(f"no route to {station}")
            return False
        legs = pre + self.g.legs_for(path) + (list(node.maneuver.entry) if node.maneuver else [])
        self.legs = legs
        self.current_path = path
        if self.station:
            st = self.site.stations[self.station]
            self._leg_start = (st.x, st.y)
        elif self.pose is not None:
            # a new route starts where the robot is (a re-plan after a back-off, a collision recovery or a reroute
            # left the old leg start behind, and the turn-at-planned-point rule would drive the robot back to it)
            self._leg_start = (self.pose[0], self.pose[1])
        self.station = None
        self.goal = station
        self.mode = mode
        self._wait_since = None
        return True

    def _reroute(self, why: str) -> None:
        if self.goal is None or self.mode not in MOVING:
            return
        self.stats.reroutes += 1
        self._log(f"reroute: {why}")
        self._go(self.goal, self.mode)

    # ============================================================ L3 schedule

    def _skip_passed_legs(self) -> None:
        """After a local manoeuvre: drop route legs whose targets the robot has already driven past."""
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        while self.legs and self.legs[0].kind == "move" and (self.legs[0].x1 - x) * c + (self.legs[0].y1 - y) * s < 0.1:
            lg = self.legs.pop(0)
            while self.legs and self.legs[0].kind == "rotate" and abs(wrap(self.legs[0].th1 - th)) < math.radians(10):
                self.legs.pop(0)
            self._leg_start = (lg.x1, lg.y1)
        self._leg_start = (x, y) if not self.legs else self._leg_start
        self._wait_since = None

    def _follow_ok(self, rid: str, res) -> bool:
        """A moving peer driving along this one-way lane in its direction: follow it, do not wait for its claims."""
        key = res[0]
        if "/" not in key or res in self.g.crossing_cells():
            return False
        p = self.peers.get(rid)
        if p is None or abs(p.v) < 0.15 or self.t - self.peer_heard.get(rid, -1e9) > 1.0 or p.bypass:
            return False
        line, lane = key.split("/")
        ln = self.g.lines.get(line)
        if ln is None:
            return False
        h = ln.lane_heading(lane)
        if abs(wrap(p.th - h)) >= 0.35:
            return False
        # only a peer AHEAD of us can be followed: a robot merging into the lane (undocking, turning in) in
        # front of an approaching peer must respect that peer's claims, not assume it will stay ahead
        return (p.x - self.pose[0]) * math.cos(h) + (p.y - self.pose[1]) * math.sin(h) > 0.5 * self.amr.length

    def _sync_static_blocks(self) -> None:
        for ep in [ep for ep, p in self._drp_plans.items() if p.expires <= self.t]:
            self._drp_plans.pop(ep, None)
        frozen = tuple(sorted(ep for ep, p in self._drp_plans.items() if self.rid not in p.members))
        sig = (tuple(sorted(self.known_obstacles.items())), frozen)
        if sig == getattr(self, "_blocks_sig", None):
            return
        self._blocks_sig = sig
        blocks = {}
        for name, (ox, oy, L, W) in sorted(self.known_obstacles.items()):
            for res in sorted(self.g.cover_box(OrientedBox(ox, oy, L + 0.1, W + 0.1, 0.0))):
                blocks[res] = name
        # DRP frozen zones: robots that are not part of the resolution keep out until it is done
        for ep in frozen:
            for b in zone_boxes(self._drp_plans[ep]):
                for res in sorted(self.g.cover_box(b)):
                    blocks.setdefault(res, f"drp:{ep}")
        self.book.static_blocks = blocks

    def _progress(self) -> None:
        x, y, th = self.pose
        while self.legs:
            lg = self.legs[0]
            if lg.kind == "rotate":
                if abs(wrap(th - lg.th1)) < math.radians(4):
                    self.legs.pop(0)
                    continue
                # turn only where the route planned it (the spot the lane graph checked for clearance): after
                # an early stop or an overshoot, first drive back onto the turning point
                px, py = self._leg_start
                d = math.hypot(px - x, py - y)
                if 0.15 < d < 1.5:
                    ahead = (px - x) * math.cos(th) + (py - y) * math.sin(th)
                    self.legs.insert(0, Leg("move", px, py, reverse=ahead < 0, v_max=0.3, tag="to_turn_point"))
                    self._leg_start = (x, y)
                break
            if lg.kind == "wait":
                self.legs.pop(0)
                continue
            if lg.kind == "shift":
                ex, ey = math.cos(lg.th1), math.sin(lg.th1)
            else:
                dx, dy = lg.x1 - self._leg_start[0], lg.y1 - self._leg_start[1]
                n = math.hypot(dx, dy)
                if n < 1e-6:
                    self.legs.pop(0)
                    continue
                ex, ey = dx / n, dy / n
            rem = (lg.x1 - x) * ex + (lg.y1 - y) * ey
            if rem < 0.06:
                self._on_leg_done(lg)
                self.legs.pop(0)
                self._leg_start = (lg.x1, lg.y1)
                continue
            break

    def _on_leg_done(self, lg: Leg) -> None:
        """Congestion learning: compare the time a lane edge really took with its nominal cost."""
        if not self.cfg.congestion_learning or lg.kind != "move" or lg.tag not in self.g.points:
            return
        if self._edge_enter is not None:
            (src, t0) = self._edge_enter
            key = (src, lg.tag)
            nominal = math.dist((self.g.points[src].x, self.g.points[src].y),
                                (self.g.points[lg.tag].x, self.g.points[lg.tag].y)) / self.amr.v_cruise
            extra = max(self.t - t0 - nominal - 1.5, 0.0)
            old = self.edge_extra.get(key, 0.0)
            self.edge_extra[key] = 0.7 * old + 0.3 * extra
        self._edge_enter = (lg.tag, self.t)

    def _stationary(self) -> bool:
        return self.estop or self.paused or self.mode in ("loading", "unloading", "charging", "parked",
                                                           "parked_charger", "idle", "fault")

    def _l3(self, t: float) -> None:
        if self.bypass is not None:
            if self.bypass.active() and not (self.estop or self.paused):
                self.bypass.tick(t)
                self._hold_body_claims(t)
                self._bypass_waits(t)
                self.publish(self.book.snapshot(t))
                if self.bypass.active():
                    return
            else:
                # dropped without a tick (e-stopped, paused, or the session expired between ticks), so
                # BypassSession.tick never ran its own clean-up: the borrowed cells would stay booked.
                self.book.extra.clear()
            self.bypass = None
            self._wait_since = None
        self._sync_static_blocks()
        self._progress()
        x, y, th = self.pose
        v = self.v if abs(self.v) > 0.02 else 0.0
        if self._stationary() or not self.legs or t < self._drp_hold_until:
            hold = self.cfg.window + (max(self.dwell_until - t, 0.0) if self.mode in ("loading", "unloading") else 0.0)
            if self.mode in ("charging", "parked", "parked_charger"):
                hold = 3600.0
            docked = self.mode in ("charging", "parked", "parked_charger", "loading", "unloading") and bool(self.station)
            self._station_blocker = ""  # nothing is being scheduled, so no station cut our route
            plan = self.sched.plan(t, x, y, th, max(v, 0.0), [], hold_final=hold, docked=docked)
        else:
            goal_hold = 0.0
            if self.goal:
                st = self.g.stations[self.goal].station
                goal_hold = st.dwell + 2.0
            plan = self.sched.plan(t, x, y, th, self._speed_along_legs(v), self._legs_to_plan(),
                                   hold_final=goal_hold, w=self.w)
        self.plan = plan
        self.stats.replans += 1
        self.stats.plan_ms.append(plan.compute_ms)
        claims = list(plan.claims)
        if self.goal and self.site.stations[self.goal].kind in ("charger", "parking"):
            claims.append(((f"S:{self.goal}", 0), t, t + 3600.0))  # booking
        # stuck at the start (first wait begins now): everything after that wait is only tentative
        first = plan.waits[0] if plan.waits else None
        tentative = first.until if first is not None and first.t_start <= t + 0.5 else math.inf
        self.book.set_route_claims(claims, t, tentative)
        sig = self.book.signature()
        if sig != self._claim_sig or t >= self._next["claims"]:
            self._claim_sig = sig
            self._next["claims"] = t + 1.0  # keep-alive even when nothing changed
            self.publish(self.book.snapshot(t))
        if not (self.estop or self.paused):
            kind = Kind.HOLD if self._stationary() else Kind.FOLLOW
            self.arbiter.request(MotionIntent(Source.PLANNER, kind, plan.traj, stamp=t), t)
        self._handle_waits(t, plan)

    def _bypass_waits(self, t: float) -> None:
        """Keep the gossiped wait edge honest while an L2 manoeuvre owns the robot.

        ``_handle_waits`` is not reached during a manoeuvre, so whatever ``waiting_for`` held when the
        bypass began stays on the air until it ends. Observed in Gazebo: a robot frozen mid-pass
        advertised a wait on a peer that had since parked 56 m away, which turned a real stall into a
        wait-for *chain* ending at an idle robot - no cycle, so the DRP declined it by construction.

        While the manoeuvre is moving, the robot is not waiting for anyone. Once it has stopped, the
        only honest edge is the blocker the manoeuvre is actually about, and only while that is a peer
        we can still hear from.
        """
        if self.bypass is None:
            self.waiting_for = []
            return
        blocker = self.bypass.obstacle_id
        stopped = abs(self.v) < 0.05
        if stopped and blocker in self.peers and t - self.peer_heard.get(blocker, -1e9) <= self.cfg.peer_timeout:
            self.waiting_for = [blocker]
        else:
            self.waiting_for = []

    def _hold_body_claims(self, t: float) -> None:
        """Keep the cells the body really covers booked while an L2 manoeuvre owns the robot.

        L3 does not schedule during a manoeuvre, so ``set_route_claims`` is not called and the route
        claims age out: a robot halfway across the aisle ends up holding *no* claims at all. All that is
        left of it in the booking system is the "covered but unclaimed" momentary block in
        :meth:`~nrda_fleet.claims.ClaimBook.blocking`, which a peer that has run out of patience drops
        after a second. Book the footprint instead, refreshed every planning tick, so a robot that is
        standing between the lanes is a real reservation to its peers and keeps its precedence to resume.
        """
        self.book.set_route_claims([(res, t, t + self.cfg.window) for res in self.book.occupied], t)

    def _legs_to_plan(self) -> list:
        """The route legs to schedule now: the route, cut at the last legal waiting spot before a busy doorway.

        * the goal station is occupied: a robot waiting at its root blocks the occupant's only way out;
        * the route crosses the exit path of another station whose robot is being served or is leaving
          (the packing stations share one approach): driving in there locks the two robots face to face.
        """
        if not self.legs or not self.goal:
            self._station_blocker = ""
            return self.legs
        gates = []  # (leg index, station, rid of the robot in it)
        node = self.g.stations.get(self.goal)
        k_goal = None
        if node is not None and node.maneuver is not None:
            k_goal = next((i for i, lg in enumerate(self.legs) if lg.tag == node.maneuver.root), None)
            occupant = self._station_taken_rid(self.goal) if k_goal is not None else ""
            if occupant:
                gates.append((k_goal, self.goal, occupant))
        x, y, th = self.pose
        here = self.g.cover(x, y, th, 0.15, 0.05)
        # a robot that is itself leaving a station never waits for its neighbours' exit paths: bays stand 1.8 m
        # apart, so robots undocking side by side each held for the other, with nothing in the wait list
        # (the west bays froze half-way out, 2026-09-30). Their lane claims order them as usual.
        leaving = self._in_station(self.rid) or any(lg.tag == "undock" for lg in self.legs[:3])
        for name, other in (() if leaving else self.g.stations.items()):
            if name == self.goal or other.maneuver is None or name == self.station:
                continue
            if abs(other.station.x - x) > 9.0 or abs(other.station.y - y) > 9.0:
                continue
            busy = self._station_active_rid(name)
            if not busy:
                continue
            need = self._exit_need(name)
            if here & need:
                continue  # already in its exit path: keep going and clear it
            k = self._first_leg_into(need)
            if k is not None:
                gates.append((k, name, busy))
        if not gates:
            self._station_blocker = ""
            return self.legs
        key = (tuple(gates), len(self.legs), self._leg_start)
        if self.__dict__.get("_queue_key") != key:
            cuts = [(self._queue_legs(k, self.g.stations[name].maneuver.root, name), rid) for k, name, rid in gates]
            self._queue_key, (self._queue_cut, self._queue_rid) = key, min(cuts, key=lambda c: len(c[0]))
        # the route stops short of a doorway: the scheduler reports no wait for it, so name the occupant here
        self._station_blocker = self._queue_rid if len(self._queue_cut) < len(self.legs) else ""
        return self._queue_cut

    def _station_active(self, name: str) -> bool:
        """A robot is in the station and will come out soon (being served, idle after service, or leaving)."""
        return bool(self._station_active_rid(name))

    def _station_active_rid(self, name: str) -> str:
        """Rid of that robot, "" if none: a cut route must be able to name who it is waiting for."""
        res = (f"S:{name}", 0)
        for rid, p in self.book.peers.items():
            if rid != self.rid and res in p.occupied and self.book.alive(rid):
                st = self.peers.get(rid)
                if st is not None and st.mode not in ("parked", "charging", "parked_charger", "fault", "paused", "estop"):
                    return rid
        return ""

    def _exit_need(self, name: str) -> frozenset:
        """Lane cells the robot in station ``name`` books on its way out (padded, with its turn on the root)."""
        cache = self.__dict__.setdefault("_exit_cells", {})
        need = cache.get(name)
        if need is None:
            m = self.g.stations[name].maneuver
            rp = self.g.points.get(m.root)
            cells = set()
            for sx, sy, sth in m.samples:
                cells |= self.g.cover(sx, sy, sth, 0.4, 0.05)
            if rp is not None:
                for k_ in range(24):
                    cells |= self.g.cover(rp.x, rp.y, k_ * math.pi / 12, 0.4, 0.05)
            need = cache[name] = frozenset(r for r in cells if "/" in r[0])
        return need

    def _first_leg_into(self, need: frozenset, reach: float = 10.0) -> int | None:
        """Index of the first route leg (within ``reach`` m) on which the body enters the cells ``need``."""
        px, py = self._leg_start
        gone = 0.0
        for i, lg in enumerate(self.legs):
            if lg.kind != "move":
                if lg.kind == "shift":
                    return None
                continue
            n = math.hypot(lg.x1 - px, lg.y1 - py)
            if n > 1e-6:
                ex, ey = (lg.x1 - px) / n, (lg.y1 - py) / n
                th = math.atan2(ey, ex)
                s0 = max((self.pose[0] - px) * ex + (self.pose[1] - py) * ey, 0.0) if i == 0 else 0.0
                s = s0
                while s <= n + 1e-6:
                    if self.g.cover(px + ex * s, py + ey * s, th, 0.15, 0.05) & need:
                        return i
                    s += 0.3
                gone += n - s0
                if gone > reach:
                    return None
            px, py = lg.x1, lg.y1
        return None

    def _queue_legs(self, k: int, root: str, station: str) -> list:
        """Legs up to the last legal waiting spot before the doorway at ``legs[k]``.

        The doorway (root) of a dead-end station is also its exit, and it often is a crossing corner: a robot
        waiting there (or anywhere inside that crossing, or in the turning circle the occupant needs to leave)
        blocks the occupant's only way out. Walk back along the route from the root to the first spot where
        the body is clear of every crossing core, every station keep-clear zone and that turning circle.
        """
        segs = []  # (x0, y0, x1, y1, index of the leg ending there)
        px, py = self._leg_start
        for i, lg in enumerate(self.legs[:k + 1]):
            if lg.kind == "shift":
                return self.legs[:k]
            if lg.kind == "move":
                segs.append((px, py, lg.x1, lg.y1, i))
                px, py = lg.x1, lg.y1
        rp = self.g.points.get(root)
        if rp is None or not segs:
            return self.legs[:k]
        half = self.amr.length / 2
        turn = math.hypot(self.amr.length, self.amr.width) / 2 + 0.1  # the occupant turns on the root
        need = self._exit_need(station)
        for x0, y0, x1, y1, i in reversed(segs):
            n = math.hypot(x1 - x0, y1 - y0)
            if n < 1e-6:
                continue
            ex, ey = (x1 - x0) / n, (y1 - y0) / n
            th = math.atan2(ey, ex)
            s = n
            # never back up: on the robot's current segment the spot must still lie ahead of it
            s_min = (self.pose[0] - x0) * ex + (self.pose[1] - y0) * ey + 0.05 if i == segs[0][4] else 0.0
            while s >= s_min:
                cx, cy = x0 + ex * s, y0 + ey * s
                if math.hypot(cx + ex * half - rp.x, cy + ey * half - rp.y) > turn \
                        and math.hypot(cx - rp.x, cy - rp.y) > half + turn \
                        and not self.g.in_intersection(cx, cy, th) \
                        and not (self.g.cover(cx, cy, th, 0.15, 0.05) & need):
                    if s >= n - 0.06:
                        return self.legs[:i + 1]
                    return self.legs[:i] + [Leg("move", cx, cy, tag="queue")]
                s -= 0.1
        return []  # no legal spot between here and the doorway: hold where the robot is

    def _station_taken(self, name: str) -> bool:
        return bool(self._station_taken_rid(name))

    def _station_taken_rid(self, name: str) -> str:
        """Rid of the robot booked into station ``name``, "" if it is free."""
        res = (f"S:{name}", 0)
        return next((rid for rid, p in self.book.peers.items()
                     if rid != self.rid and res in p.occupied and self.book.alive(rid)), "")

    def _in_station(self, rid: str) -> bool:
        if rid == self.rid:
            x, y = self.pose[0], self.pose[1]
            mode = self.mode
            station = self.station
            undocking = any(lg.tag in ("dock", "undock") for lg in self.legs[:3])
        else:
            p = self.peers.get(rid)
            if p is None:
                return False
            x, y = p.x, p.y
            mode = p.mode
            station = p.station
            undocking = False
        if not self.g._at_station(x, y, 0.8):
            return False
        if mode in ("loading", "unloading", "charging", "parked", "parked_charger") or undocking:
            return True
        st = self.site.stations.get(station)
        if st is not None and st.approach == "reverse":
            return True
        for s in self.site.stations.values():
            if abs(x - s.x) < 0.8 and abs(y - s.y) < 0.8:
                if s.approach == "reverse" and mode not in ("to_drop", "to_pickup"):
                    return True
        return False

    def _speed_along_legs(self, v: float) -> float:
        """Current speed along the direction of the first driving leg (0 if the robot moves the other way)."""
        for lg in self.legs:
            if lg.kind == "rotate":
                return 0.0
            if lg.kind in ("move", "shift"):
                return max(-v, 0.0) if lg.reverse else max(v, 0.0)
        return 0.0

    def _at_safe_point(self) -> bool:
        """Safe-point rule (FINAL_ALGORITHM 5.3): own lane, not docking, not in a crossing, no L2 manoeuvre."""
        if self.bypass is not None and self.bypass.active():
            return False
        if self.legs and self.legs[0].tag in ("dock", "undock"):
            return False
        x, y, th = self.pose
        return not self.g.in_intersection(x, y, th)

    def _handle_waits(self, t: float, plan: Plan) -> None:
        # patience: a robot stuck behind bookings for `patience` s drives on bodies + lidar for a while
        if self.book.impatient and t >= self._impatient_until:
            self.book.impatient = False
        blockers = list(plan.blocked_by)
        if self.cfg.bypass and self.bypass is None and t >= getattr(self, "_l2_retry", 0.0) \
                and any(b in self.known_obstacles for b in blockers):
            # L2 early: an unmapped obstacle ahead in our lane is planned around while there is room
            from .bypass import obstacle_ahead_l2
            ctl = obstacle_ahead_l2(self, t, plan)
            if ctl is not None:
                self.bypass = ctl
                self.stats.bypasses += 1
                self.waiting_for = []
                self._wait_since = None
                return
        is_blocked = (plan.waits and plan.waits[0].t_start <= t + 1.0) or (bool(plan.auth_blockers) and plan.u_authority <= 0.05)
        self.waiting_for = blockers if abs(self.v) < 0.05 and is_blocked else []
        if not self.waiting_for and t < self._drp_hold_until and getattr(self, "_drp_prio_rid", ""):
            self.waiting_for = [self._drp_prio_rid]
        # last resort: the route was cut short of an occupied station, so no wait names the occupant. Without
        # this the robot stands still with an empty wait list and is invisible to the fleet's wait-for graph.
        if not self.waiting_for and abs(self.v) < 0.05 and self.mode in MOVING and self._station_blocker:
            self.waiting_for = [self._station_blocker]
        if not self.waiting_for:
            self._wait_since = None
            return
        if self._wait_since is None:
            self._wait_since = t
            self.stats.waits += 1
        waited = t - self._wait_since
        self.stats.wait_time += self.cfg.plan_period
        if waited >= self.cfg.patience and not self.book.impatient and not any(b in self.known_obstacles for b in blockers):
            self.book.impatient = True
            self._impatient_until = t + 8.0
            self._log(f"patience: waited {waited:.0f} s for {', '.join(blockers)}; driving on bodies and lidar")
        # L2: a stopped robot or an unmapped obstacle in our lane -> ask the visison bypass planner
        obstacle = next((b for b in blockers if b in self.known_obstacles), None)
        if self.cfg.bypass and self.bypass is None and (obstacle or waited >= self.cfg.bypass_trigger) \
                and t >= getattr(self, "_l2_retry", 0.0):
            from .bypass import try_bypass
            if obstacle:
                ox, oy, L, W = self.known_obstacles[obstacle]
                ctl = try_bypass(self, t, OrientedBox(ox, oy, L, W, 0.0), obstacle)
            else:
                ctl = try_bypass(self, t)
            if ctl is not None:
                self.bypass = ctl
                self.stats.bypasses += 1
                # the robot is manoeuvring now, not queueing behind a peer. Leaving the old edge in place
                # gossips a wait on whichever robot happened to block the route when L2 was asked, and
                # nothing refreshes it until the manoeuvre ends (the early-L2 path above already does this).
                self.waiting_for = []
                self._wait_since = None
                return
            self._l2_retry = t + 2.0
        if obstacle and waited < 20.0:
            return  # L2 owns unmapped obstacles; re-route only if it keeps declining
        # stuck behind a failed / long-stopped robot -> reroute around its lane stretch
        stale = [b for b in blockers if not b.startswith("drp:") and t - self.peer_heard.get(b, -1e9) > self.cfg.peer_timeout
                 or (self.peers.get(b) and self.peers[b].health not in ("HEALTHY", "DEGRADED"))]
        long_wait = (plan.waits[0].until - t > self.cfg.reroute_wait) if plan.waits else False
        if (stale or long_wait) and waited > 3.0 and plan.waits:
            w = plan.waits[0]
            key, idx = w.res
            if "/" in key:
                self._forbid_resource(key, idx)
                self._reroute(f"waiting {w.until - t:.0f} s for {key}[{idx}] held by {','.join(w.blockers)}")


    def _forbid_resource(self, lane_key: str, idx: int) -> None:
        line, lane = lane_key.split("/")
        ln = self.g.lines.get(line)
        if ln is None:
            return
        a0 = ln.lo + idx * 0.5
        a1 = a0 + 0.5
        hn = heading_name(ln.lane_heading(lane))
        for (pid, h), edges in self.g.edges.items():
            if h != hn:
                continue
            for e in edges:
                if e.kind != "move" or e.line != line or e.lane != lane:
                    continue
                p, q = self.g.points[e.src[0]], self.g.points[e.dst[0]]
                lo, hi = sorted((ln.along(p.x, p.y), ln.along(q.x, q.y)))
                if lo - 1.0 <= a1 and a0 <= hi + 1.0:
                    self.forbid[(e.src[0], e.dst[0])] = self.t + 60.0
        res = (lane_key, idx)
        xing = self.g.core_cell_map().get(res)
        cell_cx, cell_cy = ln.lane_point(lane, a0 + 0.25)
        for xg in self.g.xings.values():
            if xg.v.name != line and xg.h.name != line:
                continue
            if xing == xg.name or math.hypot(xg.center[0] - cell_cx, xg.center[1] - cell_cy) <= 2.5:
                for (pid, h), edges in self.g.edges.items():
                    pt = self.g.points.get(pid)
                    if pt and pt.xing == xg.name:
                        for e in edges:
                            if e.kind == "rotate":
                                self.forbid[(e.src[0], e.dst[0])] = self.t + 60.0
                if getattr(self, "current_path", None):
                    for a, b in zip(self.current_path, self.current_path[1:]):
                        pa = self.g.points.get(a[0])
                        pb = self.g.points.get(b[0])
                        if (pa and pa.xing == xg.name) or (pb and pb.xing == xg.name):
                            self.forbid[(a[0], b[0])] = self.t + 60.0

    def _check_deadlock(self, t: float) -> None:
        """Cross-aisle deadlocks: wait-for graph from the gossiped waiting_for edges (:mod:`.waitgraph`)."""
        wfg = WaitForGraph()
        for b in self.waiting_for:
            wfg.add_wait(self.rid, b, (0, 0), 0, t)
        for rid, p in self.peers.items():
            for b in p.waiting_for:
                wfg.add_wait(rid, b, (0, 0), 0, t)
        for cyc in wfg.find_cycles():
            if self.rid not in cyc:
                continue
            key = tuple(cyc)
            since = self._cycle_since.setdefault(key, t)
            if t - since < self.cfg.deadlock_persist:
                continue
            # deadlock-scope rule: an in-aisle standoff is L2's, not ours
            if self.bypass is not None:
                continue
            # the lowest-precedence member yields (lowest priority, then highest AMR number)
            # the member that yields must be able to move out of the way: never one boxed in a station
            loser = max(cyc, key=lambda r: (not self._in_station(r), precedence_key(0.0, self._peer_priority(r), r)))
            if loser == self.rid:
                self.stats.deadlocks += 1
                self._log(f"deadlock {' -> '.join(cyc)}: yielding")
                self._cycle_since.pop(key, None)
                if self._back_off(t):
                    return
                for w in (self.plan.waits if self.plan else []):
                    key_, idx = w.res
                    if "/" in key_:
                        self._forbid_resource(key_, idx)
                # give up precedence too: the new claims start fresh (newest stamp), so the robots we were
                # blocking now win every contested cell instead of meeting the same old claims again
                self.book.own.clear()
                self._reroute("wait-for cycle")
                self._cycle_since.pop(key, None)

    # ------------------------------------------------------------ Deadlock Resolution Protocol

    def _drp_member(self, rid: str) -> Member | None:
        if rid == self.rid:
            x, y, th = self.pose
            v, wait = self.v, tuple(self.waiting_for)
            prio, byp = self._priority(), self.bypass is not None and self.bypass.active()
            path = tuple((lg.x1, lg.y1) for lg in self.legs if lg.kind in ("move", "shift"))[:8]
        else:
            p = self.peers.get(rid)
            if p is None or self.t - self.peer_heard.get(rid, -1e9) > 1.5:
                return None
            x, y, th, v, wait = p.x, p.y, p.th, p.v, tuple(p.waiting_for)
            prio, byp, path = self._peer_priority(rid), bool(p.bypass), tuple(tuple(q) for q in p.path)[:8]
        where = self.g.line_of(x, y, th)
        lane = where[0].lane_key(where[1]) if where else ""
        core = bool(self.g.cover(x, y, th, 0.0, 0.0) & self.g.core_cells())
        single_file = self.g.in_single_file(x, y)
        return Member(rid, x, y, th, v, prio, lane, core, self._in_station(rid), byp, wait, path, single_file)

    def _chain_head(self) -> tuple[str, str]:
        """Follow this robot's wait chain to its end: ``(last robot on the chain, why it ends)``.

        ``"cycle"`` means the chain closed, which is the DRP's business, not this rule's.
        """
        seen = {self.rid}
        rid = self.waiting_for[0] if self.waiting_for else ""
        while rid:
            if rid in seen:
                return rid, "cycle"
            seen.add(rid)
            p = self.peers.get(rid)
            if p is None or self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout:
                return rid, "silent"  # the chain ends at a robot nobody can hear
            nxt = p.waiting_for[0] if p.waiting_for else ""
            if not nxt:
                if p.mode not in MOVING:
                    return rid, f"{p.mode}"  # parked / idle / charging: it is never going to move on
                return rid, "driving"  # genuinely working its way forward: a queue, so keep waiting
            rid = nxt
        return "", "none"

    def _check_dead_chain(self, t: float) -> None:
        """Stalled behind a wait chain that cannot resolve itself (P4).

        The DRP only acts on *cycles*. A chain that never closes is left alone, which is right for a
        queue: the robot in front is moving and we will follow it. But when the chain ends at a robot
        that is parked, idle or silent, nobody in it is going anywhere, and every robot behind waits
        for ever. Observed in Gazebo: ``AMR_1 -> AMR_3 -> AMR_19 -> AMR_17``, where AMR_17 sat parked
        56 m away; three robots held station for 264 s and the console watched it at p=0.999.

        A robot can only command itself, so each one independently stops honouring an unsatisfiable
        wait: it drops precedence, forbids the stretch it was queued for and routes again from here.
        """
        head, why = self._chain_head()
        if why in ("cycle", "driving", "none"):
            self._chain_since = None
            return
        if self._chain_since is None:
            self._chain_since = t
            return
        if t - self._chain_since < self.cfg.chain_timeout:
            return
        self._chain_since = None
        self.stats.dead_chains += 1
        self._log(f"wait chain ends at {head} ({why}): it will not move, so re-routing instead of waiting")
        # on the bus as well as in the log: this is a liveness recovery, and a run where it fires often is
        # telling you the fleet keeps queueing behind robots that have stopped for good
        self.publish(AlertMsg(self.rid, t, "dead_chain", head, self.pose[0], self.pose[1], 0.0, why))
        for w in (self.plan.waits if self.plan else []):
            key_, idx = w.res
            if "/" in key_:
                self._forbid_resource(key_, idx)
        # give up precedence, like the cycle case: start fresh so we no longer meet our own old claims
        self.book.own.clear()
        self.waiting_for = []
        self._wait_since = None
        self._reroute(f"wait chain dead at {head} ({why})")

    def _drp_tick(self, t: float) -> None:
        """Detect deadlocks (WFG cycle or no-progress stall), elect the coordinator, plan, supervise, escalate."""
        # a robot with no legs is not "driving": it is re-planning (see _l5), and treating it as a stalled
        # member pulled it into deadlock resolutions it was not part of (3 -> 26 incidents per 480 s)
        driving = self.mode in MOVING and not (self.paused or self.estop) and bool(self.legs) \
            and self.mode not in ("loading", "unloading")
        stalled_me = self._drp_watch.update(t, self.pose[0], self.pose[1], driving)
        if self._drp_mine is not None:
            self._drp_supervise(t)
            return
        if self._drp_prio_rid and t >= self._drp_hold_until:
            self._drp_prio_rid = ""  # the hold lapsed: the edge goes with it
        elif t < self._drp_hold_until and self._drp_prio_rid:
            if self.book.waiting_on_me(self._drp_prio_rid):
                self._drp_hold_until = 0.0
                self._drp_prio_rid = ""
        # cycles through me that persisted
        wfg = WaitForGraph()
        for b in self.waiting_for:
            wfg.add_wait(self.rid, b, (0, 0), 0, t)
        for rid, p in self.peers.items():
            for b in p.waiting_for:
                wfg.add_wait(rid, b, (0, 0), 0, t)
        cycles = []
        for cyc in wfg.find_cycles():
            if self.rid in cyc:
                since = self._cycle_since.setdefault(tuple(cyc), t)
                if t - since >= self.cfg.deadlock_persist:
                    cycles.append(list(cyc))
        if not cycles:
            # no wait loop: this is a queue (behind a busy station, behind slower traffic), not a deadlock. A
            # coordinated plan here only made queued robots back off or re-route off their lane. A robot that
            # stands with nothing to wait for has lost its route (after a manoeuvre): plan it again from here.
            if stalled_me and not self.waiting_for and self.goal and self.bypass is None \
                    and t - self.__dict__.get("_unstick_t", -1e9) > 20.0:
                self._unstick_t = t
                self._log("no progress and nothing to wait for: planning the route again from here")
                self._go(self.goal, self.mode)
            elif stalled_me and self.waiting_for and self.goal and self.bypass is None:
                self._check_dead_chain(t)
            else:
                self._chain_since = None
            return
        members = {m.rid: m for m in (self._drp_member(r) for r in [self.rid, *self.peers]) if m is not None}
        stalled = [self.rid] if stalled_me else []
        # peers standing still in a driving mode near me count as stalled with me
        for rid, p in self.peers.items():
            if rid in members and p.mode in MOVING and abs(p.v) < 0.05 \
                    and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) < 3.0 and (stalled_me or p.waiting_for):
                stalled.append(rid)
        group = next((g for g in deadlock_group(cycles, stalled, members) if self.rid in g), None)
        if not group or len(group) < 1:
            return
        key = frozenset(group)
        if t < self._drp_cooldown.get(key, 0.0):
            return
        was = self._drp_escalated.get(key)
        if was is not None:
            if was == self._group_signature(group):
                return  # already escalated and nothing has moved: it is the operator's, not ours to replay
            self._drp_escalated.pop(key, None)  # the situation changed, so it is worth another attempt
        if any(key & set(p.members) and p.expires > t for p in self._drp_plans.values()):
            return  # a resolution covering us is already running
        since = self._drp_detect.setdefault(key, t)
        rank = sorted(group, key=id_key).index(self.rid)
        if t - since < 3.0 * rank:
            return  # the lower-numbered members get the first chance to coordinate
        rung = self._drp_rung.get(key, 1)
        grp = [members[r] for r in group if r in members]
        blocked = [r for r in group for b in (members[r].waiting_for if r in members else ())
                   if b in self.known_obstacles]
        plan = make_plan(grp, members, self.rid, rung, t, bool(blocked),
                         lambda x, y, th: bool(self.site.body_hits(x, y, th, 0.05)),
                         self.amr.length, self.amr.width, blocked[0] if blocked else "")
        self._drp_mine = plan
        prio = members.get(plan.priority)
        self._drp_prio_start = (prio.x, prio.y) if prio else None
        self.stats.deadlocks += 1
        self._log(f"DRP {plan.kind} rung {plan.rung} for {', '.join(sorted(group, key=id_key))}: {plan.reason}; "
                  f"priority {plan.priority}; moves {[(m.rid, m.action, m.dist) for m in plan.moves]}")
        self.publish(AlertMsg(self.rid, t, "drp_plan", plan.priority, prio.x if prio else 0.0,
                              prio.y if prio else 0.0, float(plan.rung), plan.to_json()))
        self._drp_on_plan(plan)

    def _cycle_broken(self, plan: DRPPlan) -> bool:
        """Is there no longer a wait-for cycle among this plan's members?

        That is what a rung is for. Judging only the priority robot's travel conflates "the deadlock is
        still there" with "the priority robot has somewhere else to queue", and the second one is not a
        failure of the rung.

        Deliberately cautious: an *absent* edge is not evidence. The members need time to act and to
        gossip the result, and a member whose state is missing or stale tells us nothing, so in both
        cases this reports "not broken" and the rung keeps running to its own timeout.
        """
        if self.t - plan.t_plan < CYCLE_SETTLE:
            return False
        members = set(plan.members)
        wfg = WaitForGraph()
        for rid in members:
            if rid == self.rid:
                waits = self.waiting_for
            else:
                p = self.peers.get(rid)
                if p is None or self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout:
                    return False  # no fresh word from this member: draw no conclusion
                waits = p.waiting_for
            for b in waits:
                if b in members:
                    wfg.add_wait(rid, b, (0, 0), 0, self.t)
        return not any(set(c) <= members for c in wfg.find_cycles())

    def _group_signature(self, group) -> tuple:
        """Where this group's robots are, to 0.5 m. Used to tell a changed situation from a replay."""
        out = []
        for rid in sorted(group, key=id_key):
            if rid == self.rid:
                x, y = self.pose[0], self.pose[1]
            elif rid in self.peers:
                x, y = self.peers[rid].x, self.peers[rid].y
            else:
                continue
            out.append((rid, round(x * 2), round(y * 2)))
        return tuple(out)

    def _drp_supervise(self, t: float) -> None:
        """Coordinator: close the incident when the priority robot got through, escalate if nothing moves."""
        plan = self._drp_mine
        prio = self._drp_member(plan.priority)
        moved = prio is not None and self._drp_prio_start is not None and \
            math.hypot(prio.x - self._drp_prio_start[0], prio.y - self._drp_prio_start[1]) > 1.5
        # The point of a rung is to break the circular wait, and the priority robot travelling 1.5 m was
        # the only thing that counted as success. When it is held up by something *outside* the group -
        # a queue ahead of it, a station still busy - a rung that worked perfectly still read as a
        # timeout, so the group was re-planned at a higher rung and the yielding was thrown away. Live
        # run: 15 plans, 3 resolved, 11 timed out. Treat a broken cycle as the success it is.
        why = ""
        if moved:
            why = f"{plan.priority} got through"
        elif self._cycle_broken(plan):
            why = f"circular wait broken, {plan.priority} still queued behind something outside the group"
        if why:
            self._log(f"DRP {plan.epoch}: resolved ({why})")
            self.publish(AlertMsg(self.rid, t, "drp_done", plan.epoch, 0.0, 0.0, float(plan.rung), plan.kind))
            self._drp_on_done(plan.epoch)
            key = frozenset(plan.members)
            self._drp_rung.pop(key, None)
            self._drp_detect.pop(key, None)
            self._drp_mine = None
            return
        if t - plan.t_plan < RUNG_S:
            return
        key = frozenset(plan.members)
        rung = plan.rung + 1
        self.publish(AlertMsg(self.rid, t, "drp_done", plan.epoch, 0.0, 0.0, float(plan.rung), "timeout"))
        self._drp_on_done(plan.epoch, token=False)
        self._drp_mine = None
        if rung > 3:
            # rung 4: hand it to the operator (console alert), try again later
            self._log(f"DRP {plan.epoch}: escalated to the operator after {rung - 1} rungs")
            x, y = (prio.x, prio.y) if prio else self.pose[:2]
            self.publish(AlertMsg(self.rid, t, "drp_escalated", plan.priority, x, y, 4.0,
                                  f"{plan.kind} {', '.join(plan.members)}"))
            self._drp_cooldown[key] = t + 60.0
            self._drp_rung.pop(key, None)
            # Escalation used to leave no trace, so after the 60 s cooldown the rung read back as 1 and the
            # identical four-rung sequence replayed for ever - a ~120 s lap that inflated `escalated` while
            # resolving nothing. Remember the situation we gave up on: it is only worth trying again once
            # something has actually changed (a member moved, joined or left).
            self._drp_escalated[key] = self._group_signature(plan.members)
            return
        self._drp_rung[key] = rung
        self._drp_detect[key] = t - 1e3  # re-plan at once at the next rung

    def _drp_on_plan(self, plan: DRPPlan) -> None:
        if plan.expires <= self.t:
            return
        for ep, old in list(self._drp_plans.items()):
            if set(old.members) & set(plan.members):  # a newer rung replaces the older plan of the same group
                self._drp_plans.pop(ep, None)
        self._drp_plans[plan.epoch] = plan
        mv = plan.move_of(self.rid)
        if mv is None:
            return
        if mv.action == "backoff" and mv.dist > 0 and self.mode in MOVING:
            x, y, th = self.pose
            self.book.own.clear()
            self.legs = [Leg("move", x - mv.dist * math.cos(th), y - mv.dist * math.sin(th), reverse=True,
                             v_max=0.3, tag="drp_backoff")]
            self._leg_start = (x, y)
            self._backed_off = True
            self._drp_hold_until = 0.0
            self._drp_prio_rid = ""
            self._log(f"DRP {plan.epoch}: backing off {mv.dist:.1f} m for {plan.priority}")
        elif mv.action == "hold":
            self.book.own.clear()  # our bookings ahead of the leader go
            # hold out the rung: a 10 s hold lapsed half-way through it, and the re-plan re-booked the same
            # cell keeping its original stamp, so the held robot regained precedence over the priority robot
            self._drp_hold_until = self.t + min(plan.expires - self.t, RUNG_S)
            self._drp_prio_rid = plan.priority  # advertise the edge: the wait-for graph is built from these
            self._log(f"DRP {plan.epoch}: holding for {plan.priority}")
        elif mv.action == "reroute":
            forbid_done = False
            for w in (self.plan.waits if self.plan else []):
                k_, i_ = w.res
                if "/" in k_:
                    self._forbid_resource(k_, i_)
                    forbid_done = True
            if not forbid_done:
                where = self.g.line_of(self.pose[0], self.pose[1], self.pose[2])
                if where:
                    ln, lane = where
                    idx = max(0, int((ln.along(self.pose[0], self.pose[1]) - ln.lo) / 0.5))
                    self._forbid_resource(f"{ln.name}/{lane}", idx)
            self.book.own.clear()
            self._reroute(f"DRP {plan.epoch}: re-route")
        elif mv.action == "pass":
            self._drp_hold_until = 0.0
            self._drp_prio_rid = ""

    def _drp_on_done(self, epoch: str, token: bool = True) -> None:
        plan = self._drp_plans.pop(epoch, None)
        if plan is None:
            return
        if self.rid in plan.members:
            self._drp_hold_until = 0.0
            self._drp_prio_rid = ""
        if token:
            # anti-relapse: the priority robot keeps precedence on the zone for a while
            until = self.t + TOKEN_S
            for b in zone_boxes(plan):
                for res in self.g.cover_box(b):
                    self.book.tokens[res] = (plan.priority, until)

    def _back_off(self, t: float, dist: float = 1.2) -> bool:
        """Physical deadlock resolution: reverse ``dist`` m along the way we came, if that space is free of
        static geometry and of other robots' bodies and bookings, then re-plan from there. Gives the robot we
        block room to move (the classical reverse-evasion recovery, but checked against the fleet's claims)."""
        if self.mode not in MOVING or self._in_station(self.rid):
            return False
        x, y, th = self.pose
        bx, by = x - dist * math.cos(th), y - dist * math.sin(th)
        steps = max(6, math.ceil(dist / 0.3))  # L2b asks for up to MAX_REVERSE m: keep the sampling dense
        for k in range(1, steps + 1):
            f = k / steps
            px, py = x - f * dist * math.cos(th), y - f * dist * math.sin(th)
            if self.site.body_hits(px, py, th, 0.05):
                return False
            for res in self.g.cover(px, py, th, 0.1, 0.05):
                if res in self.book.occupied:
                    continue
                if self.book.blocking(res, t, ignore_losers=False):
                    return False
        self.book.own.clear()
        self.legs = [Leg("move", bx, by, reverse=True, v_max=0.3, tag="backoff")]
        self._leg_start = (x, y)
        self._backed_off = True
        self._log(f"deadlock: backing off {dist:.1f} m to make room")
        return True

    def _collision_recover(self, t: float, dist: float = 0.4) -> None:
        """After a collision pause: move ``dist`` m away from the contact along the side the lidar shows clear
        (rear first, the way we came), then resume and re-plan. Retries every 3 s while boxed in."""
        self._collision_pause = t  # next attempt in 3 s if this one finds no clear side
        pts = self.points_body
        L, W = self.amr.length / 2, self.amr.width / 2 + 0.05
        def clear(sign: float) -> bool:
            if pts is None or len(pts) == 0:
                return True
            ahead = pts[:, 0] * sign - L
            return not ((ahead > -0.02) & (ahead < dist + 0.15) & (np.abs(pts[:, 1]) <= W)).any()
        sign = -1.0 if clear(-1.0) else (1.0 if clear(1.0) else 0.0)
        if sign == 0.0:
            self._log("collision recovery: boxed in on both sides, staying paused")
            return
        x, y, th = self.pose
        self.paused = False
        self._collision_pause = None
        if not self.estop:
            self.arbiter.release(Source.OPERATOR, t)
        if self.mode in MOVING:
            self.book.own.clear()
            bx, by = x + sign * dist * math.cos(th), y + sign * dist * math.sin(th)
            self.legs = [Leg("move", bx, by, reverse=sign < 0, v_max=0.25, tag="backoff")]
            self._leg_start = (x, y)
            self._backed_off = True
        self._log(f"collision recovery: {'reversing' if sign < 0 else 'moving forward'} {dist:.1f} m, resuming")

    def _peer_priority(self, rid: str) -> int:
        if rid == self.rid:
            return self._priority()
        p = self.book.peers.get(rid)
        return p.priority if p else 2

    def _priority(self) -> int:
        if self.job:
            return int(self.job.get("priority", 2)) + (1 if self.mode == "to_drop" else 0)
        return 1 if self.mode in ("to_park", "to_charger") else 2

    # ============================================================ L4 allocation

    def _l4(self, t: float) -> None:
        # failed peers: purge their tasks, forget their claims beyond their body
        for rid, heard in list(self.peer_heard.items()):
            if t - heard > self.cfg.peer_timeout * 2 and rid in self.peers:
                freed = self.alloc.purge_peer(rid, t)
                if freed:
                    self._log(f"{rid} silent: re-auctioning {', '.join(freed)}")
        self.alloc.eligible = (self.health.eligible_for_cbba and not self.estop and not self.paused
                               and self.battery >= self.cfg.battery_min_task and self.mode not in ("to_charger",))
        anchor = self.job["dropoff_station"] if self.job else (self.goal or self._anchor_station())
        if anchor:
            self.alloc.set_anchor(anchor)
        self.publish(self.alloc.step(t))

    # ============================================================ L1 perception, risk, ORCA

    def _perceive(self, t: float, pts: np.ndarray) -> None:
        body = to_body(pts, self.pose)
        keep = drop_self(body, self.amr.length, self.amr.width)
        pts, body = pts[keep], body[keep]
        self.points_body = body
        if self.static_map is None or len(pts) == 0:
            self.tracks = self.tracker.update(t, [])
            return
        dyn = pts[~self.static_map.occupied(pts)]
        meas = clusters(dyn, gap=0.3, min_points=3)
        self.tracks = self.tracker.update(t, meas)
        self._assess_risk(t)
        self._register_obstacles(t, dyn)

    def _register_obstacles(self, t: float, dyn: np.ndarray) -> None:
        """Static unmapped things in a lane (dropped pallet, failed robot) become fleet-wide obstacles."""
        x, y, th = self.pose
        for tr in self.tracks:
            if math.hypot(tr.vs, tr.vy) > 0.15 or self._identify(tr) is not None:
                continue
            if math.hypot(tr.s - x, tr.y - y) > 9.0:
                continue
            if not self.g.lines_near(OrientedBox(tr.s, tr.y, 0.2, 0.2, 0.0)):
                continue
            if any(math.hypot(tr.s - ox, tr.y - oy) < 1.0 for ox, oy, _L, _W in self.known_obstacles.values()):
                for name, (ox, oy, _L, _W) in self.known_obstacles.items():
                    if math.hypot(tr.s - ox, tr.y - oy) < 1.0:
                        self._obstacle_seen[name] = t
                continue
            if self._near_any_peer(tr):
                continue  # a robot, even one whose exact identity we are not sure of, is never a pallet
            key = f"{round(tr.s)}:{round(tr.y)}"
            self._obs_candidates[key] = self._obs_candidates.get(key, 0) + 1
            if self._obs_candidates[key] < 5:  # seen in 5 scans (0.5 s) before it is believed
                continue
            # the lidar sees the near faces only: assume a pallet-sized body behind them
            bx = math.atan2(tr.y - y, tr.s - x)
            L, W = max(tr.length, 1.0), max(tr.width, 1.0)
            cx, cy = tr.s + 0.35 * math.cos(bx), tr.y + 0.35 * math.sin(bx)
            name = f"lidar:{self.rid}:{len(self._obstacle_seen) + 1}"
            self.known_obstacles[name] = (cx, cy, L, W)
            self._obstacle_seen[name] = t
            self._log(f"lidar: unmapped obstacle {name} at ({cx:.1f}, {cy:.1f})")
            self.publish(AlertMsg(self.rid, t, "obstacle", name, cx, cy, 0.0, f"{L:.2f}x{W:.2f}"))
        # forget obstacles that should be visible (close, in view) but have not been seen for 3 s
        for name, (ox, oy, _L, _W) in list(self.known_obstacles.items()):
            if math.hypot(ox - x, oy - y) < 6.0 and t - self._obstacle_seen.get(name, t) > 3.0:
                near = dyn[np.hypot(dyn[:, 0] - ox, dyn[:, 1] - oy) < 0.9] if len(dyn) else dyn
                if len(near) == 0 and self._can_see(ox, oy):
                    self.known_obstacles.pop(name, None)
                    self._log(f"obstacle {name} cleared")
                    self.publish(AlertMsg(self.rid, t, "obstacle_cleared", name, ox, oy))

    def _can_see(self, px: float, py: float) -> bool:
        """Is ``(px, py)`` really in view from this robot's lidars right now?

        Clearing an obstacle is a fleet-wide assertion that a space is empty, and it overrides every
        robot that reported it - including one parked next to the thing, looking straight at it. An
        empty point cloud at some bearing is only evidence of emptiness if the beams actually reached
        there, so a view cut off by a rack or by another robot's body must not count. Observed in
        Gazebo: real pallets erased and re-discovered at the same spot four times over, and every
        clear issued by a robot other than the one that had reported it.
        """
        x, y, _th = self.pose
        d = math.hypot(px - x, py - y)
        if d < 1e-6:
            return True
        if self.static_map is not None:
            n = max(3, int(d / max(self.static_map.res, 1e-3)))
            f = np.linspace(0.0, 1.0, n)[1:-1]  # neither end: the robot itself, nor the target cell
            if len(f):
                ray = np.column_stack((x + f * (px - x), y + f * (py - y)))
                if self.static_map.occupied(ray).any():
                    return False  # a rack or wall is in the way
        for rid, p in self.peers.items():
            if self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout:
                continue
            body = OrientedBox(p.x, p.y, self.amr.length, self.amr.width, p.th)
            if segment_hits_box(x, y, px, py, body):
                return False  # another robot is standing between us and the spot
        return True

    def _near_any_peer(self, tr: TrackedObject, pad: float = 1.0) -> bool:
        """Is this track plausibly *any* robot, on a generous reading of the gossip?

        ``_identify`` has to be reasonably tight because the risk layer uses its answer to swap in the
        peer's true centre and velocity. Publishing a fleet-wide obstacle is the more expensive mistake
        and it is not reversible by the robot that made it, so it gets the looser test: anything near a
        peer's last known position is simply not announced as a pallet.
        """
        for rid, p in self.peers.items():
            if self.t - self.peer_heard.get(rid, -1e9) > self.cfg.peer_timeout:
                continue
            if math.hypot(tr.s - p.x, tr.y - p.y) <= self.amr.rotation_radius + pad:
                return True
        return False

    def _on_known_obstacle(self, px: float, py: float, pad: float = 0.35) -> bool:
        for ox, oy, L, W in self.known_obstacles.values():
            if abs(px - ox) <= L / 2 + pad and abs(py - oy) <= W / 2 + pad:
                return True
        return False

    def _identify(self, tr: TrackedObject) -> str | None:
        """Which peer this track is, or ``None`` if it is something unmapped.

        The tracker only ever sees a body's *near faces*, so the centroid it reports sits inside the
        robot it came from - 0.39 m in side-on, 0.60 m end-on, up to 0.72 m seen at a corner. The old
        test compared that centroid with the peer's *centre* under a flat 0.9 m radius, which left 0.18 m
        for peer-state age and extrapolation error in the worst case. Queueing robots therefore failed
        to be identified, and :meth:`_register_obstacles` published them to the whole fleet as dropped
        pallets: six such phantoms in one 480 s Gazebo run with nothing actually on the floor.

        Testing the centroid against the peer's *footprint* removes the body-size term entirely, so the
        only budget needed is for pose age, which scales with how fast the peer is going.
        """
        best = None
        for rid, p in self.peers.items():
            if self.t - self.peer_heard.get(rid, -1e9) > PEER_ID_MAX_AGE:
                continue
            dt = max(self.t - p.stamp, 0.0)
            px, py = p.x + p.v * math.cos(p.th) * dt, p.y + p.v * math.sin(p.th) * dt
            dx, dy = tr.s - px, tr.y - py
            c, s = math.cos(p.th), math.sin(p.th)
            lx, ly = c * dx + s * dy, -s * dx + c * dy  # the track, in the peer's body frame
            m = PEER_ID_MARGIN + abs(p.v) * dt * 0.5  # a stale pose of a moving peer is less certain
            if abs(lx) <= self.amr.length / 2 + m and abs(ly) <= self.amr.width / 2 + m:
                d = math.hypot(dx, dy)
                if best is None or d < best[0]:
                    best = (d, rid)
        return best[1] if best else None

    def _assess_risk(self, t: float) -> None:
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        ego = TrackedObject(0.0, 0.0, self.v, 0.0, self.amr.length, self.amr.width, self.rid)
        band = (-self.amr.width / 2 - 0.05, self.amr.width / 2 + 0.05)
        local, ids = [], {}
        for tr in self.tracks:
            if self._on_known_obstacle(tr.s, tr.y):
                # a reported static obstacle: the booking block and the raw-lidar fields (hard stop, protective
                # field) own it. Its track "moves" only because the robot sees different faces of it while
                # passing, and a predicted phantom velocity would brake the robot beside it for ever.
                continue
            dx, dy = tr.s - x, tr.y - y
            ls, ly = c * dx + s * dy, -s * dx + c * dy
            vs, vy = c * tr.vs + s * tr.vy, -s * tr.vs + c * tr.vy
            rid = self._identify(tr)
            if rid is not None:
                p = self.peers[rid]
                age = max(t - p.stamp, 0.0)
                px, py = p.x + p.v * math.cos(p.th) * age, p.y + p.v * math.sin(p.th) * age
                ls, ly = c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)  # peer centre, not the seen face
                rel = wrap(p.th - th)
                L = abs(self.amr.length * math.cos(rel)) + abs(self.amr.width * math.sin(rel))
                W = abs(self.amr.length * math.sin(rel)) + abs(self.amr.width * math.cos(rel))
                vs, vy = p.v * math.cos(rel), p.v * math.sin(rel)
            else:
                L, W = tr.length, tr.width
            lt = TrackedObject(ls, ly, vs, vy, L, W, tr.track_id)
            local.append(lt)
            ids[tr.track_id] = rid
        risks = self.monitor.assess(ego, band, local, identify=lambda tr: ids.get(tr.track_id),
                                    decel_of=lambda rid: self.amr.e_decel)
        worst = max(risks, key=lambda r: r.probability, default=None)
        if worst is not None and worst.level is RiskLevel.BRAKE:
            if not self.risk_active:
                self.stats.risk_brakes += 1
                self.publish(AlertMsg(self.rid, t, "risk", worst.other_id, x, y, worst.probability,
                                      f"P={worst.probability:.2f} gap={worst.gap:.2f} m"))
                self._log(f"risk brake: {worst.other_id} P={worst.probability:.2f}")
            self.risk_active = True
            self.arbiter.request(MotionIntent(Source.EMERGENCY, Kind.BRAKE, None, stamp=t, ttl=0.3), t)
        else:
            self.risk_active = False

    def _orca_inputs(self, rid: str, t: float, intent: MotionIntent):
        model = self.arbiter.model
        x, y, th = self.pose
        D = model.D
        r = model.effective_radius(self.amr.length, self.amr.width, 0.02)
        ego = OrcaAgent(rid, (x + D * math.cos(th), y + D * math.sin(th)),
                        (self.v * math.cos(th), self.v * math.sin(th)), r)
        nbs = []
        c, s = math.cos(th), math.sin(th)
        travel = -1.0 if self.legs and self.legs[0].kind in ("move", "shift") and self.legs[0].reverse else 1.0
        for pid, p in self.peers.items():
            if t - self.peer_heard.get(pid, -1e9) > 1.0:
                continue
            dx, dy = p.x - x, p.y - y
            if dx * dx + dy * dy > 36.0:
                continue
            lat = -s * dx + c * dy
            if abs(lat) > 0.6:  # only the robot's own lane band: other lanes are separated by reservations
                continue
            # a peer behind us (relative to the way we drive) is the follower: keeping the gap is its job (its
            # own ORCA, lidar fields and bookings). Counting it here froze the leader whenever a queue closed
            # up behind it: the overlapping discs made ORCA infeasible and it braked to zero.
            if (c * dx + s * dy) * travel < -0.2:
                continue
            nbs.append(OrcaAgent(pid, (p.x + D * math.cos(p.th), p.y + D * math.sin(p.th)),
                                 (p.v * math.cos(p.th), p.v * math.sin(p.th)), r, True))
        return ego, nbs, [], OrcaMode.PATH_SPEED_ONLY

    # ============================================================ reporting

    def state_msg(self, t: float) -> RobotStateMsg:
        path = []
        if self.plan is not None:
            for p in self.plan.traj.prims:
                if p.kind in ("move", "shift"):
                    path.append((round(p.x1, 2), round(p.y1, 2)))
        rest = [(round(lg.x1, 2), round(lg.y1, 2)) for lg in self.legs if lg.kind in ("move", "shift")]
        pm = sorted(self.stats.plan_ms)
        return RobotStateMsg(
            self.rid, t, round(self.pose[0], 3), round(self.pose[1], 3), round(self.pose[2], 4), round(self.v, 3),
            round(self.w, 3), self._display_mode(), round(self.battery, 4), self.job["task_id"] if self.job else "",
            self.health.value, "", self.goal or "",
            self.bypass.state_name() if self.bypass is not None and self.bypass.active() else "",
            round(pm[int(0.95 * (len(pm) - 1))], 2) if pm else 0.0, (path + rest)[:40], list(self.waiting_for),
            self.station or "")

    def _display_mode(self) -> str:
        if self.estop:
            return "estop"
        if self.paused:
            return "paused"
        if self.mode == "parked_charger":
            return "parked"
        return self.mode

    def _log(self, text: str) -> None:
        self.events.append((round(self.t, 2), text))
