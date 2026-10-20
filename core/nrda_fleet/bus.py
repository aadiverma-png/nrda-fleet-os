"""Peer-to-peer messages between edge agents, and the in-process bus used by the 2-D simulator.

Robots never share memory. Everything one robot knows about another arrives
as one of these messages. In Gazebo the same messages travel over ROS 2
topics (``nrda_fleet_ros`` converts them to ``nrda_msgs``); in the 2-D
simulator :class:`SimBus` delivers them with configurable latency and
Gilbert-Elliott burst loss (Gilbert 1960, Elliott 1963).
"""

from __future__ import annotations

import heapq
import json
import math
import random
from dataclasses import asdict, dataclass, field
from typing import Callable, Protocol

# ---------------------------------------------------------------------------- messages


@dataclass
class RobotStateMsg:
    """Broadcast at 10 Hz: pose, velocity, mode. Also serves as the heartbeat."""

    robot_id: str
    stamp: float
    x: float
    y: float
    th: float
    v: float
    w: float
    mode: str = "idle"  # idle | to_pickup | loading | to_drop | unloading | to_charger | charging | parking | paused | fault | estop
    battery: float = 1.0
    task_id: str = ""
    health: str = "HEALTHY"  # nrda_fleet.health.FaultState name
    lane: str = ""  # line/lane key the robot is driving in
    goal: str = ""  # station the robot is heading to
    bypass: str = ""  # visison controller state while a local manoeuvre runs
    compute_ms: float = 0.0  # p95 tick compute
    path: list = field(default_factory=list)  # [(x, y), ...] upcoming route polyline (for displays)
    waiting_for: list = field(default_factory=list)  # robots this one is waiting on (wait-for graph edges)
    station: str = ""  # station it is docked at / serving


@dataclass
class ClaimMsg:
    """Owner-authoritative snapshot of all of one robot's space-time claims."""

    robot_id: str
    stamp: float
    seq: int
    priority: int
    # each claim: (resource key, index, start, end, claim stamp)
    claims: list[tuple[str, int, float, float, float]] = field(default_factory=list)
    occupied: list[tuple[str, int]] = field(default_factory=list)  # resources its body covers now


@dataclass
class BidMsg:
    """CBBA consensus message: this robot's winning bids, winners and timestamps."""

    robot_id: str
    stamp: float
    iteration: int
    winning_bids: dict[str, float] = field(default_factory=dict)
    winning_robots: dict[str, str] = field(default_factory=dict)
    timestamps: dict[str, float] = field(default_factory=dict)
    bundle: list[str] = field(default_factory=list)


@dataclass
class TaskMsg:
    """A task released to the fleet (by the order gateway / scenario)."""

    task_id: str
    stamp: float
    pickup: str  # station name
    dropoff: str
    priority: int = 2  # 1 low .. 4 critical
    deadline: float = 0.0
    assign_to: str = ""  # operator override: only this robot may take it
    kind: str = "transport"
    cancelled: bool = False


@dataclass
class TaskEventMsg:
    task_id: str
    robot_id: str
    stamp: float
    status: str  # assigned | in_progress | picked | completed | failed | released
    detail: str = ""


@dataclass
class AlertMsg:
    """Emergency alerts and directives (no remote actuation: the receiver's arbiter decides)."""

    sender: str
    stamp: float
    kind: str  # brake | risk | reverse | yield | resume | obstacle | wfg_yield
    target: str = ""  # robot the directive is for ("" = everyone near)
    x: float = 0.0
    y: float = 0.0
    value: float = 0.0  # distance for reverse, probability for risk ...
    detail: str = ""


@dataclass
class ControlMsg:
    """Operator commands from the console (pause, resume, e-stop, charge, zone rules)."""

    stamp: float
    command: str  # estop | release_estop | pause | resume | charge | fault | clear_fault | zone_rule
    target: str = ""  # robot id or "" for the whole fleet
    value: str = ""


@dataclass
class SafetyEventMsg:
    event_id: str
    stamp: float
    level: str  # COLLISION | NEAR_MISS | AVOIDED
    robots: list[str]
    other: str = ""  # robot id, rack name, obstacle id
    x: float = 0.0
    y: float = 0.0
    rel_speed: float = 0.0
    source: str = ""  # contact | overlap | gap | risk | orca
    detail: str = ""


TOPICS = {
    RobotStateMsg: "state",
    ClaimMsg: "claims",
    BidMsg: "bids",
    TaskMsg: "tasks",
    TaskEventMsg: "task_events",
    AlertMsg: "alerts",
    ControlMsg: "control",
    SafetyEventMsg: "safety_events",
}
BY_TOPIC = {v: k for k, v in TOPICS.items()}


def to_json(msg) -> str:
    return json.dumps({"_t": TOPICS[type(msg)], **asdict(msg)}, separators=(",", ":"))


def from_json(text: str):
    d = json.loads(text)
    cls = BY_TOPIC[d.pop("_t")]
    if cls is ClaimMsg:
        d["claims"] = [tuple(c) for c in d.get("claims", [])]
        d["occupied"] = [tuple(c) for c in d.get("occupied", [])]
    return cls(**d)


def size_of(msg) -> int:
    """Approximate wire size in bytes (cheap: used for bandwidth accounting on every send)."""
    if isinstance(msg, ClaimMsg):
        return 64 + 44 * len(msg.claims) + 16 * len(msg.occupied)
    if isinstance(msg, BidMsg):
        return 64 + 40 * len(msg.winning_bids) + 12 * len(msg.bundle)
    if isinstance(msg, RobotStateMsg):
        return 180 + 16 * len(msg.path) + 10 * len(msg.waiting_for)
    return 140


# ------------------------------------------------------------------------------- bus


class PeerBus(Protocol):
    def publish(self, sender: str, msg) -> None: ...

    def subscribe(self, robot_id: str, handler: Callable[[object], None]) -> None: ...


@dataclass
class CommProfile:
    """Latency and Gilbert-Elliott burst loss (good/bad state), per directed link."""

    name: str = "nominal"
    latency: float = 0.02  # s
    jitter: float = 0.01  # s
    p_loss_good: float = 0.0
    p_loss_bad: float = 0.5
    p_good_to_bad: float = 0.0
    p_bad_to_good: float = 0.3
    range_m: float = math.inf

    @classmethod
    def lossy(cls, loss: float) -> "CommProfile":
        """Average loss ``loss`` with bursts (stationary share of the bad state = 2 * loss)."""
        pgb = 0.3 * min(2 * loss, 0.9) / max(1 - min(2 * loss, 0.9), 1e-6)
        return cls(f"loss{loss:.2f}", 0.03, 0.02, loss / 4, min(0.5 + loss, 0.95), pgb, 0.3)


@dataclass
class SimBus:
    """Deterministic in-process peer bus for the 2-D simulator."""

    profile: CommProfile = field(default_factory=CommProfile)
    seed: int = 0
    now: float = 0.0
    partitions: list[tuple[float, float, set[str]]] = field(default_factory=list)  # (t0, t1, isolated robots)
    positions: dict[str, tuple[float, float]] = field(default_factory=dict)

    def __post_init__(self):
        self._rng = random.Random(self.seed)
        self._subs: dict[str, Callable] = {}
        self._queue: list[tuple[float, int, str, object]] = []
        self._n = 0
        self._bad: dict[tuple[str, str], bool] = {}
        self.sent = self.delivered = self.dropped = self.bytes = 0
        self.observers: list[Callable[[object], None]] = []  # console / metrics taps (see everything)

    def subscribe(self, robot_id: str, handler: Callable[[object], None]) -> None:
        self._subs[robot_id] = handler

    def unsubscribe(self, robot_id: str) -> None:
        self._subs.pop(robot_id, None)

    def publish(self, sender: str, msg) -> None:
        nbytes = size_of(msg)
        for obs in self.observers:
            obs(msg)
        for rid in self._subs:
            if rid == sender:
                continue
            self.sent += 1
            self.bytes += nbytes
            if self._lost(sender, rid):
                self.dropped += 1
                continue
            delay = max(self.profile.latency + self._rng.uniform(-1, 1) * self.profile.jitter, 0.0)
            self._n += 1
            heapq.heappush(self._queue, (self.now + delay, self._n, rid, msg))

    def inject(self, msg) -> None:
        """Infrastructure message (tasks, operator control) delivered to every robot without loss."""
        for obs in self.observers:
            obs(msg)
        for rid in self._subs:
            self._n += 1
            heapq.heappush(self._queue, (self.now + self.profile.latency, self._n, rid, msg))

    def _lost(self, a: str, b: str) -> bool:
        for t0, t1, iso in self.partitions:
            if t0 <= self.now < t1 and ((a in iso) != (b in iso)):
                return True
        pa, pb = self.positions.get(a), self.positions.get(b)
        if pa and pb and math.dist(pa, pb) > self.profile.range_m:
            return True
        key = (a, b)
        bad = self._bad.get(key, False)
        if bad and self._rng.random() < self.profile.p_bad_to_good:
            bad = False
        elif not bad and self._rng.random() < self.profile.p_good_to_bad:
            bad = True
        self._bad[key] = bad
        return self._rng.random() < (self.profile.p_loss_bad if bad else self.profile.p_loss_good)

    def advance(self, t: float) -> None:
        """Deliver everything due by time ``t``."""
        self.now = t
        while self._queue and self._queue[0][0] <= t:
            _, _, rid, msg = heapq.heappop(self._queue)
            h = self._subs.get(rid)
            if h is not None:
                self.delivered += 1
                h(msg)
