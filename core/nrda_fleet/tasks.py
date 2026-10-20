"""Warehouse tasks and seeded workloads.

A task moves a tote/pallet from a pickup station (picker zone, decant,
overflow induct) to a drop-off station (outbound drop, packing induct).
The life-cycle is
``queued -> assigned -> in_progress (to pickup) -> picked -> completed``
(or ``failed`` / ``cancelled``).
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from .bus import TaskMsg

PRIORITY_NAMES = {1: "LOW", 2: "NORMAL", 3: "HIGH", 4: "CRITICAL"}


@dataclass
class Task:
    task_id: str
    pickup: str
    dropoff: str
    priority: int = 2
    release: float = 0.0
    deadline: float = 0.0
    assign_to: str = ""
    status: str = "queued"
    robot: str = ""
    t_assigned: float = 0.0
    t_picked: float = 0.0
    t_done: float = 0.0
    history: list[tuple[float, str, str]] = field(default_factory=list)

    def msg(self, stamp: float) -> TaskMsg:
        return TaskMsg(self.task_id, stamp, self.pickup, self.dropoff, self.priority, self.deadline, self.assign_to)


PICKUP_KINDS = ("picker", "decant", "overflow")
DROP_KINDS = ("drop", "pack")


def generate_workload(stations: dict, n: int, rate_per_min: float, seed: int, t0: float = 5.0,
                      priorities=(1, 2, 2, 2, 3, 4), deadline_s: float = 600.0) -> list[Task]:
    """``n`` tasks with Poisson releases (``rate_per_min``), random pickup and drop-off stations."""
    rng = random.Random(seed)
    pickups = sorted(n_ for n_, st in stations.items() if st.kind in PICKUP_KINDS)
    drops = sorted(n_ for n_, st in stations.items() if st.kind in DROP_KINDS)
    t = t0
    out = []
    for i in range(n):
        t += rng.expovariate(rate_per_min / 60.0) if rate_per_min > 0 else 0.0
        p = rng.choice(priorities)
        out.append(Task(f"T{i + 1:04d}", rng.choice(pickups), rng.choice(drops), p, round(t, 2),
                        round(t + deadline_s / (1 + 0.5 * (p - 1)), 2)))
    return out
