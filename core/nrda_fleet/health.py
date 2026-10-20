"""Operational health of one AMR, and the predicates that gate its behaviour.

A single enum is the robot's health, and its predicate properties are the
authority for what a robot in each state may do: ``can_move`` gates motion
(L1), ``eligible_for_cbba`` gates bidding (L4), ``holds_reservations`` gates
claim ownership (L3), ``is_chassis_obstacle`` makes a dead chassis a static
obstacle for every peer, and ``publishes_heartbeat`` /
``is_local_autonomy_permitted`` describe the communications-loss case. Keeping
them on the enum rather than scattered through ``if`` statements means one
place defines the degraded-mode policy.

The state travels between robots as the ``health`` string of
:class:`nrda_fleet.bus.RobotStateMsg` (and ``nrda_msgs/RobotState``);
:meth:`FaultState.from_str` is the inbound parse and falls back to ``HEALTHY``
for a name it does not know.
"""

from enum import Enum


class FaultState(Enum):
    """Discrete operational health and fault states for an AMR."""

    HEALTHY = 'HEALTHY'
    DEGRADED = 'DEGRADED'
    COMM_LOSS = 'COMM_LOSS'
    MOTION_FAILURE = 'MOTION_FAILURE'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    FAILED = 'FAILED'
    RECOVERING = 'RECOVERING'

    @classmethod
    def from_str(cls, val: str) -> 'FaultState':
        """Convert string to FaultState."""
        upper = val.strip().upper()
        if upper in cls.__members__:
            return cls[upper]
        return cls.HEALTHY

    @property
    def can_move(self) -> bool:
        """Return True if robot is physically capable of wheel motion."""
        return self in (FaultState.HEALTHY, FaultState.DEGRADED, FaultState.COMM_LOSS)

    @property
    def publishes_heartbeat(self) -> bool:
        """Return True if robot's compute and network stack emit heartbeats."""
        return self != FaultState.FAILED

    @property
    def can_own_tasks(self) -> bool:
        """Return True if robot can retain ownership of assigned tasks."""
        return self in (
            FaultState.HEALTHY,
            FaultState.DEGRADED,
            FaultState.COMM_LOSS,
            FaultState.EMERGENCY_STOP,
        )

    @property
    def eligible_for_cbba(self) -> bool:
        """Return True if robot is actively eligible to bid on new tasks in CBBA."""
        return self in (FaultState.HEALTHY, FaultState.DEGRADED)

    @property
    def holds_reservations(self) -> bool:
        """Return True if robot can maintain space-time corridor reservations."""
        return self in (
            FaultState.HEALTHY,
            FaultState.DEGRADED,
            FaultState.COMM_LOSS,
            FaultState.EMERGENCY_STOP,
        )

    @property
    def is_chassis_obstacle(self) -> bool:
        """Return True if disabled chassis must be rasterized as static obstacle."""
        return self in (FaultState.MOTION_FAILURE, FaultState.FAILED)

    @property
    def is_local_autonomy_permitted(self) -> bool:
        """Return True if robot may operate under local autonomy during disconnection."""
        return self == FaultState.COMM_LOSS

