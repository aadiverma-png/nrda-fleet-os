"""AMR-01 tote transporter: one place for every physical parameter.

Numbers come from ``nrds_warehouse_sim/models/amr_01/model.sdf`` (DiffDrive
plugin, chassis collision box, lidar mounts). Planning limits are set a
little below the plugin limits so the tracker always has control authority.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from visison.geometry import Footprint
from visison.kinematics import MotionLimits, QuinticLaneChange
from visison.motion_models import DiffDrive
from visison.planner import RobotSpec


@dataclass(frozen=True)
class LidarMount:
    name: str
    x: float
    y: float
    yaw: float
    fov: float = math.radians(270)
    range_max: float = 25.0


@dataclass(frozen=True)
class AMR01Spec:
    # Linear speeds run at 1.5x the original demo values (cruise 1.2 -> 1.8 m/s) with accel / decel scaled to
    # match, so the braking distance grows by 1.5 rather than 2.25 and the lidar protective field stays usable.
    # Rotation is unchanged: the rotation guard sweeps |w| * horizon, so a faster spin would stop turns next to
    # racks. Gazebo's diff-drive limits in models/amr_01/model.sdf carry the same numbers.
    length: float = 1.20
    width: float = 0.78
    wheel_separation: float = 0.62
    wheel_radius: float = 0.075
    # DiffDrive plugin hard limits (gz-sim applies max_linear_acceleration to braking as well)
    v_hw: float = 2.2
    w_hw: float = 1.5
    a_hw: float = 1.6
    alpha_hw: float = 2.0
    # planning / nominal
    v_cruise: float = 1.8  # m/s (hardware limit 2.2; the lidar protective field grows with v^2)
    accel: float = 1.2
    decel: float = 1.2
    e_decel: float = 1.6  # = a_hw: the DiffDrive plugin already brakes this hard, so do not model less
    w_turn: float = 1.0  # rad/s, in-place rotation speed used by the planner
    alpha_turn: float = 1.2
    v_reverse: float = 0.6  # reversing on station spurs and emergency back-offs
    v_station: float = 0.9  # speed on station spurs
    battery_wh: float = 1200.0
    lidars: tuple[LidarMount, ...] = field(default_factory=lambda: (
        LidarMount("lidar_front_right", 0.565, -0.355, -0.7854),
        LidarMount("lidar_rear_left", -0.565, 0.355, 2.3562),
    ))

    @property
    def half_length(self) -> float:
        return self.length / 2

    @property
    def rotation_radius(self) -> float:
        """Radius swept by the body when turning in place about the wheel axle (centre)."""
        return math.hypot(self.length / 2, self.width / 2)

    def footprint(self) -> Footprint:
        return Footprint(self.length, self.width)

    def limits(self) -> MotionLimits:
        return MotionLimits(self.v_cruise, self.accel, self.decel, self.e_decel)

    def motion_model(self) -> DiffDrive:
        return DiffDrive(
            track_width=self.wheel_separation, v_wheel_max=self.v_hw, a_acc=self.a_hw, a_dec=self.a_hw,
            alpha_max=self.alpha_hw, w_max=self.w_hw, v_rev_max=self.v_reverse, D=0.15,
        )

    def robot_spec(self) -> RobotSpec:
        """What the visison bypass planner needs."""
        return RobotSpec(
            footprint=self.footprint(), limits=self.limits(),
            lane_change=QuinticLaneChange(max_lateral_accel=0.35), bypass_speed=0.6,
        )

    def stop_distance(self, v: float, reaction: float = 0.15) -> float:
        return abs(v) * reaction + v * v / (2 * self.e_decel)


AMR01 = AMR01Spec()
