"""NRDA-FA layer L1: the lidar safety stop, independent of every coordination strategy.

It sees the **raw** lidar returns of both 270-degree scanners (nothing
filtered except the robot's own body) and can only *lower* the command the
MotionArbiter produced:

* **hard-stop zone** (unconditional, the ESPE pattern of IEC 61496-1): any return within
  ``hard_stop`` m of the bumper inside a +-``hard_arc`` cone in the direction of
  travel stops the robot, whatever its speed;

* **protective field**: a rectangle in the direction of travel, of length
  ``reaction * |v| + v^2 / (2 * e_decel) + base`` and width body + 2 x side
  margin. A return inside it means an EMERGENCY brake intent;
* **warning field**: speed is capped so the robot can still stop before the
  nearest return in its path (``v <= sqrt(2 * a * (gap - base))``);
* **rotation guard**: while turning in place, a return inside the swept
  circle stops the rotation.

The hard-stop zone is an unconditional 0.28 m / 24-degree stop; the
speed-scaled fields around it follow the ISO 3691-4 protective-field pattern
(docs/FINAL_ALGORITHM.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from visison.motion_models import Command

from .amr import AMR01Spec


@dataclass
class SafetyOutput:
    cmd: Command
    brake: bool = False
    capped: bool = False
    gap: float = math.inf
    reason: str = ""


@dataclass
class SafetySupervisor:
    amr: AMR01Spec
    base: float = 0.08  # m, always kept free in front of the bumper
    side: float = 0.06  # m, lateral margin of the fields
    reaction: float = 0.15  # s
    decel_cap: float = 0.8  # m/s^2 used for the warning-field speed cap
    hard_stop: float = 0.28  # m, unconditional stop distance from the bumper
    hard_arc: float = math.radians(12)  # half of the 24-degree forward arc
    scan_age: float = 0.1  # s, worst-case age of the newest scan (10 Hz lidars)
    enabled: bool = True

    arc_w: float = 0.05  # rad/s above which the field follows the arc instead of pointing straight ahead

    def _path_gap(self, pts_body: np.ndarray, v: float, w: float, d: float,
                  L: float, W: float) -> float | None:
        """Clear distance along the path the robot is actually on, or ``None`` if nothing is in it.

        Straight ahead this is the old rectangular band. While turning, the field follows the arc the
        command (v, w) describes: a rectangle pointing along the current heading cuts the corner, and at
        1.5 m/s it reaches 1.58 m from the robot's centre while a 3 m-radius turn has already carried the
        path 0.45 m sideways - a whole half-band. Rack faces beside a bend therefore sat inside the
        protective field and braked the robot from cruise with its path completely clear. Live run: three
        such stops with no peer within 10 m.
        """
        if abs(w) <= self.arc_w:
            ahead = pts_body[:, 0] * d - L
            band = (np.abs(pts_body[:, 1]) <= W) & (ahead > -0.02)
            return float(ahead[band].min()) if band.any() else None
        R = v / w  # signed turn radius; the centre of rotation is at (0, R) in the body frame
        rad = np.hypot(pts_body[:, 0], pts_body[:, 1] - R)
        in_corridor = (rad >= abs(R) - W) & (rad <= abs(R) + W)
        if not in_corridor.any():
            return None
        p = pts_body[in_corridor]
        # angle about the centre, from the robot's own position, measured the way it travels (dphi/dt = w)
        phi0 = math.atan2(-R, 0.0)
        sweep = np.mod((np.arctan2(p[:, 1] - R, p[:, 0]) - phi0) * math.copysign(1.0, w), 2 * math.pi)
        ahead = abs(R) * sweep - L
        fwd = ahead > -0.02  # behind the bumper: already passed, as in the straight case
        return float(ahead[fwd].min()) if fwd.any() else None

    def filter(self, cmd: Command, pts_body: np.ndarray, docking: bool = False) -> SafetyOutput:
        """``docking``: the planner is deliberately closing on a station (conveyor, charger tower) at
        station speed, so the fixed clearance shrinks to 2 cm; the speed-dependent part stays."""
        if not self.enabled or pts_body is None or len(pts_body) == 0:
            return SafetyOutput(cmd)
        base = 0.02 if docking else self.base
        L, W = self.amr.length / 2, self.amr.width / 2 + self.side
        v, w = cmd.v, cmd.w
        out = SafetyOutput(cmd)
        if abs(v) > 0.02:
            d = 1.0 if v > 0 else -1.0
            # hard-stop zone: cone from the bumper centre in the direction of travel
            fx = pts_body[:, 0] * d - L
            cone = (fx > -0.02) & (np.abs(np.arctan2(pts_body[:, 1], np.maximum(fx + 0.05, 1e-6))) <= self.hard_arc)
            if not docking and (cone & (np.hypot(np.maximum(fx, 0.0), pts_body[:, 1]) < self.hard_stop)).any():
                out.cmd, out.brake, out.reason = Command(0.0, 0.0), True, "lidar hard stop (0.28 m)"
                return out
            gap = self._path_gap(pts_body, v, w, d, L, W)
            if gap is not None:
                out.gap = gap
                protect = base * 0.6 + self.reaction * abs(v) + v * v / (2 * self.amr.e_decel)
                if gap < protect:
                    out.cmd, out.brake, out.reason = Command(0.0, 0.0), True, f"protective field ({gap:.2f} m)"
                    return out
                v_cap = math.sqrt(max(2 * self.decel_cap * (gap - base), 0.0))
                if abs(v) > v_cap:
                    v = d * v_cap
                    out.capped, out.reason = True, f"warning field ({gap:.2f} m)"
        if abs(w) > 0.05 and abs(v) < 0.15:
            # body swept until the rotation can really be stopped: reaction + scan age + braking from |w| at
            # the hardware angular deceleration. A fixed 0.4 s look-ahead was too short at 1 rad/s (0.5 s
            # to stop alone), so a turn next to a rack could brush it before the guard fired.
            horizon = self.reaction + self.scan_age + abs(w) / self.amr.alpha_hw
            # at least 15 degrees in the turning direction, whatever the commanded rate: a creeping command
            # (0.17 rad/s) swept only a few degrees, so the guard released it tick after tick and the robot
            # inched its corner into a rack
            sweep = max(abs(w) * horizon, math.radians(15.0))
            near = pts_body[np.hypot(pts_body[:, 0], pts_body[:, 1]) <= self.amr.rotation_radius + 0.08]
            if len(near):
                for frac in (0.25, 0.5, 0.75, 1.0):
                    a = -math.copysign(sweep * frac, w)
                    c, s = math.cos(a), math.sin(a)
                    px, py = c * near[:, 0] - s * near[:, 1], s * near[:, 0] + c * near[:, 1]
                    if ((np.abs(px) <= L + 0.04) & (np.abs(py) <= self.amr.width / 2 + 0.04)).any():
                        w = 0.0
                        out.capped, out.reason = True, "rotation guard"
                        break
        if abs(w) > 0.05 and abs(v) >= 0.15:
            # arc guard: driving while turning swings the body's corners outwards (the rear corner of a robot
            # turning away from a rack sweeps towards it); the straight-ahead fields cannot see that. Sweep the
            # body along the predicted arc until it could be stopped; if it would touch a return, drive straight.
            horizon = self.reaction + self.scan_age + max(abs(v) / self.amr.e_decel, abs(w) / self.amr.alpha_hw)
            reach = math.hypot(L, self.amr.width / 2) + abs(v) * horizon + 0.1
            near = pts_body[np.hypot(pts_body[:, 0], pts_body[:, 1]) <= reach]
            if len(near):
                for frac in (0.25, 0.5, 0.75, 1.0):
                    tt = horizon * frac
                    a = w * tt
                    # pose after tt on the arc (unicycle), then the returns in that future body frame
                    px = v / w * math.sin(a)
                    py = v / w * (1.0 - math.cos(a))
                    c, s_ = math.cos(-a), math.sin(-a)
                    qx, qy = near[:, 0] - px, near[:, 1] - py
                    fx, fy = c * qx - s_ * qy, s_ * qx + c * qy
                    inside = (np.abs(fx) <= L + 0.02) & (np.abs(fy) <= self.amr.width / 2 + 0.02)
                    # returns already inside the body now are the robot's own noise / contact: ignore them
                    now_inside = (np.abs(near[:, 0]) <= L) & (np.abs(near[:, 1]) <= self.amr.width / 2)
                    if (inside & ~now_inside).any():
                        w = 0.0
                        out.capped, out.reason = True, "arc guard"
                        break
        out.cmd = Command(v, w)
        return out
