"""Time-parameterised world-frame trajectories for a differential-drive AMR.

A :class:`PathTrajectory` is a chain of primitives a diff-drive robot can
follow exactly:

* ``move``   straight line, forward or reverse, trapezoidal speed profile;
* ``shift``  straight chord plus a quintic lateral offset (docking into an
             alcove), forward or reverse, from rest to rest;
* ``rotate`` turn in place, trapezoidal angular profile;
* ``wait``   stand still.

``sample(t)`` returns a :class:`visison.planner.Pose` whose ``s``/``y`` fields
carry world ``x``/``y``. That is exactly what
:func:`visison.tracking.reference` and :class:`visison.arbiter.MotionArbiter`
consume, so the arbiter tracks these like visison's own trajectories.

:class:`FramedTrajectory` wraps an aisle-frame visison ``Trajectory`` (a
bypass plan) and maps it into the world frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator, Sequence

from visison.kinematics import EPS, MotionLimits, Profile, trapezoid
from visison.planner import Pose

from .geom import wrap

TURN_TOL = math.radians(4)


def _quintic(x: float) -> tuple[float, float]:
    x = min(max(x, 0.0), 1.0)
    return x**3 * (10 - 15 * x + 6 * x * x), 30 * x * x * (1 - x) ** 2


@dataclass
class Prim:
    kind: str  # move | shift | rotate | wait
    t0: float
    t1: float
    x0: float
    y0: float
    th0: float  # body heading at the start
    x1: float = 0.0  # end of the chord (move/shift)
    y1: float = 0.0
    th1: float = 0.0  # body heading at the end
    reverse: bool = False
    profile: Profile | None = None  # distance along the chord, or |angle| for rotate
    length: float = 0.0
    offset: float = 0.0  # shift: lateral offset at the end, to the left of the chord
    u0: float = 0.0  # cumulative travelled distance at the start
    tag: str = ""

    def pose(self, t: float) -> tuple[float, float, float, float, float]:
        """(x, y, heading, signed v, u) at time ``t`` (clamped to this primitive)."""
        t = min(max(t, self.t0), self.t1)
        if self.kind == "wait":
            return self.x0, self.y0, self.th0, 0.0, self.u0
        if self.kind == "rotate":
            a, _ = self.profile.state(t)
            sgn = 1.0 if wrap(self.th1 - self.th0) >= 0 else -1.0
            return self.x0, self.y0, wrap(self.th0 + sgn * min(a, self.length)), 0.0, self.u0
        u, v = self.profile.state(t)
        u = min(u, self.length)
        L = max(self.length, EPS)
        ex, ey = (self.x1 - self.x0) / L, (self.y1 - self.y0) / L
        x, y = self.x0 + ex * u, self.y0 + ey * u
        dx, dy = ex, ey
        if self.kind == "shift":
            f, df = _quintic(u / L)
            nx, ny = -ey, ex
            x += nx * self.offset * f
            y += ny * self.offset * f
            dx, dy = ex + nx * self.offset * df / L, ey + ny * self.offset * df / L
        heading = wrap(math.atan2(dy, dx) + (math.pi if self.reverse else 0.0))
        return x, y, heading, (-v if self.reverse else v), self.u0 + u


class PathTrajectory:
    """A chain of primitives. After the last one the robot stands still."""

    turns_in_place = True  # heading is defined at rest (see visison.tracking.reference)

    def __init__(self, prims: Sequence[Prim]):
        if not prims:
            raise ValueError("empty trajectory")
        self.prims: list[Prim] = list(prims)

    # visison trajectory interface
    def sample(self, t: float) -> Pose:
        x, y, th, v, _ = self.state(t)
        return Pose(t, x, y, v, th)

    @property
    def t_start(self) -> float:
        return self.prims[0].t0

    @property
    def t_end(self) -> float:
        return self.prims[-1].t1

    @property
    def length(self) -> float:
        p = self.prims[-1]
        return p.u0 + (p.length if p.kind in ("move", "shift") else 0.0)

    def end_pose(self) -> tuple[float, float, float]:
        x, y, th, _, _ = self.prims[-1].pose(self.prims[-1].t1)
        return x, y, th

    def state(self, t: float) -> tuple[float, float, float, float, float]:
        """(x, y, heading, signed v, u) where u is the distance travelled since the start."""
        return self.primitive_at(t).pose(t)

    def primitive_at(self, t: float) -> Prim:
        for p in self.prims:
            if t <= p.t1:
                return p
        return self.prims[-1]

    def samples(self, dt: float = 0.1, t_from: float | None = None,
                t_to: float | None = None) -> Iterator[tuple[float, float, float, float, float]]:
        """(t, x, y, heading, v) every ``dt``, always including both ends. Rotations are sampled finer."""
        t0 = self.t_start if t_from is None else max(t_from, self.t_start)
        t1 = self.t_end if t_to is None else min(t_to, self.t_end)
        t = t0
        while t < t1 - 1e-9:
            x, y, th, v, _ = self.state(t)
            yield t, x, y, th, v
            step = dt * 0.5 if self.primitive_at(t).kind == "rotate" else dt
            t += step
        x, y, th, v, _ = self.state(t1)
        yield t1, x, y, th, v


@dataclass
class Leg:
    """One geometric step of a route, before timing."""

    kind: str  # move | shift | rotate | wait
    x1: float = 0.0  # move/shift: final position
    y1: float = 0.0
    th1: float = 0.0  # rotate: target heading; shift: direction of travel of the chord
    reverse: bool = False
    v_max: float | None = None
    duration: float = 0.0  # wait
    tag: str = ""


@dataclass
class Builder:
    """Times a list of :class:`Leg` into a :class:`PathTrajectory`.

    Consecutive ``move`` legs with the same body heading and direction are
    driven through without stopping. Rotations, waits, shifts and direction
    reversals start and end at rest. The trajectory always ends at rest.
    """

    limits: MotionLimits
    w_turn: float = 0.8
    alpha_turn: float = 1.2

    def build(self, t0: float, x: float, y: float, th: float, v0: float,
              legs: Sequence[Leg], w0: float = 0.0) -> PathTrajectory:
        legs = [lg for lg in legs if not (lg.kind == "wait" and lg.duration <= EPS)]
        prims: list[Prim] = []
        t, u, v = t0, 0.0, max(v0, 0.0)
        cx, cy, cth = x, y, th
        for i, lg in enumerate(legs):
            if lg.kind == "wait":
                t, u, cx, cy, v = self._stop(prims, t, u, cx, cy, cth, v)
                prims.append(Prim("wait", t, t + lg.duration, cx, cy, cth, cx, cy, cth, u0=u, tag=lg.tag))
                t += lg.duration
                continue
            if lg.kind == "rotate":
                t, u, cx, cy, v = self._stop(prims, t, u, cx, cy, cth, v)
                t = self._rotate(prims, t, u, cx, cy, cth, lg.th1, lg.tag, w0 if not prims else 0.0)
                cth = lg.th1
                continue
            L = math.hypot(lg.x1 - cx, lg.y1 - cy)
            if L < 1e-4:
                continue
            if lg.kind == "shift":
                travel = lg.th1
                ex, ey = math.cos(travel), math.sin(travel)
                along = (lg.x1 - cx) * ex + (lg.y1 - cy) * ey
                offset = -(lg.x1 - cx) * ey + (lg.y1 - cy) * ex
                if along < 0:  # chord points the other way: flip the travel direction
                    travel, along, offset = wrap(travel + math.pi), -along, -offset
                    ex, ey = -ex, -ey
                body = wrap(travel + (math.pi if lg.reverse else 0.0))
                t, u, cx, cy, v = self._stop(prims, t, u, cx, cy, cth, v)
                if abs(wrap(body - cth)) > TURN_TOL:
                    t = self._rotate(prims, t, u, cx, cy, cth, body, "align")
                vcap = min(lg.v_max or self.limits.v_max, self.limits.v_max)
                prof = self._profile(t, 0.0, along, vcap, 0.0)
                prims.append(Prim("shift", t, prof.t_end, cx, cy, body, cx + ex * along, cy + ey * along,
                                  body, lg.reverse, prof, along, offset, u, lg.tag))
                t, u, v = prof.t_end, u + along, 0.0
                cx, cy, cth, _, _ = prims[-1].pose(prims[-1].t1)
                continue
            # move
            travel = math.atan2(lg.y1 - cy, lg.x1 - cx)
            body = wrap(travel + (math.pi if lg.reverse else 0.0))
            if abs(wrap(body - cth)) > TURN_TOL:
                t, u, cx, cy, v = self._stop(prims, t, u, cx, cy, cth, v)
                t = self._rotate(prims, t, u, cx, cy, cth, body, "align")
                L = math.hypot(lg.x1 - cx, lg.y1 - cy)
            vcap = min(lg.v_max or self.limits.v_max, self.limits.v_max)
            nxt = legs[i + 1] if i + 1 < len(legs) else None
            v_end = 0.0
            if nxt is not None and nxt.kind == "move" and nxt.reverse == lg.reverse:
                n_travel = math.atan2(nxt.y1 - lg.y1, nxt.x1 - lg.x1)
                if abs(wrap(n_travel - travel)) < TURN_TOL:
                    v_end = min(vcap, nxt.v_max or self.limits.v_max, self.limits.v_max)
            prof = self._profile(t, v, L, vcap, v_end)
            prims.append(Prim("move", t, prof.t_end, cx, cy, body, lg.x1, lg.y1, body, lg.reverse, prof, L,
                              0.0, u, lg.tag))
            t, u, v = prof.t_end, u + L, prof.v_end
            cx, cy, cth = lg.x1, lg.y1, body
        t, u, cx, cy, v = self._stop(prims, t, u, cx, cy, cth, v)
        if not prims:
            prims.append(Prim("wait", t0, t0 + 1e-3, x, y, th, x, y, th))
        return PathTrajectory(prims)

    # helpers ------------------------------------------------------------------------

    def _profile(self, t: float, v: float, L: float, vcap: float, v_end: float) -> Profile:
        a, d = self.limits.accel, self.limits.decel
        v = min(v, max(vcap, v))
        if v_end > v and v_end**2 - v**2 > 2 * a * L:
            v_end = math.sqrt(v * v + 2 * a * L)
        if v > v_end and v * v - v_end * v_end > 2 * d * L:
            d = (v * v - v_end * v_end) / (2 * L) + 1e-6  # brake harder than nominal (bounded by e-decel in practice)
        prof = Profile(t, 0.0, v)
        for acc, dt in trapezoid(L, v, v_end, max(vcap, v, v_end), a, d):
            prof._push(acc, dt)  # noqa: SLF001  (visison Profile has no public "append segment")
        return prof

    def _rotate(self, prims, t, u, x, y, th0, th1, tag="", w0: float = 0.0) -> float:
        ang = abs(wrap(th1 - th0))
        if ang < math.radians(0.5):
            return t
        sgn = 1.0 if wrap(th1 - th0) >= 0 else -1.0
        wstart = max(0.0, min(w0 * sgn, self.w_turn))  # continue a turn already in progress
        lim = MotionLimits(self.w_turn, self.alpha_turn, self.alpha_turn)
        if wstart * wstart > 2 * self.alpha_turn * ang:
            wstart = math.sqrt(2 * self.alpha_turn * ang)
        prof = Profile(t, 0.0, wstart).move(ang, lim)
        prims.append(Prim("rotate", t, prof.t_end, x, y, th0, x, y, wrap(th1), profile=prof, length=ang,
                          u0=u, tag=tag))
        return prof.t_end

    def _stop(self, prims, t, u, x, y, th, v):
        """Bring the robot to rest along its heading (only needed if a caller forgot to end a leg at rest)."""
        if v <= EPS:
            return t, u, x, y, 0.0
        prof = Profile(t, 0.0, v).change_speed(0.0, self.limits.accel, self.limits.decel)
        d = prof.u_end
        ex, ey = math.cos(th), math.sin(th)
        prims.append(Prim("move", t, prof.t_end, x, y, th, x + ex * d, y + ey * d, th, False, prof, d, 0.0, u))
        return prof.t_end, u + d, x + ex * d, y + ey * d, 0.0


class FramedTrajectory:
    """An aisle-frame visison ``Trajectory`` in world coordinates.

    ``world = origin + s * t_hat + y * n_hat`` where ``t_hat`` points at ``theta``
    and ``n_hat`` is ``t_hat`` rotated by +90 degrees.
    """

    def __init__(self, traj, origin: tuple[float, float], theta: float):
        self.traj, self.origin, self.theta = traj, origin, theta
        self._c, self._s = math.cos(theta), math.sin(theta)

    def to_world(self, s: float, y: float) -> tuple[float, float]:
        return (self.origin[0] + self._c * s - self._s * y, self.origin[1] + self._s * s + self._c * y)

    def sample(self, t: float) -> Pose:
        p = self.traj.sample(t)
        x, y = self.to_world(p.s, p.y)
        return Pose(t, x, y, p.v, wrap(p.heading + self.theta))

    def lateral(self, t: float) -> float:
        return self.traj.lateral(t)

    @property
    def profile(self):
        return self.traj.profile
