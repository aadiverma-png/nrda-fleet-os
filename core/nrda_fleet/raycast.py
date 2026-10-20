"""CPU 2-D lidar model: ray casting against oriented boxes.

Used by the headless simulator and by ``nrda_fleet_ros/lidar_emulator`` (the
Gazebo ``light`` sensor profile). A laptop GPU cannot render 40 gpu_lidars
over a 24 MB warehouse mesh at 10 Hz, so both produce the AMR-01's two
270-degree scans on the CPU from the same geometry: the site's collision boxes,
the other robots' true footprints and any injected obstacles.
"""

from __future__ import annotations

import math
import random

import numpy as np

from .amr import LidarMount
from .geom import OrientedBox


class BoxSet:
    """Arrays of box parameters for vectorised ray casting."""

    def __init__(self, boxes: list[OrientedBox], names: list[str]):
        self.names = names
        self.cx = np.array([b.cx for b in boxes])
        self.cy = np.array([b.cy for b in boxes])
        self.hl = np.array([b.length / 2 for b in boxes])
        self.hw = np.array([b.width / 2 for b in boxes])
        self.c = np.cos([b.heading for b in boxes])
        self.s = np.sin([b.heading for b in boxes])

    def __len__(self) -> int:
        return len(self.names)


def cast(boxes: BoxSet, ox: float, oy: float, angles: np.ndarray, rmax: float, exclude: str = "",
         noise: float = 0.0, rng: random.Random | None = None) -> np.ndarray:
    """Range along each ray from (ox, oy); ``inf`` where nothing is hit within ``rmax``."""
    n = len(angles)
    out = np.full(n, np.inf)
    if len(boxes) == 0:
        return out
    near = (np.abs(boxes.cx - ox) < rmax + boxes.hl + boxes.hw) & (np.abs(boxes.cy - oy) < rmax + boxes.hl + boxes.hw)
    idx = [i for i in np.flatnonzero(near) if boxes.names[i] != exclude]
    if not idx:
        return out
    I = np.array(idx)
    c, s, hl, hw = boxes.c[I], boxes.s[I], boxes.hl[I], boxes.hw[I]
    px = (ox - boxes.cx[I]) * c + (oy - boxes.cy[I]) * s
    py = -(ox - boxes.cx[I]) * s + (oy - boxes.cy[I]) * c
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    ddx = dx * c + dy * s
    ddy = -dx * s + dy * c
    with np.errstate(divide="ignore", invalid="ignore"):
        t1x = (-hl - px) / ddx
        t2x = (hl - px) / ddx
        t1y = (-hw - py) / ddy
        t2y = (hw - py) / ddy
    tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
    tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
    hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.02)
    tt = np.nan_to_num(np.where(hit, tmin, np.inf), nan=np.inf).min(axis=1)
    tt[tt > rmax] = np.inf
    if noise > 0:
        fin = np.isfinite(tt)
        seed = (rng or random).getrandbits(32)
        tt[fin] += np.random.default_rng(seed).normal(0.0, noise, int(fin.sum()))
    return tt


def scan(boxes: BoxSet, pose: tuple[float, float, float], mount: LidarMount, beams: int, exclude: str = "",
         noise: float = 0.0, rng: random.Random | None = None, rmax: float = 12.0):
    """One scan: (sensor origin, beam angles in world frame, ranges)."""
    x, y, th = pose
    ox = x + math.cos(th) * mount.x - math.sin(th) * mount.y
    oy = y + math.sin(th) * mount.x + math.cos(th) * mount.y
    rel = np.linspace(-mount.fov / 2, mount.fov / 2, beams)
    ang = th + mount.yaw + rel
    return (ox, oy), ang, rel, cast(boxes, ox, oy, ang, min(rmax, mount.range_max), exclude, noise, rng)
