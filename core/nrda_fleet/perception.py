"""Onboard lidar perception (edge AI, NRDA-FA layer L1).

* :func:`scan_points` turns a 270-degree scan (per mount) into world points.
* :class:`StaticMap` is a rasterised copy of the site's collision boxes
  (5 cm cells, dilated) used to drop returns from racks and walls, so only
  *unmapped* things remain: robots, people, dropped pallets.
* :func:`clusters` groups the remaining points (scan order, gap rule, as in
  the former ``visison_gazebo.perception``) into ``(x, y, length, width)``
  measurements for ``visison.emergency.ObjectTracker``.
"""

from __future__ import annotations

import math

import numpy as np

from .site import Site


class StaticMap:
    def __init__(self, site: Site, res: float = 0.05, dilate: float = 0.12):
        self.res = res
        self.x0, self.y0 = -42.0, -27.0
        self.nx, self.ny = int(84 / res), int(54 / res)
        grid = np.zeros((self.ny, self.nx), dtype=bool)
        for nb in site.boxes:
            b = nb.box
            c, s = math.cos(b.heading), math.sin(b.heading)
            hl, hw = b.length / 2 + dilate, b.width / 2 + dilate
            xs, ys = zip(*[(b.cx + i * hl * c - j * hw * s, b.cy + i * hl * s + j * hw * c)
                           for i, j in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
            i0, i1 = self._ix(min(xs)), self._ix(max(xs)) + 1
            j0, j1 = self._iy(min(ys)), self._iy(max(ys)) + 1
            if i1 <= 0 or j1 <= 0 or i0 >= self.nx or j0 >= self.ny:
                continue
            i0, j0 = max(i0, 0), max(j0, 0)
            i1, j1 = min(i1, self.nx), min(j1, self.ny)
            gx = self.x0 + (np.arange(i0, i1) + 0.5) * res
            gy = self.y0 + (np.arange(j0, j1) + 0.5) * res
            X, Y = np.meshgrid(gx, gy)
            lx, ly = c * (X - b.cx) + s * (Y - b.cy), -s * (X - b.cx) + c * (Y - b.cy)
            grid[j0:j1, i0:i1] |= (np.abs(lx) <= hl) & (np.abs(ly) <= hw)
        self.grid = grid

    def _ix(self, x):
        return int((x - self.x0) / self.res)

    def _iy(self, y):
        return int((y - self.y0) / self.res)

    def occupied(self, pts: np.ndarray) -> np.ndarray:
        """Boolean mask: which (N, 2) world points fall on mapped structure."""
        if len(pts) == 0:
            return np.zeros(0, dtype=bool)
        i = ((pts[:, 0] - self.x0) / self.res).astype(int)
        j = ((pts[:, 1] - self.y0) / self.res).astype(int)
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        out = np.ones(len(pts), dtype=bool)
        out[inside] = self.grid[j[inside], i[inside]]
        return out


def scan_points(ranges, angle_min: float, angle_inc: float, rmin: float, rmax: float,
                mount: tuple[float, float, float], pose: tuple[float, float, float]) -> np.ndarray:
    """World (N, 2) points of valid returns, in scan order."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax * 0.999)
    r, a = r[ok], a[ok]
    mx, my, myaw = mount
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    sx, sy = x + c * mx - s * my, y + s * mx + c * my
    ang = th + myaw + a
    return np.column_stack((sx + r * np.cos(ang), sy + r * np.sin(ang)))


def to_body(pts: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    d = pts - np.array([x, y])
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def drop_self(pts_body: np.ndarray, length: float, width: float, pad: float = 0.06) -> np.ndarray:
    """Mask of points outside the robot's own body (lidars can graze the bumper)."""
    if len(pts_body) == 0:
        return np.zeros(0, dtype=bool)
    return ~((np.abs(pts_body[:, 0]) <= length / 2 + pad) & (np.abs(pts_body[:, 1]) <= width / 2 + pad))


def clusters(pts: np.ndarray, gap: float = 0.3, min_points: int = 3) -> list[tuple[float, float, float, float]]:
    """Scan-ordered points -> [(cx, cy, extent_x, extent_y)]."""
    if len(pts) == 0:
        return []
    d = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    breaks = np.flatnonzero(d > gap) + 1
    out = []
    for c in np.split(pts, breaks):
        if len(c) < min_points:
            continue
        x0, y0 = c.min(axis=0)
        x1, y1 = c.max(axis=0)
        out.append(((x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, 0.05), max(y1 - y0, 0.05)))
    return out
