"""Ground-truth evaluation metrics (pure Python / numpy, unit-tested).

These classes are the ONLY place where ground-truth maze geometry is
compared with what the robot built. They are used by the evaluator node,
never by the exploration stack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from mazemind_simulation.maze import Maze, WallBox
from mazemind_simulation.rasterize import free_space_mask


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def map_to_world(start: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """SLAM map frame -> world frame.

    SLAM Toolbox anchors the map frame at the first scan pose, which is the
    odometry origin, i.e. the robot's spawn pose in the world.
    """
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return start.x + c * x - s * y, start.y + s * x + c * y


def world_to_map(start: Pose2D, x: float, y: float) -> Tuple[float, float]:
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    dx, dy = x - start.x, y - start.y
    return c * dx + s * dy, -s * dx + c * dy


class CoverageEvaluator:
    """Fraction of the maze's traversable floor that the SLAM map marks as free.

    Procedure:
      1. Rasterise ground-truth free floor at the SLAM resolution.
      2. Erode it by ``erode_cells`` so cells straddling a wall edge (where a
         correct map may legitimately say "occupied") are not counted.
      3. Transform each remaining cell centre into the SLAM map frame using
         the spawn pose, look up the SLAM cell, and count it as covered when
         the SLAM value is known-free (0 <= v < free_threshold).
    """

    def __init__(self, maze: Maze, start: Pose2D, resolution: float,
                 erode_cells: int = 1, free_threshold: int = 25):
        self.resolution = resolution
        self.free_threshold = free_threshold
        free = free_space_mask(maze, resolution)
        if erode_cells > 0:
            free = ndimage.binary_erosion(free, iterations=erode_cells)
        iy, ix = np.nonzero(free)
        wx = (ix + 0.5) * resolution
        wy = (iy + 0.5) * resolution
        c, s = math.cos(start.yaw), math.sin(start.yaw)
        dx, dy = wx - start.x, wy - start.y
        self.map_x = c * dx + s * dy
        self.map_y = -s * dx + c * dy
        self.total = int(iy.size)

    def coverage(self, data: np.ndarray, resolution: float, origin_x: float, origin_y: float) -> float:
        if self.total == 0:
            return 0.0
        col = np.floor((self.map_x - origin_x) / resolution).astype(np.int64)
        row = np.floor((self.map_y - origin_y) / resolution).astype(np.int64)
        inside = (row >= 0) & (row < data.shape[0]) & (col >= 0) & (col < data.shape[1])
        vals = np.full(self.total, -1, dtype=np.int16)
        vals[inside] = data[row[inside], col[inside]]
        covered = (vals >= 0) & (vals < self.free_threshold)
        return float(covered.sum()) / self.total


class DistanceAccumulator:
    """Path length from successive positions, ignoring sub-``min_step`` jitter."""

    def __init__(self, min_step: float = 1e-3):
        self.min_step = min_step
        self.total = 0.0
        self._last: Optional[Tuple[float, float]] = None

    def update(self, x: float, y: float) -> None:
        if self._last is None:
            self._last = (x, y)
            return
        d = math.hypot(x - self._last[0], y - self._last[1])
        if d >= self.min_step:
            self.total += d
            self._last = (x, y)


class ContactCounter:
    """Counts physical contact *events* from the simulator's contact sensor.

    A message containing at least one contact whose collision names do not
    involve an ignored entity (the ground plane) marks the robot as touching
    something. A new event starts when contact resumes after more than
    ``debounce`` seconds without contact.
    """

    def __init__(self, debounce: float = 0.5, ignore: Sequence[str] = ('ground_plane',)):
        self.debounce = debounce
        self.ignore = tuple(ignore)
        self.events = 0
        self.contact_time = 0.0
        self.partners: set = set()
        self._last: Optional[float] = None

    def update(self, stamp: float, pairs: Sequence[Tuple[str, str]]) -> None:
        relevant = [(a, b) for a, b in pairs
                    if not any(tag in a or tag in b for tag in self.ignore)]
        if not relevant:
            return
        for a, b in relevant:
            self.partners.add(b if 'mazemind' in a else a)
        if self._last is None or stamp - self._last > self.debounce:
            self.events += 1
        elif stamp > self._last:
            self.contact_time += stamp - self._last
        self._last = stamp


class NearCollisionTracker:
    """Events where true clearance (robot body edge to nearest wall) drops below ``threshold``.

    Clearance = min distance from the robot centre to any wall box minus the
    body radius. An event is counted on each downward crossing; the tracker
    re-arms only after clearance exceeds ``threshold + hysteresis``.
    """

    def __init__(self, walls: Sequence[WallBox], body_radius: float, threshold: float = 0.05,
                 hysteresis: float = 0.02):
        self.walls = list(walls)
        self.body_radius = body_radius
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.events = 0
        self.min_clearance = math.inf
        self._armed = True

    def clearance(self, x: float, y: float) -> float:
        return min(w.distance(x, y) for w in self.walls) - self.body_radius

    def update(self, x: float, y: float) -> float:
        c = self.clearance(x, y)
        self.min_clearance = min(self.min_clearance, c)
        if self._armed and c < self.threshold:
            self.events += 1
            self._armed = False
        elif not self._armed and c > self.threshold + self.hysteresis:
            self._armed = True
        return c


def summarize(values: Sequence[float]) -> dict:
    """n / mean / median / std / min / max of finite values (empty -> n = 0)."""
    arr = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    if arr.size == 0:
        return {'n': 0, 'mean': None, 'median': None, 'std': None, 'min': None, 'max': None}
    return {'n': int(arr.size), 'mean': float(arr.mean()), 'median': float(np.median(arr)),
            'std': float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
            'min': float(arr.min()), 'max': float(arr.max())}


def wilson_interval(successes: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """95 % Wilson score interval for a success rate (robust for small n)."""
    if n == 0:
        return (0.0, 0.0)
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))
