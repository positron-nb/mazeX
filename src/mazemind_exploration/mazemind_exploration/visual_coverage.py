"""Camera coverage bookkeeping for visual search.

The exit marker can only be recognised by the camera, whose field of view
(~75 deg, ~3.5 m useful range) is far narrower than the 360-deg, 8 m LiDAR.
LiDAR-based frontiers therefore disappear long before the camera has looked
at every corridor. This module tracks which SLAM cells the camera has
actually observed, so the explorer can also target sizeable *unseen*
traversable regions ("visual frontiers").

Seen cells are stored in a world-anchored set of integer cell keys, so the
record survives SLAM map growth (which moves the map origin).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

from mazemind_exploration.grid import GridMap

_OFFSET = 1 << 20   # keeps world cell indices positive


class VisualCoverage:

    def __init__(self, resolution: float, view_range: float = 3.5, hfov: float = 1.3,
                 n_rays: int = 48):
        self.resolution = resolution
        self.view_range = view_range
        self.hfov = hfov
        self.n_rays = n_rays
        self._keys: set = set()

    def __len__(self) -> int:
        return len(self._keys)

    def _world_index(self, grid: GridMap, rows: np.ndarray, cols: np.ndarray):
        gx = np.floor((grid.origin_x + (cols + 0.5) * grid.resolution) / self.resolution).astype(np.int64)
        gy = np.floor((grid.origin_y + (rows + 0.5) * grid.resolution) / self.resolution).astype(np.int64)
        return (gx + _OFFSET) * (2 * _OFFSET) + (gy + _OFFSET)

    def observe(self, grid: GridMap, x: float, y: float, yaw: float) -> int:
        """Mark known cells inside the camera frustum up to the first obstacle as seen.

        Rays stop at the first occupied cell, which is marked (the camera sees
        the wall face). They pass through unknown cells without marking them:
        inside LiDAR range these are slivers between diverging LiDAR rays, not
        hidden structure, because the 8 m LiDAR maps every wall the 3.5 m camera
        could see. Returns the number of newly seen cells.
        """
        r0, c0 = grid.world_to_cell(x, y)
        steps = max(1, int(self.view_range / grid.resolution))
        angles = yaw + np.linspace(-self.hfov / 2, self.hfov / 2, self.n_rays)
        t = np.arange(0, steps + 1)
        rr = np.rint(r0 + np.outer(np.sin(angles), t)).astype(np.int64)
        cc = np.rint(c0 + np.outer(np.cos(angles), t)).astype(np.int64)
        rows, cols = grid.shape
        inside = (rr >= 0) & (rr < rows) & (cc >= 0) & (cc < cols)
        vals = np.full(rr.shape, -1, dtype=np.int16)
        vals[inside] = grid.data[rr[inside], cc[inside]]
        occupied = vals >= grid.occupied_threshold
        known = vals >= 0
        first = np.where(occupied.any(axis=1), occupied.argmax(axis=1), steps + 1)
        visible = t[None, :] <= first[:, None]        # includes the wall cell that stops the ray
        mark = visible & known & inside
        if not mark.any():
            return 0
        keys = self._world_index(grid, rr[mark], cc[mark])
        before = len(self._keys)
        self._keys.update(keys.tolist())
        return len(self._keys) - before

    def mask(self, grid: GridMap) -> np.ndarray:
        """Boolean array (grid shape) of cells the camera has seen."""
        out = np.zeros(grid.shape, dtype=bool)
        if not self._keys:
            return out
        keys = np.fromiter(self._keys, dtype=np.int64, count=len(self._keys))
        gx = keys // (2 * _OFFSET) - _OFFSET
        gy = keys % (2 * _OFFSET) - _OFFSET
        col = np.floor(((gx + 0.5) * self.resolution - grid.origin_x) / grid.resolution).astype(np.int64)
        row = np.floor(((gy + 0.5) * self.resolution - grid.origin_y) / grid.resolution).astype(np.int64)
        ok = (row >= 0) & (row < grid.shape[0]) & (col >= 0) & (col < grid.shape[1])
        out[row[ok], col[ok]] = True
        return out


def unseen_regions(traversable: np.ndarray, seen: np.ndarray, min_cells: int
                   ) -> List[Tuple[np.ndarray, Tuple[int, int]]]:
    """Connected traversable regions the camera has not seen.

    Returns ``(cells, target_cell)`` per region with at least ``min_cells``
    cells; the target is the region cell closest to the region centroid.
    """
    unseen = traversable & ~seen
    labels, count = ndimage.label(unseen, structure=np.ones((3, 3), dtype=bool))
    out = []
    if count == 0:
        return out
    sizes = ndimage.sum(unseen, labels, index=np.arange(1, count + 1))
    for k in np.flatnonzero(sizes >= min_cells) + 1:
        rows, cols = np.nonzero(labels == k)
        cr, cc = rows.mean(), cols.mean()
        i = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
        out.append((np.stack([rows, cols], axis=1), (int(rows[i]), int(cols[i]))))
    return out


def info_grid_with_unseen(grid: GridMap, unseen_mask: Optional[np.ndarray]) -> GridMap:
    """Copy of ``grid`` where camera-unseen cells count as unknown (for information gain)."""
    if unseen_mask is None or not unseen_mask.any():
        return grid
    data = grid.data.copy()
    data[unseen_mask] = -1
    return GridMap(data, grid.resolution, grid.origin_x, grid.origin_y,
                   grid.free_threshold, grid.occupied_threshold)
