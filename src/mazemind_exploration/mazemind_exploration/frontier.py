"""Frontier detection: boundaries between known-free and unknown space.

A frontier cell is a *free* cell with at least one 4-connected *unknown*
neighbour (cells outside the map array count as unknown). Frontier cells
are grouped into clusters with 8-connectivity; clusters smaller than
``min_size`` cells are discarded as sensor noise / unobservable slivers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

from mazemind_exploration.grid import GridMap, nearest_cell_in_mask


@dataclass
class Frontier:
    id: int
    cells: np.ndarray                 # (N, 2) int rows/cols
    centroid: Tuple[float, float]     # world (map frame) x, y
    target_cell: Tuple[int, int]      # where the robot is sent
    target: Tuple[float, float]       # world x, y of target_cell
    kind: str = 'map'                 # 'map' (LiDAR frontier) | 'visual' (camera-unseen region)

    @property
    def size(self) -> int:
        return int(self.cells.shape[0])


def frontier_mask(grid: GridMap) -> np.ndarray:
    unknown = np.pad(grid.unknown, 1, constant_values=True)
    near_unknown = (unknown[:-2, 1:-1] | unknown[2:, 1:-1]
                    | unknown[1:-1, :-2] | unknown[1:-1, 2:])
    return grid.free & near_unknown


def detect_frontiers(grid: GridMap, min_size: int,
                     traversable: Optional[np.ndarray] = None,
                     snap_radius_cells: int = 8) -> List[Frontier]:
    """Cluster frontier cells and choose one reachable-looking target per cluster.

    The target is the cluster cell closest to the cluster centroid (so it lies
    on the frontier even for L-shaped clusters). If ``traversable`` is given and
    that cell is not traversable (e.g. hugging a wall), the target is snapped to
    the nearest traversable cell within ``snap_radius_cells``; clusters with no
    such cell are dropped.
    """
    mask = frontier_mask(grid)
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if count == 0:
        return []
    flat_labels = labels.ravel()
    order = np.argsort(flat_labels, kind='stable')
    sorted_labels = flat_labels[order]
    starts = np.searchsorted(sorted_labels, np.arange(1, count + 1), side='left')
    ends = np.searchsorted(sorted_labels, np.arange(1, count + 1), side='right')

    frontiers: List[Frontier] = []
    for k, (s, e) in enumerate(zip(starts, ends), start=1):
        if e - s < min_size:
            continue
        flat = order[s:e]
        rows, cols = np.unravel_index(flat, grid.shape)
        cells = np.stack([rows, cols], axis=1)
        cr, cc = rows.mean(), cols.mean()
        i = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
        target_cell: Optional[Tuple[int, int]] = (int(rows[i]), int(cols[i]))
        if traversable is not None and not traversable[target_cell]:
            # prefer another cluster cell that is traversable, then any nearby traversable cell
            ok = traversable[rows, cols]
            if ok.any():
                j = np.flatnonzero(ok)[np.argmin(((rows[ok] - cr) ** 2 + (cols[ok] - cc) ** 2))]
                target_cell = (int(rows[j]), int(cols[j]))
            else:
                target_cell = nearest_cell_in_mask(traversable, target_cell, snap_radius_cells)
            if target_cell is None:
                continue
        frontiers.append(Frontier(
            id=k,
            cells=cells,
            centroid=grid.cell_to_world(cr, cc),
            target_cell=target_cell,
            target=grid.cell_to_world(*target_cell),
        ))
    return frontiers


def visible_unknown_cells(grid: GridMap, cell: Tuple[int, int], radius_m: float,
                          n_rays: int = 72) -> int:
    """Information-gain estimate: unknown cells visible from ``cell``.

    Casts ``n_rays`` rays out to ``radius_m``; each ray stops at the first
    occupied cell. Unknown cells hit by at least one ray are counted once.
    Unknown cells hidden behind known walls therefore do not count, which
    matters in mazes where the space behind a wall is a different corridor.
    """
    steps = max(1, int(radius_m / grid.resolution))
    angles = np.linspace(0.0, 2 * np.pi, n_rays, endpoint=False)
    t = np.arange(1, steps + 1)
    dr = np.rint(np.outer(np.sin(angles), t)).astype(np.int64)
    dc = np.rint(np.outer(np.cos(angles), t)).astype(np.int64)
    rr = cell[0] + dr
    cc = cell[1] + dc
    rows, cols = grid.shape
    inside = (rr >= 0) & (rr < rows) & (cc >= 0) & (cc < cols)
    vals = np.full(rr.shape, -1, dtype=np.int16)
    vals[inside] = grid.data[rr[inside], cc[inside]]
    blocked = vals >= grid.occupied_threshold
    # cells strictly before the first obstacle on each ray are visible
    first_block = np.where(blocked.any(axis=1), blocked.argmax(axis=1), steps)
    visible = t[None, :] - 1 < first_block[:, None]
    # cells outside the map array have never been observed: they count as unknown
    unknown_visible = visible & (vals < 0)
    if not unknown_visible.any():
        return 0
    coords = np.stack([rr[unknown_visible], cc[unknown_visible]], axis=1)
    return int(np.unique(coords, axis=0).shape[0])
