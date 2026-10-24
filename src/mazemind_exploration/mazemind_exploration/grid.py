"""Occupancy-grid helpers built on the live SLAM map (never on ground truth)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

UNKNOWN = -1


@dataclass
class GridMap:
    """Row-major occupancy grid: ``data[row, col]`` with row = y index, col = x index.

    Values follow nav_msgs/OccupancyGrid: -1 unknown, 0..100 occupancy probability.
    The map origin is assumed to be unrotated (true for SLAM Toolbox).
    """

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    free_threshold: int = 25
    occupied_threshold: int = 65

    @classmethod
    def from_msg(cls, msg, free_threshold: int = 25, occupied_threshold: int = 65) -> 'GridMap':
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(data, float(info.resolution), float(info.origin.position.x),
                   float(info.origin.position.y), free_threshold, occupied_threshold)

    @property
    def shape(self) -> Tuple[int, int]:
        return self.data.shape

    @property
    def unknown(self) -> np.ndarray:
        return self.data < 0

    @property
    def free(self) -> np.ndarray:
        return (self.data >= 0) & (self.data < self.free_threshold)

    @property
    def occupied(self) -> np.ndarray:
        return self.data >= self.occupied_threshold

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def cell_to_world(self, row: float, col: float) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)


@dataclass
class Traversability:
    """Where the robot centre may go, and how far each cell is from obstacles."""

    mask: np.ndarray            # bool, traversable cells
    clearance: np.ndarray       # metres from each cell to the nearest occupied cell


def compute_traversability(grid: GridMap, robot_radius: float, gap_fill_cells: int = 2,
                           border_cells: int = 0) -> Traversability:
    """Traversable = (known free, or unknown within ``gap_fill_cells`` of known free)
    and farther than ``robot_radius`` from every occupied cell.

    The gap fill bridges the unobserved slivers between diverging LiDAR rays
    at long range, which otherwise split one corridor into disconnected free
    islands. It only reaches a few cells into unknown space, so paths cannot
    wander through unexplored regions. ``border_cells`` excludes the outer rim
    of the map array (the global costmap may lag the SLAM map by a cell).
    """
    occupied = grid.occupied
    if occupied.any():
        clearance = ndimage.distance_transform_edt(~occupied) * grid.resolution
    else:
        clearance = np.full(grid.shape, np.inf)
    free = grid.free
    if gap_fill_cells > 0:
        free = free | (grid.unknown & ndimage.binary_dilation(free, iterations=gap_fill_cells))
    mask = free & (clearance > robot_radius)
    if border_cells > 0:
        mask[:border_cells, :] = False
        mask[-border_cells:, :] = False
        mask[:, :border_cells] = False
        mask[:, -border_cells:] = False
    return Traversability(mask=mask, clearance=clearance)


class GeodesicField:
    """Travel costs (metres) over traversable cells, 8-connected, via compiled Dijkstra.

    With ``clearance_penalty > 0`` each step costs
    ``length * (1 + clearance_penalty * p)``, where ``p`` in [0, 1] grows as the
    cells' clearance drops from ``safe_clearance`` to ``robot_radius``. Paths then
    keep to corridor centres the way Nav2's inflation-aware planner does, instead
    of hugging corners. A pure shortest path touches every corner at exactly the
    robot radius, which made the path-bottleneck feature meaningless.
    """

    def __init__(self, trav: Traversability, resolution: float, source: Tuple[int, int],
                 clearance_penalty: float = 0.0, robot_radius: float = 0.15,
                 safe_clearance: float = 0.35):
        mask = trav.mask
        self.shape = mask.shape
        self.resolution = resolution
        self.clearance = trav.clearance
        rows, cols = self.shape
        index = -np.ones(self.shape, dtype=np.int64)
        flat = np.flatnonzero(mask)
        index.flat[flat] = np.arange(flat.size)
        self._index = index
        self._cells = flat
        self.distance = np.full(self.shape, np.inf)
        self._pred = None
        self.source = source
        if flat.size == 0 or index[source] < 0:
            return

        if clearance_penalty > 0 and safe_clearance > robot_radius:
            pen = np.clip((safe_clearance - trav.clearance) / (safe_clearance - robot_radius), 0.0, 1.0)
        else:
            pen = np.zeros(self.shape)
        src, dst, w = [], [], []
        for dr, dc, cost in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
            # cells (r, c) whose neighbour (r + dr, c + dc) is inside the grid
            r_lo, r_hi = max(0, -dr), rows - max(0, dr)
            c_lo, c_hi = max(0, -dc), cols - max(0, dc)
            a = mask[r_lo:r_hi, c_lo:c_hi]
            b = mask[r_lo + dr:r_hi + dr, c_lo + dc:c_hi + dc]
            ra, ca = np.nonzero(a & b)
            ra = ra + r_lo
            ca = ca + c_lo
            src.append(index[ra, ca])
            dst.append(index[ra + dr, ca + dc])
            step_pen = 0.5 * (pen[ra, ca] + pen[ra + dr, ca + dc])
            w.append(cost * resolution * (1.0 + clearance_penalty * step_pen))
        src = np.concatenate(src)
        dst = np.concatenate(dst)
        w = np.concatenate(w)
        n = flat.size
        graph = csr_matrix((w, (src, dst)), shape=(n, n))
        dist, pred = dijkstra(graph, directed=False, indices=int(index[source]),
                              return_predecessors=True)
        self.distance.flat[flat] = dist
        self._pred = pred

    def reachable(self, cell: Tuple[int, int]) -> bool:
        return bool(np.isfinite(self.distance[cell]))

    def path(self, cell: Tuple[int, int]) -> np.ndarray:
        """Cells (row, col) from the source to ``cell`` (empty if unreachable)."""
        if self._pred is None or not self.reachable(cell):
            return np.empty((0, 2), dtype=np.int64)
        node = int(self._index[cell])
        out = []
        while node >= 0:
            out.append(self._cells[node])
            node = int(self._pred[node])
        flat = np.array(out[::-1], dtype=np.int64)
        return np.stack(np.unravel_index(flat, self.shape), axis=1)

    def bottleneck_clearance(self, cell: Tuple[int, int]) -> float:
        """Smallest obstacle clearance along the shortest path to ``cell``."""
        p = self.path(cell)
        if p.size == 0:
            return 0.0
        return float(self.clearance[p[:, 0], p[:, 1]].min())


def nearest_cell_in_mask(mask: np.ndarray, cell: Tuple[int, int], max_radius_cells: int
                         ) -> Optional[Tuple[int, int]]:
    """Closest True cell of ``mask`` to ``cell`` within a square search window."""
    r, c = cell
    rows, cols = mask.shape
    r0, r1 = max(0, r - max_radius_cells), min(rows, r + max_radius_cells + 1)
    c0, c1 = max(0, c - max_radius_cells), min(cols, c + max_radius_cells + 1)
    if r0 >= r1 or c0 >= c1:
        return None
    rr, cc = np.nonzero(mask[r0:r1, c0:c1])
    if rr.size == 0:
        return None
    rr = rr + r0
    cc = cc + c0
    d2 = (rr - r) ** 2 + (cc - c) ** 2
    k = int(np.argmin(d2))
    if d2[k] > max_radius_cells ** 2:
        return None
    return int(rr[k]), int(cc[k])
