"""Rasterize ground-truth maze geometry (evaluation only)."""

from __future__ import annotations

import numpy as np

from mazemind_simulation.maze import Maze


def free_space_mask(maze: Maze, resolution: float, wall_margin: float = 0.0) -> np.ndarray:
    """Boolean grid of traversable floor inside the maze.

    Index ``[iy, ix]`` covers the square whose centre is
    ``((ix + 0.5) * resolution, (iy + 0.5) * resolution)`` in the world
    frame. A cell is free when its centre lies inside the outer walls and
    farther than ``wall_margin`` from every wall box.
    """
    nx = int(np.ceil(maze.width / resolution))
    ny = int(np.ceil(maze.height / resolution))
    xs = (np.arange(nx) + 0.5) * resolution
    ys = (np.arange(ny) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    free = np.ones((ny, nx), dtype=bool)
    for b in maze.wall_boxes():
        inside = ((np.abs(gx - b.cx) <= b.sx / 2 + wall_margin)
                  & (np.abs(gy - b.cy) <= b.sy / 2 + wall_margin))
        free &= ~inside
    return free
