"""Offline integration test of the full frontier pipeline (no ROS, no Gazebo).

A generated maze is rasterised; an idealised 360-degree range sensor
ray-casts into it to build an occupancy grid incrementally; the robot is
moved along the GeodesicField path to each selected target. Every
strategy must explore (almost) all free space and only ever travel through
cells that are traversable in the *built* map.

Ground truth appears only in this test harness (as the simulated world),
never inside the code under test.
"""

import math

import numpy as np
import pytest

mazemind_simulation = pytest.importorskip('mazemind_simulation')
from mazemind_simulation.maze import MazeConfig, generate_maze  # noqa: E402
from mazemind_simulation.rasterize import free_space_mask  # noqa: E402

from mazemind_exploration.frontier import detect_frontiers  # noqa: E402
from mazemind_exploration.grid import (GeodesicField, GridMap, compute_traversability,  # noqa: E402
                                       nearest_cell_in_mask)
from mazemind_exploration.memory import REACHED, GoalMemory  # noqa: E402
from mazemind_exploration.selection import (EvaluationParams, Selector,  # noqa: E402
                                            evaluate_candidates)

RES = 0.05
PAD = 10  # cells of unknown border around the maze


class FakeWorld:
    def __init__(self, maze):
        free = free_space_mask(maze, RES)
        self.truth = np.zeros((free.shape[0] + 2 * PAD, free.shape[1] + 2 * PAD), dtype=bool)
        self.truth[PAD:-PAD, PAD:-PAD] = free
        self.grid = GridMap(np.full(self.truth.shape, -1, dtype=np.int16), RES, -PAD * RES, -PAD * RES)

    def sense(self, x, y, max_range=3.0, rays=180):
        r0, c0 = self.grid.world_to_cell(x, y)
        steps = int(max_range / RES)
        for a in np.linspace(0, 2 * math.pi, rays, endpoint=False):
            for k in range(1, steps + 1):
                r = int(round(r0 + k * math.sin(a)))
                c = int(round(c0 + k * math.cos(a)))
                if not self.grid.in_bounds(r, c):
                    break
                if self.truth[r, c]:
                    self.grid.data[r, c] = 0
                else:
                    self.grid.data[r, c] = 100
                    break


def explore(strategy, seed=3, max_goals=80):
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=seed))
    world = FakeWorld(maze)
    x, y = maze.cell_center(maze.config.start)
    world.sense(x, y)
    params = EvaluationParams()
    memory = GoalMemory()
    selector = Selector(strategy, seed=seed)
    travelled = 0.0
    for _ in range(max_goals):
        trav = compute_traversability(world.grid, params.robot_radius)
        cell = nearest_cell_in_mask(trav.mask, world.grid.world_to_cell(x, y), 8)
        assert cell is not None
        field = GeodesicField(trav, RES, cell)
        fronts = detect_frontiers(world.grid, 8, trav.mask)
        cands = evaluate_candidates(fronts, world.grid, trav, field, memory, params)
        best = selector.select(cands)
        if best is None:
            break
        path = field.path(best.frontier.target_cell)
        assert trav.mask[path[:, 0], path[:, 1]].all()       # only known-safe cells
        for r, c in path[::5]:
            x, y = world.grid.cell_to_world(r, c)
            world.sense(x, y)
        x, y = best.target
        world.sense(x, y)
        travelled += best.travel_cost
        memory.record(x, y, REACHED, 0.0)
    truth_free = world.truth[PAD:-PAD, PAD:-PAD]
    # ignore the one-cell rind next to walls (rasterisation aliasing)
    from scipy import ndimage
    core = ndimage.binary_erosion(truth_free, iterations=2)
    known_free = world.grid.data[PAD:-PAD, PAD:-PAD] == 0
    coverage = (known_free & core).sum() / core.sum()
    return coverage, travelled


@pytest.mark.parametrize('strategy', ['random', 'nearest', 'weighted'])
def test_each_strategy_explores_the_whole_maze(strategy):
    coverage, travelled = explore(strategy)
    assert coverage > 0.97, f'{strategy}: coverage {coverage:.3f}'
    assert travelled > 0
