import math

import numpy as np

from mazemind_exploration.grid import GridMap
from mazemind_exploration.visual_coverage import (VisualCoverage, info_grid_with_unseen,
                                                  unseen_regions)

RES = 0.05


def room(rows=60, cols=60):
    g = GridMap(np.zeros((rows, cols), dtype=np.int16), RES, 0.0, 0.0)
    g.data[0, :] = g.data[-1, :] = g.data[:, 0] = g.data[:, -1] = 100
    return g


def test_frustum_marks_cells_in_front_only():
    g = room()
    cov = VisualCoverage(RES, view_range=1.0, hfov=1.0, n_rays=30)
    x, y = g.cell_to_world(30, 30)
    assert cov.observe(g, x, y, 0.0) > 0                # facing +x
    seen = cov.mask(g)
    assert seen[30, 45]                                 # 0.75 m ahead
    assert not seen[30, 15]                             # behind
    assert not seen[30, 55]                             # beyond view range


def test_rays_stop_at_walls_but_mark_them():
    g = room()
    g.data[20:40, 35] = 100                             # wall 0.25 m ahead
    cov = VisualCoverage(RES, view_range=1.5, hfov=0.6, n_rays=30)
    x, y = g.cell_to_world(30, 30)
    cov.observe(g, x, y, 0.0)
    seen = cov.mask(g)
    assert seen[30, 35]                                 # wall face seen
    assert not seen[30, 40]                             # behind the wall


def test_unknown_cells_are_passed_through_not_marked():
    g = room()
    g.data[30, 34] = -1                                 # sliver between LiDAR rays
    cov = VisualCoverage(RES, view_range=1.0, hfov=0.2, n_rays=10)
    x, y = g.cell_to_world(30, 30)
    cov.observe(g, x, y, 0.0)
    seen = cov.mask(g)
    assert not seen[30, 34]
    assert seen[30, 40]


def test_seen_cells_survive_map_growth():
    g = room()
    cov = VisualCoverage(RES, view_range=1.0, hfov=1.0, n_rays=30)
    x, y = g.cell_to_world(30, 30)
    cov.observe(g, x, y, math.pi / 2)
    before = cov.mask(g)
    # SLAM grows the map by 10 cells to the west/south: origin moves, indices shift
    bigger = GridMap(np.zeros((70, 70), dtype=np.int16), RES, -10 * RES, -10 * RES)
    after = cov.mask(bigger)
    assert after.sum() == before.sum()
    assert np.array_equal(after[10:70, 10:70], before)


def test_unseen_regions_and_info_grid():
    g = room()
    trav = np.ones(g.shape, dtype=bool)
    seen = np.zeros(g.shape, dtype=bool)
    seen[:, :40] = True
    regions = unseen_regions(trav, seen, min_cells=50)
    assert len(regions) == 1
    cells, target = regions[0]
    assert cells.shape[0] == 60 * 20
    assert target[1] >= 40
    tiny = np.zeros(g.shape, dtype=bool)
    tiny[5:7, 5:7] = True
    assert unseen_regions(tiny, np.zeros_like(tiny), min_cells=50) == []
    ig = info_grid_with_unseen(g, ~seen)
    assert (ig.data[:, 40:] == -1).all() and (g.data[1:-1, 40:-1] == 0).all()
