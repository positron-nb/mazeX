import math

import numpy as np
import pytest

from mazemind_exploration.frontier import detect_frontiers, frontier_mask, visible_unknown_cells
from mazemind_exploration.grid import (GeodesicField, GridMap, compute_traversability,
                                       nearest_cell_in_mask)

RES = 0.05


def make_grid(rows=40, cols=40, fill=-1):
    return GridMap(np.full((rows, cols), fill, dtype=np.int16), RES, 0.0, 0.0)


def corridor_grid():
    """Horizontal corridor (rows 10-29) known free for cols 0-24, unknown beyond, walls around."""
    g = make_grid()
    g.data[10:30, 0:25] = 0
    g.data[8:10, 0:25] = 100
    g.data[30:32, 0:25] = 100
    g.data[8:32, 0] = 100
    return g


def test_frontier_mask_marks_free_cells_next_to_unknown():
    g = corridor_grid()
    mask = frontier_mask(g)
    assert mask[10:30, 24].all()           # boundary column of known corridor
    assert not mask[10:30, 1:23].any()     # interior free cells are not frontier
    assert not mask[8:10, :].any()         # walls are never frontier


def test_detect_frontiers_clusters_and_filters_small():
    g = corridor_grid()
    g.data[35, 35] = 0                     # isolated free cell: tiny frontier
    fronts = detect_frontiers(g, min_size=5)
    assert len(fronts) == 1
    f = fronts[0]
    assert f.size == 20
    assert f.target_cell[1] == 24
    assert 10 <= f.target_cell[0] < 30
    assert math.isclose(f.target[0], (24 + 0.5) * RES)


def test_target_snaps_to_traversable_cell():
    g = corridor_grid()
    trav = compute_traversability(g, robot_radius=0.15)
    fronts = detect_frontiers(g, min_size=5, traversable=trav.mask)
    assert len(fronts) == 1
    assert trav.mask[fronts[0].target_cell]


def test_traversability_respects_robot_radius():
    g = corridor_grid()
    trav = compute_traversability(g, robot_radius=0.15)    # 3 cells
    assert not trav.mask[10, 12]                          # hugging the wall
    assert trav.mask[20, 12]                              # corridor centre
    assert trav.clearance[20, 12] == pytest.approx(10 * RES, abs=1e-9)   # nearest wall row 30


def test_geodesic_distance_goes_around_walls():
    g = make_grid(60, 60, fill=0)
    g.data[0:45, 30] = 100                 # wall with a gap at the top
    trav = compute_traversability(g, robot_radius=0.1)
    field = GeodesicField(trav, RES, (10, 10))
    target = (10, 50)
    straight = 40 * RES
    assert field.reachable(target)
    assert field.distance[target] > straight * 1.5
    path = field.path(target)
    assert tuple(path[0]) == (10, 10) and tuple(path[-1]) == target
    assert (path[:, 0] >= 45).any()        # path passes through the gap
    assert field.bottleneck_clearance(target) > 0.1


def test_unreachable_target():
    g = make_grid(40, 40, fill=0)
    g.data[:, 20] = 100
    trav = compute_traversability(g, robot_radius=0.05)
    field = GeodesicField(trav, RES, (5, 5))
    assert not field.reachable((5, 35))
    assert field.path((5, 35)).size == 0


def test_visible_unknown_cells_blocked_by_walls():
    g = make_grid(60, 60, fill=0)
    g.data[:, 40:] = -1                    # unknown region to the east
    open_view = visible_unknown_cells(g, (30, 30), radius_m=1.0)
    g.data[:, 35] = 100                    # wall between robot and unknown region
    blocked = visible_unknown_cells(g, (30, 30), radius_m=1.0)
    assert open_view > 50
    assert blocked == 0


def test_cells_outside_map_count_as_unknown():
    g = make_grid(20, 20, fill=0)
    assert visible_unknown_cells(g, (10, 18), radius_m=1.0) > 0


def test_nearest_cell_in_mask():
    mask = np.zeros((20, 20), dtype=bool)
    mask[15, 15] = True
    assert nearest_cell_in_mask(mask, (12, 12), 5) == (15, 15)
    assert nearest_cell_in_mask(mask, (0, 0), 5) is None


def test_clearance_penalty_keeps_path_off_the_corner():
    """An L-shaped corridor: the plain shortest path cuts the inner corner at the robot
    radius; the clearance-aware path keeps away from it, raising the bottleneck."""
    g = make_grid(60, 60, fill=100)
    g.data[5:23, 5:55] = 0          # horizontal corridor, 0.9 m wide
    g.data[5:55, 37:55] = 0         # vertical corridor, 0.9 m wide
    trav = compute_traversability(g, robot_radius=0.15, gap_fill_cells=0)
    start, goal = (14, 14), (45, 46)    # both on corridor centre lines (clearance >= 0.4 m)
    plain = GeodesicField(trav, RES, start)
    aware = GeodesicField(trav, RES, start, clearance_penalty=2.0, robot_radius=0.15,
                          safe_clearance=0.35)
    assert plain.reachable(goal) and aware.reachable(goal)
    assert plain.bottleneck_clearance(goal) < 0.2
    assert aware.bottleneck_clearance(goal) > plain.bottleneck_clearance(goal) + 0.1
