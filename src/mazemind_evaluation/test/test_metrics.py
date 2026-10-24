import math

import numpy as np
import pytest

from mazemind_evaluation.analyze_results import flatten, markdown_summary, summarize_by_strategy
from mazemind_evaluation.metrics import (ContactCounter, CoverageEvaluator, DistanceAccumulator,
                                         NearCollisionTracker, Pose2D, map_to_world, summarize,
                                         wilson_interval, world_to_map)
from mazemind_simulation.maze import MazeConfig, WallBox, generate_maze
from mazemind_simulation.rasterize import free_space_mask


def test_map_world_roundtrip():
    start = Pose2D(0.5, 1.5, math.pi / 2)
    wx, wy = map_to_world(start, 1.0, 0.0)
    assert (wx, wy) == pytest.approx((0.5, 2.5))
    assert world_to_map(start, wx, wy) == pytest.approx((1.0, 0.0))


def _slam_grid_from_truth(maze, start, res=0.05, known_fraction=1.0):
    """Build a perfect 'SLAM' map in the map frame from ground truth (test helper)."""
    free = free_space_mask(maze, res)
    pad = 10
    grid = np.full((free.shape[0] + 2 * pad, free.shape[1] + 2 * pad), 100, dtype=np.int16)
    grid[pad:-pad, pad:-pad] = np.where(free, 0, 100)
    if known_fraction < 1.0:
        cut = int(grid.shape[1] * known_fraction)
        grid[:, cut:] = -1
    # map origin such that world (0,0) corner sits at grid[pad, pad]; start yaw = 0
    origin_x, origin_y = -start.x - pad * res, -start.y - pad * res
    return grid, origin_x, origin_y


def test_coverage_full_and_partial():
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=1))
    start = Pose2D(*maze.cell_center(maze.config.start), 0.0)
    cov = CoverageEvaluator(maze, start, 0.05)
    grid, ox, oy = _slam_grid_from_truth(maze, start)
    assert cov.coverage(grid, 0.05, ox, oy) == pytest.approx(1.0)
    half, ox, oy = _slam_grid_from_truth(maze, start, known_fraction=0.5)
    assert 0.3 < cov.coverage(half, 0.05, ox, oy) < 0.7
    unknown = np.full_like(grid, -1)
    assert cov.coverage(unknown, 0.05, ox, oy) == 0.0


def test_coverage_detects_misaligned_map():
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=1))
    start = Pose2D(*maze.cell_center(maze.config.start), 0.0)
    cov = CoverageEvaluator(maze, start, 0.05)
    grid, ox, oy = _slam_grid_from_truth(maze, start)
    shifted = cov.coverage(grid, 0.05, ox + 0.5, oy)   # map shifted by half a cell
    assert shifted < 0.9


def test_distance_accumulator_ignores_jitter():
    d = DistanceAccumulator(min_step=0.01)
    for i in range(101):
        d.update(i * 0.01, 0.0)
    for _ in range(100):
        d.update(1.0 + 1e-4, 0.0)       # jitter
    assert d.total == pytest.approx(1.0, abs=0.02)


def test_contact_counter_events_and_ground_filter():
    c = ContactCounter(debounce=0.5)
    c.update(0.0, [('mazemind::base::chassis', 'ground_plane::link::collision')])
    assert c.events == 0
    c.update(1.0, [('mazemind::base::chassis', 'maze::walls::wall_7_collision')])
    c.update(1.1, [('mazemind::base::chassis', 'maze::walls::wall_7_collision')])
    c.update(1.2, [('mazemind::base::chassis', 'maze::walls::wall_7_collision')])
    assert c.events == 1
    assert c.contact_time == pytest.approx(0.2)
    c.update(3.0, [('maze::walls::wall_2_collision', 'mazemind::base::chassis')])
    assert c.events == 2
    assert c.partners == {'maze::walls::wall_7_collision', 'maze::walls::wall_2_collision'}


def test_near_collision_tracker_hysteresis():
    wall = WallBox(cx=0.0, cy=0.0, sx=0.1, sy=4.0)   # wall face at x = 0.05
    t = NearCollisionTracker([wall], body_radius=0.14, threshold=0.05, hysteresis=0.02)
    assert t.update(0.5, 0.0) == pytest.approx(0.31)
    t.update(0.22, 0.0)     # clearance 0.03 -> event
    t.update(0.21, 0.0)     # still close, no new event
    t.update(0.25, 0.0)     # 0.06: not re-armed yet (needs > 0.07)
    t.update(0.23, 0.0)
    assert t.events == 1
    t.update(0.4, 0.0)      # re-armed
    t.update(0.2, 0.0)
    assert t.events == 2
    assert t.min_clearance == pytest.approx(0.01)


def test_summary_and_wilson():
    s = summarize([1.0, 2.0, 3.0, None, float('nan')])
    assert s['n'] == 3 and s['mean'] == 2.0 and s['median'] == 2.0 and s['std'] == 1.0
    assert summarize([])['n'] == 0
    lo, hi = wilson_interval(5, 5)
    assert hi == 1.0 and 0.5 < lo < 0.6
    assert wilson_interval(0, 0) == (0.0, 0.0)


def test_strategy_summary_counts_only_successes_for_distance():
    runs = [
        {'run_id': 'a', 'strategy': 'nearest', 'final_status': 'EXIT_REACHED', 'exit_success': True,
         'distance_gt_m': 10.0, 'time_to_exit_s': 100.0, 'coverage': 0.8, 'goals_failed': 1,
         'recoveries': 0, 'collision_events': 0, 'near_collision_events': 0,
         'maze': {'seed': 0}},
        {'run_id': 'b', 'strategy': 'nearest', 'final_status': 'TIMEOUT', 'exit_success': False,
         'distance_gt_m': 50.0, 'time_to_exit_s': None, 'coverage': 0.6, 'goals_failed': 4,
         'recoveries': 3, 'collision_events': 1, 'near_collision_events': 2,
         'maze': {'seed': 1}},
    ]
    summary = summarize_by_strategy([flatten(r) for r in runs])
    d = summary['nearest']
    assert d['n'] == 2 and d['successes'] == 1
    assert d['distance_success']['mean'] == 10.0
    assert d['distance_all']['mean'] == 30.0
    assert d['statuses'] == {'EXIT_REACHED': 1, 'TIMEOUT': 1}
    assert 'nearest' in markdown_summary(summary, 'test')
