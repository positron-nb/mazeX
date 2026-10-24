import math
import xml.etree.ElementTree as ET

import pytest

from mazemind_simulation.maze import (DIFFICULTY_PRESETS, Maze, MazeConfig, MazeValidationError,
                                      generate_maze)
from mazemind_simulation.rasterize import free_space_mask
from mazemind_simulation.world_writer import world_sdf


@pytest.mark.parametrize('seed', range(20))
@pytest.mark.parametrize('difficulty', sorted(DIFFICULTY_PRESETS))
def test_generated_maze_is_valid_and_solvable(difficulty, seed):
    maze = generate_maze(MazeConfig.from_difficulty(difficulty, seed=seed))
    cfg = maze.config
    assert len(maze.bfs_distances(cfg.start)) == cfg.rows * cfg.cols
    assert maze.solution[0] == cfg.start and maze.solution[-1] == maze.exit_cell
    # consecutive solution cells must be open neighbours
    for a, b in zip(maze.solution, maze.solution[1:]):
        assert b in maze.open_neighbors(a)


def test_same_seed_is_reproducible_and_different_seeds_differ():
    a = generate_maze(MazeConfig(rows=6, cols=6, seed=3))
    b = generate_maze(MazeConfig(rows=6, cols=6, seed=3))
    c = generate_maze(MazeConfig(rows=6, cols=6, seed=4))
    assert a.to_dict() == b.to_dict()
    assert a.h_walls != c.h_walls or a.v_walls != c.v_walls


def test_perfect_maze_is_a_spanning_tree():
    maze = generate_maze(MazeConfig(rows=7, cols=5, seed=11))
    openings = sum(len(maze.open_neighbors((r, c))) for r in range(7) for c in range(5)) // 2
    assert openings == 7 * 5 - 1


def test_loops_add_openings():
    tree = generate_maze(MazeConfig(rows=6, cols=6, seed=1))
    loopy = generate_maze(MazeConfig(rows=6, cols=6, seed=1, loop_fraction=0.3))

    def count(m):
        return sum(len(m.open_neighbors((r, c))) for r in range(6) for c in range(6))

    assert count(loopy) > count(tree)


def test_default_exit_is_farthest_boundary_cell():
    maze = generate_maze(MazeConfig(rows=5, cols=5, seed=2))
    dist = maze.bfs_distances(maze.config.start)
    boundary = [(r, c) for r in range(5) for c in range(5)
                if r in (0, 4) or c in (0, 4)]
    assert dist[maze.exit_cell] == max(dist[c] for c in boundary)


def test_explicit_exit_and_validation_errors():
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=0, exit=(3, 3)))
    assert maze.exit_cell == (3, 3)
    with pytest.raises(MazeValidationError):
        generate_maze(MazeConfig(rows=4, cols=4, exit=(1, 1)))  # interior exit
    with pytest.raises(MazeValidationError):
        generate_maze(MazeConfig(rows=4, cols=4, cell_size=0.5))  # corridor too narrow
    with pytest.raises(MazeValidationError):
        generate_maze(MazeConfig(rows=1, cols=4))


def test_wall_boxes_cover_every_wall_segment():
    maze = generate_maze(MazeConfig(rows=5, cols=5, seed=9))
    boxes = maze.wall_boxes()
    s = maze.config.cell_size
    for i, line in enumerate(maze.h_walls):
        for c, present in enumerate(line):
            mid = ((c + 0.5) * s, i * s)
            assert present == any(b.contains(*mid) for b in boxes)
    for r, line in enumerate(maze.v_walls):
        for j, present in enumerate(line):
            mid = (j * s, (r + 0.5) * s)
            assert present == any(b.contains(*mid) for b in boxes)


def test_door_lies_on_exit_boundary_wall_facing_inward():
    maze = generate_maze(MazeConfig(rows=5, cols=5, seed=5))
    door = maze.door_pose()
    cx, cy = maze.cell_center(maze.exit_cell)
    # door is inside the exit cell, and its normal points back toward the cell centre
    assert maze.world_to_cell(door['x'], door['y']) == maze.exit_cell
    to_center = math.atan2(cy - door['y'], cx - door['x'])
    assert abs(math.remainder(to_center - door['yaw'], 2 * math.pi)) < 1e-6


def test_json_roundtrip(tmp_path):
    maze = generate_maze(MazeConfig(rows=4, cols=6, seed=8, loop_fraction=0.2))
    path = tmp_path / 'm.json'
    maze.save_json(str(path))
    loaded = Maze.load_json(str(path))
    assert loaded.to_dict() == maze.to_dict()


def test_world_sdf_is_well_formed():
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=1))
    root = ET.fromstring(world_sdf(maze))
    models = {m.get('name') for m in root.iter('model')}
    assert {'ground_plane', 'maze', 'exit_door'} <= models
    collisions = [c for c in root.iter('collision') if c.get('name', '').startswith('wall_')]
    assert len(collisions) == len(maze.wall_boxes())


def test_free_space_mask_area_matches_geometry():
    maze = generate_maze(MazeConfig(rows=4, cols=4, seed=1))
    res = 0.05
    free = free_space_mask(maze, res)
    # Free area is less than the full square (walls) but more than 70 % of it.
    total = maze.width * maze.height
    free_area = free.sum() * res * res
    assert 0.7 * total < free_area < total
    # start cell centre is free; a point on the outer wall is not
    sx, sy = maze.cell_center(maze.config.start)
    assert free[int(sy / res), int(sx / res)]
    assert not free[0, int(sx / res)]
