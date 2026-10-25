"""Command-line maze generator: writes the Gazebo world and ground-truth JSON.

Example::

    ros2 run mazemind_simulation generate_maze --difficulty easy --seed 7 --out ~/.mazemind/worlds
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, Optional, Sequence

from mazemind_simulation.maze import DIFFICULTY_PRESETS, MazeConfig, MazeValidationError, generate_maze
from mazemind_simulation.world_writer import write_world


def default_data_dir() -> str:
    return os.environ.get('MAZEMIND_DATA_DIR', os.path.join(os.path.expanduser('~'), '.mazemind'))


def maze_basename(cfg: MazeConfig) -> str:
    return f'maze_{cfg.rows}x{cfg.cols}_s{cfg.seed}_l{int(round(cfg.loop_fraction * 100))}'


def build_and_write(cfg: MazeConfig, out_dir: str, real_time_factor: float = 1.0) -> Dict[str, str]:
    """Generate the maze for ``cfg`` and write ``<name>.sdf`` and ``<name>.json``."""
    maze = generate_maze(cfg)
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, maze_basename(cfg))
    write_world(maze, base + '.sdf', real_time_factor=real_time_factor)
    maze.save_json(base + '.json')
    start = maze.to_dict()['start']
    return {'world': base + '.sdf', 'ground_truth': base + '.json',
            'x': str(start['x']), 'y': str(start['y']), 'yaw': str(start['yaw'])}


def config_from_args(args: argparse.Namespace) -> MazeConfig:
    overrides = {'seed': args.seed,
                 'wall_thickness': args.wall_thickness, 'wall_height': args.wall_height,
                 'start': (args.start_row, args.start_col)}
    if args.exit_row is not None and args.exit_col is not None:
        overrides['exit'] = (args.exit_row, args.exit_col)
    if args.cell_size is not None:
        overrides['cell_size'] = args.cell_size
    if args.rows is not None:
        overrides['rows'] = args.rows
    if args.cols is not None:
        overrides['cols'] = args.cols
    if args.loop_fraction is not None:
        overrides['loop_fraction'] = args.loop_fraction
    return MazeConfig.from_difficulty(args.difficulty, **overrides)


def make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Generate a reproducible MazeMind maze.')
    p.add_argument('--difficulty', default='easy', choices=sorted(DIFFICULTY_PRESETS))
    p.add_argument('--rows', type=int)
    p.add_argument('--cols', type=int)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--loop-fraction', type=float)
    p.add_argument('--cell-size', type=float, help='override the preset cell size (m)')
    p.add_argument('--wall-thickness', type=float, default=0.1)
    p.add_argument('--wall-height', type=float, default=0.5)
    p.add_argument('--start-row', type=int, default=0)
    p.add_argument('--start-col', type=int, default=0)
    p.add_argument('--exit-row', type=int)
    p.add_argument('--exit-col', type=int)
    p.add_argument('--out', default=os.path.join(default_data_dir(), 'worlds'))
    p.add_argument('--print', action='store_true', help='print an ASCII rendering')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = make_parser().parse_args(argv)
    try:
        cfg = config_from_args(args)
        paths = build_and_write(cfg, os.path.expanduser(args.out))
    except MazeValidationError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.print:
        print(generate_maze(cfg).to_ascii())
    print(f"world:        {paths['world']}")
    print(f"ground truth: {paths['ground_truth']}")
    print(f"spawn pose:   x={float(paths['x']):.3f} y={float(paths['y']):.3f} "
          f"yaw={float(paths['yaw']):.3f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
