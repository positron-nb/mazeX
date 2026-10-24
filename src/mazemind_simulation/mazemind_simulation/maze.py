"""Procedural maze generation with reproducible seeds.

The maze is a grid of ``rows x cols`` square cells. Walls live on the grid
lines between cells. A perfect maze (exactly one route between any two
cells) is carved with an iterative randomized depth-first search
("recursive backtracker"); optional loops are then added by removing a
fraction of the remaining interior walls, which creates alternative routes
and makes nearest-frontier exploration less trivially optimal.

World frame convention
----------------------
* The maze's lower-left outer corner is the world origin.
* Cell ``(r, c)`` has its centre at ``((c + 0.5) * s, (r + 0.5) * s)``
  where ``s`` is the cell size. Rows grow along +y, columns along +x.
* ``h_walls[i][c]`` is the wall on the horizontal line ``y = i * s``
  spanning column ``c``  (``i`` in ``0..rows``).
* ``v_walls[r][j]`` is the wall on the vertical line ``x = j * s``
  spanning row ``r``  (``j`` in ``0..cols``).

The generated :class:`Maze` is *ground truth*. It is used to write the
Gazebo world and by the evaluator. The exploration stack never receives it.
"""

from __future__ import annotations

import json
import math
import random
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

Cell = Tuple[int, int]

# Direction name -> (d_row, d_col)
DIRECTIONS: Dict[str, Cell] = {
    'south': (-1, 0),
    'north': (1, 0),
    'west': (0, -1),
    'east': (0, 1),
}

DIFFICULTY_PRESETS: Dict[str, Dict[str, float]] = {
    'tiny': {'rows': 3, 'cols': 3, 'loop_fraction': 0.0},
    'easy': {'rows': 4, 'cols': 4, 'loop_fraction': 0.0},
    'medium': {'rows': 6, 'cols': 6, 'loop_fraction': 0.1},
    'hard': {'rows': 8, 'cols': 8, 'loop_fraction': 0.15},
    # demo-scale maze: 12 x 12 cells of 1.5 m = 18 m x 18 m, 1.4 m corridors
    'showcase': {'rows': 12, 'cols': 12, 'loop_fraction': 0.1, 'cell_size': 1.5},
}


class MazeValidationError(ValueError):
    """Raised when a maze configuration or generated maze is invalid."""


@dataclass(frozen=True)
class MazeConfig:
    rows: int = 4
    cols: int = 4
    cell_size: float = 1.0          # metres between adjacent wall centrelines
    wall_thickness: float = 0.1     # metres
    wall_height: float = 0.5        # metres
    seed: int = 0
    loop_fraction: float = 0.0      # fraction of interior walls removed after carving
    start: Cell = (0, 0)
    exit: Optional[Cell] = None     # None -> farthest boundary cell from start
    robot_radius: float = 0.15      # used only for corridor validation
    min_clearance: float = 0.15     # free space required on each side of the robot
    door_width: float = 0.6
    door_height: float = 0.35
    door_thickness: float = 0.02
    door_bottom: float = 0.02

    @property
    def corridor_width(self) -> float:
        return self.cell_size - self.wall_thickness

    @classmethod
    def from_difficulty(cls, difficulty: str, **overrides) -> 'MazeConfig':
        if difficulty not in DIFFICULTY_PRESETS:
            raise MazeValidationError(
                f'unknown difficulty {difficulty!r}; choose from {sorted(DIFFICULTY_PRESETS)}')
        params = dict(DIFFICULTY_PRESETS[difficulty])
        params['rows'] = int(params['rows'])
        params['cols'] = int(params['cols'])
        params.update(overrides)
        return cls(**params)


@dataclass(frozen=True)
class WallBox:
    """Axis-aligned wall box (centre and full extents) in the world frame."""

    cx: float
    cy: float
    sx: float
    sy: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (abs(x - self.cx) <= self.sx / 2 + margin
                and abs(y - self.cy) <= self.sy / 2 + margin)

    def distance(self, x: float, y: float) -> float:
        """Euclidean distance from a point to the box (0 inside)."""
        dx = max(abs(x - self.cx) - self.sx / 2, 0.0)
        dy = max(abs(y - self.cy) - self.sy / 2, 0.0)
        return math.hypot(dx, dy)


@dataclass
class Maze:
    config: MazeConfig
    h_walls: List[List[bool]]
    v_walls: List[List[bool]]
    exit_cell: Cell
    exit_side: str
    solution: List[Cell] = field(default_factory=list)

    # ------------------------------------------------------------------ geometry
    @property
    def rows(self) -> int:
        return self.config.rows

    @property
    def cols(self) -> int:
        return self.config.cols

    @property
    def width(self) -> float:
        return self.cols * self.config.cell_size

    @property
    def height(self) -> float:
        return self.rows * self.config.cell_size

    def cell_center(self, cell: Cell) -> Tuple[float, float]:
        r, c = cell
        s = self.config.cell_size
        return ((c + 0.5) * s, (r + 0.5) * s)

    def world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        s = self.config.cell_size
        c = int(math.floor(x / s))
        r = int(math.floor(y / s))
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    # ------------------------------------------------------------------ topology
    def has_wall(self, cell: Cell, direction: str) -> bool:
        r, c = cell
        if direction == 'south':
            return self.h_walls[r][c]
        if direction == 'north':
            return self.h_walls[r + 1][c]
        if direction == 'west':
            return self.v_walls[r][c]
        if direction == 'east':
            return self.v_walls[r][c + 1]
        raise ValueError(direction)

    def open_neighbors(self, cell: Cell) -> List[Cell]:
        out = []
        for name, (dr, dc) in DIRECTIONS.items():
            nr, nc = cell[0] + dr, cell[1] + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and not self.has_wall(cell, name):
                out.append((nr, nc))
        return out

    def bfs_distances(self, start: Cell) -> Dict[Cell, int]:
        dist = {start: 0}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            for nxt in self.open_neighbors(cur):
                if nxt not in dist:
                    dist[nxt] = dist[cur] + 1
                    queue.append(nxt)
        return dist

    def shortest_path(self, start: Cell, goal: Cell) -> List[Cell]:
        parent: Dict[Cell, Optional[Cell]] = {start: None}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            if cur == goal:
                break
            for nxt in self.open_neighbors(cur):
                if nxt not in parent:
                    parent[nxt] = cur
                    queue.append(nxt)
        if goal not in parent:
            return []
        path = [goal]
        while parent[path[-1]] is not None:
            path.append(parent[path[-1]])
        return path[::-1]

    def dead_end_count(self) -> int:
        return sum(1 for r in range(self.rows) for c in range(self.cols)
                   if len(self.open_neighbors((r, c))) == 1)

    def start_yaw(self) -> float:
        """Spawn heading: face the first open direction (E, N, W, S order)."""
        yaw_for = {'east': 0.0, 'north': math.pi / 2, 'west': math.pi, 'south': -math.pi / 2}
        for name in ('east', 'north', 'west', 'south'):
            dr, dc = DIRECTIONS[name]
            nr, nc = self.config.start[0] + dr, self.config.start[1] + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and not self.has_wall(self.config.start, name):
                return yaw_for[name]
        return 0.0

    # ------------------------------------------------------------------ walls
    def wall_boxes(self) -> List[WallBox]:
        """Merge collinear wall segments into as few boxes as possible."""
        s = self.config.cell_size
        t = self.config.wall_thickness
        boxes: List[WallBox] = []
        for i, line in enumerate(self.h_walls):
            for c0, c1 in _runs(line):
                x0, x1 = c0 * s - t / 2, (c1 + 1) * s + t / 2
                boxes.append(WallBox((x0 + x1) / 2, i * s, x1 - x0, t))
        for j in range(self.cols + 1):
            column = [self.v_walls[r][j] for r in range(self.rows)]
            for r0, r1 in _runs(column):
                y0, y1 = r0 * s - t / 2, (r1 + 1) * s + t / 2
                boxes.append(WallBox(j * s, (y0 + y1) / 2, t, y1 - y0))
        return boxes

    def door_pose(self) -> Dict[str, float]:
        """Exit door panel, mounted on the inner face of the boundary wall."""
        cfg = self.config
        cx, cy = self.cell_center(self.exit_cell)
        half = cfg.cell_size / 2 - cfg.wall_thickness / 2 - cfg.door_thickness / 2
        # yaw is the direction of the door's outward normal (pointing into the maze)
        offsets = {
            'south': (0.0, -half, math.pi / 2),
            'north': (0.0, half, -math.pi / 2),
            'west': (-half, 0.0, 0.0),
            'east': (half, 0.0, math.pi),
        }
        dx, dy, yaw = offsets[self.exit_side]
        return {
            'x': cx + dx,
            'y': cy + dy,
            'z': cfg.door_bottom + cfg.door_height / 2,
            'yaw': yaw,
            'width': cfg.door_width,
            'height': cfg.door_height,
            'thickness': cfg.door_thickness,
        }

    # ------------------------------------------------------------------ export
    def to_dict(self) -> Dict:
        start_xy = self.cell_center(self.config.start)
        exit_xy = self.cell_center(self.exit_cell)
        return {
            'format': 'mazemind-ground-truth-v1',
            'config': {**asdict(self.config),
                       'start': list(self.config.start),
                       'exit': list(self.config.exit) if self.config.exit else None},
            'width': self.width,
            'height': self.height,
            'h_walls': self.h_walls,
            'v_walls': self.v_walls,
            'wall_boxes': [asdict(b) for b in self.wall_boxes()],
            'start': {'cell': list(self.config.start), 'x': start_xy[0], 'y': start_xy[1],
                      'yaw': self.start_yaw()},
            'exit': {'cell': list(self.exit_cell), 'side': self.exit_side,
                     'x': exit_xy[0], 'y': exit_xy[1], 'door': self.door_pose()},
            'solution_cells': [list(c) for c in self.solution],
            'solution_length_m': max(len(self.solution) - 1, 0) * self.config.cell_size,
            'dead_ends': self.dead_end_count(),
        }

    def save_json(self, path: str) -> None:
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(self.to_dict(), fh, indent=2)

    @classmethod
    def from_dict(cls, data: Dict) -> 'Maze':
        cfg = dict(data['config'])
        cfg['start'] = tuple(cfg['start'])
        cfg['exit'] = tuple(cfg['exit']) if cfg.get('exit') else None
        maze = cls(config=MazeConfig(**cfg),
                   h_walls=[list(map(bool, row)) for row in data['h_walls']],
                   v_walls=[list(map(bool, row)) for row in data['v_walls']],
                   exit_cell=tuple(data['exit']['cell']),
                   exit_side=data['exit']['side'],
                   solution=[tuple(c) for c in data.get('solution_cells', [])])
        return maze

    @classmethod
    def load_json(cls, path: str) -> 'Maze':
        with open(path, 'r', encoding='utf-8') as fh:
            return cls.from_dict(json.load(fh))

    def to_ascii(self) -> str:
        """Top-down ASCII rendering (north at the top). S = start, E = exit."""
        lines = []
        for i in range(self.rows, -1, -1):
            row = '+'
            for c in range(self.cols):
                row += ('---' if self.h_walls[i][c] else '   ') + '+'
            lines.append(row)
            if i == 0:
                break
            r = i - 1
            row = ''
            for c in range(self.cols):
                row += '|' if self.v_walls[r][c] else ' '
                mark = ' S ' if (r, c) == self.config.start else (
                    ' E ' if (r, c) == self.exit_cell else '   ')
                row += mark
            row += '|' if self.v_walls[r][self.cols] else ' '
            lines.append(row)
        return '\n'.join(lines)


def _runs(flags: Sequence[bool]) -> List[Tuple[int, int]]:
    """Inclusive index ranges of consecutive True values."""
    runs, start = [], None
    for i, flag in enumerate(flags):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def _boundary_sides(cell: Cell, rows: int, cols: int) -> List[str]:
    r, c = cell
    sides = []
    if r == 0:
        sides.append('south')
    if r == rows - 1:
        sides.append('north')
    if c == 0:
        sides.append('west')
    if c == cols - 1:
        sides.append('east')
    return sides


def validate_config(cfg: MazeConfig) -> None:
    if cfg.rows < 2 or cfg.cols < 2:
        raise MazeValidationError('maze must be at least 2x2 cells')
    if not 0.0 <= cfg.loop_fraction <= 1.0:
        raise MazeValidationError('loop_fraction must be within [0, 1]')
    needed = 2 * (cfg.robot_radius + cfg.min_clearance)
    if cfg.corridor_width < needed:
        raise MazeValidationError(
            f'corridor width {cfg.corridor_width:.2f} m < required {needed:.2f} m '
            f'(2 x (robot_radius + min_clearance)); increase cell_size or reduce wall_thickness')
    if cfg.door_width > cfg.corridor_width:
        raise MazeValidationError('door_width must fit inside the corridor')
    for name, cell in (('start', cfg.start), ('exit', cfg.exit)):
        if cell is None:
            continue
        if not (0 <= cell[0] < cfg.rows and 0 <= cell[1] < cfg.cols):
            raise MazeValidationError(f'{name} cell {cell} outside the maze')
    if cfg.exit is not None:
        if cfg.exit == cfg.start:
            raise MazeValidationError('exit and start must differ')
        if not _boundary_sides(cfg.exit, cfg.rows, cfg.cols):
            raise MazeValidationError('exit must be a boundary cell (the door is on the outer wall)')


def generate_maze(cfg: MazeConfig) -> Maze:
    """Generate a reproducible maze for ``cfg`` (same config -> same maze)."""
    validate_config(cfg)
    rng = random.Random(cfg.seed)
    rows, cols = cfg.rows, cfg.cols
    h_walls = [[True] * cols for _ in range(rows + 1)]
    v_walls = [[True] * (cols + 1) for _ in range(rows)]

    def remove_wall(a: Cell, b: Cell) -> None:
        (r1, c1), (r2, c2) = a, b
        if r1 == r2:
            v_walls[r1][max(c1, c2)] = False
        else:
            h_walls[max(r1, r2)][c1] = False

    # Iterative randomized depth-first search (recursive backtracker).
    visited = {cfg.start}
    stack = [cfg.start]
    while stack:
        r, c = stack[-1]
        options = [(r + dr, c + dc) for dr, dc in DIRECTIONS.values()
                   if 0 <= r + dr < rows and 0 <= c + dc < cols
                   and (r + dr, c + dc) not in visited]
        if not options:
            stack.pop()
            continue
        nxt = rng.choice(options)
        remove_wall((r, c), nxt)
        visited.add(nxt)
        stack.append(nxt)

    # Optional loops: remove a seeded fraction of the remaining interior walls.
    if cfg.loop_fraction > 0:
        interior = [('h', i, c) for i in range(1, rows) for c in range(cols) if h_walls[i][c]]
        interior += [('v', r, j) for r in range(rows) for j in range(1, cols) if v_walls[r][j]]
        rng.shuffle(interior)
        for kind, a, b in interior[:int(round(cfg.loop_fraction * len(interior)))]:
            if kind == 'h':
                h_walls[a][b] = False
            else:
                v_walls[a][b] = False

    maze = Maze(config=cfg, h_walls=h_walls, v_walls=v_walls, exit_cell=(0, 0), exit_side='south')
    dist = maze.bfs_distances(cfg.start)

    if cfg.exit is not None:
        exit_cell = cfg.exit
    else:
        boundary = [(r, c) for r in range(rows) for c in range(cols)
                    if _boundary_sides((r, c), rows, cols) and (r, c) != cfg.start]
        # Farthest boundary cell; ties broken deterministically by cell index.
        exit_cell = max(boundary, key=lambda cell: (dist.get(cell, -1), cell))
    maze.exit_cell = exit_cell
    maze.exit_side = _boundary_sides(exit_cell, rows, cols)[0]
    maze.solution = maze.shortest_path(cfg.start, exit_cell)
    validate_maze(maze)
    return maze


def validate_maze(maze: Maze) -> None:
    """Structural checks used after generation and when loading from disk."""
    cfg = maze.config
    rows, cols = cfg.rows, cfg.cols
    if any(not maze.h_walls[0][c] or not maze.h_walls[rows][c] for c in range(cols)):
        raise MazeValidationError('outer boundary must be closed (south/north)')
    if any(not maze.v_walls[r][0] or not maze.v_walls[r][cols] for r in range(rows)):
        raise MazeValidationError('outer boundary must be closed (west/east)')
    dist = maze.bfs_distances(cfg.start)
    if len(dist) != rows * cols:
        raise MazeValidationError('not every cell is reachable from the start')
    if maze.exit_cell not in dist:
        raise MazeValidationError('exit is not reachable from the start')
    if maze.exit_side not in _boundary_sides(maze.exit_cell, rows, cols):
        raise MazeValidationError('exit door must be on an outer wall')
    if not maze.solution or maze.solution[0] != cfg.start or maze.solution[-1] != maze.exit_cell:
        raise MazeValidationError('solution path missing or inconsistent')
