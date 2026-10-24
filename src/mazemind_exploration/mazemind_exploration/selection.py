"""Frontier candidate evaluation and target-selection strategies.

All three strategies receive the *same* filtered candidate list (reachable
in the SLAM-derived traversability graph, not blacklisted, not trivially
close), so experiments isolate the effect of the selection rule:

* ``random``   - uniform choice (seeded).
* ``nearest``  - minimum geodesic travel cost.
* ``weighted`` - information-gain and safety-aware score::

    Score(F) =  w_info      * info_gain_n(F)
              + w_size      * size_n(F)
              + w_clearance * clearance_n(F)
              + w_success   * success_estimate(F)
              - w_distance  * travel_cost_n(F)
              - w_revisit   * revisit_penalty(F)
              - w_failure   * failure_penalty(F)

  ``*_n`` features are divided by the maximum over the current candidates
  (ratio-preserving, in [0, 1]); clearance is divided by ``clearance_cap``;
  the penalties and the success estimate are already in [0, 1].
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from mazemind_exploration.frontier import Frontier, visible_unknown_cells
from mazemind_exploration.grid import GeodesicField, GridMap, Traversability
from mazemind_exploration.memory import GoalMemory

STRATEGIES = ('random', 'nearest', 'weighted')


@dataclass
class Weights:
    info: float = 1.0
    size: float = 0.3
    clearance: float = 0.3
    success: float = 0.5
    distance: float = 1.2
    revisit: float = 0.8
    failure: float = 1.5


@dataclass
class Candidate:
    frontier: Frontier
    travel_cost: float              # metres along the geodesic path
    info_gain: int                  # visible unknown cells
    clearance: float                # metres to nearest obstacle at the target
    bottleneck: float               # min clearance along the path (metres)
    success_estimate: float         # [0, 1] traversability proxy
    revisit_penalty: float          # [0, 1]
    failure_penalty: float          # [0, 1]
    score: float = 0.0
    features: Dict[str, float] = field(default_factory=dict)

    @property
    def target(self):
        return self.frontier.target


@dataclass
class EvaluationParams:
    robot_radius: float = 0.15
    safe_clearance: float = 0.35     # bottleneck clearance treated as "certainly passable"
    clearance_cap: float = 0.45      # clearance normalisation (half a corridor)
    info_radius: float = 2.0
    info_rays: int = 72
    min_goal_distance: float = 0.35  # skip frontiers the robot is already standing on


def success_estimate(bottleneck: float, robot_radius: float, safe_clearance: float) -> float:
    """Heuristic proxy (not a calibrated probability) that Nav2 can traverse the path.

    0 when the narrowest point of the path only just admits the robot, 1 when
    every point keeps at least ``safe_clearance`` from obstacles.
    """
    if safe_clearance <= robot_radius:
        return 1.0
    return float(np.clip((bottleneck - robot_radius) / (safe_clearance - robot_radius), 0.0, 1.0))


def evaluate_candidates(frontiers: Sequence[Frontier], grid: GridMap, trav: Traversability,
                        field_: GeodesicField, memory: GoalMemory,
                        params: EvaluationParams,
                        rejected: Optional[List] = None) -> List[Candidate]:
    """Compute raw features for every reachable, non-blacklisted frontier.

    If ``rejected`` is given, ``(frontier, reason)`` pairs are appended for
    frontiers that were filtered out (for logging and diagnostics).
    """
    out: List[Candidate] = []
    for f in frontiers:
        cell = f.target_cell
        x, y = f.target
        if not field_.reachable(cell):
            if rejected is not None:
                rejected.append((f, 'unreachable'))
            continue
        cost = float(field_.distance[cell])
        # visual targets next to the robot are handled by turning to look, not filtered
        if cost < params.min_goal_distance and f.kind != 'visual':
            if rejected is not None:
                rejected.append((f, f'too close ({cost:.2f} m)'))
            continue
        if memory.is_blacklisted(x, y):
            if rejected is not None:
                rejected.append((f, 'blacklisted'))
            continue
        bottleneck = field_.bottleneck_clearance(cell)
        out.append(Candidate(
            frontier=f,
            travel_cost=cost,
            info_gain=visible_unknown_cells(grid, cell, params.info_radius, params.info_rays),
            clearance=float(trav.clearance[cell]),
            bottleneck=bottleneck,
            success_estimate=success_estimate(bottleneck, params.robot_radius, params.safe_clearance),
            revisit_penalty=memory.revisit_penalty(x, y),
            failure_penalty=memory.failure_penalty(x, y),
        ))
    return out


def _max_norm(values: Sequence[float]) -> List[float]:
    top = max(values) if values else 0.0
    if top <= 0:
        return [0.0 for _ in values]
    return [v / top for v in values]


def score_candidates(cands: List[Candidate], w: Weights, clearance_cap: float) -> None:
    """Fill ``score`` and ``features`` in place using the weighted formulation."""
    if not cands:
        return
    info_n = _max_norm([c.info_gain for c in cands])
    size_n = _max_norm([c.frontier.size for c in cands])
    cost_n = _max_norm([c.travel_cost for c in cands])
    for c, info, size, cost in zip(cands, info_n, size_n, cost_n):
        clear = min(c.clearance / clearance_cap, 1.0) if clearance_cap > 0 else 0.0
        c.features = {'info': info, 'size': size, 'clearance': clear,
                      'success': c.success_estimate, 'distance': cost,
                      'revisit': c.revisit_penalty, 'failure': c.failure_penalty}
        c.score = (w.info * info + w.size * size + w.clearance * clear
                   + w.success * c.success_estimate - w.distance * cost
                   - w.revisit * c.revisit_penalty - w.failure * c.failure_penalty)


class Selector:
    """Applies one of :data:`STRATEGIES` to a list of evaluated candidates."""

    def __init__(self, strategy: str, weights: Optional[Weights] = None,
                 clearance_cap: float = 0.45, seed: int = 0):
        if strategy not in STRATEGIES:
            raise ValueError(f'unknown strategy {strategy!r}; choose from {STRATEGIES}')
        self.strategy = strategy
        self.weights = weights or Weights()
        self.clearance_cap = clearance_cap
        self._rng = random.Random(seed)

    def rank(self, cands: List[Candidate]) -> List[Candidate]:
        """Best-first ordering. ``score`` is always filled (for logging and markers)."""
        score_candidates(cands, self.weights, self.clearance_cap)
        if self.strategy == 'weighted':
            return sorted(cands, key=lambda c: (-c.score, c.travel_cost))
        if self.strategy == 'nearest':
            return sorted(cands, key=lambda c: (c.travel_cost, -c.score))
        shuffled = list(cands)
        self._rng.shuffle(shuffled)
        return shuffled

    def select(self, cands: List[Candidate]) -> Optional[Candidate]:
        ranked = self.rank(cands)
        return ranked[0] if ranked else None

    def preference(self, cand: Candidate) -> float:
        """Larger is better; used for hysteresis when deciding to switch targets."""
        if self.strategy == 'nearest':
            return -cand.travel_cost
        return cand.score


def distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
