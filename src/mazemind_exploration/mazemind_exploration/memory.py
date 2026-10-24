"""Bookkeeping of past navigation goals: revisits, failures and blacklisting."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

# Goal outcomes recorded by the explorer
REACHED = 'reached'        # Nav2 reported success
RESOLVED = 'resolved'      # frontier disappeared before arrival (not a failure)
PREEMPTED = 'preempted'    # replaced by a clearly better target (not a failure)
FAILED = 'failed'          # Nav2 aborted / rejected the goal
TIMED_OUT = 'timed_out'    # exceeded the per-goal time budget
STUCK = 'stuck'            # robot made no progress while navigating

FAILURE_OUTCOMES = frozenset({FAILED, TIMED_OUT, STUCK})


@dataclass
class GoalRecord:
    x: float
    y: float
    outcome: str
    stamp: float


class GoalMemory:
    """Remembers where goals were sent and how they ended.

    A *region* is a disc of ``region_radius`` metres. A target is
    blacklisted when its region has accumulated ``max_failures`` failures, or
    ``max_attempts`` attempts of any kind (a frontier that survives repeated
    visits is treated as unobservable rather than chased forever).
    """

    def __init__(self, region_radius: float = 0.5, max_failures: int = 2, max_attempts: int = 3):
        self.region_radius = region_radius
        self.max_failures = max_failures
        self.max_attempts = max_attempts
        self.records: List[GoalRecord] = []

    def record(self, x: float, y: float, outcome: str, stamp: float) -> None:
        self.records.append(GoalRecord(x, y, outcome, stamp))

    def _near(self, x: float, y: float) -> List[GoalRecord]:
        r = self.region_radius
        return [g for g in self.records if math.hypot(g.x - x, g.y - y) <= r]

    def failures_near(self, x: float, y: float) -> int:
        return sum(1 for g in self._near(x, y) if g.outcome in FAILURE_OUTCOMES)

    def attempts_near(self, x: float, y: float) -> int:
        """Attempts that should have resolved the region (excludes preemption/resolution)."""
        return sum(1 for g in self._near(x, y) if g.outcome not in (PREEMPTED, RESOLVED))

    def is_blacklisted(self, x: float, y: float) -> bool:
        return (self.failures_near(x, y) >= self.max_failures
                or self.attempts_near(x, y) >= self.max_attempts)

    def failure_penalty(self, x: float, y: float) -> float:
        return min(1.0, self.failures_near(x, y) / max(self.max_failures, 1))

    def revisit_penalty(self, x: float, y: float) -> float:
        return min(1.0, self.attempts_near(x, y) / max(self.max_attempts, 1))

    def count(self, outcome: str) -> int:
        return sum(1 for g in self.records if g.outcome == outcome)

    def blacklisted_points(self) -> List[Tuple[float, float]]:
        seen: List[Tuple[float, float]] = []
        for g in self.records:
            if self.is_blacklisted(g.x, g.y) and all(
                    math.hypot(g.x - sx, g.y - sy) > self.region_radius for sx, sy in seen):
                seen.append((g.x, g.y))
        return seen


class StuckDetector:
    """Flags lack of progress: the robot stayed within ``min_displacement``
    metres of where it was ``window`` seconds ago while a goal was active."""

    def __init__(self, window: float = 20.0, min_displacement: float = 0.15):
        self.window = window
        self.min_displacement = min_displacement
        self._samples: Deque[Tuple[float, float, float]] = deque()

    def reset(self) -> None:
        self._samples.clear()

    def update(self, stamp: float, x: float, y: float) -> None:
        self._samples.append((stamp, x, y))
        while self._samples and stamp - self._samples[0][0] > self.window * 1.5:
            self._samples.popleft()

    def is_stuck(self, stamp: Optional[float] = None) -> bool:
        if len(self._samples) < 2:
            return False
        now = stamp if stamp is not None else self._samples[-1][0]
        if now - self._samples[0][0] < self.window:
            return False
        window = [s for s in self._samples if now - s[0] <= self.window]
        if not window:
            return False
        _, x0, y0 = window[0]
        return all(math.hypot(x - x0, y - y0) < self.min_displacement for _, x, y in window)
