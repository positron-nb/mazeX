import numpy as np
import pytest

from mazemind_exploration import memory as mem
from mazemind_exploration.frontier import Frontier
from mazemind_exploration.memory import GoalMemory, StuckDetector
from mazemind_exploration.selection import (Candidate, Selector, Weights, score_candidates,
                                            success_estimate)


def cand(x, y, cost, info, size=10, clearance=0.4, success=1.0, revisit=0.0, failure=0.0):
    f = Frontier(id=0, cells=np.zeros((size, 2), dtype=int), centroid=(x, y),
                 target_cell=(0, 0), target=(x, y))
    return Candidate(frontier=f, travel_cost=cost, info_gain=info, clearance=clearance,
                     bottleneck=clearance, success_estimate=success,
                     revisit_penalty=revisit, failure_penalty=failure)


def test_nearest_picks_shortest_travel_cost():
    cands = [cand(5, 0, 5.0, 900), cand(1, 0, 1.0, 10), cand(3, 0, 3.0, 50)]
    assert Selector('nearest').select(cands).target == (1, 0)


def test_weighted_prefers_information_at_similar_cost():
    cands = [cand(2, 0, 2.0, 50), cand(0, 2, 2.1, 400)]
    assert Selector('weighted').select(cands).target == (0, 2)


def test_weighted_avoids_long_backtracking_for_small_gain():
    cands = [cand(1, 0, 1.0, 300), cand(9, 0, 9.0, 330)]
    assert Selector('weighted').select(cands).target == (1, 0)


def test_failure_penalty_changes_choice():
    a = cand(2, 0, 2.0, 300, failure=1.0)
    b = cand(0, 2, 2.0, 250)
    assert Selector('weighted').select([a, b]).target == (0, 2)


def test_features_are_normalised():
    cands = [cand(1, 0, 1.0, 100, size=5), cand(2, 0, 4.0, 400, size=20)]
    score_candidates(cands, Weights(), clearance_cap=0.45)
    for c in cands:
        for name, value in c.features.items():
            assert 0.0 <= value <= 1.0, name
    assert cands[1].features['info'] == 1.0
    assert cands[0].features['distance'] == pytest.approx(0.25)


def test_random_strategy_is_seeded():
    cands = [cand(i, 0, float(i + 1), 10) for i in range(10)]
    a = [c.target for c in Selector('random', seed=4).rank(list(cands))]
    b = [c.target for c in Selector('random', seed=4).rank(list(cands))]
    c = [c.target for c in Selector('random', seed=5).rank(list(cands))]
    assert a == b and a != c


def test_unknown_strategy_rejected():
    with pytest.raises(ValueError):
        Selector('greedy')


def test_success_estimate_range():
    assert success_estimate(0.15, 0.15, 0.35) == 0.0
    assert success_estimate(0.40, 0.15, 0.35) == 1.0
    assert success_estimate(0.25, 0.15, 0.35) == pytest.approx(0.5)


def test_goal_memory_blacklists_after_failures_and_attempts():
    m = GoalMemory(region_radius=0.5, max_failures=2, max_attempts=3)
    m.record(1.0, 1.0, mem.FAILED, 0.0)
    assert not m.is_blacklisted(1.1, 1.0)
    assert m.failure_penalty(1.1, 1.0) == 0.5
    m.record(1.2, 1.0, mem.STUCK, 1.0)
    assert m.is_blacklisted(1.0, 1.0)
    assert not m.is_blacklisted(3.0, 3.0)
    # preempted / resolved goals never blacklist
    for t in range(5):
        m.record(5.0, 5.0, mem.PREEMPTED, t)
    assert not m.is_blacklisted(5.0, 5.0)
    for t in range(3):
        m.record(7.0, 7.0, mem.REACHED, t)
    assert m.is_blacklisted(7.0, 7.0)          # reached 3x and still a frontier
    assert len(m.blacklisted_points()) == 2


def test_stuck_detector():
    s = StuckDetector(window=10.0, min_displacement=0.2)
    for t in range(0, 12):
        s.update(float(t), 0.01 * t, 0.0)
    assert s.is_stuck()
    s.reset()
    for t in range(0, 12):
        s.update(float(t), 0.1 * t, 0.0)
    assert not s.is_stuck()
    s.reset()
    s.update(0.0, 0.0, 0.0)
    s.update(5.0, 0.0, 0.0)
    assert not s.is_stuck()                     # window not yet covered
