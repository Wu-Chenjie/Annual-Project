"""WindowReplanner subgoal 选择单测。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.obstacles import OccupancyGrid
from core.planning.replanner import WindowReplanner


class DummyPlanner:
    def plan(self, start, goal, grid, **kw):
        return np.array([start, goal], dtype=float)

    def smooth(self, path, grid=None):
        return np.asarray(path, dtype=float)


class BoxObstacleField:
    def __init__(self, min_corner, max_corner):
        self.min_corner = np.asarray(min_corner, dtype=float)
        self.max_corner = np.asarray(max_corner, dtype=float)

    def signed_distance(self, pos):
        p = np.asarray(pos, dtype=float)
        c = (self.min_corner + self.max_corner) / 2.0
        h = (self.max_corner - self.min_corner) / 2.0
        q = np.abs(p - c) - h
        outside = np.linalg.norm(np.maximum(q, 0.0))
        inside = min(max(q[0], q[1], q[2]), 0.0)
        return float(outside + inside)


def test_subgoal_prefers_safe_reference_point():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(12, 12, 4))
    obstacle_field = BoxObstacleField([4.0, -0.5, -1.0], [6.0, 0.5, 2.0])
    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=5.0,
        obstacle_field=obstacle_field,
    )
    replanner._global_ref_path = np.array([
        [0.0, 0.0, 0.0],
        [2.0, 2.0, 0.0],
        [4.0, 2.0, 0.0],
        [6.0, 2.0, 0.0],
        [8.0, 2.0, 0.0],
    ], dtype=float)

    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([8.0, 0.0, 0.0], dtype=float)
    subgoal = replanner._compute_subgoal(pose, goal)

    assert np.linalg.norm(subgoal - np.array([4.0, 2.0, 0.0])) < 1e-6


def test_subgoal_avoids_blocked_straight_segment():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(12, 12, 4))
    # 在 x 方向正前方放一个体素障碍，阻断直线 subgoal
    grid.data[2:5, 0:2, 0:2] = 1
    obstacle_field = BoxObstacleField([2.0, 0.0, 0.0], [5.0, 2.0, 2.0])

    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=5.0,
        obstacle_field=obstacle_field,
    )

    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([8.0, 0.0, 0.0], dtype=float)
    subgoal = replanner._compute_subgoal(pose, goal)

    assert not np.allclose(subgoal, np.array([5.0, 0.0, 0.0], dtype=float))
    assert replanner._segment_is_safe(pose, subgoal)

