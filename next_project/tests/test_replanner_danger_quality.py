from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.obstacles import OccupancyGrid
from core.planning.replanner import WindowReplanner


class LocalPlanner:
    def plan(self, start, goal, grid, **kw):
        return np.array([start, goal], dtype=float)

    def smooth(self, path, grid=None):
        return np.asarray(path, dtype=float)


class DangerPlanner:
    def __init__(self, path):
        self._path = np.asarray(path, dtype=float)

    def plan(self, start, goal, grid, **kw):
        return self._path.copy()

    def smooth(self, path, grid=None):
        return np.asarray(path, dtype=float)


def test_danger_path_rejected_when_much_longer_than_local():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(32, 32, 4))
    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([5.0, 0.0, 0.0], dtype=float)
    danger_path = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [5.0, 5.0, 0.0],
        [5.0, 0.0, 0.0],
    ], dtype=float)
    replanner = WindowReplanner(
        planner=LocalPlanner(),
        grid=grid,
        danger_planner=DangerPlanner(danger_path),
        gnn_path_ratio_limit=1.2,
    )
    path = replanner._replan_danger(pose, goal)
    assert path is None
    assert replanner.last_fallback_reason == "danger_path_too_long"
