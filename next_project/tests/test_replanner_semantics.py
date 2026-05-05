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


def test_dynamic_sensor_obstacle_expires_after_ttl():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(16, 16, 4))
    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=4.0,
        sensor_obstacle_ttl_steps=2,
        sensor_clear_confirm_steps=1,
    )
    pose = np.array([2.0, 2.0, 1.0], dtype=float)
    readings = np.array([2.0, 4.0, 4.0, 4.0, 4.0, 4.0], dtype=float)
    replanner._update_grid_from_sensor(pose, readings)
    hit_idx = grid.world_to_index(np.array([4.0, 2.0, 1.0], dtype=float))
    assert replanner._sensor_occupied[hit_idx]
    replanner._decay_sensor_obstacles()
    assert replanner._sensor_occupied[hit_idx]
    replanner._decay_sensor_obstacles()
    assert not replanner._sensor_occupied[hit_idx]


def test_static_obstacle_is_never_cleared_by_sensor_decay():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(16, 16, 4))
    static_idx = (5, 5, 1)
    grid.data[static_idx] = 1
    replanner = WindowReplanner(planner=DummyPlanner(), grid=grid, horizon=4.0)
    replanner._static_occupied[static_idx] = True
    replanner._sensor_occupied[static_idx] = True
    replanner._sensor_ttl[static_idx] = 0
    replanner._decay_sensor_obstacles()
    assert grid.data[static_idx] == 1
    assert not replanner._sensor_occupied[static_idx]
