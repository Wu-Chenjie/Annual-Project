from __future__ import annotations

import numpy as np
import pytest

from core.obstacles import ObstacleField
from core.planning import AStar, PlannerError


def test_occupancy_grid_contains_world_rejects_outside_points():
    grid = ObstacleField().to_voxel_grid(np.array([[0, 0, 0], [2, 2, 2]], dtype=float), 1.0)

    assert grid.contains_world(np.array([0.5, 0.5, 0.5]))
    assert not grid.contains_world(np.array([-0.1, 0.5, 0.5]))
    assert not grid.contains_world(np.array([2.1, 0.5, 0.5]))


def test_astar_rejects_out_of_bounds_start_or_goal():
    grid = ObstacleField().to_voxel_grid(np.array([[0, 0, 0], [3, 3, 3]], dtype=float), 1.0)
    planner = AStar()

    with pytest.raises(PlannerError, match="start"):
        planner.plan(np.array([-1.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]), grid)
    with pytest.raises(PlannerError, match="goal"):
        planner.plan(np.array([1.0, 1.0, 1.0]), np.array([9.0, 2.0, 2.0]), grid)
