from __future__ import annotations

import sys
import json
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
_project = _here.parent
if str(_project) not in sys.path:
    sys.path.insert(0, str(_project))

from core.planning.trajectory_optimizer import TrajectoryOptimizer
from simulations.formation_simulation import SimulationConfig
from simulations.obstacle_scenario import ObstacleScenarioSimulation


def test_trajectory_optimizer_returns_time_parameterized_samples():
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.2, smoothing_window=3)
    path = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 1.0],
    ], dtype=float)
    result = optimizer.optimize(path, method="moving_average")
    assert result.accepted is True
    assert len(result.positions) >= len(path)
    assert len(result.samples) == len(result.timestamps)
    assert result.timestamps[0] == 0.0
    assert result.path_length > 0.0
    assert result.max_speed > 0.0
    assert result.max_acceleration >= 0.0
    assert result.mean_jerk >= 0.0
    assert result.max_jerk >= result.mean_jerk
    assert result.jerk_squared_integral >= 0.0
    assert result.snap_squared_integral >= 0.0
    payload = result.to_dict()
    assert payload["max_speed"] == result.max_speed
    assert "max_jerk" in payload
    assert "jerk_squared_integral" in payload
    assert "snap_squared_integral" in payload
    json.dumps(result.to_dict())


def test_trajectory_optimizer_falls_back_when_clearance_gate_rejects_smoothed_path():
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.2, smoothing_window=5)
    path = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [2.0, 0.0, 1.0],
    ], dtype=float)
    result = optimizer.optimize(
        path,
        method="moving_average",
        clearance_checker=lambda candidate: len(candidate) == len(path) and np.allclose(candidate, path),
        fallback_to_raw=True,
    )
    assert result.accepted is False
    assert result.fallback_reason == "optimized_path_failed_clearance_gate"
    assert np.allclose(result.positions[0], path[0])
    assert np.allclose(result.positions[-1], path[-1])


def test_obstacle_scenario_emits_planned_trajectory_when_enabled():
    config = SimulationConfig(
        max_sim_time=0.5,
        use_smc=True,
        num_followers=1,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        trajectory_optimizer_enabled=True,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )
    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    planned_trajectory = result.get("planned_trajectory")
    assert planned_trajectory is not None
    assert planned_trajectory["timestamps"][0] == 0.0
    assert len(planned_trajectory["samples"]) == len(planned_trajectory["timestamps"])
    assert "max_speed" in planned_trajectory
    assert "mean_jerk" in planned_trajectory
    assert "jerk_squared_integral" in planned_trajectory
    assert "snap_squared_integral" in planned_trajectory


def test_trajectory_optimizer_supports_minimum_jerk_method():
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.1, smoothing_window=3)
    path = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 1.0],
    ], dtype=float)

    result = optimizer.optimize(path, method="minimum_jerk")

    assert result.method == "minimum_jerk"
    assert result.accepted is True
    assert len(result.positions) > len(path)
    assert result.jerk_squared_integral >= 0.0
    assert np.allclose(result.positions[0], path[0])
    assert np.allclose(result.positions[-1], path[-1])


def test_trajectory_optimizer_can_select_by_snap_proxy_cost():
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.1, smoothing_window=3)
    path = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [2.0, 1.0, 1.0],
    ], dtype=float)

    result = optimizer.optimize(path, method="min_snap_proxy")

    assert result.method.startswith("min_snap_proxy:")
    assert result.accepted is True
    assert result.jerk_squared_integral >= 0.0
    assert result.snap_squared_integral >= 0.0
