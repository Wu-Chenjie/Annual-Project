from __future__ import annotations

import numpy as np

from core.planning.mpc_tracker import MPCFeasibilityEvaluator


def test_mpc_feasibility_evaluator_accepts_within_limits():
    evaluator = MPCFeasibilityEvaluator(max_speed=2.0, max_acceleration=3.0)
    positions = np.array([[0.0, 0.0, 1.0], [0.5, 0.0, 1.0], [1.0, 0.0, 1.0]], dtype=float)
    timestamps = np.array([0.0, 0.5, 1.0], dtype=float)

    result = evaluator.evaluate_arrays(positions=positions, timestamps=timestamps)

    assert result.evaluated is True
    assert result.feasible is True
    assert result.max_velocity_violation == 0.0
    assert result.max_acceleration_violation == 0.0
    assert result.saturation_ratio == 0.0
    assert result.recommendation == "continue_mpc_prototype"


def test_mpc_feasibility_evaluator_flags_velocity_and_acceleration_violations():
    evaluator = MPCFeasibilityEvaluator(max_speed=1.0, max_acceleration=1.0)
    positions = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0], [3.0, 3.0, 1.0]], dtype=float)
    timestamps = np.array([0.0, 0.5, 1.0], dtype=float)

    result = evaluator.evaluate_arrays(positions=positions, timestamps=timestamps)

    assert result.evaluated is True
    assert result.feasible is False
    assert result.max_velocity_violation > 0.0
    assert result.max_acceleration_violation > 0.0
    assert result.saturation_ratio > 0.0
    assert result.recommendation == "defer_online_mpc"
    assert result.to_dict()["feasible"] is False
