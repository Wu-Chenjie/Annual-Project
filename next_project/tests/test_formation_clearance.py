from __future__ import annotations

import numpy as np

from core.formation_clearance import FormationClearancePolicy
from core.obstacles import ObstacleField


def test_formation_clearance_flags_follower_margin_that_leader_only_misses():
    obstacles = ObstacleField()
    obstacles.add_aabb([0.8, -0.2, -0.2], [1.2, 0.2, 0.2])
    leader_path = np.array([[0.0, -0.5, 0.0], [0.0, 0.5, 0.0]], dtype=float)

    leader_only = FormationClearancePolicy(
        signed_distance=obstacles.signed_distance,
        follower_offsets=[],
        base_clearance=0.1,
        sample_spacing=0.1,
        formation_aware=False,
    )
    formation_aware = FormationClearancePolicy(
        signed_distance=obstacles.signed_distance,
        follower_offsets=[np.array([1.0, 0.0, 0.0], dtype=float)],
        base_clearance=0.1,
        sample_spacing=0.1,
        formation_aware=True,
    )

    leader_eval = leader_only.evaluate_path(leader_path)
    formation_eval = formation_aware.evaluate_path(leader_path)

    assert leader_eval.mode == "leader_only_planner"
    assert leader_eval.violation_count == 0
    assert leader_eval.min_leader_margin > 0.0
    assert formation_eval.mode == "formation_aware_clearance"
    assert formation_eval.violation_count > 0
    assert formation_eval.follower_violation_count > 0
    assert formation_eval.min_follower_margin < 0.0
    assert formation_eval.min_formation_margin <= formation_eval.min_leader_margin


def test_required_leader_clearance_includes_follower_envelope_radius():
    policy = FormationClearancePolicy(
        signed_distance=lambda point: 10.0,
        follower_offsets=[
            np.array([-0.3, 0.4, 0.0], dtype=float),
            np.array([-1.2, 0.0, 0.0], dtype=float),
        ],
        base_clearance=0.2,
        sample_spacing=0.2,
        formation_aware=True,
    )

    assert policy.required_leader_clearance == 1.4
