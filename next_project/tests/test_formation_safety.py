from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
_project = _here.parent
if str(_project) not in sys.path:
    sys.path.insert(0, str(_project))

from core.formation_safety import (
    DownwashZone,
    deconflict_follower_target,
    downwash_zone,
    follower_safety_correction,
    is_in_downwash_zone,
    min_inter_drone_distance,
    nominal_target_ready_for_recovery,
)


def test_min_inter_drone_distance_returns_smallest_pair():
    positions = [
        np.array([0.0, 0.0, 1.0], dtype=float),
        np.array([1.0, 0.0, 1.0], dtype=float),
        np.array([0.4, 0.0, 1.0], dtype=float),
    ]
    assert abs(min_inter_drone_distance(positions) - 0.4) < 1e-9


def test_downwash_zone_detects_vertical_stack():
    zone = downwash_zone(radius=0.5, height=1.0)
    assert is_in_downwash_zone(
        np.array([0.0, 0.0, 2.0], dtype=float),
        np.array([0.2, 0.1, 1.3], dtype=float),
        zone,
    )
    assert not is_in_downwash_zone(
        np.array([0.0, 0.0, 2.0], dtype=float),
        np.array([0.8, 0.0, 1.3], dtype=float),
        zone,
    )


def test_follower_safety_correction_shrinks_toward_leader_when_target_blocked():
    leader = np.array([0.0, 0.0, 1.0], dtype=float)
    raw_target = np.array([1.0, 0.0, 1.0], dtype=float)

    def signed_distance(p: np.ndarray) -> float:
        return -0.2 if p[0] > 0.6 else 1.0

    corrected = follower_safety_correction(
        leader,
        raw_target,
        current_pos=np.array([0.1, 0.0, 1.0], dtype=float),
        signed_distance=signed_distance,
        segment_is_safe=lambda path, clearance: True,
        project_to_free=None,
        min_clearance=0.3,
        shrink_steps=10,
    )
    assert corrected[0] <= 0.6 + 1e-9
    assert corrected[0] >= 0.0


def test_follower_safety_correction_keeps_current_when_no_candidate_safe():
    current = np.array([0.2, 0.0, 1.0], dtype=float)
    corrected = follower_safety_correction(
        np.array([0.0, 0.0, 1.0], dtype=float),
        np.array([1.0, 0.0, 1.0], dtype=float),
        current_pos=current,
        signed_distance=lambda p: -1.0 if p[0] > 0.2 else 0.5,
        segment_is_safe=lambda path, clearance: False,
        project_to_free=None,
        min_clearance=0.3,
        shrink_steps=4,
    )
    np.testing.assert_allclose(corrected, current)


def test_deconflict_follower_target_avoids_reserved_position():
    corrected = deconflict_follower_target(
        np.array([1.0, 0.0, 1.0], dtype=float),
        leader_pos=np.array([0.0, 0.0, 1.0], dtype=float),
        reserved_positions=[np.array([1.0, 0.0, 1.0], dtype=float)],
        current_pos=np.array([0.8, 0.0, 1.0], dtype=float),
        signed_distance=lambda p: 1.0,
        segment_is_safe=lambda path, clearance: True,
        min_clearance=0.2,
        min_inter_distance=0.4,
        downwash=DownwashZone(radius=0.3, height=0.6),
        vertical_step=0.5,
        lateral_step=0.4,
    )
    assert np.linalg.norm(corrected - np.array([1.0, 0.0, 1.0], dtype=float)) > 1e-6
    assert np.linalg.norm(corrected - np.array([1.0, 0.0, 1.0], dtype=float)) >= 0.4 - 1e-9


def test_nominal_target_ready_for_recovery_accepts_clear_nominal_target():
    ready = nominal_target_ready_for_recovery(
        np.array([1.0, 0.0, 1.0], dtype=float),
        reserved_positions=[np.array([0.0, 0.0, 1.0], dtype=float)],
        current_pos=np.array([0.8, 0.0, 1.0], dtype=float),
        signed_distance=lambda p: 1.0,
        segment_is_safe=lambda path, clearance: True,
        min_clearance=0.2,
        min_inter_distance=0.35,
        downwash=DownwashZone(radius=0.2, height=0.5),
        recovery_margin=0.05,
    )
    assert ready
