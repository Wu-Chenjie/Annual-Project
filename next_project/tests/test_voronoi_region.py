from __future__ import annotations

import numpy as np

from core.planning.voronoi_region import VoronoiRegionSelector


def test_voronoi_region_prefers_previous_side_for_same_region():
    selector = VoronoiRegionSelector(weight=0.5)
    pose = np.array([0.0, 0.0, 1.0])
    goal = np.array([8.0, 0.0, 1.0])
    obstacles = np.array([[3.0, -1.0, 1.0], [3.0, 1.0, 1.0], [5.0, -1.0, 1.0], [5.0, 1.0, 1.0]])

    upper = selector.score(pose, goal, np.array([4.0, 1.5, 1.0]), obstacles, previous_side=1)
    lower = selector.score(pose, goal, np.array([4.0, -1.5, 1.0]), obstacles, previous_side=1)

    assert upper.enabled is True
    assert upper.region_id == lower.region_id
    assert upper.stability_bonus > lower.stability_bonus
    assert upper.side == 1
    assert lower.side == -1


def test_voronoi_region_disables_when_obstacles_are_insufficient():
    selector = VoronoiRegionSelector()
    result = selector.score(
        np.array([0.0, 0.0, 1.0]),
        np.array([8.0, 0.0, 1.0]),
        np.array([4.0, 1.0, 1.0]),
        np.array([[3.0, 0.0, 1.0]]),
    )

    assert result.enabled is False
    assert result.stability_bonus == 0.0
