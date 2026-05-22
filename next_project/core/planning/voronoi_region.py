"""Lightweight Voronoi-region stability scoring for online replanning."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VoronoiRegionScore:
    enabled: bool
    region_id: str
    side: int
    stability_bonus: float
    obstacle_count: int


class VoronoiRegionSelector:
    """Score whether a subgoal stays in a stable local Voronoi proxy region."""

    def __init__(self, *, weight: float = 0.25):
        self.weight = max(float(weight), 0.0)

    def score(
        self,
        pose: np.ndarray,
        goal: np.ndarray,
        candidate: np.ndarray,
        obstacle_centers: np.ndarray,
        *,
        previous_region_id: str | None = None,
        previous_side: int = 0,
    ) -> VoronoiRegionScore:
        centers = np.asarray(obstacle_centers, dtype=float)
        obstacle_count = int(len(centers)) if centers.ndim else 0
        if centers.ndim != 2 or centers.shape[1] < 2 or obstacle_count < 2:
            return VoronoiRegionScore(False, "", 0, 0.0, obstacle_count)

        pose_xy = np.asarray(pose, dtype=float)[:2]
        goal_xy = np.asarray(goal, dtype=float)[:2]
        cand_xy = np.asarray(candidate, dtype=float)[:2]
        centers_xy = centers[:, :2]

        axis = goal_xy - pose_xy
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1e-9:
            side = int(np.sign(previous_side))
            distances = np.linalg.norm(centers_xy - cand_xy, axis=1)
        else:
            axis_unit = axis / axis_norm
            center_s = (centers_xy - pose_xy) @ axis_unit
            cand_s = float((cand_xy - pose_xy) @ axis_unit)
            distances = np.abs(center_s - cand_s)
            rel = cand_xy - pose_xy
            cross_z = axis[0] * rel[1] - axis[1] * rel[0]
            side = 1 if cross_z > 1e-9 else -1 if cross_z < -1e-9 else 0

        nearest = np.argsort(distances)[:2]
        pair = tuple(sorted(int(i) for i in nearest))
        region_id = f"{pair[0]}:{pair[1]}"

        bonus = 0.0
        if previous_region_id and previous_region_id == region_id and previous_side and side:
            bonus = self.weight if side == previous_side else -self.weight
        elif previous_side and side:
            bonus = 0.5 * self.weight if side == previous_side else -0.5 * self.weight

        return VoronoiRegionScore(True, region_id, side, float(bonus), obstacle_count)
