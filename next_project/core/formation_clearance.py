"""Formation-aware obstacle clearance constraints.

The module makes the follower clearance rule explicit:

    min(SDF(x), SDF(x + d_1), ..., SDF(x + d_n)) >= r_clear

where ``x`` is the leader reference point, ``d_i`` are follower offsets, and
``r_clear`` is the configured vehicle/safety clearance. A leader-only planner
uses only ``SDF(x) >= r_clear``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


ArrayLike = np.ndarray
SignedDistanceFn = Callable[[ArrayLike], float]


@dataclass(frozen=True)
class FormationClearanceEvaluation:
    mode: str
    required_clearance: float
    min_leader_signed_distance: float
    min_follower_signed_distance: float | None
    min_formation_signed_distance: float
    min_leader_margin: float
    min_follower_margin: float | None
    min_formation_margin: float
    violation_count: int
    follower_violation_count: int
    sample_count: int
    probe_count: int

    def to_dict(self) -> dict[str, float | int | str | None]:
        return {
            "mode": self.mode,
            "required_clearance": float(self.required_clearance),
            "min_leader_signed_distance": float(self.min_leader_signed_distance),
            "min_follower_signed_distance": (
                None if self.min_follower_signed_distance is None else float(self.min_follower_signed_distance)
            ),
            "min_formation_signed_distance": float(self.min_formation_signed_distance),
            "min_leader_margin": float(self.min_leader_margin),
            "min_follower_margin": None if self.min_follower_margin is None else float(self.min_follower_margin),
            "min_formation_margin": float(self.min_formation_margin),
            "violation_count": int(self.violation_count),
            "follower_violation_count": int(self.follower_violation_count),
            "sample_count": int(self.sample_count),
            "probe_count": int(self.probe_count),
        }


class FormationClearancePolicy:
    """Evaluate leader-only and formation-aware clearance constraints."""

    def __init__(
        self,
        *,
        signed_distance: SignedDistanceFn,
        follower_offsets: Iterable[ArrayLike],
        base_clearance: float,
        sample_spacing: float = 0.1,
        formation_aware: bool = True,
    ) -> None:
        self.signed_distance = signed_distance
        self.follower_offsets = [np.asarray(offset, dtype=float).reshape(3) for offset in follower_offsets]
        self.base_clearance = float(base_clearance)
        self.sample_spacing = max(float(sample_spacing), 1e-6)
        self.formation_aware = bool(formation_aware)

    @property
    def mode(self) -> str:
        return "formation_aware_clearance" if self.formation_aware else "leader_only_planner"

    @property
    def envelope_radius(self) -> float:
        if not self.follower_offsets:
            return 0.0
        return float(max(np.linalg.norm(offset) for offset in self.follower_offsets))

    @property
    def required_leader_clearance(self) -> float:
        if not self.formation_aware:
            return self.base_clearance
        return float(self.base_clearance + self.envelope_radius)

    def evaluate_path(
        self,
        path: ArrayLike,
        *,
        base_clearance: float | None = None,
        sample_spacing: float | None = None,
    ) -> FormationClearanceEvaluation:
        clearance = self.base_clearance if base_clearance is None else float(base_clearance)
        samples = self._sample_path(path, sample_spacing=sample_spacing)
        if len(samples) == 0:
            samples = np.zeros((1, 3), dtype=float)

        min_leader_sd = float("inf")
        min_follower_sd: float | None = None
        violation_count = 0
        follower_violation_count = 0
        probe_count = 0

        for point in samples:
            leader_sd = float(self.signed_distance(point))
            min_leader_sd = min(min_leader_sd, leader_sd)
            probe_count += 1
            if leader_sd < clearance - 1e-9:
                violation_count += 1

            if not self.formation_aware:
                continue

            for offset in self.follower_offsets:
                follower_sd = float(self.signed_distance(point + offset))
                min_follower_sd = follower_sd if min_follower_sd is None else min(min_follower_sd, follower_sd)
                probe_count += 1
                if follower_sd < clearance - 1e-9:
                    violation_count += 1
                    follower_violation_count += 1

        min_formation_sd = min_leader_sd if min_follower_sd is None else min(min_leader_sd, min_follower_sd)
        min_follower_margin = None if min_follower_sd is None else float(min_follower_sd - clearance)
        return FormationClearanceEvaluation(
            mode=self.mode,
            required_clearance=float(clearance + (self.envelope_radius if self.formation_aware else 0.0)),
            min_leader_signed_distance=float(min_leader_sd),
            min_follower_signed_distance=None if min_follower_sd is None else float(min_follower_sd),
            min_formation_signed_distance=float(min_formation_sd),
            min_leader_margin=float(min_leader_sd - clearance),
            min_follower_margin=min_follower_margin,
            min_formation_margin=float(min_formation_sd - clearance),
            violation_count=int(violation_count),
            follower_violation_count=int(follower_violation_count),
            sample_count=int(len(samples)),
            probe_count=int(probe_count),
        )

    def is_path_safe(
        self,
        path: ArrayLike,
        *,
        base_clearance: float | None = None,
        sample_spacing: float | None = None,
    ) -> bool:
        return self.evaluate_path(
            path,
            base_clearance=base_clearance,
            sample_spacing=sample_spacing,
        ).violation_count == 0

    def _sample_path(self, path: ArrayLike, *, sample_spacing: float | None = None) -> np.ndarray:
        points = np.asarray(path, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            return np.zeros((0, 3), dtype=float)
        if len(points) < 2:
            return points.copy()

        spacing = max(float(sample_spacing or self.sample_spacing), 1e-6)
        samples: list[np.ndarray] = [points[0].copy()]
        for idx in range(len(points) - 1):
            start = points[idx]
            end = points[idx + 1]
            dist = float(np.linalg.norm(end - start))
            count = max(1, int(np.ceil(dist / spacing)))
            for step in range(1, count + 1):
                samples.append(start + (end - start) * (step / count))
        return np.asarray(samples, dtype=float)


__all__ = ["FormationClearanceEvaluation", "FormationClearancePolicy"]
