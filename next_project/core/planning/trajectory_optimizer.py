from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    t: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class TrajectoryResult:
    method: str
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    timestamps: np.ndarray
    samples: list[TrajectorySample]
    path_length: float
    clearance_checked: bool
    accepted: bool
    fallback_reason: str | None = None

    def to_dict(self) -> dict:
        return {
            "method": self.method,
            "positions": self.positions.tolist(),
            "velocities": self.velocities.tolist(),
            "accelerations": self.accelerations.tolist(),
            "timestamps": self.timestamps.tolist(),
            "samples": [sample.to_dict() for sample in self.samples],
            "path_length": self.path_length,
            "clearance_checked": self.clearance_checked,
            "accepted": self.accepted,
            "fallback_reason": self.fallback_reason,
        }


class TrajectoryOptimizer:
    """Lightweight trajectory post-processor for discrete planner paths."""

    def __init__(
        self,
        *,
        nominal_speed: float = 1.0,
        sample_dt: float = 0.2,
        smoothing_window: int = 5,
    ):
        self.nominal_speed = max(float(nominal_speed), 1e-3)
        self.sample_dt = max(float(sample_dt), 1e-3)
        self.smoothing_window = max(int(smoothing_window), 1)

    def optimize(
        self,
        path: np.ndarray,
        *,
        clearance_checker=None,
        method: str = "moving_average",
        fallback_to_raw: bool = True,
    ) -> TrajectoryResult:
        raw = np.asarray(path, dtype=float)
        if raw.ndim != 2 or raw.shape[1] != 3:
            raise ValueError("path must be shaped (N, 3)")
        if len(raw) == 0:
            empty = np.zeros((0, 3), dtype=float)
            return TrajectoryResult(
                method=method,
                positions=empty,
                velocities=empty,
                accelerations=empty,
                timestamps=np.zeros(0, dtype=float),
                samples=[],
                path_length=0.0,
                clearance_checked=clearance_checker is not None,
                accepted=True,
            )

        resampled = self._resample_by_speed(raw)
        optimized = self._smooth_positions(resampled) if method != "none" else resampled.copy()
        accepted = True
        fallback_reason = None
        clearance_checked = clearance_checker is not None

        if clearance_checker is not None and len(optimized) > 1 and not bool(clearance_checker(optimized)):
            if fallback_to_raw:
                optimized = resampled
                accepted = False
                fallback_reason = "optimized_path_failed_clearance_gate"
            else:
                raise ValueError("optimized path failed clearance gate")

        timestamps = self._build_timestamps(optimized)
        velocities, accelerations = self._differentiate(optimized, timestamps)
        samples = [
            TrajectorySample(
                t=float(t),
                position=tuple(pos.tolist()),
                velocity=tuple(vel.tolist()),
                acceleration=tuple(acc.tolist()),
            )
            for t, pos, vel, acc in zip(timestamps, optimized, velocities, accelerations)
        ]
        return TrajectoryResult(
            method=method,
            positions=optimized,
            velocities=velocities,
            accelerations=accelerations,
            timestamps=timestamps,
            samples=samples,
            path_length=self._path_length(optimized),
            clearance_checked=clearance_checked,
            accepted=accepted,
            fallback_reason=fallback_reason,
        )

    def _resample_by_speed(self, path: np.ndarray) -> np.ndarray:
        if len(path) < 2:
            return path.copy()
        spacing = self.nominal_speed * self.sample_dt
        diffs = np.diff(path, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        total = float(np.sum(seg_lens))
        if total < 1e-9:
            return np.vstack([path[0], path[-1]])
        count = max(2, int(np.ceil(total / spacing)) + 1)
        cumulative = np.concatenate([[0.0], np.cumsum(seg_lens)])
        targets = np.linspace(0.0, total, count)
        result = np.zeros((count, 3), dtype=float)
        for axis in range(3):
            result[:, axis] = np.interp(targets, cumulative, path[:, axis])
        result[0] = path[0]
        result[-1] = path[-1]
        return result

    def _smooth_positions(self, path: np.ndarray) -> np.ndarray:
        if len(path) <= 2 or self.smoothing_window <= 1:
            return path.copy()
        radius = self.smoothing_window // 2
        smoothed = path.copy()
        for idx in range(1, len(path) - 1):
            left = max(0, idx - radius)
            right = min(len(path), idx + radius + 1)
            smoothed[idx] = np.mean(path[left:right], axis=0)
        smoothed[0] = path[0]
        smoothed[-1] = path[-1]
        return smoothed

    def _build_timestamps(self, path: np.ndarray) -> np.ndarray:
        if len(path) == 0:
            return np.zeros(0, dtype=float)
        if len(path) == 1:
            return np.array([0.0], dtype=float)
        seg_lens = np.linalg.norm(np.diff(path, axis=0), axis=1)
        seg_dt = np.maximum(seg_lens / self.nominal_speed, self.sample_dt)
        return np.concatenate([[0.0], np.cumsum(seg_dt)])

    @staticmethod
    def _differentiate(positions: np.ndarray, timestamps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n = len(positions)
        velocities = np.zeros((n, 3), dtype=float)
        accelerations = np.zeros((n, 3), dtype=float)
        if n <= 1:
            return velocities, accelerations
        for i in range(1, n):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            velocities[i] = (positions[i] - positions[i - 1]) / dt
        velocities[0] = velocities[1]
        for i in range(1, n):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            accelerations[i] = (velocities[i] - velocities[i - 1]) / dt
        accelerations[0] = accelerations[1] if n > 1 else np.zeros(3, dtype=float)
        return velocities, accelerations

    @staticmethod
    def _path_length(path: np.ndarray) -> float:
        if len(path) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
