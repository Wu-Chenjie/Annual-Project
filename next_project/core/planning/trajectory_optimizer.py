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
    max_speed: float
    max_acceleration: float
    mean_jerk: float
    max_jerk: float
    jerk_squared_integral: float
    snap_squared_integral: float
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
            "max_speed": self.max_speed,
            "max_acceleration": self.max_acceleration,
            "mean_jerk": self.mean_jerk,
            "max_jerk": self.max_jerk,
            "jerk_squared_integral": self.jerk_squared_integral,
            "snap_squared_integral": self.snap_squared_integral,
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
                max_speed=0.0,
                max_acceleration=0.0,
                mean_jerk=0.0,
                max_jerk=0.0,
                jerk_squared_integral=0.0,
                snap_squared_integral=0.0,
                clearance_checked=clearance_checker is not None,
                accepted=True,
            )
        if method in {"min_snap_proxy", "min_jerk_cost"}:
            return self._select_by_smoothness_cost(
                raw,
                clearance_checker=clearance_checker,
                fallback_to_raw=fallback_to_raw,
                selector=method,
            )

        resampled = self._resample_by_speed(raw)
        if method == "none":
            optimized = resampled.copy()
        elif method in {"minimum_jerk", "quintic_minimum_jerk"}:
            optimized = self._minimum_jerk_positions(raw)
        else:
            optimized = self._smooth_positions(resampled)
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
        mean_jerk, max_jerk, jerk_squared_integral = self._jerk_metrics(accelerations, timestamps)
        snap_squared_integral = self._snap_squared_integral(accelerations, timestamps)
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
            max_speed=self._max_norm(velocities),
            max_acceleration=self._max_norm(accelerations),
            mean_jerk=mean_jerk,
            max_jerk=max_jerk,
            jerk_squared_integral=jerk_squared_integral,
            snap_squared_integral=snap_squared_integral,
            clearance_checked=clearance_checked,
            accepted=accepted,
            fallback_reason=fallback_reason,
        )

    def _select_by_smoothness_cost(
        self,
        raw: np.ndarray,
        *,
        clearance_checker=None,
        fallback_to_raw: bool,
        selector: str,
    ) -> TrajectoryResult:
        candidates = [
            self.optimize(
                raw,
                clearance_checker=clearance_checker,
                method="moving_average",
                fallback_to_raw=fallback_to_raw,
            ),
            self.optimize(
                raw,
                clearance_checker=clearance_checker,
                method="minimum_jerk",
                fallback_to_raw=fallback_to_raw,
            ),
        ]
        accepted = [candidate for candidate in candidates if candidate.accepted]
        pool = accepted or candidates
        if selector == "min_jerk_cost":
            # Backward-compatible selector. Kept for experiments that explicitly
            # want a translational smoothness proxy, but not recommended as the
            # primary quadrotor dynamics criterion.
            key = lambda candidate: candidate.jerk_squared_integral
        else:
            key = lambda candidate: candidate.snap_squared_integral
        best = min(pool, key=key)
        return TrajectoryResult(
            method=selector + ":" + best.method,
            positions=best.positions,
            velocities=best.velocities,
            accelerations=best.accelerations,
            timestamps=best.timestamps,
            samples=best.samples,
            path_length=best.path_length,
            max_speed=best.max_speed,
            max_acceleration=best.max_acceleration,
            mean_jerk=best.mean_jerk,
            max_jerk=best.max_jerk,
            jerk_squared_integral=best.jerk_squared_integral,
            snap_squared_integral=best.snap_squared_integral,
            clearance_checked=best.clearance_checked,
            accepted=best.accepted,
            fallback_reason=best.fallback_reason,
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

    def _minimum_jerk_positions(self, path: np.ndarray) -> np.ndarray:
        if len(path) < 2:
            return path.copy()
        samples: list[np.ndarray] = []
        for idx in range(len(path) - 1):
            start = path[idx]
            end = path[idx + 1]
            distance = float(np.linalg.norm(end - start))
            duration = max(distance / self.nominal_speed, self.sample_dt)
            count = max(2, int(np.ceil(duration / self.sample_dt)) + 1)
            if idx > 0:
                # Drop the duplicated knot shared with the previous segment.
                local_s = np.linspace(0.0, 1.0, count)[1:]
            else:
                local_s = np.linspace(0.0, 1.0, count)
            blend = 10.0 * local_s**3 - 15.0 * local_s**4 + 6.0 * local_s**5
            segment = start + (end - start) * blend[:, None]
            samples.extend(segment)
        if not samples:
            return path.copy()
        result = np.asarray(samples, dtype=float)
        result[0] = path[0]
        result[-1] = path[-1]
        return result

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

    @staticmethod
    def _max_norm(values: np.ndarray) -> float:
        if len(values) == 0:
            return 0.0
        return float(np.max(np.linalg.norm(values, axis=1)))

    @staticmethod
    def _jerk_metrics(accelerations: np.ndarray, timestamps: np.ndarray) -> tuple[float, float, float]:
        if len(accelerations) < 2:
            return 0.0, 0.0, 0.0
        jerk_norms = []
        jerk_squared_integral = 0.0
        for i in range(1, len(accelerations)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            jerk = (accelerations[i] - accelerations[i - 1]) / dt
            jerk_sq = float(np.dot(jerk, jerk))
            jerk_norms.append(float(np.sqrt(jerk_sq)))
            jerk_squared_integral += jerk_sq * dt
        if not jerk_norms:
            return 0.0, 0.0, 0.0
        return float(np.mean(jerk_norms)), float(np.max(jerk_norms)), float(jerk_squared_integral)

    @staticmethod
    def _snap_squared_integral(accelerations: np.ndarray, timestamps: np.ndarray) -> float:
        if len(accelerations) < 3:
            return 0.0
        jerk_values = []
        jerk_times = []
        for i in range(1, len(accelerations)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            jerk_values.append((accelerations[i] - accelerations[i - 1]) / dt)
            jerk_times.append(float(timestamps[i]))
        snap_integral = 0.0
        for i in range(1, len(jerk_values)):
            dt = max(float(jerk_times[i] - jerk_times[i - 1]), 1e-6)
            snap = (jerk_values[i] - jerk_values[i - 1]) / dt
            snap_integral += float(np.dot(snap, snap)) * dt
        return float(snap_integral)
