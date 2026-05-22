from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class MPCFeasibilityResult:
    evaluated: bool
    feasible: bool
    tracking_rms_proxy: float
    max_velocity_violation: float
    max_acceleration_violation: float
    saturation_ratio: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


class MPCFeasibilityEvaluator:
    """Offline trajectory feasibility scan for a future MPC tracking layer."""

    def __init__(self, *, max_speed: float, max_acceleration: float, rms_limit: float = 0.75):
        self.max_speed = max(float(max_speed), 1e-6)
        self.max_acceleration = max(float(max_acceleration), 1e-6)
        self.rms_limit = max(float(rms_limit), 0.0)

    def evaluate_trajectory(self, trajectory) -> MPCFeasibilityResult:
        return self.evaluate_arrays(
            positions=np.asarray(trajectory.positions, dtype=float),
            timestamps=np.asarray(trajectory.timestamps, dtype=float),
        )

    def evaluate_arrays(self, *, positions: np.ndarray, timestamps: np.ndarray) -> MPCFeasibilityResult:
        positions = np.asarray(positions, dtype=float)
        timestamps = np.asarray(timestamps, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 2:
            return MPCFeasibilityResult(
                evaluated=False,
                feasible=False,
                tracking_rms_proxy=0.0,
                max_velocity_violation=0.0,
                max_acceleration_violation=0.0,
                saturation_ratio=0.0,
                recommendation="missing_trajectory",
            )

        velocities, accelerations = self._differentiate(positions, timestamps)
        speed_norms = np.linalg.norm(velocities, axis=1)
        acc_norms = np.linalg.norm(accelerations, axis=1)
        max_velocity_violation = float(max(0.0, float(np.max(speed_norms)) - self.max_speed))
        max_acceleration_violation = float(max(0.0, float(np.max(acc_norms)) - self.max_acceleration))
        tracking_rms_proxy = float(np.sqrt(np.mean(np.minimum(acc_norms / self.max_acceleration, 2.0) ** 2)))
        saturated = (speed_norms > self.max_speed) | (acc_norms > self.max_acceleration)
        saturation_ratio = float(np.mean(saturated))
        feasible = (
            max_velocity_violation == 0.0
            and max_acceleration_violation == 0.0
            and tracking_rms_proxy <= self.rms_limit
        )
        return MPCFeasibilityResult(
            evaluated=True,
            feasible=bool(feasible),
            tracking_rms_proxy=tracking_rms_proxy,
            max_velocity_violation=max_velocity_violation,
            max_acceleration_violation=max_acceleration_violation,
            saturation_ratio=saturation_ratio,
            recommendation="continue_mpc_prototype" if feasible else "defer_online_mpc",
        )

    @staticmethod
    def _differentiate(positions: np.ndarray, timestamps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if len(timestamps) != len(positions):
            timestamps = np.arange(len(positions), dtype=float)
        velocities = np.zeros_like(positions, dtype=float)
        accelerations = np.zeros_like(positions, dtype=float)
        for i in range(1, len(positions)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            velocities[i] = (positions[i] - positions[i - 1]) / dt
        velocities[0] = velocities[1]
        for i in range(1, len(positions)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            accelerations[i] = (velocities[i] - velocities[i - 1]) / dt
        accelerations[0] = accelerations[1]
        return velocities, accelerations
