"""Runtime topology observability helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np

from core.topology import TopologyGraph


@dataclass
class TopologyRuntimeMetrics:
    available: bool = False
    sample_count: int = 0
    mean_algebraic_connectivity: float = 0.0
    min_algebraic_connectivity: float = 0.0
    final_algebraic_connectivity: float = 0.0
    leader_control_energy_proxy: float = 0.0
    follower_control_energy_proxy: float = 0.0
    fault_event_count: int = 0
    reconfiguration_event_count: int = 0

    def to_dict(self) -> dict[str, bool | int | float]:
        return {
            "available": bool(self.available),
            "sample_count": int(self.sample_count),
            "mean_algebraic_connectivity": float(self.mean_algebraic_connectivity),
            "min_algebraic_connectivity": float(self.min_algebraic_connectivity),
            "final_algebraic_connectivity": float(self.final_algebraic_connectivity),
            "leader_control_energy_proxy": float(self.leader_control_energy_proxy),
            "follower_control_energy_proxy": float(self.follower_control_energy_proxy),
            "fault_event_count": int(self.fault_event_count),
            "reconfiguration_event_count": int(self.reconfiguration_event_count),
        }


class TopologyMetricAccumulator:
    """Accumulates lightweight topology metrics during a simulation run."""

    def __init__(self, dt: float):
        self.dt = max(float(dt), 0.0)
        self._lambda2_values: list[float] = []
        self._leader_energy = 0.0
        self._follower_energy = 0.0

    def add_sample(
        self,
        offsets: Iterable[Any],
        *,
        leader_control: Any | None = None,
        follower_controls: Iterable[Any] | None = None,
    ) -> None:
        try:
            offset_list = [np.asarray(offset, dtype=float).flatten()[:3] for offset in offsets]
            lambda2 = float(TopologyGraph(offset_list).algebraic_connectivity)
        except Exception:
            lambda2 = float("nan")

        if np.isfinite(lambda2):
            self._lambda2_values.append(lambda2)

        self._leader_energy += self._control_energy(leader_control)
        if follower_controls is not None:
            for control in follower_controls:
                self._follower_energy += self._control_energy(control)

    def to_dict(self, fault_log: Iterable[Any] | None = None) -> dict[str, bool | int | float]:
        metrics = self._finalize(fault_log=fault_log)
        return metrics.to_dict()

    def _finalize(self, fault_log: Iterable[Any] | None = None) -> TopologyRuntimeMetrics:
        values = self._lambda2_values
        fault_count, reconfig_count = self._count_fault_events(fault_log)
        if values:
            return TopologyRuntimeMetrics(
                available=True,
                sample_count=len(values),
                mean_algebraic_connectivity=float(np.mean(values)),
                min_algebraic_connectivity=float(np.min(values)),
                final_algebraic_connectivity=float(values[-1]),
                leader_control_energy_proxy=float(self._leader_energy),
                follower_control_energy_proxy=float(self._follower_energy),
                fault_event_count=fault_count,
                reconfiguration_event_count=reconfig_count,
            )
        return TopologyRuntimeMetrics(
            leader_control_energy_proxy=float(self._leader_energy),
            follower_control_energy_proxy=float(self._follower_energy),
            fault_event_count=fault_count,
            reconfiguration_event_count=reconfig_count,
        )

    def _control_energy(self, control: Any | None) -> float:
        if control is None:
            return 0.0
        arr = np.asarray(control, dtype=float).ravel()
        finite = arr[np.isfinite(arr)]
        if finite.size == 0:
            return 0.0
        return float(np.sum(finite * finite) * self.dt)

    @staticmethod
    def _count_fault_events(fault_log: Iterable[Any] | None) -> tuple[int, int]:
        fault_count = 0
        reconfig_count = 0
        for event in fault_log or []:
            event_type = ""
            if isinstance(event, dict):
                event_type = str(event.get("type", ""))
            else:
                event_type = str(event).split(":", 1)[0]

            if event_type in {"inject", "detect"}:
                fault_count += 1
            elif event_type == "reconfigure":
                reconfig_count += 1
        return fault_count, reconfig_count
