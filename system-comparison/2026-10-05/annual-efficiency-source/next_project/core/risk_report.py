from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any

import numpy as np

from .safety_profiles import SafetyProfile, get_safety_profile


@dataclass(frozen=True)
class RiskReport:
    safety_profile: dict[str, Any]
    conops: dict[str, Any]
    observed_events: dict[str, Any]
    compliance_checks: dict[str, Any]
    residual_risk: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _as_path_array(value: Any) -> np.ndarray:
    if value is None:
        return np.zeros((0, 3), dtype=float)
    arr = np.asarray(value, dtype=float)
    if arr.size == 0:
        return np.zeros((0, 3), dtype=float)
    return arr.reshape((-1, 3))


def build_risk_report(sim_result: dict[str, Any], safety_profile: str | SafetyProfile = "indoor_demo") -> RiskReport:
    profile = safety_profile if isinstance(safety_profile, SafetyProfile) else get_safety_profile(safety_profile)

    executed_path_raw = sim_result.get("executed_path")
    task_waypoints_raw = sim_result.get("task_waypoints")
    replanned_waypoints_raw = sim_result.get("replanned_waypoints")
    executed_path = _as_path_array(executed_path_raw)
    task_waypoints = _as_path_array(task_waypoints_raw)
    replanned_waypoints = _as_path_array(replanned_waypoints_raw)
    collision_log = list(sim_result.get("collision_log") or [])
    replan_events = list(sim_result.get("replan_events") or [])
    fault_log = list(sim_result.get("fault_log") or [])
    sensor_logs = sim_result.get("sensor_logs")
    metrics = sim_result.get("metrics") or {}
    safety_metrics = sim_result.get("safety_metrics") or {}

    final_goal_distance = None
    if executed_path.size and task_waypoints.size:
        final_goal_distance = float(np.linalg.norm(executed_path[-1] - task_waypoints[-1]))

    min_sensor_clearance = None
    if sensor_logs is not None:
        sensor_arr = np.asarray(sensor_logs, dtype=float)
        if sensor_arr.size:
            min_sensor_clearance = float(np.min(sensor_arr))

    compliance_checks = {
        "collision_free": len(collision_log) == 0,
        "horizontal_separation_documented": profile.horizontal_separation > 0.0,
        "vertical_separation_documented": profile.vertical_separation > 0.0,
        "surveillance_period_documented": profile.surveillance_period > 0.0,
        "indoor_scale_match": profile.scenario_scale == "indoor",
        "final_goal_within_horizontal_separation": (
            final_goal_distance is None or final_goal_distance <= profile.horizontal_separation
        ),
    }

    remaining_risk_level = "low"
    if collision_log:
        remaining_risk_level = "high"
    elif fault_log or (min_sensor_clearance is not None and min_sensor_clearance < profile.horizontal_separation):
        remaining_risk_level = "medium"

    return RiskReport(
        safety_profile=profile.to_dict(),
        conops={
            "task_waypoint_count": int(len(task_waypoints)),
            "replanned_waypoint_count": int(len(replanned_waypoints)),
            "executed_path_samples": int(len(executed_path)),
            "profile_scale": profile.scenario_scale,
        },
        observed_events={
            "collisions": len(collision_log),
            "replans": len(replan_events),
            "faults": len(fault_log),
            "min_sensor_clearance": min_sensor_clearance,
            "final_goal_distance": final_goal_distance,
            "mean_tracking_error": metrics.get("mean"),
            "max_tracking_error": metrics.get("max"),
            "min_inter_drone_distance": safety_metrics.get("min_inter_drone_distance"),
            "downwash_hits": safety_metrics.get("downwash_hits"),
        },
        compliance_checks=compliance_checks,
        residual_risk={
            "level": remaining_risk_level,
            "summary": _summarize_residual_risk(
                remaining_risk_level,
                collision_log=collision_log,
                fault_log=fault_log,
                profile=profile,
            ),
        },
    )


def _summarize_residual_risk(level: str, *, collision_log: list[Any], fault_log: list[Any], profile: SafetyProfile) -> str:
    if level == "high":
        return "Collision events observed; scenario does not satisfy residual risk expectations."
    if level == "medium":
        return (
            "No collision observed, but faults or reduced clearance remain. "
            f"Review profile {profile.name} before external-facing claims."
        )
    return "No collision observed and residual risk is acceptable for the selected profile."
