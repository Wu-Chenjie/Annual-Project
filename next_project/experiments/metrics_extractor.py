from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np


def extract_metrics(sim_result: Mapping[str, Any]) -> dict[str, float | int | str | None]:
    """Extract scalar experiment metrics from a standardized sim_result payload."""
    summary = sim_result.get("summary") or {}
    raw_metrics = sim_result.get("metrics") or {}
    planned_path = _points(
        sim_result.get("planned_path")
        or sim_result.get("replanned_waypoints")
        or sim_result.get("task_waypoints")
        or sim_result.get("waypoints")
    )
    executed_path = _points(sim_result.get("executed_path") or sim_result.get("leader"))
    planned_trajectory = sim_result.get("planned_trajectory") or {}
    derived_trajectory = _derive_trajectory_metrics(sim_result, planned_path, executed_path)
    mean_errors = _float_list(raw_metrics.get("mean"))
    max_errors = _float_list(raw_metrics.get("max"))
    final_errors = _float_list(raw_metrics.get("final"))

    metrics: dict[str, float | int | str | None] = {
        "preset": str(sim_result.get("preset") or ""),
        "runtime_engine": str(sim_result.get("runtime_engine") or ""),
        "runtime_s": _float_or_none(sim_result.get("runtime_s")),
        "completed_waypoint_count": int(sim_result.get("completed_waypoint_count") or 0),
        "mean_error_overall": _summary_or_aggregate(summary, "mean_error_overall", mean_errors, "mean"),
        "max_error_overall": _summary_or_aggregate(summary, "max_error_overall", max_errors, "max"),
        "final_error_overall": _summary_or_aggregate(summary, "final_error_overall", final_errors, "mean"),
        "collision_count": int(summary.get("collision_count") or 0),
        "collision_step_count": int(summary.get("collision_step_count") or summary.get("collision_count") or 0),
        "hard_collision_count": int(summary.get("hard_collision_count") or 0),
        "hard_collision_step_count": int(summary.get("hard_collision_step_count") or 0),
        "clearance_warning_count": int(
            summary.get("clearance_warning_count")
            or summary.get("collision_count")
            or 0
        ),
        "clearance_warning_step_count": int(
            summary.get("clearance_warning_step_count")
            or summary.get("collision_step_count")
            or summary.get("collision_count")
            or 0
        ),
        "min_obstacle_signed_distance": _float_or_none(summary.get("min_obstacle_signed_distance")),
        "replan_count": int(summary.get("replan_count") or 0),
        "fault_count": int(summary.get("fault_count") or 0),
        "planned_path_length": _path_length(planned_path),
        "executed_path_length": _path_length(executed_path),
        "trajectory_path_length": _first_float(
            planned_trajectory.get("path_length"), derived_trajectory.get("path_length")
        ),
        "trajectory_max_speed": _first_float(
            planned_trajectory.get("max_speed"), derived_trajectory.get("max_speed")
        ),
        "trajectory_max_acceleration": _first_float(
            planned_trajectory.get("max_acceleration"), derived_trajectory.get("max_acceleration")
        ),
        "trajectory_mean_jerk": _first_float(
            planned_trajectory.get("mean_jerk"), derived_trajectory.get("mean_jerk")
        ),
        "trajectory_max_jerk": _first_float(
            planned_trajectory.get("max_jerk"), derived_trajectory.get("max_jerk")
        ),
        "trajectory_jerk_squared_integral": _first_float(
            planned_trajectory.get("jerk_squared_integral"),
            derived_trajectory.get("jerk_squared_integral"),
        ),
        "trajectory_snap_squared_integral": _first_float(
            planned_trajectory.get("snap_squared_integral"),
            derived_trajectory.get("snap_squared_integral"),
        ),
    }

    safety = sim_result.get("safety_metrics") or {}
    metrics["min_inter_drone_distance"] = _float_or_none(safety.get("min_inter_drone_distance"))
    metrics["downwash_hits"] = int(safety.get("downwash_hits") or 0)
    clearance = safety.get("formation_clearance") or {}
    metrics["formation_clearance_required"] = _float_or_none(clearance.get("required_clearance"))
    metrics["formation_clearance_min_margin"] = _float_or_none(clearance.get("min_formation_margin"))
    metrics["formation_clearance_min_leader_margin"] = _float_or_none(clearance.get("min_leader_margin"))
    metrics["formation_clearance_min_follower_margin"] = _float_or_none(clearance.get("min_follower_margin"))
    metrics["formation_clearance_violation_count"] = int(clearance.get("violation_count") or 0)
    metrics["formation_clearance_follower_violation_count"] = int(clearance.get("follower_violation_count") or 0)
    metrics["formation_clearance_sample_count"] = int(clearance.get("sample_count") or 0)
    metrics["formation_clearance_mode"] = str(clearance.get("mode") or "")
    posthoc = safety.get("formation_clearance_posthoc") or {}
    metrics["formation_clearance_posthoc_required"] = _float_or_none(posthoc.get("required_clearance"))
    metrics["formation_clearance_posthoc_min_margin"] = _float_or_none(posthoc.get("min_formation_margin"))
    metrics["formation_clearance_posthoc_violation_count"] = int(posthoc.get("violation_count") or 0)
    metrics["formation_clearance_posthoc_follower_violation_count"] = int(
        posthoc.get("follower_violation_count") or 0
    )
    adaptation_events = sim_result.get("formation_adaptation_events") or []
    metrics["formation_adaptation_count"] = len(adaptation_events)
    last_event = adaptation_events[-1] if adaptation_events else {}
    metrics["formation_adaptation_last"] = str(last_event.get("to") or "")
    metrics["formation_adaptation_last_reason"] = str(last_event.get("reason") or "")
    return metrics


def extract_metrics_file(path: str | Path) -> dict[str, float | int | str | None]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return extract_metrics(payload)


def _points(value: Any) -> np.ndarray:
    if value is None:
        return np.zeros((0, 3), dtype=float)
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        return np.zeros((0, 3), dtype=float)
    return arr


def _path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _derive_trajectory_metrics(
    sim_result: Mapping[str, Any],
    planned_path: np.ndarray,
    executed_path: np.ndarray,
) -> dict[str, float]:
    path_source = planned_path if len(planned_path) >= 2 else executed_path
    motion_source = executed_path if len(executed_path) >= 2 else path_source
    path_length = _path_length(path_source)
    if len(motion_source) < 2:
        return {
            "path_length": path_length,
            "max_speed": 0.0,
            "max_acceleration": 0.0,
            "mean_jerk": 0.0,
            "max_jerk": 0.0,
            "jerk_squared_integral": 0.0,
            "snap_squared_integral": 0.0,
        }

    timestamps = _time_array(sim_result.get("time"), len(motion_source))
    if timestamps is None:
        timestamps = _synthetic_timestamps(sim_result, motion_source)

    velocities = _differentiate(motion_source, timestamps)
    accelerations = _differentiate(velocities, timestamps)
    jerks = _differentiate(accelerations, timestamps)
    snaps = _differentiate(jerks, timestamps)
    jerk_norms = np.linalg.norm(jerks, axis=1) if len(jerks) else np.zeros(0, dtype=float)

    return {
        "path_length": path_length,
        "max_speed": _max_norm(velocities),
        "max_acceleration": _max_norm(accelerations),
        "mean_jerk": float(np.mean(jerk_norms)) if len(jerk_norms) else 0.0,
        "max_jerk": _max_norm(jerks),
        "jerk_squared_integral": _squared_integral(jerks, timestamps),
        "snap_squared_integral": _squared_integral(snaps, timestamps),
    }


def _time_array(value: Any, length: int) -> np.ndarray | None:
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if len(arr) < length:
        return None
    arr = arr[:length]
    if not np.all(np.isfinite(arr)):
        return None
    if np.any(np.diff(arr) <= 0.0):
        return None
    return arr


def _synthetic_timestamps(sim_result: Mapping[str, Any], points: np.ndarray) -> np.ndarray:
    runtime_s = _float_or_none(sim_result.get("runtime_s"))
    if runtime_s is not None and runtime_s > 0.0:
        return np.linspace(0.0, runtime_s, len(points), dtype=float)

    cfg = sim_result.get("config_snapshot") or {}
    nominal_speed = _first_float(cfg.get("nominal_speed"), cfg.get("leader_max_vel"), 1.0) or 1.0
    nominal_speed = max(nominal_speed, 1e-6)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    increments = np.maximum(distances / nominal_speed, 1e-6)
    return np.concatenate([[0.0], np.cumsum(increments)])


def _differentiate(values: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    out = np.zeros_like(values, dtype=float)
    if len(values) < 2:
        return out
    for idx in range(1, len(values)):
        dt = max(float(timestamps[idx] - timestamps[idx - 1]), 1e-6)
        out[idx] = (values[idx] - values[idx - 1]) / dt
    out[0] = out[1]
    return out


def _max_norm(values: np.ndarray) -> float:
    if len(values) == 0:
        return 0.0
    return float(np.max(np.linalg.norm(values, axis=1)))


def _squared_integral(values: np.ndarray, timestamps: np.ndarray) -> float:
    if len(values) == 0:
        return 0.0
    integral = 0.0
    for idx in range(1, len(values)):
        dt = max(float(timestamps[idx] - timestamps[idx - 1]), 1e-6)
        integral += float(np.dot(values[idx], values[idx])) * dt
    return float(integral)


def _first_float(*values: Any) -> float | None:
    for value in values:
        number = _float_or_none(value)
        if number is not None:
            return number
    return None


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _float_list(value: Any) -> list[float]:
    if value is None:
        return []
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return []


def _summary_or_aggregate(summary: Mapping[str, Any], key: str, values: list[float], mode: str) -> float:
    value = _float_or_none(summary.get(key))
    if value is not None:
        return value
    if not values:
        return 0.0
    if mode == "max":
        return float(np.max(values))
    return float(np.mean(values))


__all__ = ["extract_metrics", "extract_metrics_file"]
