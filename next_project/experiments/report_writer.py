from __future__ import annotations

import csv
from pathlib import Path
from typing import Mapping, Sequence


MetricRow = Mapping[str, object]


def write_summary_csv(rows: Sequence[MetricRow], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows)
    with target.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})
    return target


def write_markdown_report(rows: Sequence[MetricRow], path: str | Path, *, title: str = "Ablation Report") -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows)
    lines = [f"# {title}", ""]
    if not rows:
        lines.append("No rows.")
    else:
        lines.append("| " + " | ".join(fieldnames) + " |")
        lines.append("| " + " | ".join("---" for _ in fieldnames) + " |")
        for row in rows:
            lines.append("| " + " | ".join(_format_cell(row.get(key, "")) for key in fieldnames) + " |")
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target


def _fieldnames(rows: Sequence[MetricRow]) -> list[str]:
    preferred = [
        "scenario",
        "variant",
        "preset",
        "runtime_engine",
        "runtime_s",
        "completed_waypoint_count",
        "collision_count",
        "replan_count",
        "fault_count",
        "planned_path_length",
        "executed_path_length",
        "trajectory_path_length",
        "trajectory_max_speed",
        "trajectory_max_acceleration",
        "trajectory_mean_jerk",
        "trajectory_max_jerk",
        "trajectory_jerk_squared_integral",
        "trajectory_snap_squared_integral",
        "mean_error_overall",
        "max_error_overall",
        "final_error_overall",
        "min_inter_drone_distance",
        "downwash_hits",
        "formation_clearance_mode",
        "formation_clearance_required",
        "formation_clearance_min_margin",
        "formation_clearance_min_leader_margin",
        "formation_clearance_min_follower_margin",
        "formation_clearance_violation_count",
        "formation_clearance_follower_violation_count",
        "formation_clearance_sample_count",
        "formation_clearance_posthoc_required",
        "formation_clearance_posthoc_min_margin",
        "formation_clearance_posthoc_violation_count",
        "formation_clearance_posthoc_follower_violation_count",
        "formation_adaptation_count",
        "formation_adaptation_last",
        "formation_adaptation_last_reason",
    ]
    keys = set()
    for row in rows:
        keys.update(str(key) for key in row.keys())
    return [key for key in preferred if key in keys] + sorted(keys.difference(preferred))


def _format_cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value).replace("|", "\\|")


__all__ = ["write_markdown_report", "write_summary_csv"]
