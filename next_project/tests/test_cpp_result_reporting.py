from __future__ import annotations

import json
import pytest
from pathlib import Path

from experiments.report_cpp_results import generate_cpp_result_report
from experiments.run_cpp_report import find_sim_result_path, run_cpp_and_report


def _cpp_payload() -> dict:
    return {
        "schema_version": "1.0.0",
        "preset": "warehouse",
        "runtime_engine": "cpp",
        "runtime_s": 0.12,
        "metrics": {"mean": [0.1], "max": [0.2], "final": [0.05]},
        "summary": {
            "mean_error_overall": 0.1,
            "max_error_overall": 0.2,
            "final_error_overall": 0.05,
            "collision_count": 0,
            "replan_count": 0,
            "fault_count": 0,
        },
        "completed_waypoint_count": 2,
        "task_waypoints": [[0, 0, 1], [2, 0, 1]],
        "replanned_waypoints": [[0, 0, 1], [1, 0, 1], [2, 0, 1]],
        "executed_path": [[0, 0, 1], [1, 0.1, 1], [2, 0, 1]],
        "timing": {"planning_s": 0.03, "simulation_s": 0.09, "total_s": 0.12},
        "obstacle_model": {
            "bounds": [[-1, -1, 0], [3, 3, 2]],
            "primitives": [
                {"id": "sphere_0", "type": "sphere", "center": [1, 0, 1], "radius": 0.4},
                {"id": "box_0", "type": "aabb", "min": [1.5, -0.3, 0], "max": [1.9, 0.3, 1.2]},
                {"id": "cyl_0", "type": "cylinder", "center": [0.5, 1.0, 0], "radius": 0.2, "z_min": 0, "z_max": 1.5},
            ],
        },
        "planning_events": [
            {"t": 0.0, "phase": "offline_segment", "planner": "cpp", "wall_time_s": 0.03, "point_count": 3, "accepted": True}
        ],
        "waypoint_events": [
            {"t": 0.2, "type": "waypoint_reached", "index": 0, "distance": 0.1},
        ],
        "collision_log": [],
        "safety_metrics": {"min_inter_drone_distance": 0.4, "downwash_hits": 0},
    }


def test_generate_cpp_result_report_uses_cpp_output_names(tmp_path: Path):
    payload_path = tmp_path / "sim_result.json"
    payload_path.write_text(json.dumps(_cpp_payload(), ensure_ascii=False), encoding="utf-8")

    report_path = generate_cpp_result_report(payload_path)

    assert report_path == tmp_path / "cpp_report.md"
    assert (tmp_path / "cpp_metrics.json").is_file()
    assert (tmp_path / "cpp_report_figures" / "场景与路线.png").is_file()
    text = report_path.read_text(encoding="utf-8")
    assert "# C++ 仿真结果报告" in text
    assert "| 运行引擎 | cpp |" in text
    assert "| 规划器 | cpp |" in text
    assert "offline_segment" in text
    assert "## 障碍物建模" in text
    assert "| sphere | 1 |" in text
    metrics = json.loads((tmp_path / "cpp_metrics.json").read_text(encoding="utf-8"))
    assert metrics["planned_path_length"] == 2.0


def test_find_sim_result_path_from_cpp_stdout(tmp_path: Path):
    result_path = tmp_path / "outputs" / "warehouse" / "20260512-130000" / "sim_result.json"
    result_path.parent.mkdir(parents=True)
    result_path.write_text("{}", encoding="utf-8")
    stdout = f"结果文件: {result_path}\n"

    found = find_sim_result_path(stdout, tmp_path)

    assert found == result_path


def test_cpp_warehouse_output_contains_observer_and_obstacle_model(tmp_path: Path):
    exe = Path("cpp/build/sim_warehouse.exe")
    if not exe.is_file():
        pytest.skip("C++ warehouse executable is not built")

    sim_result, report_path = run_cpp_and_report(
        exe,
        cwd=Path.cwd(),
        output_dir=tmp_path / "cpp_report",
        title="C++仓库观察器报告",
    )
    payload = json.loads(sim_result.read_text(encoding="utf-8"))
    metrics = json.loads((tmp_path / "cpp_report" / "cpp_metrics.json").read_text(encoding="utf-8"))
    summary = payload["summary"]

    assert payload["runtime_engine"] == "cpp"
    assert payload["time"]
    assert payload["planned_path"]
    assert payload["planning_events"]
    assert payload["waypoint_events"]
    assert "collision_log" in payload
    assert payload["config_snapshot"]["planner_kind"]
    assert payload["obstacle_model"]["primitives"]
    assert summary["collision_step_count"] >= summary["collision_count"]
    assert summary["hard_collision_count"] == 0
    assert summary["collision_count"] == summary["hard_collision_count"]
    assert summary["clearance_warning_count"] >= summary["collision_count"]
    assert summary["min_obstacle_signed_distance"] >= 0.0
    assert metrics["collision_step_count"] >= metrics["collision_count"]
    assert metrics["hard_collision_count"] == 0
    assert metrics["clearance_warning_count"] >= metrics["collision_count"]
    assert metrics["trajectory_path_length"] is not None
    assert metrics["trajectory_max_speed"] is not None
    assert metrics["trajectory_max_acceleration"] is not None
    assert metrics["trajectory_jerk_squared_integral"] is not None
    assert metrics["trajectory_snap_squared_integral"] is not None
    assert report_path.is_file()
    assert (tmp_path / "cpp_report" / "cpp_report_figures" / "场景与路线.png").is_file()
