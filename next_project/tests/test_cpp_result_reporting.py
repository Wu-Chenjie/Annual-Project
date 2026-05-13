from __future__ import annotations

import json
import subprocess
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
        "config_snapshot": {
            "planner_kind": "cpp",
            "planner_mode": "online",
            "formation_adaptation_enabled": True,
            "formation_lookahead_enabled": True,
            "formation_lookahead_rrt_enabled": True,
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
        "formation_adaptation_events": [
            {
                "t": 0.1,
                "kind": "lookahead_reference_blocked",
                "from": "diamond",
                "to": "diamond",
                "reason": "lookahead_sharp_turn",
                "clearance_margin": -0.05,
                "max_turn_angle_rad": 1.4,
            },
            {
                "t": 0.1,
                "from": "diamond",
                "to": "line",
                "reason": "lookahead_sharp_turn",
                "channel_width": [0.8, 4.0, 3.0],
                "clearance_margin": -0.05,
            },
            {
                "t": 0.1,
                "kind": "rrt_escape_attempt",
                "from": "line",
                "to": "line",
                "reason": "lookahead_sharp_turn",
                "planner": "rrt_star_escape",
                "goal_count": 2,
            },
            {
                "t": 0.1,
                "kind": "rrt_escape_accepted",
                "from": "line",
                "to": "line",
                "reason": "lookahead_sharp_turn",
                "planner": "direct_clear_escape",
                "point_count": 3,
            },
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
    assert "## 编队调控机制" in text
    assert "| 队形自适应 | 是 |" in text
    assert "| 前瞻 RRT escape | 是 |" in text
    assert "| 前瞻阻断次数 | 1 |" in text
    assert "| RRT escape 接受数 | 1 |" in text
    assert "lookahead_reference_blocked" in text
    assert "offline_segment" in text
    assert "## 障碍物建模" in text
    assert "| sphere | 1 |" in text
    metrics = json.loads((tmp_path / "cpp_metrics.json").read_text(encoding="utf-8"))
    assert metrics["planned_path_length"] == 2.0
    assert metrics["formation_adaptation_count"] == 4
    assert metrics["rrt_escape_accepted_count"] == 1


def test_cpp_sources_expose_formation_adaptation_presets_and_fields():
    config_h = Path("cpp/include/config.hpp").read_text(encoding="utf-8")
    dynamic_main = Path("cpp/src/dynamic_main.cpp").read_text(encoding="utf-8")
    warehouse_main = Path("cpp/src/warehouse_main.cpp").read_text(encoding="utf-8")

    assert "config_rrt_dual_channel_online" in config_h
    assert "config_formation_maze_stress_online" in config_h
    assert "config_unknown_map_online" in config_h
    assert 'preset == "rrt_dual_channel_online"' in config_h
    assert 'preset == "formation_maze_stress_online"' in config_h
    assert 'preset == "unknown_map_online"' in config_h
    assert "planner_initial_map_unknown" in config_h
    assert "formation_adaptation_enabled" in dynamic_main
    assert "formation_lookahead_enabled" in dynamic_main
    assert "formation_lookahead_rrt_enabled" in dynamic_main
    assert "formation_adaptation_events" in warehouse_main
    assert "map_knowledge" in warehouse_main


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


def test_cpp_dynamic_replay_outputs_formation_metadata_for_new_presets(tmp_path: Path):
    exe = Path("cpp/build/sim_dynamic_replay.exe")
    if not exe.is_file():
        pytest.skip("C++ dynamic replay executable is not built")

    input_path = tmp_path / "input.json"
    output_path = tmp_path / "replay_output.json"
    input_path.write_text(
        json.dumps(
            {
                "preset": "rrt_dual_channel_online",
                "base_config": {
                    "preset": "rrt_dual_channel_online",
                    "max_sim_time": 0.2,
                    "planner_horizon": 2.0,
                    "planner_replan_interval": 0.2,
                },
                "compare_planners": ["astar"],
                "repeat_count": 1,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [str(exe), str(input_path), "-o", str(output_path)],
        cwd=Path.cwd(),
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    replay = json.loads(output_path.read_text(encoding="utf-8"))[0]
    metadata = replay["metadata"]

    assert metadata["formation_adaptation_enabled"] is True
    assert metadata["formation_lookahead_enabled"] is True
    assert metadata["formation_lookahead_rrt_enabled"] is True
    assert metadata["formation_lookahead_distance"] > 0.0
    assert "formation_adaptation_events" in replay
    assert isinstance(replay["formation_adaptation_events"], list)


def test_cpp_standard_sim_accepts_new_formation_preset(tmp_path: Path):
    exe = Path("cpp/build/sim_warehouse.exe")
    if not exe.is_file():
        pytest.skip("C++ warehouse executable is not built")

    sim_result, report_path = run_cpp_and_report(
        exe,
        cwd=Path.cwd(),
        output_dir=tmp_path / "cpp_report",
        title="C++ 编队调控预设报告",
        extra_args=["--preset", "rrt_dual_channel_online", "--max-sim-time", "0.5"],
    )
    payload = json.loads(sim_result.read_text(encoding="utf-8"))
    snapshot = payload["config_snapshot"]

    assert payload["preset"] == "rrt_dual_channel_online"
    assert snapshot["formation_adaptation_enabled"] is True
    assert snapshot["formation_lookahead_enabled"] is True
    assert snapshot["formation_lookahead_rrt_enabled"] is True
    assert "formation_adaptation_events" in payload
    assert report_path.is_file()
    assert "## 编队调控机制" in report_path.read_text(encoding="utf-8")
    assert (tmp_path / "cpp_report" / "cpp_report_figures" / "场景与路线.png").is_file()
