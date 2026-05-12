from __future__ import annotations

import json
from pathlib import Path

from experiments.result_reporter import FIGURE_NAMES, generate_result_report


def _sample_payload() -> dict:
    return {
        "schema_version": "1.0.0",
        "preset": "warehouse_online",
        "runtime_engine": "python",
        "runtime_s": 3.2,
        "metrics": {
            "mean": [0.1, 0.2],
            "max": [0.3, 0.4],
            "final": [0.05, 0.07],
        },
        "summary": {
            "mean_error_overall": 0.15,
            "max_error_overall": 0.4,
            "final_error_overall": 0.06,
            "collision_count": 0,
            "replan_count": 1,
            "fault_count": 1,
        },
        "completed_waypoint_count": 2,
        "config_snapshot": {
            "planner_kind": "astar",
            "planner_mode": "online",
            "planner_resolution": 0.4,
            "planner_replan_interval": 0.4,
            "planner_horizon": 6.0,
            "num_followers": 2,
            "initial_formation": "diamond",
            "sensor_enabled": True,
            "danger_mode_enabled": False,
            "trajectory_optimizer_enabled": True,
            "trajectory_optimizer_method": "min_snap_proxy",
            "waypoints": [[0, 0, 1], [2, 0, 1]],
        },
        "planned_path": [[0, 0, 1], [1, 0, 1], [2, 0, 1]],
        "executed_path": [[0, 0, 1], [1, 0.1, 1], [2, 0, 1]],
        "planned_trajectory": {
            "path_length": 2.1,
            "max_speed": 1.2,
            "max_acceleration": 0.8,
            "mean_jerk": 0.3,
            "max_jerk": 0.6,
            "jerk_squared_integral": 1.4,
            "snap_squared_integral": 2.8,
        },
        "planning_events": [
            {
                "t": 0.0,
                "phase": "offline_segment",
                "planner": "astar",
                "wall_time_s": 0.012,
                "point_count": 3,
                "accepted": True,
            },
            {
                "t": 1.0,
                "phase": "online_replan",
                "planner": "astar",
                "wall_time_s": 0.02,
                "point_count": 2,
                "accepted": True,
            },
        ],
        "waypoint_events": [
            {"t": 0.5, "type": "waypoint_reached", "index": 0},
            {"t": 2.0, "type": "waypoint_reached", "index": 1},
        ],
        "replan_events": [{"t": 1.0, "phase": "local"}],
        "collision_log": [],
        "fault_log": [{"t": 1.5, "type": "inject"}],
        "safety_metrics": {"min_inter_drone_distance": 0.7, "downwash_hits": 0},
    }


def test_generate_result_report_writes_chinese_markdown_and_figures(tmp_path: Path):
    payload_path = tmp_path / "sim_result.json"
    payload_path.write_text(json.dumps(_sample_payload(), ensure_ascii=False), encoding="utf-8")

    report_path = generate_result_report(payload_path, title="仓库在线测试报告")

    assert report_path == tmp_path / "report.md"
    text = report_path.read_text(encoding="utf-8")
    assert "# 仓库在线测试报告" in text
    assert "## 预设场景" in text
    assert "## 路径规划路线参数" in text
    assert "## 规划器与规划耗时" in text
    assert "![场景与路线](report_figures/场景与路线.png)" in text
    assert "![飞行事件时间线](report_figures/飞行事件时间线.png)" in text
    assert (tmp_path / "report_figures" / "场景与路线.png").is_file()
    assert (tmp_path / "report_figures" / "飞行参数.png").is_file()
    assert (tmp_path / "report_figures" / "规划器耗时.png").is_file()
    assert (tmp_path / "metrics.json").is_file()


def test_generate_result_report_handles_missing_optional_sections(tmp_path: Path):
    payload = _sample_payload()
    for key in ("planned_path", "executed_path", "planned_trajectory", "planning_events"):
        payload.pop(key, None)
    payload_path = tmp_path / "sim_result.json"
    payload_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    report_path = generate_result_report(payload_path)

    text = report_path.read_text(encoding="utf-8")
    assert "未提供" in text
    assert (tmp_path / "report_figures" / "误差统计.png").is_file()


def test_generate_result_report_fills_trajectory_rows_without_planned_trajectory(tmp_path: Path):
    payload = _sample_payload()
    payload.pop("planned_trajectory", None)
    payload["planned_path"] = [[0, 0, 0], [0, 7.5, 0]]
    payload["executed_path"] = [[0, 0, 0], [2, 0, 0], [4, 3, 0]]
    payload["time"] = [0.0, 1.0, 2.0]
    payload_path = tmp_path / "sim_result.json"
    payload_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    report_path = generate_result_report(payload_path)

    text = report_path.read_text(encoding="utf-8")
    metrics = json.loads((tmp_path / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["trajectory_path_length"] == 7.5
    assert metrics["trajectory_max_speed"] == 3.605551275463989
    assert "路径/时间序列估算" in text
    assert "7.5" in text
    assert "3.60555" in text


def test_generate_result_report_compacts_dense_planning_events(tmp_path: Path):
    payload = _sample_payload()
    payload["planning_events"] = [
        {
            "t": idx * 0.3,
            "phase": "online_replan",
            "planner": "astar",
            "wall_time_s": 0.001 + idx * 0.00001,
            "point_count": 20 + idx,
            "accepted": idx % 3 == 0,
            "fallback_reason": "" if idx % 3 == 0 else "clearance_rejected",
        }
        for idx in range(80)
    ]
    payload_path = tmp_path / "sim_result.json"
    payload_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    report_path = generate_result_report(payload_path)

    text = report_path.read_text(encoding="utf-8")
    assert "规划事件汇总" in text
    assert "明细已压缩" in text
    assert text.count("online_replan") < 30
    assert (tmp_path / "report_figures" / FIGURE_NAMES["planning"]).is_file()


def test_run_with_config_generates_report_by_default(tmp_path: Path):
    from config import get_config
    from main import run_with_config

    config = get_config("basic")
    config.max_sim_time = 0.2
    out_root = tmp_path / "outputs"

    run_with_config(
        config,
        output_dir=out_root,
        plot=False,
        preset="basic",
        run_name="report-smoke",
        validate_schema=True,
    )

    run_dir = out_root / "basic" / "report-smoke"
    assert (run_dir / "report.md").is_file()
    assert (run_dir / "report_figures" / "飞行参数.png").is_file()
    assert "## 预设场景" in (run_dir / "report.md").read_text(encoding="utf-8")


def test_obstacle_simulation_emits_planning_and_waypoint_events():
    from config import get_config
    from simulations.obstacle_scenario import ObstacleScenarioSimulation

    config = get_config("obstacle")
    config.max_sim_time = 0.5

    result = ObstacleScenarioSimulation(config).run()

    assert result["planning_events"]
    assert result["planning_events"][0]["planner"] == config.planner_kind
    assert "wall_time_s" in result["planning_events"][0]
    assert "waypoint_events" in result
