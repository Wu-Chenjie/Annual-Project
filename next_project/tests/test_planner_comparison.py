from __future__ import annotations

from pathlib import Path

from experiments.planner_comparison import choose_best_planner, planner_score_reason, write_planner_comparison_report


def test_choose_best_planner_prioritizes_safety_and_completion():
    rows = [
        {
            "planner": "rrt_star",
            "completed_waypoint_count": 1,
            "collision_count": 0,
            "max_error_overall": 0.2,
            "executed_path_length": 8.0,
            "planning_wall_time_s": 0.1,
            "trajectory_snap_squared_integral": 2.0,
        },
        {
            "planner": "hybrid_astar",
            "completed_waypoint_count": 2,
            "collision_count": 1,
            "max_error_overall": 0.1,
            "executed_path_length": 4.0,
            "planning_wall_time_s": 0.1,
            "trajectory_snap_squared_integral": 1.0,
        },
        {
            "planner": "astar",
            "completed_waypoint_count": 2,
            "collision_count": 0,
            "max_error_overall": 0.15,
            "executed_path_length": 5.0,
            "planning_wall_time_s": 0.2,
            "trajectory_snap_squared_integral": 1.5,
        },
    ]

    best = choose_best_planner(rows)

    assert best["planner"] == "astar"
    reason = planner_score_reason(best, rows)
    assert "综合最优" in reason
    assert "碰撞数为 0" in reason
    assert "完成航点数最高" in reason


def test_write_planner_comparison_report_keeps_planner_names(tmp_path: Path):
    rows = [
        {"planner": "astar", "completed_waypoint_count": 2, "collision_count": 0, "max_error_overall": 0.1},
        {"planner": "dijkstra", "completed_waypoint_count": 2, "collision_count": 0, "max_error_overall": 0.2},
    ]

    report_path = write_planner_comparison_report(rows, tmp_path / "report.md", tmp_path / "规划模式对比.png")

    text = report_path.read_text(encoding="utf-8")
    assert "| astar |" in text
    assert "| dijkstra |" in text
    assert "。。" not in text
