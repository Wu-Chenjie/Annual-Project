from __future__ import annotations

from pathlib import Path

import pytest

from config import get_config
from core.result_schema import json_safe
from experiments.ablation import apply_variant, list_variants
from experiments.metrics_extractor import extract_metrics
from experiments.report_writer import write_markdown_report, write_summary_csv
from experiments.scenario_registry import get_scenario_config, list_scenarios
from simulations.obstacle_scenario import ObstacleScenarioSimulation


def test_scenario_registry_returns_quick_config():
    scenarios = list_scenarios("online")
    assert any(spec.name == "warehouse_online" for spec in scenarios)
    assert any(spec.name == "warehouse" for spec in list_scenarios("offline"))

    cfg = get_scenario_config("warehouse_online", quick=True)

    assert cfg.max_sim_time <= 5.0
    assert cfg.planner_horizon <= 3.0


def test_apply_ablation_variant_changes_config():
    cfg = get_scenario_config("warehouse_danger", quick=True)

    apply_variant(cfg, "no_gnn")

    assert cfg.danger_mode_enabled is False
    assert any(variant.name == "with_trajectory_optimizer" for variant in list_variants())

    apply_variant(cfg, "with_trajectory_optimizer")

    assert cfg.trajectory_optimizer_enabled is True
    assert cfg.trajectory_optimizer_method == "min_snap_proxy"


def test_formation_clearance_ablation_variants_are_explicit():
    leader_only = get_scenario_config("warehouse", quick=True)
    apply_variant(leader_only, "leader_only_planner")

    assert leader_only.planner_use_formation_envelope is False
    assert leader_only.formation_safety_enabled is False
    assert leader_only.plan_clearance_extra == 0.0

    formation_aware = get_scenario_config("warehouse", quick=True)
    apply_variant(formation_aware, "formation_aware_clearance")

    assert formation_aware.planner_use_formation_envelope is True
    assert formation_aware.formation_safety_enabled is True
    assert formation_aware.plan_clearance_extra > leader_only.plan_clearance_extra


def test_formation_adaptation_ablation_variants_are_explicit():
    static = get_scenario_config("meeting_room", quick=True)
    apply_variant(static, "formation_aware_static")

    assert static.planner_use_formation_envelope is True
    assert static.formation_safety_enabled is True
    assert static.formation_adaptation_enabled is False

    adaptive = get_scenario_config("meeting_room", quick=True)
    apply_variant(adaptive, "formation_aware_adaptive")

    assert adaptive.planner_use_formation_envelope is True
    assert adaptive.formation_safety_enabled is True
    assert adaptive.formation_adaptation_enabled is True
    assert adaptive.plan_clearance_extra == static.plan_clearance_extra


def test_lookahead_adaptation_ablation_variant_is_explicit():
    cfg = get_scenario_config("meeting_room", quick=True)

    apply_variant(cfg, "formation_aware_lookahead_adaptive")

    assert cfg.planner_use_formation_envelope is True
    assert cfg.formation_safety_enabled is True
    assert cfg.formation_adaptation_enabled is True
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True


def test_non_lookahead_ablation_variants_disable_rrt_lookahead():
    leader_only = get_scenario_config("rrt_dual_channel_online", quick=True)
    apply_variant(leader_only, "leader_only_planner")

    assert leader_only.formation_lookahead_enabled is False
    assert leader_only.formation_lookahead_rrt_enabled is False

    adaptive = get_scenario_config("rrt_dual_channel_online", quick=True)
    apply_variant(adaptive, "formation_aware_adaptive")

    assert adaptive.formation_adaptation_enabled is True
    assert adaptive.formation_lookahead_enabled is False
    assert adaptive.formation_lookahead_rrt_enabled is False


def test_rrt_dual_channel_online_scenario_is_registered():
    scenarios = list_scenarios("online")
    assert any(spec.name == "rrt_dual_channel_online" for spec in scenarios)

    cfg = get_scenario_config("rrt_dual_channel_online", quick=True)

    assert cfg.enable_obstacles is True
    assert cfg.planner_mode == "online"
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True
    assert cfg.map_file is not None
    assert cfg.map_file.endswith("rrt_dual_channel_escape.json")
    assert cfg.max_sim_time <= 5.0
    assert cfg.planner_horizon <= 3.0


def test_rrt_dual_channel_quick_run_records_rrt_escape():
    cfg = get_scenario_config("rrt_dual_channel_online", quick=True)
    apply_variant(cfg, "formation_aware_lookahead_adaptive")
    cfg.max_sim_time = min(cfg.max_sim_time, 5.0)

    result = ObstacleScenarioSimulation(cfg).run()
    metrics = extract_metrics(json_safe(result))

    assert metrics["lookahead_reference_blocked_count"] > 0
    assert metrics["rrt_escape_attempt_count"] > 0
    assert metrics["rrt_escape_accepted_count"] > 0
    assert metrics["collision_count"] == 0


def test_formation_maze_stress_online_scenario_is_registered():
    scenarios = list_scenarios("online")
    assert any(spec.name == "formation_maze_stress_online" for spec in scenarios)

    cfg = get_scenario_config("formation_maze_stress_online", quick=True)

    assert cfg.enable_obstacles is True
    assert cfg.planner_mode == "online"
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True
    assert cfg.map_file is not None
    assert cfg.map_file.endswith("formation_maze_stress.json")


def test_unknown_map_online_scenario_starts_with_empty_planner_map():
    scenarios = list_scenarios("online")
    assert any(spec.name == "unknown_map_online" for spec in scenarios)

    cfg = get_scenario_config("unknown_map_online", quick=True)

    assert cfg.enable_obstacles is True
    assert cfg.planner_mode == "online"
    assert cfg.sensor_enabled is True
    assert cfg.planner_initial_map_unknown is True
    assert cfg.map_file is not None
    assert cfg.map_file.endswith("unknown_map_arena.json")

    sim = ObstacleScenarioSimulation(cfg)

    assert len(list(sim.obstacles)) > 0
    assert int(sim.replanner._static_occupied.sum()) == 0
    assert min(float(sim.obstacles.signed_distance(wp)) for wp in cfg.waypoints) > sim._collision_margin


def test_report_writer_outputs_csv_and_markdown(tmp_path: Path):
    rows = [
        {
            "scenario": "basic",
            "variant": "baseline",
            "collision_count": 0,
            "trajectory_max_jerk": "0.250000000000",
            "trajectory_snap_squared_integral": 0.5,
        }
    ]

    csv_path = write_summary_csv(rows, tmp_path / "summary.csv")
    md_path = write_markdown_report(rows, tmp_path / "report.md")

    assert "trajectory_max_jerk" in csv_path.read_text(encoding="utf-8")
    markdown = md_path.read_text(encoding="utf-8")
    assert "# 消融对比报告" in markdown
    assert "## 消融设置" in markdown
    assert "## 快速消融结果" in markdown
    assert "## 结论" in markdown
    assert "| 场景 | 变体 | 碰撞次数 | 最大 jerk | snap 平方积分 |" in markdown
    assert "| 基础编队 | 基线 | 0 | 0.25 | 0.5 |" in markdown


def test_config_error_lists_new_rrt_test_presets():
    with pytest.raises(ValueError) as exc_info:
        get_config("__missing_preset__")

    message = str(exc_info.value)
    assert "rrt_dual_channel_online" in message
    assert "formation_maze_stress_online" in message
    assert "unknown_map_online" in message
