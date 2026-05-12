from __future__ import annotations

from pathlib import Path

from experiments.ablation import apply_variant, list_variants
from experiments.report_writer import write_markdown_report, write_summary_csv
from experiments.scenario_registry import get_scenario_config, list_scenarios


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


def test_report_writer_outputs_csv_and_markdown(tmp_path: Path):
    rows = [
        {
            "scenario": "basic",
            "variant": "baseline",
            "collision_count": 0,
            "trajectory_max_jerk": 0.25,
            "trajectory_snap_squared_integral": 0.5,
        }
    ]

    csv_path = write_summary_csv(rows, tmp_path / "summary.csv")
    md_path = write_markdown_report(rows, tmp_path / "report.md")

    assert "trajectory_max_jerk" in csv_path.read_text(encoding="utf-8")
    assert "| basic | baseline |" in md_path.read_text(encoding="utf-8")
