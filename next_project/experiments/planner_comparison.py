from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from config import get_config
from core.result_schema import build_sim_result_payload, json_safe
from experiments.metrics_extractor import extract_metrics
from experiments.report_writer import write_summary_csv
from simulations.formation_simulation import FormationSimulation, SimulationConfig
from simulations.obstacle_scenario import ObstacleScenarioSimulation

plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "Arial Unicode MS",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


def choose_best_planner(rows: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    if not rows:
        raise ValueError("planner rows must not be empty")
    max_completed = max(_num(row.get("completed_waypoint_count")) for row in rows)
    return min(
        rows,
        key=lambda row: (
            _num(row.get("collision_count")),
            -_num(row.get("completed_waypoint_count")),
            0 if _num(row.get("completed_waypoint_count")) >= max_completed else 1,
            _num(row.get("max_error_overall")),
            _num(row.get("executed_path_length")),
            _num(row.get("planning_wall_time_s")),
            _num(row.get("replan_count")),
            _num(row.get("trajectory_snap_squared_integral")),
        ),
    )


def planner_score_reason(best: Mapping[str, Any], rows: Sequence[Mapping[str, Any]]) -> str:
    max_completed = max(_num(row.get("completed_waypoint_count")) for row in rows) if rows else 0.0
    parts = [
        f"`{best.get('planner', '')}` 被判定为本轮综合最优",
    ]
    if _num(best.get("collision_count")) == 0:
        parts.append("碰撞数为 0")
    else:
        parts.append(f"碰撞数为 {_fmt(best.get('collision_count'))}")
    if _num(best.get("completed_waypoint_count")) >= max_completed:
        parts.append("完成航点数最高")
    parts.append(f"最大误差为 {_fmt(best.get('max_error_overall'))} m")
    parts.append(f"执行路径长度为 {_fmt(best.get('executed_path_length'))} m")
    parts.append(f"规划耗时为 {_fmt(best.get('planning_wall_time_s'))} s")
    parts.append("该结论是当前预设、地图和参数下的实验相对最优，不代表理论全局最优")
    return "，".join(parts) + "。"


def run_planner_comparison(
    *,
    preset: str,
    planners: Sequence[str],
    output_dir: str | Path,
    quick: bool = False,
) -> tuple[Path, Path]:
    out_dir = Path(output_dir)
    rows: list[dict[str, Any]] = []
    raw_dir = out_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    for planner in planners:
        cfg = get_config(preset)
        cfg.planner_kind = planner
        if quick:
            cfg.max_sim_time = min(float(cfg.max_sim_time), 5.0)
            cfg.planner_horizon = min(float(getattr(cfg, "planner_horizon", 3.0)), 3.0)
        sim = _make_simulation(cfg)
        result = sim.run()
        payload = build_sim_result_payload(
            preset=f"{preset}_{planner}",
            sim_result=result,
            runtime_s=None,
            runtime_engine="python",
            config_snapshot=json_safe(asdict(cfg)),
            include_trajectories=False,
        )
        payload_path = raw_dir / f"{planner}.json"
        payload_path.write_text(json.dumps(json_safe(payload), ensure_ascii=False, indent=2), encoding="utf-8")
        row = dict(extract_metrics(payload))
        row["planner"] = planner
        row["planning_wall_time_s"] = _planning_wall_time(payload.get("planning_events") or [])
        rows.append(row)

    summary_path = write_summary_csv(rows, out_dir / "summary.csv")
    figure_path = _write_comparison_figure(rows, out_dir / "规划模式对比.png")
    report_path = write_planner_comparison_report(rows, out_dir / "report.md", figure_path)
    return summary_path, report_path


def _make_simulation(cfg: SimulationConfig):
    return ObstacleScenarioSimulation(cfg) if cfg.enable_obstacles else FormationSimulation(cfg)


def write_planner_comparison_report(rows: Sequence[Mapping[str, Any]], path: Path, figure_path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    best = choose_best_planner(rows)
    fieldnames = [
        "planner",
        "completed_waypoint_count",
        "collision_count",
        "replan_count",
        "max_error_overall",
        "executed_path_length",
        "planning_wall_time_s",
        "trajectory_snap_squared_integral",
    ]
    lines = [
        "# 规划模式对比报告",
        "",
        "## 为什么最优",
        "",
        planner_score_reason(best, rows),
        "",
        "## 对比表",
        "",
        "| " + " | ".join(fieldnames) + " |",
        "| " + " | ".join("---" for _ in fieldnames) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(key)) for key in fieldnames) + " |")
    lines.extend([
        "",
        "## 对比图片",
        "",
        f"![规划模式对比]({figure_path.name})",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _write_comparison_figure(rows: Sequence[Mapping[str, Any]], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [str(row.get("planner", "")) for row in rows]
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.5))
    specs = [
        ("collision_count", "碰撞数"),
        ("max_error_overall", "最大误差"),
        ("planning_wall_time_s", "规划耗时"),
    ]
    for ax, (key, title) in zip(axes, specs):
        ax.bar(labels, [_num(row.get(key)) for row in rows])
        ax.set_title(title)
        ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _planning_wall_time(events: Sequence[Mapping[str, Any]]) -> float:
    return float(sum(_num(event.get("wall_time_s")) for event in events))


def _num(value: Any) -> float:
    if value is None:
        return float("inf")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("inf")


def _fmt(value: Any) -> str:
    if isinstance(value, str):
        return value
    number = _num(value)
    if number == float("inf"):
        return ""
    if number.is_integer():
        return str(int(number))
    return f"{number:.6g}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="运行多个规划器并生成中文对比报告。")
    parser.add_argument("--preset", default="warehouse_online", help="预设场景名称。")
    parser.add_argument("--planner", action="append", default=None, help="规划器名称，可重复。")
    parser.add_argument("--output-dir", default="outputs/planner_compare", help="输出目录。")
    parser.add_argument("--quick", action="store_true", help="缩短仿真时间用于烟测。")
    args = parser.parse_args(argv)

    planners = args.planner or ["astar", "hybrid_astar", "rrt_star"]
    summary_path, report_path = run_planner_comparison(
        preset=args.preset,
        planners=planners,
        output_dir=args.output_dir,
        quick=args.quick,
    )
    print(f"汇总已生成: {summary_path}")
    print(f"报告已生成: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "choose_best_planner",
    "planner_score_reason",
    "run_planner_comparison",
    "write_planner_comparison_report",
]
