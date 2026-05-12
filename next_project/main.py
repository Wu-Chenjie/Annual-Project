"""Command-line entry point for the UAV formation simulation project."""

from __future__ import annotations

import argparse
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from config import get_config
from core.result_schema import (
    build_sim_result_payload,
    json_safe,
    validate,
    write_json,
)
from experiments.result_reporter import generate_result_report
from simulations.formation_simulation import FormationSimulation, SimulationConfig
from simulations.obstacle_scenario import ObstacleScenarioSimulation
from simulations.visualization import SimulationVisualizer


DEFAULT_PRESET = "warehouse_danger"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run indoor multi-UAV formation simulation presets.",
    )
    parser.add_argument(
        "--preset",
        default=DEFAULT_PRESET,
        help=f"Configuration preset from config.py. Default: {DEFAULT_PRESET}",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs",
        help=(
            "结果根目录。最终输出会落到 <output-dir>/<preset>/<timestamp>/ ；"
            "可用 --run-name 覆盖时间戳子目录。默认: outputs"
        ),
    )
    parser.add_argument(
        "--run-name",
        default=None,
        help="覆盖 timestamp 子目录名（用于 CI/复现脚本生成稳定路径）。",
    )
    parser.add_argument(
        "--max-sim-time",
        type=float,
        default=None,
        help="Override the preset simulation duration in seconds.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Run the simulation without writing visualization files.",
    )
    parser.add_argument(
        "--no-validate",
        action="store_true",
        help="跳过 sim_result.json 的 JSON Schema 校验（默认开启）。",
    )
    parser.add_argument(
        "--include-trajectories",
        action="store_true",
        help="把 time/leader/followers 轨迹时序写入 sim_result.json（文件会变大）。",
    )
    parser.add_argument(
        "--no-report",
        action="store_true",
        help="Do not generate the Chinese Markdown result report after simulation.",
    )
    parser.add_argument(
        "--report-title",
        default=None,
        help="Override the generated Chinese report title.",
    )
    return parser


def make_simulation(config: SimulationConfig):
    if config.enable_obstacles:
        return ObstacleScenarioSimulation(config=config)
    return FormationSimulation(config=config)


def _resolve_run_dir(output_dir: str | Path, preset: str, run_name: str | None) -> Path:
    """生成 ``<output_dir>/<preset>/<run_name|timestamp>/`` 目录。"""
    base = Path(output_dir)
    stamp = run_name or datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = base / preset / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def _config_snapshot(config: SimulationConfig) -> dict:
    """把 SimulationConfig 转成可 JSON 化字典（数组转 list）。"""
    if is_dataclass(config):
        raw = asdict(config)
    else:  # pragma: no cover
        raw = dict(getattr(config, "__dict__", {}))
    return json_safe(raw)


def run_with_config(
    config: SimulationConfig,
    output_dir: str | Path = "outputs",
    plot: bool = True,
    *,
    preset: str = "custom",
    run_name: str | None = None,
    validate_schema: bool = True,
    include_trajectories: bool = False,
    generate_report: bool = True,
    report_title: str | None = None,
) -> dict:
    sim = make_simulation(config)

    start_time = time.perf_counter()
    result = sim.run()
    elapsed = time.perf_counter() - start_time

    run_dir = _resolve_run_dir(output_dir, preset, run_name)

    figure_paths: dict[str, str] = {}
    if plot:
        visualizer = SimulationVisualizer(output_dir=str(run_dir))
        figure_paths = visualizer.plot_all(result, show=False)

    payload = build_sim_result_payload(
        preset=preset,
        sim_result=result,
        runtime_s=elapsed,
        runtime_engine="python",
        config_snapshot=_config_snapshot(config),
        include_trajectories=include_trajectories,
    )
    if validate_schema:
        validate(payload, "sim_result", strict=True)

    sim_result_path = write_json(payload, run_dir / "sim_result.json")
    report_path = None
    if generate_report:
        report_path = generate_result_report(sim_result_path, title=report_title)

    print_summary(
        config,
        result,
        elapsed,
        figure_paths,
        sim_result_path=sim_result_path,
        report_path=report_path,
    )
    return result


def print_summary(
    config: SimulationConfig,
    result: dict,
    elapsed: float,
    figure_paths: dict[str, str],
    *,
    sim_result_path: Path | None = None,
    report_path: Path | None = None,
) -> None:
    means = result["metrics"]["mean"]
    maxs = result["metrics"]["max"]
    finals = result["metrics"]["final"]
    overall_max = float(np.max(maxs)) if len(maxs) > 0 else 0.0
    collision_count = len(result.get("collision_log", []))
    replan_count = len(result.get("replan_events", []))
    mode_label = "online" if getattr(config, "planner_mode", "offline") == "online" else "offline"
    scene_label = "obstacle" if config.enable_obstacles else "formation"

    print("=" * 56)
    print(f"{scene_label} simulation completed [{mode_label}]")
    print("=" * 56)
    print(f"elapsed:             {elapsed:.2f} s")
    print(f"completed waypoints: {result.get('completed_waypoint_count', 'N/A')}")
    if config.enable_obstacles:
        print(f"collisions:          {collision_count}")
        print(f"replan events:       {replan_count}")
    print(f"overall max error:   {overall_max:.4f} m")
    print("-" * 56)
    for idx in range(len(means)):
        print(
            f"Follower {idx + 1}: "
            f"mean={means[idx]:.4f} m, "
            f"max={maxs[idx]:.4f} m, "
            f"final={finals[idx]:.4f} m"
        )
    if figure_paths:
        print("-" * 56)
        print("figures:")
        for key, path in figure_paths.items():
            print(f"  {key}: {path}")
    if sim_result_path is not None:
        print("-" * 56)
        print(f"sim_result.json: {sim_result_path}")
    if report_path is not None:
        print(f"report.md:       {report_path}")
    print("=" * 56)


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    config = get_config(args.preset)
    if args.max_sim_time is not None:
        config.max_sim_time = float(args.max_sim_time)
    run_with_config(
        config,
        output_dir=args.output_dir,
        plot=not args.no_plot,
        preset=args.preset,
        run_name=args.run_name,
        validate_schema=not args.no_validate,
        include_trajectories=args.include_trajectories,
        generate_report=not args.no_report,
        report_title=args.report_title,
    )


if __name__ == "__main__":
    main()
