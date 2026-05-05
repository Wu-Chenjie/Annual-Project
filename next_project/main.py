"""Command-line entry point for the UAV formation simulation project."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from config import get_config
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
        help="Directory for generated figures and result files. Default: outputs",
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
    return parser


def make_simulation(config: SimulationConfig):
    if config.enable_obstacles:
        return ObstacleScenarioSimulation(config=config)
    return FormationSimulation(config=config)


def run_with_config(
    config: SimulationConfig,
    output_dir: str | Path = "outputs",
    plot: bool = True,
) -> dict:
    sim = make_simulation(config)

    start_time = time.perf_counter()
    result = sim.run()
    elapsed = time.perf_counter() - start_time

    figure_paths: dict[str, str] = {}
    if plot:
        visualizer = SimulationVisualizer(output_dir=str(output_dir))
        figure_paths = visualizer.plot_all(result, show=False)

    print_summary(config, result, elapsed, figure_paths)
    return result


def print_summary(
    config: SimulationConfig,
    result: dict,
    elapsed: float,
    figure_paths: dict[str, str],
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
    print("=" * 56)


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    config = get_config(args.preset)
    if args.max_sim_time is not None:
        config.max_sim_time = float(args.max_sim_time)
    run_with_config(config, output_dir=args.output_dir, plot=not args.no_plot)


if __name__ == "__main__":
    main()
