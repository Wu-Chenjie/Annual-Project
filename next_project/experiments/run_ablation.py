from __future__ import annotations

import argparse
import json
from pathlib import Path

from experiments.ablation import apply_variant
from experiments.metrics_extractor import extract_metrics_file
from experiments.report_writer import write_markdown_report, write_summary_csv
from experiments.scenario_registry import get_scenario_config
from core.result_schema import json_safe
from simulations.formation_simulation import FormationSimulation
from simulations.obstacle_scenario import ObstacleScenarioSimulation


def main() -> int:
    parser = argparse.ArgumentParser(description="Run small ablation batches and collect scalar metrics.")
    parser.add_argument("--scenario", action="append", default=None, help="Scenario preset name. Repeatable.")
    parser.add_argument("--variant", action="append", default=None, help="Ablation variant. Repeatable.")
    parser.add_argument("--output-dir", default="outputs/ablation", help="Output directory.")
    parser.add_argument("--quick", action="store_true", help="Clamp long scenarios to a short smoke-test horizon.")
    args = parser.parse_args()
    scenarios = args.scenario or ["basic"]
    variants = args.variant or ["baseline"]

    out_dir = Path(args.output_dir)
    rows = []
    for scenario in scenarios:
        for variant in variants:
            cfg = get_scenario_config(scenario, quick=args.quick)
            cfg = apply_variant(cfg, variant)
            run_name = f"{scenario}_{variant}"
            cfg.max_sim_time = min(float(cfg.max_sim_time), 5.0) if args.quick else cfg.max_sim_time
            sim = ObstacleScenarioSimulation(cfg) if cfg.enable_obstacles else FormationSimulation(cfg)
            result = sim.run()
            payload_path = out_dir / "raw" / f"{run_name}.json"
            payload_path.parent.mkdir(parents=True, exist_ok=True)
            payload_path.write_text(
                json.dumps(json_safe(result), ensure_ascii=False, indent=2, default=str),
                encoding="utf-8",
            )
            metrics = extract_metrics_file(payload_path)
            metrics["scenario"] = scenario
            metrics["variant"] = variant
            rows.append(metrics)

    write_summary_csv(rows, out_dir / "summary.csv")
    write_markdown_report(rows, out_dir / "report.md", title="Ablation Report")
    print(f"wrote {out_dir / 'summary.csv'}")
    print(f"wrote {out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
