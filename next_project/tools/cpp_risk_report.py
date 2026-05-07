from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from core.risk_report import build_risk_report


def render_markdown(report: dict[str, Any], *, source_name: str) -> str:
    observed = report["observed_events"]
    checks = report["compliance_checks"]
    residual = report["residual_risk"]
    profile = report["safety_profile"]
    conops = report["conops"]
    lines = [
        f"# C++ Risk Report: {source_name}",
        "",
        f"- safety_profile: `{profile['name']}`",
        f"- residual_risk: `{residual['level']}`",
        f"- task_waypoint_count: {conops['task_waypoint_count']}",
        f"- replanned_waypoint_count: {conops['replanned_waypoint_count']}",
        f"- executed_path_samples: {conops['executed_path_samples']}",
        "",
        "## Observed Events",
        "",
        f"- collisions: {observed['collisions']}",
        f"- replans: {observed['replans']}",
        f"- faults: {observed['faults']}",
        f"- final_goal_distance: {observed['final_goal_distance']}",
        f"- min_inter_drone_distance: {observed['min_inter_drone_distance']}",
        f"- downwash_hits: {observed['downwash_hits']}",
        "",
        "## Compliance Checks",
        "",
        f"- collision_free: {checks['collision_free']}",
        f"- indoor_scale_match: {checks['indoor_scale_match']}",
        f"- final_goal_within_horizontal_separation: {checks['final_goal_within_horizontal_separation']}",
        "",
        "## Residual Risk",
        "",
        residual["summary"],
        "",
    ]
    return "\n".join(lines)


def build_cpp_risk_report(
    input_path: str | Path,
    *,
    safety_profile: str = "indoor_demo",
    output_dir: str | Path | None = None,
) -> tuple[Path, Path]:
    input_path = Path(input_path)
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    report = build_risk_report(payload, safety_profile).to_dict()

    out_dir = Path(output_dir) if output_dir is not None else Path("outputs") / "reports"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = input_path.stem
    json_path = out_dir / f"{stem}_risk_report.json"
    md_path = out_dir / f"{stem}_risk_report.md"

    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    md_path.write_text(render_markdown(report, source_name=input_path.name), encoding="utf-8")
    return json_path, md_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Markdown/JSON risk reports from C++ obstacle result JSON.")
    parser.add_argument("input", help="Path to C++ obstacle result json, e.g. cpp/outputs/warehouse_result.json")
    parser.add_argument("--safety-profile", default="indoor_demo", help="Safety profile name")
    parser.add_argument("--output-dir", default=None, help="Directory for generated reports")
    args = parser.parse_args()

    json_path, md_path = build_cpp_risk_report(
        args.input,
        safety_profile=args.safety_profile,
        output_dir=args.output_dir,
    )
    print(f"JSON report: {json_path}")
    print(f"Markdown report: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
