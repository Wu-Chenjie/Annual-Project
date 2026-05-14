from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# Allow execution from any working directory (e.g. C++ calls from cpp/)
_script_dir = Path(__file__).resolve().parent
_root_dir = _script_dir.parent
if str(_root_dir) not in sys.path:
    sys.path.insert(0, str(_root_dir))

from experiments.result_reporter import generate_result_report


def generate_cpp_result_report(
    input_json: str | Path,
    output_dir: str | Path | None = None,
    *,
    title: str = "C++ 仿真结果报告",
) -> Path:
    input_path = Path(input_json)
    _validate_cpp_like_payload(input_path)
    return generate_result_report(
        input_path,
        output_dir,
        title=title,
        report_name="cpp_report.md",
        metrics_name="cpp_metrics.json",
        figure_dir_name="cpp_report_figures",
    )


def _validate_cpp_like_payload(input_path: Path) -> None:
    payload: dict[str, Any] = json.loads(input_path.read_text(encoding="utf-8"))
    engine = str(payload.get("runtime_engine") or "")
    if engine and engine != "cpp":
        raise ValueError(f"expected runtime_engine='cpp', got {engine!r}")
    required = ["metrics", "summary", "completed_waypoint_count"]
    missing = [key for key in required if key not in payload]
    if missing:
        raise ValueError("C++ result JSON missing required fields: " + ", ".join(missing))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="从 C++ sim_result.json 生成中文图片报告。")
    parser.add_argument("input_json", help="C++ sim_result.json 路径。")
    parser.add_argument("--output-dir", default=None, help="输出目录；默认写到输入文件所在目录。")
    parser.add_argument("--title", default="C++ 仿真结果报告", help="Markdown 报告标题。")
    args = parser.parse_args(argv)

    report_path = generate_cpp_result_report(args.input_json, args.output_dir, title=args.title)
    print(f"C++ 报告已生成: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["generate_cpp_result_report"]
