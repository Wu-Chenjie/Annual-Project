"""批量评测脚本。

用途：对编队仿真进行多次重复实验，输出误差与耗时统计，并支持保存结果文件。
原理：固定同一套控制和动力学参数，仅改变风场随机种子，统计均值、标准差和极值，形成可对比、可复现实验数据。
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

# 允许通过 `python simulations/benchmark.py` 直接运行。
if __package__ is None or __package__ == "":
    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

from simulations.formation_simulation import FormationSimulation, SimulationConfig


def _json_safe(obj):
    """递归将 numpy 对象转换为可 JSON 序列化结构。"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    return obj


def run_benchmark(runs: int = 5, output_file: str | None = None) -> dict:
    records = []
    for i in range(runs):
        cfg = SimulationConfig(
            max_sim_time=30.0,
            use_smc=True,
            leader_wind_seed=42 + i,
            follower_wind_seed_start=100 + 10 * i,
        )
        start = time.perf_counter()
        result = FormationSimulation(config=cfg).run()
        elapsed = time.perf_counter() - start
        records.append(
            {
                "run_index": i,
                "runtime_s": float(elapsed),
                "mean_error": [float(v) for v in result["metrics"]["mean"]],
                "max_error": [float(v) for v in result["metrics"]["max"]],
                "final_error": [float(v) for v in result["metrics"]["final"]],
                "completed_waypoint_count": int(result["completed_waypoint_count"]),
            }
        )

    mean_errors = np.array([r["mean_error"] for r in records], dtype=float)
    max_errors = np.array([r["max_error"] for r in records], dtype=float)
    runtimes = np.array([r["runtime_s"] for r in records], dtype=float)

    summary = {
        "runs": runs,
        "runtime_mean_s": float(np.mean(runtimes)),
        "runtime_std_s": float(np.std(runtimes)),
        "mean_error_mean": [float(v) for v in np.mean(mean_errors, axis=0)],
        "mean_error_std": [float(v) for v in np.std(mean_errors, axis=0)],
        "max_error_mean": [float(v) for v in np.mean(max_errors, axis=0)],
        "max_error_std": [float(v) for v in np.std(max_errors, axis=0)],
        "worst_case_max_error": float(np.max(max_errors)),
        "all_runs_pass_0_3m": bool(np.all(max_errors < 0.3)),
    }

    payload = {
        "config_template": _json_safe(asdict(SimulationConfig(max_sim_time=30.0, use_smc=True))),
        "summary": summary,
        "records": records,
    }

    if output_file is not None:
        path = Path(output_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    return payload


def main() -> None:
    output_path = Path("outputs") / "benchmark_results.json"
    result = run_benchmark(runs=5, output_file=str(output_path))

    print("批量评测完成")
    print(f"结果文件: {output_path}")
    print(f"平均运行时间: {result['summary']['runtime_mean_s']:.3f} s")
    print("从机平均误差均值:", [round(v, 4) for v in result["summary"]["mean_error_mean"]])
    print("从机最大误差均值:", [round(v, 4) for v in result["summary"]["max_error_mean"]])
    print("最差工况最大误差:", round(result["summary"]["worst_case_max_error"], 4))
    print("0.3m 阈值全通过:", result["summary"]["all_runs_pass_0_3m"])


if __name__ == "__main__":
    main()
