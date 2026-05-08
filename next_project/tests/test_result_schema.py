"""Tests for ``core.result_schema`` and the standardized output pipeline.

覆盖：
- 两份 JSON Schema 可加载、与 build_*_payload 输出兼容；
- ``validate(strict=True)`` 在缺字段时会抛错；
- ``main.run_with_config`` 写出的 ``sim_result.json`` 落在 ``outputs/<preset>/<run_name>/``
  且通过 schema 校验。
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
import pytest

from core.result_schema import (
    SCHEMA_VERSION,
    build_benchmark_payload,
    build_sim_result_payload,
    load_schema,
    validate,
)


def _fake_sim_result() -> dict:
    return {
        "metrics": {
            "mean": np.array([0.10, 0.12]),
            "max": np.array([0.20, 0.22]),
            "final": np.array([0.05, 0.06]),
        },
        "completed_waypoint_count": 4,
        "collision_log": [],
        "replan_events": [],
        "fault_log": [],
        "safety_metrics": {"min_inter_drone_distance": 0.42, "downwash_hits": 0},
    }


def test_schemas_are_loadable():
    sim = load_schema("sim_result")
    bench = load_schema("benchmark_result")
    assert sim["title"] == "SimulationResult"
    assert bench["title"] == "BenchmarkResult"


def test_build_sim_result_payload_validates():
    payload = build_sim_result_payload(
        preset="basic",
        sim_result=_fake_sim_result(),
        runtime_s=1.23,
    )
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["runtime_engine"] == "python"
    assert payload["metrics"]["mean"] == [0.10, 0.12]
    assert payload["summary"]["collision_count"] == 0
    assert validate(payload, "sim_result", strict=False) == []


def test_build_benchmark_payload_validates():
    records = [
        {
            "run_index": 0,
            "runtime_s": 1.0,
            "mean_error": [0.1],
            "max_error": [0.2],
            "final_error": [0.05],
            "completed_waypoint_count": 3,
        }
    ]
    summary = {
        "runs": 1,
        "runtime_mean_s": 1.0,
        "runtime_std_s": 0.0,
        "mean_error_mean": [0.1],
        "mean_error_std": [0.0],
        "max_error_mean": [0.2],
        "max_error_std": [0.0],
        "worst_case_max_error": 0.2,
        "all_runs_pass_0_3m": True,
    }
    payload = build_benchmark_payload(
        preset="basic",
        runs=1,
        records=records,
        summary=summary,
    )
    assert validate(payload, "benchmark_result", strict=False) == []


def test_validate_strict_raises_on_missing_required_field():
    bad = {
        "schema_version": SCHEMA_VERSION,
        "preset": "basic",
        # runtime_engine missing
        "metrics": {"mean": [0.1], "max": [0.2], "final": [0.05]},
        "completed_waypoint_count": 1,
        "summary": {
            "mean_error_overall": 0.1,
            "max_error_overall": 0.2,
            "final_error_overall": 0.05,
        },
    }
    with pytest.raises(ValueError):
        validate(bad, "sim_result", strict=True)


def test_runtime_engine_enum_rejects_unknown_value():
    payload = build_sim_result_payload(
        preset="basic",
        sim_result=_fake_sim_result(),
        runtime_s=1.0,
    )
    payload["runtime_engine"] = "matlab"
    errs = validate(payload, "sim_result", strict=False)
    assert errs, "应当因为 enum 不匹配而失败"


def test_run_with_config_writes_standard_layout(tmp_path: Path):
    """快速冒烟：用 basic 预设跑一次，检查输出目录与 sim_result.json 内容。"""
    from config import get_config
    from main import run_with_config

    config = get_config("basic")
    config.max_sim_time = 1.0  # 加速测试
    out_root = tmp_path / "outputs"

    run_with_config(
        config,
        output_dir=str(out_root),
        plot=False,
        preset="basic",
        run_name="testrun",
        validate_schema=True,
    )

    expected_dir = out_root / "basic" / "testrun"
    assert expected_dir.is_dir(), f"期望目录 {expected_dir} 不存在"

    sim_result_path = expected_dir / "sim_result.json"
    assert sim_result_path.is_file()

    payload = json.loads(sim_result_path.read_text(encoding="utf-8"))
    assert payload["preset"] == "basic"
    assert payload["runtime_engine"] == "python"
    assert payload["schema_version"] == SCHEMA_VERSION
    # 严格校验
    assert validate(payload, "sim_result", strict=False) == []


def test_cli_argparse_parses_run_name_and_no_validate():
    from main import build_parser

    parser = build_parser()
    args = parser.parse_args([
        "--preset", "basic",
        "--run-name", "abc",
        "--no-validate",
        "--no-plot",
    ])
    assert args.preset == "basic"
    assert args.run_name == "abc"
    assert args.no_validate is True
    assert args.no_plot is True
