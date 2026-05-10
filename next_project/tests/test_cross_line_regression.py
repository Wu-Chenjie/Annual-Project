from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from core.result_schema import validate
from scripts.compare_results import compare_benchmark, compare_sim_results


ROOT = Path(__file__).resolve().parents[1]


def _latest_json(root: Path, pattern: str) -> Path:
    candidates = sorted(root.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        raise AssertionError(f"no JSON files matched {pattern!r} under {root}")
    return candidates[0]


def _cpp_exe(name: str) -> Path | None:
    suffix = ".exe" if os.name == "nt" else ""
    for rel in (
        f"cpp/build/{name}{suffix}",
        f"cpp/build_windows/{name}{suffix}",
        f"cpp/build_codex/{name}{suffix}",
        f"cpp/build/{name}",
    ):
        path = ROOT / rel
        if path.exists():
            return path
    return None


def _run(cmd: list[str], *, timeout: int = 180) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=True,
    )


def test_cross_line_regression_contract_is_documented() -> None:
    doc = (ROOT / "docs" / "capability-matrix.md").read_text(encoding="utf-8")
    assert "basic" in doc
    assert "warehouse" in doc
    assert "school_corridor" in doc
    assert "Python / C++ / Web" in doc
    assert "task_waypoints" in doc
    assert "replanned_waypoints" in doc


@pytest.mark.skipif(
    os.environ.get("RUN_CROSS_LINE_REGRESSION") != "1",
    reason="set RUN_CROSS_LINE_REGRESSION=1 to execute Python/C++ runtime parity checks",
)
def test_basic_python_cpp_sim_result_regression() -> None:
    exe = _cpp_exe("sim_main")
    if exe is None:
        pytest.skip("C++ sim_main executable is not built")

    run_name = "cross-line-basic"
    py_dir = ROOT / "outputs" / "basic" / run_name
    cpp_before = set((ROOT / "outputs" / "basic").glob("*/sim_result.json"))

    _run([sys.executable, "main.py", "--preset", "basic", "--run-name", run_name, "--no-plot"])
    _run([str(exe)])

    py_payload = json.loads((py_dir / "sim_result.json").read_text(encoding="utf-8"))
    cpp_after = set((ROOT / "outputs" / "basic").glob("*/sim_result.json"))
    new_cpp = sorted(cpp_after - cpp_before, key=lambda p: p.stat().st_mtime, reverse=True)
    cpp_path = new_cpp[0] if new_cpp else _latest_json(ROOT / "outputs" / "basic", "*/sim_result.json")
    cpp_payload = json.loads(cpp_path.read_text(encoding="utf-8"))

    validate(py_payload, "sim_result", strict=True)
    validate(cpp_payload, "sim_result", strict=True)

    rows = compare_sim_results(py_payload, cpp_payload, rel_tol=0.75, abs_tol=0.35)
    failures = [row for row in rows if not row["ok"] and row["name"] not in {"runtime_engine"}]
    assert not failures


@pytest.mark.skipif(
    os.environ.get("RUN_CROSS_LINE_REGRESSION") != "1",
    reason="set RUN_CROSS_LINE_REGRESSION=1 to execute Python/C++ runtime parity checks",
)
def test_benchmark_python_cpp_schema_regression() -> None:
    exe = _cpp_exe("sim_benchmark")
    if exe is None:
        pytest.skip("C++ sim_benchmark executable is not built")

    py_path = ROOT / "outputs" / "benchmark_default" / "cross-line-benchmark" / "benchmark_results.json"
    cpp_before = set((ROOT / "outputs" / "benchmark_default").glob("*/benchmark_results.json"))

    code = (
        "from simulations.benchmark import run_benchmark; "
        f"run_benchmark(runs=1, output_file={str(py_path)!r}, preset='benchmark_default')"
    )
    _run([sys.executable, "-c", code])
    _run([str(exe)], timeout=240)

    py_payload = json.loads(py_path.read_text(encoding="utf-8"))
    cpp_after = set((ROOT / "outputs" / "benchmark_default").glob("*/benchmark_results.json"))
    new_cpp = sorted(cpp_after - cpp_before, key=lambda p: p.stat().st_mtime, reverse=True)
    cpp_path = new_cpp[0] if new_cpp else _latest_json(ROOT / "outputs" / "benchmark_default", "*/benchmark_results.json")
    cpp_payload = json.loads(cpp_path.read_text(encoding="utf-8"))

    validate(py_payload, "benchmark_result", strict=True)
    validate(cpp_payload, "benchmark_result", strict=True)

    rows = compare_benchmark(py_payload, cpp_payload, rel_tol=1.0, abs_tol=0.5)
    failures = [row for row in rows if not row["ok"] and row["name"] != "runs"]
    assert not failures
