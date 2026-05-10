from __future__ import annotations

import json
import os
import subprocess
import time
import sys
from contextlib import contextmanager
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.risk_report import build_risk_report
from tools.cpp_risk_report import build_cpp_risk_report


_PROJECT = Path(__file__).resolve().parent.parent


@contextmanager
def _file_lock(path: Path, timeout_s: float = 120.0):
    path.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.time() + timeout_s
    handle = None
    while handle is None:
        try:
            fd = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_RDWR)
            handle = os.fdopen(fd, "w")
        except FileExistsError:
            if time.time() >= deadline:
                raise TimeoutError(f"timed out waiting for lock: {path}")
            time.sleep(0.1)
    try:
        yield
    finally:
        handle.close()
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def _build_cpp_target(target: str) -> Path:
    build_dir = _PROJECT / "cpp" / "build"
    exe_name = f"{target}.exe" if os.name == "nt" else target
    with _file_lock(build_dir / ".pytest_cmake_build.lock"):
        subprocess.run(
            ["cmake", "--build", str(build_dir), "--target", target],
            cwd=_PROJECT / "cpp",
            check=True,
            capture_output=True,
            text=True,
        )
    return build_dir / exe_name


def test_risk_report_builds_from_obstacle_scenario_like_result():
    sim_result = {
        "task_waypoints": np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]], dtype=float),
        "replanned_waypoints": np.array([[0.0, 0.0, 1.0], [1.0, 0.2, 1.0], [2.0, 0.0, 1.0]], dtype=float),
        "executed_path": np.array([[0.0, 0.0, 1.0], [1.0, 0.1, 1.0], [2.0, 0.0, 1.0]], dtype=float),
        "replan_events": [{"t": 0.4, "mode": "online"}],
        "sensor_logs": np.array([[1.2, 1.4, 2.0, 2.1, 1.8, 1.9]], dtype=float),
        "collision_log": [],
        "fault_log": [],
        "metrics": {
            "mean": [0.2, 0.25],
            "max": [0.4, 0.5],
        },
    }
    report = build_risk_report(sim_result, "indoor_research").to_dict()
    assert report["safety_profile"]["name"] == "indoor_research"
    assert report["observed_events"]["collisions"] == 0
    assert report["observed_events"]["replans"] == 1
    assert report["compliance_checks"]["collision_free"] is True
    assert report["residual_risk"]["level"] == "low"


def test_risk_report_marks_regulatory_profile_as_scale_mismatch_for_indoor_result():
    sim_result = {
        "task_waypoints": np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]], dtype=float),
        "executed_path": np.array([[0.0, 0.0, 1.0], [1.8, 0.0, 1.0]], dtype=float),
        "replanned_waypoints": np.zeros((0, 3), dtype=float),
        "replan_events": [],
        "sensor_logs": None,
        "collision_log": [],
        "fault_log": [],
        "metrics": {},
    }
    report = build_risk_report(sim_result, "low_altitude_regulatory").to_dict()
    assert report["compliance_checks"]["indoor_scale_match"] is False


def test_risk_report_accepts_cpp_warehouse_result_json_subset():
    root = _PROJECT
    exe = _build_cpp_target("sim_warehouse")

    output_root = root / "outputs" / "warehouse"
    before = set(output_root.glob("*/sim_result.json"))

    subprocess.run(
        [str(exe)],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    after = set(output_root.glob("*/sim_result.json"))
    new_outputs = sorted(after - before, key=lambda p: p.stat().st_mtime, reverse=True)
    assert new_outputs
    output_path = new_outputs[0]
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    report = build_risk_report(payload, "indoor_demo").to_dict()
    assert report["conops"]["task_waypoint_count"] == len(payload["task_waypoints"])
    assert report["conops"]["executed_path_samples"] == len(payload["executed_path"])
    assert report["observed_events"]["faults"] == len(payload["fault_log"])
    assert report["observed_events"]["min_inter_drone_distance"] == payload["safety_metrics"]["min_inter_drone_distance"]
    assert report["observed_events"]["downwash_hits"] == payload["safety_metrics"]["downwash_hits"]


def test_cpp_risk_report_tool_writes_json_and_markdown(tmp_path: Path):
    root = _PROJECT
    exe = _build_cpp_target("sim_warehouse")

    output_root = root / "outputs" / "warehouse"
    before = set(output_root.glob("*/sim_result.json"))

    subprocess.run(
        [str(exe)],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    after = set(output_root.glob("*/sim_result.json"))
    new_outputs = sorted(after - before, key=lambda p: p.stat().st_mtime, reverse=True)
    assert new_outputs
    output_path = new_outputs[0]

    json_path, md_path = build_cpp_risk_report(
        output_path,
        safety_profile="indoor_demo",
        output_dir=tmp_path,
    )
    assert json_path.exists()
    assert md_path.exists()

    report = json.loads(json_path.read_text(encoding="utf-8"))
    markdown = md_path.read_text(encoding="utf-8")
    assert report["safety_profile"]["name"] == "indoor_demo"
    assert "min_inter_drone_distance" in report["observed_events"]
    assert "downwash_hits" in report["observed_events"]
    assert "# C++ Risk Report:" in markdown
    assert "## Observed Events" in markdown
