# Competition Map Demo Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the competition-ready code pipeline that turns imported reconstruction models into audited UAV maps, reproducible simulation runs, demo evidence pages, and ROS2-ready interface metadata without requiring a full ROS2 migration.

**Architecture:** Keep the existing planner and obstacle model unchanged: arbitrary imported shapes still enter the system through `.ply/.obj/.stl -> voxelized AABB map -> existing planner/replay/report pipeline`. Add small, testable tools around the current core: one map-quality auditor, one direct import CLI, one imported-map simulation runner, one demo index builder, and one ROS2 bridge-contract exporter. ROS2 stays as an interface contract and downstream bridge target, not a runtime dependency for the competition demo.

**Tech Stack:** Python 3.14, NumPy, pytest, existing `web.server` model parsers, existing `core.map_loader`, existing `main.run_with_config`, Markdown/JSON outputs.

---

## File Structure

- Create `experiments/map_quality.py`
  - Responsibility: audit imported `maps/*.json` files for bounds, obstacle count, approximate occupied volume, waypoint clearance, and map-scale warnings.
- Create `tools/photo_reconstruction_import.py`
  - Responsibility: import `.ply/.obj/.stl` directly from CLI by reusing the existing Web parser and `_model_to_map()` conversion logic.
- Create `tools/run_imported_map_smoke.py`
  - Responsibility: run an imported map with explicit waypoints and stable output naming through `main.run_with_config()`.
- Create `experiments/competition_demo.py`
  - Responsibility: build a Markdown index that ties photos, 3DGS/mesh assets, map audits, sim results, reports, and demo videos together scene-by-scene.
- Create `tools/export_ros2_bridge_manifest.py`
  - Responsibility: generate a ROS2-ready topic/frame contract JSON from a `sim_result.json` without importing ROS2 packages.
- Create `docs/ros2_bridge_contract.md`
  - Responsibility: document the future ROS2 interface boundary, topics, frames, and what is explicitly not implemented yet.
- Add tests:
  - `tests/test_map_quality.py`
  - `tests/test_photo_reconstruction_import.py`
  - `tests/test_imported_map_smoke.py`
  - `tests/test_competition_demo.py`
  - `tests/test_ros2_bridge_manifest.py`

Implementation boundary:
- Do not add new obstacle shape types for competition.
- Do not make the planner consume raw 3DGS, mesh, or point cloud geometry.
- Do not require ROS2, Gazebo, PX4, MAVROS, or RViz to run the competition demo.
- Do not tune planner safety radii to hide an imported-map scale error.

---

### Task 1: Imported Map Quality Audit

**Files:**
- Create: `experiments/map_quality.py`
- Create: `tests/test_map_quality.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_map_quality.py`:

```python
from __future__ import annotations

import json
from pathlib import Path

from experiments.map_quality import audit_map, write_audit_files


def _write_demo_map(path: Path) -> None:
    payload = {
        "bounds": [[0, 0, 0], [5, 4, 3]],
        "description": "Imported demo map; voxel_size=0.5, scale=1.0",
        "obstacles": [
            {"type": "aabb", "min": [1.0, 1.0, 0.0], "max": [2.0, 1.5, 2.0]},
            {"type": "cylinder", "center_xy": [3.0, 3.0], "radius": 0.25, "z_range": [0.0, 2.0]},
        ],
        "waypoints": [
            [0.5, 0.5, 1.5],
            [4.5, 3.5, 1.5],
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_audit_map_reports_core_quality_metrics(tmp_path: Path):
    map_path = tmp_path / "meeting_room_photo.json"
    _write_demo_map(map_path)

    audit = audit_map(map_path, safety_margin=0.3)

    assert audit["map_name"] == "meeting_room_photo"
    assert audit["obstacle_count"] == 2
    assert audit["bounds_extent"] == [5.0, 4.0, 3.0]
    assert audit["bounds_volume"] == 60.0
    assert audit["approx_obstacle_volume"] > 0.0
    assert 0.0 < audit["approx_occupied_ratio"] < 1.0
    assert audit["waypoint_count"] == 2
    assert audit["waypoints_clear"] is True
    assert audit["status"] == "pass"
    assert audit["warnings"] == []


def test_audit_map_flags_close_waypoint(tmp_path: Path):
    map_path = tmp_path / "blocked_photo.json"
    payload = {
        "bounds": [[0, 0, 0], [4, 4, 3]],
        "obstacles": [
            {"type": "aabb", "min": [1.0, 1.0, 0.0], "max": [2.0, 2.0, 2.0]},
        ],
        "waypoints": [
            [1.2, 1.2, 1.0],
        ],
    }
    map_path.write_text(json.dumps(payload), encoding="utf-8")

    audit = audit_map(map_path, safety_margin=0.3)

    assert audit["status"] == "fail"
    assert audit["waypoints_clear"] is False
    assert audit["waypoint_clearances"][0]["clearance"] < 0.3
    assert any("waypoint 0 clearance" in warning for warning in audit["warnings"])


def test_write_audit_files_writes_json_and_markdown(tmp_path: Path):
    map_path = tmp_path / "demo_map.json"
    _write_demo_map(map_path)

    json_path, md_path = write_audit_files(map_path, tmp_path / "audit", safety_margin=0.3)

    assert json_path == tmp_path / "audit" / "demo_map_map_audit.json"
    assert md_path == tmp_path / "audit" / "demo_map_map_audit.md"
    data = json.loads(json_path.read_text(encoding="utf-8"))
    text = md_path.read_text(encoding="utf-8")
    assert data["status"] == "pass"
    assert "| obstacle_count | 2 |" in text
    assert "## Waypoint Clearance" in text
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```powershell
python -m pytest tests/test_map_quality.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'experiments.map_quality'`.

- [ ] **Step 3: Implement the map audit module**

Create `experiments/map_quality.py`:

```python
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from core.map_loader import load_from_json


def _aabb_volume(obs: dict[str, Any]) -> float:
    mins = np.array(obs["min"], dtype=float)
    maxs = np.array(obs["max"], dtype=float)
    return float(np.prod(np.maximum(maxs - mins, 0.0)))


def _cylinder_volume(obs: dict[str, Any]) -> float:
    radius = float(obs["radius"])
    z0, z1 = obs["z_range"]
    return float(math.pi * radius * radius * max(float(z1) - float(z0), 0.0))


def _sphere_volume(obs: dict[str, Any]) -> float:
    radius = float(obs["radius"])
    return float(4.0 * math.pi * radius ** 3 / 3.0)


def _obstacle_volume(obs: dict[str, Any]) -> float:
    obs_type = obs.get("type")
    if obs_type == "aabb":
        return _aabb_volume(obs)
    if obs_type == "cylinder":
        return _cylinder_volume(obs)
    if obs_type == "sphere":
        return _sphere_volume(obs)
    return 0.0


def _round_float(value: float, digits: int = 4) -> float:
    return round(float(value), digits)


def audit_map(map_path: str | Path, *, safety_margin: float = 0.3) -> dict[str, Any]:
    path = Path(map_path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    bounds = np.array(raw["bounds"], dtype=float)
    field, _ = load_from_json(str(path))

    extent = np.maximum(bounds[1] - bounds[0], 0.0)
    bounds_volume = float(np.prod(extent))
    obstacles = list(raw.get("obstacles") or [])
    approx_obstacle_volume = sum(_obstacle_volume(obs) for obs in obstacles)
    approx_occupied_ratio = 0.0 if bounds_volume <= 0.0 else approx_obstacle_volume / bounds_volume

    warnings: list[str] = []
    if bounds_volume <= 0.0:
        warnings.append("bounds volume is zero; check imported model scale and bounds")
    if len(obstacles) == 0:
        warnings.append("map contains no obstacles; imported model may be empty")
    if len(obstacles) > 8000:
        warnings.append("obstacle count exceeds 8000; increase voxel_size or clean the model")
    if approx_occupied_ratio > 0.55:
        warnings.append("occupied ratio exceeds 55%; map may be over-filled or corridors may be blocked")

    waypoint_clearances: list[dict[str, Any]] = []
    for idx, waypoint in enumerate(raw.get("waypoints") or []):
        point = np.array(waypoint, dtype=float)
        clearance = float(field.signed_distance(point))
        clear = clearance >= safety_margin
        waypoint_clearances.append(
            {
                "index": idx,
                "point": [_round_float(x) for x in point],
                "clearance": _round_float(clearance),
                "clear": clear,
            }
        )
        if not clear:
            warnings.append(
                f"waypoint {idx} clearance {clearance:.3f} is below safety_margin {safety_margin:.3f}"
            )

    waypoints_clear = all(item["clear"] for item in waypoint_clearances)
    status = "pass" if not warnings else "fail"
    return {
        "map_name": path.stem,
        "map_path": str(path),
        "status": status,
        "bounds": [[_round_float(x) for x in row] for row in bounds.tolist()],
        "bounds_extent": [_round_float(x) for x in extent],
        "bounds_volume": _round_float(bounds_volume),
        "obstacle_count": len(obstacles),
        "approx_obstacle_volume": _round_float(approx_obstacle_volume),
        "approx_occupied_ratio": _round_float(approx_occupied_ratio),
        "safety_margin": _round_float(safety_margin),
        "waypoint_count": len(waypoint_clearances),
        "waypoints_clear": waypoints_clear,
        "waypoint_clearances": waypoint_clearances,
        "warnings": warnings,
    }


def _markdown_table(audit: dict[str, Any]) -> str:
    rows = [
        ("status", audit["status"]),
        ("bounds_extent", audit["bounds_extent"]),
        ("bounds_volume", audit["bounds_volume"]),
        ("obstacle_count", audit["obstacle_count"]),
        ("approx_occupied_ratio", audit["approx_occupied_ratio"]),
        ("waypoint_count", audit["waypoint_count"]),
        ("waypoints_clear", audit["waypoints_clear"]),
    ]
    lines = ["| metric | value |", "|---|---|"]
    lines.extend(f"| {key} | {value} |" for key, value in rows)
    return "\n".join(lines)


def render_markdown(audit: dict[str, Any]) -> str:
    lines = [
        f"# Map Audit: {audit['map_name']}",
        "",
        _markdown_table(audit),
        "",
        "## Waypoint Clearance",
        "",
        "| index | point | clearance | clear |",
        "|---:|---|---:|---|",
    ]
    for item in audit["waypoint_clearances"]:
        lines.append(
            f"| {item['index']} | {item['point']} | {item['clearance']} | {item['clear']} |"
        )
    lines.extend(["", "## Warnings", ""])
    if audit["warnings"]:
        lines.extend(f"- {warning}" for warning in audit["warnings"])
    else:
        lines.append("- none")
    lines.append("")
    return "\n".join(lines)


def write_audit_files(
    map_path: str | Path,
    output_dir: str | Path,
    *,
    safety_margin: float = 0.3,
) -> tuple[Path, Path]:
    audit = audit_map(map_path, safety_margin=safety_margin)
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(map_path).stem
    json_path = out_dir / f"{stem}_map_audit.json"
    md_path = out_dir / f"{stem}_map_audit.md"
    json_path.write_text(json.dumps(audit, indent=2, ensure_ascii=False), encoding="utf-8")
    md_path.write_text(render_markdown(audit), encoding="utf-8")
    return json_path, md_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit an imported UAV map JSON file.")
    parser.add_argument("map", help="Path to maps/<name>.json")
    parser.add_argument("--output-dir", default="outputs/map_audits")
    parser.add_argument("--safety-margin", type=float, default=0.3)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    json_path, md_path = write_audit_files(
        args.map,
        args.output_dir,
        safety_margin=args.safety_margin,
    )
    print(f"map audit json: {json_path}")
    print(f"map audit markdown: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run tests to verify they pass**

Run:

```powershell
python -m pytest tests/test_map_quality.py -q
```

Expected: `3 passed`.

- [ ] **Step 5: Commit**

```powershell
git add experiments/map_quality.py tests/test_map_quality.py
git commit -m "feat: add imported map quality audit"
```

---

### Task 2: Direct Reconstruction Model Import CLI

**Files:**
- Create: `tools/photo_reconstruction_import.py`
- Create: `tests/test_photo_reconstruction_import.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_photo_reconstruction_import.py`:

```python
from __future__ import annotations

import json
from pathlib import Path

from tools.photo_reconstruction_import import import_model_file


ASCII_PLY = """ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
element face 2
property list uchar int vertex_indices
end_header
0 0 0
1 0 0
1 1 0
0 1 0
3 0 1 2
3 0 2 3
"""


def test_import_model_file_writes_map_json(tmp_path: Path):
    model_path = tmp_path / "table_top.ply"
    model_path.write_text(ASCII_PLY, encoding="utf-8")

    map_path, summary = import_model_file(
        model_path,
        output_dir=tmp_path / "maps",
        map_name="table_top_photo",
        voxel_size=0.5,
        scale=2.0,
        padding=1.0,
        max_obstacles=100,
    )

    payload = json.loads(map_path.read_text(encoding="utf-8"))
    assert map_path == tmp_path / "maps" / "table_top_photo.json"
    assert summary["map"] == "table_top_photo"
    assert summary["vertex_count"] == 4
    assert summary["triangle_count"] == 2
    assert summary["obstacle_count"] == len(payload["obstacles"])
    assert payload["bounds"][1][0] >= 3.0
    assert payload["bounds"][1][1] >= 3.0
    assert all(obs["type"] == "aabb" for obs in payload["obstacles"])
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_photo_reconstruction_import.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tools.photo_reconstruction_import'`.

- [ ] **Step 3: Implement the direct import tool**

Create `tools/photo_reconstruction_import.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from web.server import _model_to_map, _parse_model, _safe_new_map_name


def import_model_file(
    model_path: str | Path,
    *,
    output_dir: str | Path = "maps",
    map_name: str = "imported_model",
    voxel_size: float = 0.5,
    scale: float = 1.0,
    padding: float = 1.0,
    max_obstacles: int = 4000,
) -> tuple[Path, dict[str, Any]]:
    source = Path(model_path)
    if not source.exists():
        raise FileNotFoundError(f"model file does not exist: {source}")
    data = source.read_bytes()
    vertices, triangles = _parse_model(source.name, data)
    safe_name = _safe_new_map_name(map_name)
    map_json = _model_to_map(
        vertices,
        triangles,
        voxel_size=voxel_size,
        scale=scale,
        padding=max(0.0, padding),
        max_obstacles=max(1, max_obstacles),
        source_name=source.name,
    )
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    map_path = out_dir / f"{safe_name}.json"
    map_path.write_text(json.dumps(map_json, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {
        "ok": True,
        "map": safe_name,
        "path": str(map_path),
        "vertex_count": len(vertices),
        "triangle_count": len(triangles),
        "obstacle_count": len(map_json["obstacles"]),
        "bounds": map_json["bounds"],
        "voxel_size": voxel_size,
        "scale": scale,
        "padding": max(0.0, padding),
    }
    return map_path, summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Import a reconstruction model into maps/<name>.json.")
    parser.add_argument("model", help="Path to .ply, .obj, or .stl")
    parser.add_argument("--output-dir", default="maps")
    parser.add_argument("--map-name", default="imported_model")
    parser.add_argument("--voxel-size", type=float, default=0.5)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--padding", type=float, default=1.0)
    parser.add_argument("--max-obstacles", type=int, default=4000)
    parser.add_argument("--summary-json", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    map_path, summary = import_model_file(
        args.model,
        output_dir=args.output_dir,
        map_name=args.map_name,
        voxel_size=args.voxel_size,
        scale=args.scale,
        padding=args.padding,
        max_obstacles=args.max_obstacles,
    )
    print(f"imported map: {map_path}")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if args.summary_json:
        Path(args.summary_json).write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```powershell
python -m pytest tests/test_photo_reconstruction_import.py -q
```

Expected: `1 passed`.

- [ ] **Step 5: Commit**

```powershell
git add tools/photo_reconstruction_import.py tests/test_photo_reconstruction_import.py
git commit -m "feat: add direct photo reconstruction import tool"
```

---

### Task 3: Imported Map Simulation Smoke Runner

**Files:**
- Create: `tools/run_imported_map_smoke.py`
- Create: `tests/test_imported_map_smoke.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_imported_map_smoke.py`:

```python
from __future__ import annotations

import json
from pathlib import Path

from tools.run_imported_map_smoke import build_imported_map_config, load_waypoints


def test_load_waypoints_reads_json_arrays(tmp_path: Path):
    path = tmp_path / "waypoints.json"
    path.write_text(json.dumps([[0, 0, 1.5], [3, 2, 1.5]]), encoding="utf-8")

    waypoints = load_waypoints(path)

    assert len(waypoints) == 2
    assert waypoints[0].tolist() == [0.0, 0.0, 1.5]
    assert waypoints[1].tolist() == [3.0, 2.0, 1.5]


def test_build_imported_map_config_overrides_map_and_waypoints(tmp_path: Path):
    map_path = tmp_path / "meeting_room_photo.json"
    map_path.write_text(
        json.dumps({"bounds": [[0, 0, 0], [5, 5, 3]], "obstacles": []}),
        encoding="utf-8",
    )
    waypoint_path = tmp_path / "waypoints.json"
    waypoint_path.write_text(json.dumps([[0.5, 0.5, 1.5], [4.0, 4.0, 1.5]]), encoding="utf-8")

    cfg = build_imported_map_config(
        map_path=map_path,
        waypoints_path=waypoint_path,
        base_preset="custom",
        planner_mode="offline",
        max_sim_time=12.0,
    )

    assert cfg.enable_obstacles is True
    assert cfg.map_file == str(map_path)
    assert cfg.planner_mode == "offline"
    assert cfg.max_sim_time == 12.0
    assert [wp.tolist() for wp in cfg.waypoints] == [[0.5, 0.5, 1.5], [4.0, 4.0, 1.5]]
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```powershell
python -m pytest tests/test_imported_map_smoke.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tools.run_imported_map_smoke'`.

- [ ] **Step 3: Implement the smoke runner**

Create `tools/run_imported_map_smoke.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from config import get_config
from main import run_with_config
from simulations.formation_simulation import SimulationConfig


def load_waypoints(path: str | Path) -> list[np.ndarray]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise ValueError("waypoints file must contain a JSON array")
    waypoints: list[np.ndarray] = []
    for idx, item in enumerate(raw):
        if not isinstance(item, list) or len(item) != 3:
            raise ValueError(f"waypoint {idx} must be [x, y, z]")
        waypoints.append(np.array([float(item[0]), float(item[1]), float(item[2])], dtype=float))
    return waypoints


def build_imported_map_config(
    *,
    map_path: str | Path,
    waypoints_path: str | Path,
    base_preset: str = "custom",
    planner_mode: str = "offline",
    max_sim_time: float = 45.0,
) -> SimulationConfig:
    cfg = get_config(base_preset)
    cfg.enable_obstacles = True
    cfg.map_file = str(Path(map_path))
    cfg.waypoints = load_waypoints(waypoints_path)
    cfg.planner_mode = planner_mode
    cfg.max_sim_time = float(max_sim_time)
    cfg.sensor_enabled = planner_mode == "online"
    cfg.formation_safety_enabled = True
    cfg.formation_min_inter_drone_distance = max(
        float(getattr(cfg, "formation_min_inter_drone_distance", 0.35)),
        0.35,
    )
    return cfg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a smoke simulation on an imported map.")
    parser.add_argument("--map", required=True, help="Path to imported maps/<name>.json")
    parser.add_argument("--waypoints", required=True, help="JSON file containing [[x,y,z], ...]")
    parser.add_argument("--base-preset", default="custom")
    parser.add_argument("--planner-mode", choices=["offline", "online"], default="offline")
    parser.add_argument("--max-sim-time", type=float, default=45.0)
    parser.add_argument("--output-dir", default="outputs/competition_demo")
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--preset-name", default="imported_map_smoke")
    parser.add_argument("--no-plot", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = build_imported_map_config(
        map_path=args.map,
        waypoints_path=args.waypoints,
        base_preset=args.base_preset,
        planner_mode=args.planner_mode,
        max_sim_time=args.max_sim_time,
    )
    run_with_config(
        cfg,
        output_dir=args.output_dir,
        plot=not args.no_plot,
        preset=args.preset_name,
        run_name=args.run_name,
        generate_report=True,
        report_title=f"导入地图仿真报告：{Path(args.map).stem}",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run tests to verify they pass**

Run:

```powershell
python -m pytest tests/test_imported_map_smoke.py -q
```

Expected: `2 passed`.

- [ ] **Step 5: Document the smoke command inside the tool help**

Run:

```powershell
python tools/run_imported_map_smoke.py --help
```

Expected: output contains `--map`, `--waypoints`, `--planner-mode`, and `--output-dir`.

- [ ] **Step 6: Commit**

```powershell
git add tools/run_imported_map_smoke.py tests/test_imported_map_smoke.py
git commit -m "feat: add imported map simulation smoke runner"
```

---

### Task 4: Competition Demo Evidence Index

**Files:**
- Create: `experiments/competition_demo.py`
- Create: `tests/test_competition_demo.py`
- Create: `docs/competition_demo_manifest_example.json`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_competition_demo.py`:

```python
from __future__ import annotations

import json
from pathlib import Path

from experiments.competition_demo import write_demo_index


def test_write_demo_index_links_scene_assets(tmp_path: Path):
    manifest = {
        "title": "空域智构参赛演示",
        "summary": "同一组照片同时生成 3DGS 展示和 UAV 安全仿真证据。",
        "scenes": [
            {
                "name": "meeting_room_photo",
                "photo_count": 96,
                "visual_asset": "media/meeting_room_photo.mp4",
                "map": "maps/meeting_room_photo.json",
                "map_audit": "outputs/competition_demo/meeting_room_photo_map_audit.md",
                "sim_result": "outputs/competition_demo/imported_map_smoke/meeting_room/sim_result.json",
                "report": "outputs/competition_demo/imported_map_smoke/meeting_room/report.md",
                "status": "pass",
                "notes": "scale=1.08, voxel_size=0.35, waypoints=5/5",
            }
        ],
    }
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")

    output = write_demo_index(manifest_path, tmp_path / "index.md")

    text = output.read_text(encoding="utf-8")
    assert "# 空域智构参赛演示" in text
    assert "同一组照片同时生成" in text
    assert "| meeting_room_photo | 96 | pass |" in text
    assert "[3DGS/Video](media/meeting_room_photo.mp4)" in text
    assert "[Map](maps/meeting_room_photo.json)" in text
    assert "视觉展示保留真实重建结果，规划验证使用体素化障碍地图" in text
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_competition_demo.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'experiments.competition_demo'`.

- [ ] **Step 3: Implement the demo index builder**

Create `experiments/competition_demo.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _link(label: str, path: str | None) -> str:
    if not path:
        return ""
    return f"[{label}]({path})"


def render_demo_index(manifest: dict[str, Any]) -> str:
    title = str(manifest.get("title") or "Competition Demo")
    summary = str(manifest.get("summary") or "")
    lines = [
        f"# {title}",
        "",
        summary,
        "",
        "> 视觉展示保留真实重建结果，规划验证使用体素化障碍地图；导入地图不直接替代规划器，也不伪造任务航点。",
        "",
        "| Scene | Photos | Status | Visual | Map | Audit | Sim Result | Report | Notes |",
        "|---|---:|---|---|---|---|---|---|---|",
    ]
    for scene in manifest.get("scenes") or []:
        lines.append(
            "| {name} | {photos} | {status} | {visual} | {map_link} | {audit} | {sim_result} | {report} | {notes} |".format(
                name=scene.get("name", ""),
                photos=scene.get("photo_count", ""),
                status=scene.get("status", ""),
                visual=_link("3DGS/Video", scene.get("visual_asset")),
                map_link=_link("Map", scene.get("map")),
                audit=_link("Audit", scene.get("map_audit")),
                sim_result=_link("sim_result.json", scene.get("sim_result")),
                report=_link("report.md", scene.get("report")),
                notes=scene.get("notes", ""),
            )
        )
    lines.extend(
        [
            "",
            "## Demo Boundary",
            "",
            "- 3DGS/mesh assets are used for visual inspection and explanation.",
            "- UAV planning consumes `bounds + obstacles` JSON maps.",
            "- Complex imported geometry is represented as conservative voxelized AABB obstacles.",
            "- ROS2 is treated as an interface target; this demo does not require ROS2 runtime.",
            "",
        ]
    )
    return "\n".join(lines)


def write_demo_index(manifest_path: str | Path, output_path: str | Path) -> Path:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_demo_index(manifest), encoding="utf-8")
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a competition demo Markdown index.")
    parser.add_argument("manifest", help="Path to competition demo manifest JSON")
    parser.add_argument("--output", default="outputs/competition_demo/index.md")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output = write_demo_index(args.manifest, args.output)
    print(f"competition demo index: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Add an example manifest**

Create `docs/competition_demo_manifest_example.json`:

```json
{
  "title": "空域智构参赛演示",
  "summary": "同一组室内照片双链路产出：3DGS/mesh 用于视觉展示，体素化 JSON 地图用于无人机编队仿真与风险报告。",
  "scenes": [
    {
      "name": "meeting_room_photo",
      "photo_count": 96,
      "visual_asset": "media/meeting_room_photo.mp4",
      "map": "maps/meeting_room_photo.json",
      "map_audit": "outputs/competition_demo/meeting_room_photo_map_audit.md",
      "sim_result": "outputs/competition_demo/imported_map_smoke/meeting_room/sim_result.json",
      "report": "outputs/competition_demo/imported_map_smoke/meeting_room/report.md",
      "status": "pass",
      "notes": "scale=1.08, voxel_size=0.35, waypoints=5/5"
    }
  ]
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run:

```powershell
python -m pytest tests/test_competition_demo.py -q
```

Expected: `1 passed`.

- [ ] **Step 6: Commit**

```powershell
git add experiments/competition_demo.py tests/test_competition_demo.py docs/competition_demo_manifest_example.json
git commit -m "feat: add competition demo evidence index"
```

---

### Task 5: ROS2-Ready Bridge Manifest Without ROS2 Dependency

**Files:**
- Create: `tools/export_ros2_bridge_manifest.py`
- Create: `tests/test_ros2_bridge_manifest.py`
- Create: `docs/ros2_bridge_contract.md`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_ros2_bridge_manifest.py`:

```python
from __future__ import annotations

import json
from pathlib import Path

from tools.export_ros2_bridge_manifest import build_bridge_manifest, write_bridge_manifest


def test_build_bridge_manifest_describes_topics_without_ros2_dependency(tmp_path: Path):
    sim_result = tmp_path / "sim_result.json"
    sim_result.write_text(
        json.dumps(
            {
                "preset": "meeting_room_photo",
                "runtime_engine": "python",
                "task_waypoints": [[0, 0, 1.5], [3, 2, 1.5]],
                "metrics": {"mean_error_overall": 0.12},
                "safety_metrics": {"min_inter_drone_distance": 0.41, "downwash_hits": 0},
            }
        ),
        encoding="utf-8",
    )

    manifest = build_bridge_manifest(sim_result)

    assert manifest["source"] == str(sim_result)
    assert manifest["ros2_required_to_generate"] is False
    assert manifest["frames"] == ["map", "world", "uav_0/base_link"]
    assert "/planned_path" in [topic["name"] for topic in manifest["topics"]]
    assert "/risk_events" in [topic["name"] for topic in manifest["topics"]]
    assert manifest["available_fields"]["task_waypoints"] is True
    assert manifest["available_fields"]["safety_metrics"] is True


def test_write_bridge_manifest_writes_json(tmp_path: Path):
    sim_result = tmp_path / "sim_result.json"
    sim_result.write_text(json.dumps({"preset": "demo"}), encoding="utf-8")

    out = write_bridge_manifest(sim_result, tmp_path / "ros2_bridge_manifest.json")

    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["preset"] == "demo"
    assert payload["bridge_scope"] == "offline_export_contract"
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```powershell
python -m pytest tests/test_ros2_bridge_manifest.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tools.export_ros2_bridge_manifest'`.

- [ ] **Step 3: Implement the bridge manifest exporter**

Create `tools/export_ros2_bridge_manifest.py`:

```python
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


TOPICS = [
    {"name": "/uav_0/odom", "type": "nav_msgs/Odometry", "source": "leader trajectory"},
    {"name": "/uav_i/odom", "type": "nav_msgs/Odometry", "source": "follower trajectories"},
    {"name": "/planned_path", "type": "nav_msgs/Path", "source": "planned_path or task_waypoints"},
    {"name": "/executed_path", "type": "nav_msgs/Path", "source": "executed_path"},
    {"name": "/map_obstacles", "type": "visualization_msgs/MarkerArray", "source": "bounds + obstacles map"},
    {"name": "/risk_events", "type": "diagnostic_msgs/DiagnosticArray", "source": "collision, clearance, safety metrics"},
]


def build_bridge_manifest(sim_result_path: str | Path) -> dict[str, Any]:
    source = Path(sim_result_path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    return {
        "bridge_scope": "offline_export_contract",
        "ros2_required_to_generate": False,
        "source": str(source),
        "preset": payload.get("preset"),
        "runtime_engine": payload.get("runtime_engine"),
        "frames": ["map", "world", "uav_0/base_link"],
        "topics": TOPICS,
        "available_fields": {
            "task_waypoints": "task_waypoints" in payload,
            "planned_path": "planned_path" in payload,
            "executed_path": "executed_path" in payload,
            "safety_metrics": "safety_metrics" in payload,
            "risk_report": "risk_report" in payload,
        },
        "boundary": [
            "This file is an interface contract for a future ROS2 bridge.",
            "It does not publish ROS2 topics.",
            "It does not imply real UAV closed-loop deployment.",
        ],
    }


def write_bridge_manifest(sim_result_path: str | Path, output_path: str | Path) -> Path:
    manifest = build_bridge_manifest(sim_result_path)
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export a ROS2-ready bridge manifest from sim_result.json.")
    parser.add_argument("sim_result", help="Path to sim_result.json")
    parser.add_argument("--output", default="outputs/ros2_bridge_manifest.json")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output = write_bridge_manifest(args.sim_result, args.output)
    print(f"ros2 bridge manifest: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Add the ROS2 contract doc**

Create `docs/ros2_bridge_contract.md`:

```markdown
# ROS2 Bridge Contract

This project does not require ROS2 for the competition demo. The current stable runtime remains Python/C++/Web.

## Purpose

The bridge contract records how existing `sim_result.json` evidence can be mapped to ROS2 concepts when a later deployment phase needs RViz, rosbag2, Gazebo, PX4, or real sensor integration.

## Frames

| Frame | Meaning |
|---|---|
| `map` | Imported `bounds + obstacles` map coordinate frame |
| `world` | Simulation world frame, aligned to `map` for the current project |
| `uav_0/base_link` | Leader body frame |
| `uav_i/base_link` | Follower body frames, one per follower |

## Topics

| Topic | ROS2 Type | Source |
|---|---|---|
| `/uav_0/odom` | `nav_msgs/Odometry` | leader trajectory |
| `/uav_i/odom` | `nav_msgs/Odometry` | follower trajectories |
| `/planned_path` | `nav_msgs/Path` | `planned_path` or task waypoints |
| `/executed_path` | `nav_msgs/Path` | executed leader path |
| `/map_obstacles` | `visualization_msgs/MarkerArray` | imported map obstacles |
| `/risk_events` | `diagnostic_msgs/DiagnosticArray` | collision, clearance, and safety events |

## Boundary

- This contract is not a ROS2 node.
- This contract does not prove real UAV safety.
- Full ROS2 migration remains a later deployment task.
- The competition demo should use this contract only to show that the data model is ready for future integration.
```

- [ ] **Step 5: Run tests to verify they pass**

Run:

```powershell
python -m pytest tests/test_ros2_bridge_manifest.py -q
```

Expected: `2 passed`.

- [ ] **Step 6: Commit**

```powershell
git add tools/export_ros2_bridge_manifest.py tests/test_ros2_bridge_manifest.py docs/ros2_bridge_contract.md
git commit -m "feat: add ros2-ready bridge manifest export"
```

---

### Task 6: Documentation And End-To-End Competition Commands

**Files:**
- Modify: `docs/photo_reconstruction_integration.md`
- Modify: `README.md`
- Create: `tests/test_competition_docs_static.py`

- [ ] **Step 1: Write the failing documentation guard test**

Create `tests/test_competition_docs_static.py`:

```python
from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_competition_docs_state_imported_map_boundaries():
    photo_doc = (ROOT / "docs" / "photo_reconstruction_integration.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    combined = photo_doc + "\n" + readme
    assert "tools/photo_reconstruction_import.py" in combined
    assert "experiments.map_quality" in combined
    assert "tools/run_imported_map_smoke.py" in combined
    assert "3DGS" in combined
    assert "体素化" in combined
    assert "不要手改规划器安全半径" in combined
    assert "ROS2" in combined
    assert "不需要 ROS2" in combined
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_competition_docs_static.py -q
```

Expected: FAIL because the new command names and ROS2 boundary are not documented yet.

- [ ] **Step 3: Add the competition command block to `docs/photo_reconstruction_integration.md`**

Append this section near the existing `/api/maps/import-model` guidance:

```markdown
## 9. Competition Demo Command Chain

For competition demos, keep the stable chain offline-friendly:

```powershell
python tools/photo_reconstruction_import.py `
  experiments/photogrammetry/reconstruction/meeting_room_photo/openmvs/scene_dense_mesh.ply `
  --map-name meeting_room_photo `
  --output-dir maps `
  --voxel-size 0.35 `
  --scale 1.0 `
  --padding 1.0 `
  --max-obstacles 8000

python -m experiments.map_quality `
  maps/meeting_room_photo.json `
  --output-dir outputs/competition_demo/meeting_room_photo `
  --safety-margin 0.3

python tools/run_imported_map_smoke.py `
  --map maps/meeting_room_photo.json `
  --waypoints docs/competition_waypoints/meeting_room_photo.json `
  --planner-mode offline `
  --output-dir outputs/competition_demo `
  --run-name meeting_room_photo
```

3DGS or mesh assets are visual evidence. The UAV planner consumes the voxelized `bounds + obstacles` map. If the imported map scale is wrong, fix `scale` and rerun the audit; do not hand-edit planner safety radii to hide a scale error.

ROS2 is not required for the competition demo. Use `tools/export_ros2_bridge_manifest.py` only to describe the future bridge contract.
```

- [ ] **Step 4: Add the short command block to `README.md`**

Add this subsection under the existing mid-term experiment/reporting area:

```markdown
### Competition imported-map demo

The competition demo path keeps real reconstruction and UAV planning separated:

```text
3DGS / mesh visual asset
  -> voxelized bounds + obstacles map
  -> map quality audit
  -> imported-map smoke simulation
  -> report.md / sim_result.json / realtime error curve
```

Useful commands:

```powershell
python tools/photo_reconstruction_import.py <model.ply> --map-name meeting_room_photo --output-dir maps --voxel-size 0.35 --scale 1.0
python -m experiments.map_quality maps/meeting_room_photo.json --output-dir outputs/competition_demo/meeting_room_photo
python tools/run_imported_map_smoke.py --map maps/meeting_room_photo.json --waypoints docs/competition_waypoints/meeting_room_photo.json --output-dir outputs/competition_demo
python tools/export_ros2_bridge_manifest.py outputs/competition_demo/imported_map_smoke/meeting_room_photo/sim_result.json --output outputs/competition_demo/ros2_bridge_manifest.json
```

For competition delivery, ROS2 is a later bridge target, not a required runtime. The planner should not consume raw 3DGS geometry directly; imported shapes are converted to conservative voxelized AABB obstacles.
```

- [ ] **Step 5: Run the documentation guard**

Run:

```powershell
python -m pytest tests/test_competition_docs_static.py -q
```

Expected: `1 passed`.

- [ ] **Step 6: Run the focused competition-tool tests**

Run:

```powershell
python -m pytest `
  tests/test_map_quality.py `
  tests/test_photo_reconstruction_import.py `
  tests/test_imported_map_smoke.py `
  tests/test_competition_demo.py `
  tests/test_ros2_bridge_manifest.py `
  tests/test_competition_docs_static.py `
  -q
```

Expected: all listed tests pass.

- [ ] **Step 7: Commit**

```powershell
git add README.md docs/photo_reconstruction_integration.md tests/test_competition_docs_static.py
git commit -m "docs: document competition imported-map workflow"
```

---

## End-To-End Manual Validation

After all tasks pass, run the chain on one real or sample model:

```powershell
python tools/photo_reconstruction_import.py `
  experiments/photogrammetry/reconstruction/meeting_room_photo/openmvs/scene_dense_mesh.ply `
  --map-name meeting_room_photo `
  --output-dir maps `
  --voxel-size 0.35 `
  --scale 1.0 `
  --padding 1.0 `
  --max-obstacles 8000
```

Expected:

```text
imported map: maps\meeting_room_photo.json
```

Run the audit:

```powershell
python -m experiments.map_quality maps/meeting_room_photo.json --output-dir outputs/competition_demo/meeting_room_photo --safety-margin 0.3
```

Expected:

```text
map audit json: outputs\competition_demo\meeting_room_photo\meeting_room_photo_map_audit.json
map audit markdown: outputs\competition_demo\meeting_room_photo\meeting_room_photo_map_audit.md
```

Run smoke simulation:

```powershell
python tools/run_imported_map_smoke.py `
  --map maps/meeting_room_photo.json `
  --waypoints docs/competition_waypoints/meeting_room_photo.json `
  --planner-mode offline `
  --output-dir outputs/competition_demo `
  --run-name meeting_room_photo
```

Expected outputs:

- `outputs/competition_demo/imported_map_smoke/meeting_room_photo/sim_result.json`
- `outputs/competition_demo/imported_map_smoke/meeting_room_photo/report.md`
- `outputs/competition_demo/imported_map_smoke/meeting_room_photo/report_figures/*.png`

Build the demo index:

```powershell
python -m experiments.competition_demo docs/competition_demo_manifest_example.json --output outputs/competition_demo/index.md
```

Expected:

```text
competition demo index: outputs\competition_demo\index.md
```

Build the ROS2 bridge contract:

```powershell
python tools/export_ros2_bridge_manifest.py `
  outputs/competition_demo/imported_map_smoke/meeting_room_photo/sim_result.json `
  --output outputs/competition_demo/ros2_bridge_manifest.json
```

Expected:

```text
ros2 bridge manifest: outputs\competition_demo\ros2_bridge_manifest.json
```

---

## Self-Review

**Spec coverage**
- Imported map handling is covered by Task 1 and Task 2.
- Other geometric shapes are handled by the existing model-to-voxelized-AABB conversion and explicitly kept out of planner shape expansion.
- Competition demo evidence is covered by Task 3 and Task 4.
- ROS2 is handled as a bridge contract in Task 5, matching the decision not to migrate before competition.
- Documentation and repeatable commands are covered by Task 6.

**Placeholder scan**
- The plan contains no `TBD`, no `TODO`, and no unspecified implementation steps.
- Every code task includes exact paths, test code, implementation code, commands, expected outcomes, and commit commands.

**Type consistency**
- `audit_map()`, `write_audit_files()`, `import_model_file()`, `build_imported_map_config()`, `write_demo_index()`, and `write_bridge_manifest()` are introduced before later tasks refer to them.
- JSON outputs use stable dict/list primitives only.
- ROS2 bridge export does not import ROS2 packages and is consistently described as an offline contract.
