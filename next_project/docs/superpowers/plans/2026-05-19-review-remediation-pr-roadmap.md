# Review Remediation PR Roadmap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the repository review findings into small, independently reviewable PRs that improve performance, safety, correctness, and maintainability without changing simulation behavior unless explicitly tested.

**Architecture:** Keep each PR focused on one subsystem boundary. Start with the already implemented sensor-window optimization, then land low-risk safety/correctness fixes before larger deduplication and configuration-source work. Each PR must include regression tests and a short A/B or invariant check when it touches simulation behavior.

**Tech Stack:** Python, pytest, C++/CMake/MSVC, FastAPI-style Web service code, repository docs.

---

## PR Sequence

1. **PR 1: Sensor Window Performance** - the focused sensor-window optimization already specified and validated in this session.
2. **PR 2: Web Safety Guardrails** - local-only default, upload/request limits, simulate concurrency guard.
3. **PR 3: Planning Bounds And Drone Arm Length Correctness** - explicit map-bound checks and remove hard-coded arm length.
4. **PR 4: Profiling Metrics For Planning/Clearance** - add wall-time and query counters before deeper optimization.
5. **PR 5: Model Importer Deduplication** - Web calls `core.model_importer` instead of duplicating parsers.
6. **PR 6: Preset Single Source Preparation** - introduce machine-readable preset metadata and generate/serve lists from it.
7. **PR 7: Test And Packaging Hygiene** - pytest markers, schema packaging, stricter schema fallback behavior.

---

## PR 1: Sensor Window Performance

**Goal:** Resolve high time complexity when sensors are enabled while preserving physical ray-intersection behavior.

**Files:**
- Modify: `next_project/core/obstacles.py`
- Modify: `next_project/core/sensors.py`
- Modify: `next_project/core/planning/replanner.py`
- Modify: `next_project/simulations/obstacle_scenario.py`
- Modify: `next_project/cpp/include/obstacles.hpp`
- Modify: `next_project/cpp/include/sensor.hpp`
- Modify: `next_project/cpp/include/replanner.hpp`
- Modify: `next_project/cpp/src/obstacle_scenario.cpp`
- Test: `next_project/tests/test_sensor_window_candidates.py`
- Test: `next_project/tests/test_obstacle_scenario.py`
- Test: `next_project/tests/test_cpp_sync_static.py`

- [x] **Step 1: Use one window for sensing and planning**

Online obstacle scenarios construct `RangeSensor6` with `planner_horizon`, not `sensor_max_range`, so sensing and `WindowReplanner` use the same spatial window.

- [x] **Step 2: Add obstacle broad-phase query**

Add `obstacles_in_ray_window(origin, direction, max_range)` to Python and C++ `ObstacleField`. It returns only obstacles whose bounding boxes overlap the finite sensor segment window.

- [x] **Step 3: Keep exact geometry as the source of truth**

`RangeSensor6` still calls exact AABB/sphere/cylinder ray intersection after broad-phase filtering. The broad phase never decides hit distance.

- [x] **Step 4: Avoid repeated unknown-map obstacle-field rebuilds**

Track dirty sensor-grid changes and rebuild discovered obstacles only when the grid actually changed.

- [x] **Step 5: Verify no behavioral drift**

Run:

```powershell
pytest next_project/tests/test_sensor_window_candidates.py next_project/tests/test_replanner_semantics.py next_project/tests/test_cpp_sync_static.py -q
pytest next_project/tests/test_obstacle_scenario.py::test_obstacle_simulation_zero_collision next_project/tests/test_obstacle_scenario.py::test_obstacle_simulation_dijkstra next_project/tests/test_obstacle_scenario.py::test_rrt_star_obstacle next_project/tests/test_obstacle_scenario.py::test_hybrid_astar_obstacle next_project/tests/test_obstacle_scenario.py::test_regression_baseline -q
cmake --build next_project/cpp/build_pr_check --config Release
```

Expected:
- pytest passes.
- C++ build exits 0.
- A/B metric spot checks against `HEAD` keep collision count and error metrics unchanged.

---

## PR 2: Web Safety Guardrails

**Goal:** Make local Web tooling safer by default and harder to abuse accidentally.

**Files:**
- Modify: `next_project/web/server.py`
- Modify: `next_project/README.md` or Web usage docs if present
- Test: `next_project/tests/test_web_server_safety.py`

- [x] **Step 1: Add failing tests for safety defaults**

Create `next_project/tests/test_web_server_safety.py` with tests that import Web configuration helpers and assert:

```python
def test_web_defaults_to_loopback_host():
    from web.server import DEFAULT_HOST
    assert DEFAULT_HOST == "127.0.0.1"

def test_upload_size_limit_is_defined():
    from web.server import MAX_UPLOAD_BYTES
    assert MAX_UPLOAD_BYTES <= 50 * 1024 * 1024

def test_simulate_concurrency_limit_is_defined():
    from web.server import SIMULATE_CONCURRENCY_LIMIT
    assert SIMULATE_CONCURRENCY_LIMIT == 1
```

Run:

```powershell
pytest next_project/tests/test_web_server_safety.py -q
```

Expected before implementation: import or assertion failure.

- [x] **Step 2: Add explicit Web constants**

In `next_project/web/server.py`, define:

```python
DEFAULT_HOST = "127.0.0.1"
MAX_UPLOAD_BYTES = 50 * 1024 * 1024
SIMULATE_CONCURRENCY_LIMIT = 1
```

Use `DEFAULT_HOST` in the CLI/server startup path instead of `0.0.0.0`.

- [x] **Step 3: Enforce upload size before parsing**

In upload endpoints, reject requests whose file bytes exceed `MAX_UPLOAD_BYTES` before model parsing.

Expected behavior:
- Oversized uploads return HTTP 413 or a structured JSON error.
- Existing small model tests still pass.

- [x] **Step 4: Guard `/api/simulate` concurrency**

Add a process-local semaphore around simulation execution:

```python
_simulate_semaphore = threading.Semaphore(SIMULATE_CONCURRENCY_LIMIT)
```

If the semaphore cannot be acquired immediately, return HTTP 429 or an equivalent JSON error.

- [x] **Step 5: Document safe deployment boundary**

Update the Web docs to say the server is intended for local use by default and should not be exposed publicly without auth and rate limiting.

- [x] **Step 6: Verify PR 2**

Run:

```powershell
pytest next_project/tests/test_web_server_safety.py -q
pytest next_project/tests/test_result_schema.py next_project/tests/test_cpp_result_reporting.py -q
```

Expected: all selected tests pass.

---

## PR 3: Planning Bounds And Drone Arm Length Correctness

**Goal:** Prevent silent planning target clipping and ensure collision margin uses the configured drone profile.

**Files:**
- Modify: `next_project/core/obstacles.py`
- Modify: `next_project/core/planning/astar.py`
- Modify: other planners that call `world_to_index(start/goal)` directly
- Modify: `next_project/simulations/obstacle_scenario.py`
- Test: `next_project/tests/test_planner_bounds.py`
- Test: `next_project/tests/test_obstacle_scenario.py`

- [x] **Step 1: Add `contains_world` tests**

Create `next_project/tests/test_planner_bounds.py`:

```python
import numpy as np
import pytest

from core.obstacles import ObstacleField
from core.planning import AStar, PlannerError


def test_occupancy_grid_contains_world_rejects_outside_points():
    grid = ObstacleField().to_voxel_grid(np.array([[0, 0, 0], [2, 2, 2]], dtype=float), 1.0)
    assert grid.contains_world(np.array([0.5, 0.5, 0.5]))
    assert not grid.contains_world(np.array([-0.1, 0.5, 0.5]))
    assert not grid.contains_world(np.array([2.1, 0.5, 0.5]))


def test_astar_rejects_out_of_bounds_start_or_goal():
    grid = ObstacleField().to_voxel_grid(np.array([[0, 0, 0], [3, 3, 3]], dtype=float), 1.0)
    planner = AStar()
    with pytest.raises(PlannerError):
        planner.plan(np.array([-1.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0]), grid)
    with pytest.raises(PlannerError):
        planner.plan(np.array([1.0, 1.0, 1.0]), np.array([9.0, 2.0, 2.0]), grid)
```

- [x] **Step 2: Implement `OccupancyGrid.contains_world`**

Add:

```python
def contains_world(self, p: np.ndarray) -> bool:
    p = np.asarray(p, dtype=float)
    idx_float = (p - self.origin) / self.resolution
    return bool(np.all(idx_float >= 0.0) and np.all(idx_float < np.asarray(self.shape, dtype=float)))
```

- [x] **Step 3: Reject out-of-bounds planner endpoints**

In planners that convert endpoints to grid indices, check `contains_world(start)` and `contains_world(goal)` before clipping. Raise `PlannerError` with a message that names the offending endpoint.

- [x] **Step 4: Add drone arm-length regression**

In `next_project/tests/test_obstacle_scenario.py`, add a test that sets a non-default drone profile or monkeypatches loaded drone params and asserts `_collision_margin` reflects the configured arm length.

- [x] **Step 5: Replace hard-coded arm length**

In `ObstacleScenarioSimulation.__init__`, replace:

```python
arm_length = 0.2
```

with:

```python
arm_length = float(getattr(self.drone_params, "arm_length", 0.2))
```

- [x] **Step 6: Verify PR 3**

Run:

```powershell
pytest next_project/tests/test_planner_bounds.py next_project/tests/test_obstacle_scenario.py::test_obstacle_simulation_zero_collision -q
```

Expected: tests pass and previous zero-collision scenario remains unchanged.

---

## PR 4: Profiling Metrics For Planning/Clearance

**Goal:** Add measurement before deeper performance rewrites, without changing planning decisions.

**Files:**
- Modify: `next_project/simulations/obstacle_scenario.py`
- Modify: `next_project/experiments/metrics_extractor.py`
- Modify: result schema if required
- Test: `next_project/tests/test_metrics_extractor.py`
- Test: `next_project/tests/test_obstacle_scenario.py`

- [ ] **Step 1: Add tests for new metrics**

Extend metrics tests to expect numeric fields:

```python
def test_metrics_extractor_includes_planning_perf_fields():
    result = {
        "planning_events": [{"wall_time_ms": 12.5, "planner": "astar", "path_points": 8}],
        "metrics": {"collision_count": 0},
        "completed_waypoint_count": 1,
    }
    metrics = extract_metrics(result)
    assert metrics["planning_wall_time_ms_total"] == 12.5
    assert metrics["planning_event_count"] == 1
```

- [ ] **Step 2: Record planner wall time**

Around existing online/offline planning calls, record:

```python
start = time.perf_counter()
...
wall_time_ms = (time.perf_counter() - start) * 1000.0
```

Append planner type, phase, path point count, accepted/rejected status.

- [ ] **Step 3: Record clearance/SDF query counters**

Add simple counters on `ObstacleScenarioSimulation`:

```python
self._sdf_query_count = 0
self._clearance_check_count = 0
```

Increment them inside `_planning_signed_distance()` and `_path_segment_clearance()`.

- [ ] **Step 4: Include counters in results**

Add fields to result payload:

```python
"performance_counters": {
    "sdf_query_count": self._sdf_query_count,
    "clearance_check_count": self._clearance_check_count,
}
```

- [ ] **Step 5: Verify PR 4**

Run:

```powershell
pytest next_project/tests/test_metrics_extractor.py next_project/tests/test_obstacle_scenario.py::test_online_mode_preserves_task_waypoint_semantics -q
```

Expected: tests pass and no path/physics assertions change.

---

## PR 5: Model Importer Deduplication

**Goal:** Make Web model import use the core importer instead of maintaining duplicate OBJ/STL/PLY parsing logic.

**Files:**
- Modify: `next_project/core/model_importer.py`
- Modify: `next_project/web/server.py`
- Test: existing model importer/Web tests or create `next_project/tests/test_model_importer_web_reuse.py`

- [ ] **Step 1: Add a core importer bytes API test**

Test:

```python
def test_parse_model_bytes_obj_minimal_triangle():
    payload = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    mesh = parse_model_bytes(payload, filename="tri.obj")
    assert len(mesh.vertices) == 3
    assert len(mesh.faces) == 1
```

- [ ] **Step 2: Add core API wrappers**

Expose functions from `core/model_importer.py`:

```python
def parse_model_bytes(data: bytes, filename: str) -> ImportedMesh: ...
def model_to_map_json(mesh: ImportedMesh, *, max_obstacles: int, resolution: float) -> dict: ...
```

- [ ] **Step 3: Replace Web duplicate parser calls**

In `web/server.py`, delete or deprecate private `_parse_obj`, `_parse_stl`, `_parse_ply`, and `_model_to_map` call sites. Use `core.model_importer` wrappers instead.

- [ ] **Step 4: Verify PR 5**

Run:

```powershell
pytest next_project/tests/test_model_importer_web_reuse.py next_project/tests/test_cpp_result_reporting.py -q
```

Expected: importer tests pass and Web-facing payload shape is unchanged.

---

## PR 6: Preset Single Source Preparation

**Goal:** Start reducing preset drift without replacing every preset in one PR.

**Files:**
- Create: `next_project/config/preset_metadata.json` or `next_project/config/preset_metadata.yaml`
- Modify: `next_project/config.py`
- Modify: `next_project/web/server.py`
- Modify: docs that list presets
- Test: `next_project/tests/test_preset_metadata.py`

- [ ] **Step 1: Add metadata consistency test**

Test that all available presets appear in metadata and Web list:

```python
from config import AVAILABLE_PRESETS
from web.server import PRESETS


def test_web_preset_list_covers_config_presets():
    missing = set(AVAILABLE_PRESETS) - set(PRESETS)
    assert not missing
```

- [ ] **Step 2: Add preset metadata file**

Create a metadata file with fields:

```json
{
  "warehouse_online": {
    "label": "仓库在线",
    "mode": "online",
    "description": "仓库场景在线重规划"
  }
}
```

Include every preset currently listed by `AVAILABLE_PRESETS`.

- [ ] **Step 3: Load Web preset list from metadata**

Replace Web-local manual preset labels with metadata loading. Keep existing API response shape stable.

- [ ] **Step 4: Verify PR 6**

Run:

```powershell
pytest next_project/tests/test_preset_metadata.py next_project/tests/test_experiment_workflow.py::test_scenario_registry_returns_quick_config -q
```

Expected: no preset drift between CLI and Web metadata.

---

## PR 7: Test And Packaging Hygiene

**Goal:** Make routine checks faster and make schema validation failures less silent.

**Files:**
- Modify: `pyproject.toml`
- Modify: `next_project/core/result_schema.py`
- Create or modify: `pytest.ini`
- Test: `next_project/tests/test_result_schema.py`

- [ ] **Step 1: Add pytest markers**

Add marker declarations:

```ini
[pytest]
markers =
    unit: fast unit tests
    integration: simulation integration tests
    slow: long-running tests
    cpp: C++ build or executable tests
    static_sync: source-level synchronization checks
```

- [ ] **Step 2: Mark slow and C++ tests**

Add decorators to long simulation and C++ tests:

```python
import pytest

@pytest.mark.integration
@pytest.mark.slow
def test_obstacle_simulation_zero_collision():
    ...
```

- [ ] **Step 3: Package schemas**

Update `pyproject.toml` package-data settings so schema JSON files are included in built packages.

- [ ] **Step 4: Tighten schema fallback**

In `result_schema.py`, only fall back to lightweight validation on `ImportError` for optional dependencies. Missing schema files or invalid schema should raise a clear exception.

- [ ] **Step 5: Verify PR 7**

Run:

```powershell
pytest next_project/tests/test_result_schema.py -q
pytest -m "not slow" next_project/tests -q
```

Expected: schema tests pass and marker selection runs without unknown-marker warnings.

---

## Execution Rule

Each PR should start from a clean branch based on the previous merged PR:

```powershell
git status --short
git checkout main
git pull
git checkout -b pr-N-short-name
```

Do not mix PRs. If a later task needs an earlier helper, merge or rebase on the earlier PR first.

## Review Checklist For Every PR

- [ ] Scope is one topic only.
- [ ] New or changed behavior has tests.
- [ ] Collision/error-sensitive changes include an invariant or A/B check.
- [ ] `git diff --check` passes.
- [ ] Relevant pytest command passes.
- [ ] C++ build runs when C++ files change.
- [ ] PR body includes verification output and known unrelated failures.
