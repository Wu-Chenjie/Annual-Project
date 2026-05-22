# A1/A2 Trajectory MPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete A-line slice 1 by adding FIRI-aware trajectory metrics/acceptance and offline MPC feasibility evidence, with Python and C++ result fields synchronized.

**Architecture:** Python remains the behavioral source of truth: `TrajectoryOptimizer` gains curvature and corridor-gate behavior, then `mpc_tracker.py` evaluates planned trajectories offline. C++ mirrors public metrics/result schema with lightweight utilities and static synchronization tests so cross-line claims stay honest.

**Tech Stack:** Python 3, NumPy, pytest, C++20, CMake, existing `SimulationConfig`/`SimulationResult` JSON writers.

---

## File Structure

- Modify: `core/planning/trajectory_optimizer.py`
  - Add curvature metrics and optional corridor gate.
- Modify: `core/planning/firi.py`
  - Add public corridor builder used by simulation and tests.
- Create: `core/planning/mpc_tracker.py`
  - Offline feasibility evaluator with serializable result.
- Modify: `core/planning/__init__.py`
  - Export MPC evaluator/result.
- Modify: `simulations/formation_simulation.py`
  - Add config switches and limits for MPC feasibility evaluation.
- Modify: `simulations/obstacle_scenario.py`
  - Pass FIRI corridors into trajectory optimization and emit `mpc_feasibility`.
- Modify: `experiments/metrics_extractor.py`
  - Extract curvature and MPC feasibility metrics.
- Modify: `tests/test_trajectory_optimizer.py`
  - Add curvature and corridor-gate tests.
- Create: `tests/test_mpc_tracker.py`
  - Add feasible and infeasible evaluator tests.
- Modify: `tests/test_metrics_extractor.py`
  - Add curvature and MPC metric extraction expectations.
- Modify: `tests/test_cpp_sync_static.py`
  - Add C++ synchronization assertions.
- Create: `cpp/include/trajectory_metrics.hpp`
- Create: `cpp/src/trajectory_metrics.cpp`
- Create: `cpp/include/mpc_tracker.hpp`
- Create: `cpp/src/mpc_tracker.cpp`
- Modify: `cpp/include/formation_simulation.hpp`
  - Add result/config fields if not already centralized there.
- Modify: `cpp/include/obstacle_scenario.hpp`
  - Add obstacle config switches if obstacle-only config owns them.
- Modify: `cpp/src/obstacle_scenario.cpp`
  - Populate C++ planned trajectory metrics and MPC feasibility result.
- Modify: `cpp/include/result_writer.hpp`
  - Emit synchronized JSON keys.
- Modify: `cpp/CMakeLists.txt`
  - Add new C++ source files to `CORE_SOURCES`.

---

### Task 1: Python Trajectory Curvature Metrics

**Files:**
- Modify: `tests/test_trajectory_optimizer.py`
- Modify: `core/planning/trajectory_optimizer.py`
- Modify: `experiments/metrics_extractor.py`
- Modify: `tests/test_metrics_extractor.py`

- [ ] **Step 1: Write failing trajectory metric test**

Add to `tests/test_trajectory_optimizer.py`:

```python
def test_trajectory_optimizer_reports_curvature_metrics():
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.2, smoothing_window=3)
    path = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [2.0, 1.0, 1.0],
    ], dtype=float)

    result = optimizer.optimize(path, method="none")
    payload = result.to_dict()

    assert result.mean_curvature >= 0.0
    assert result.max_curvature >= result.mean_curvature
    assert result.curvature_squared_integral >= 0.0
    assert payload["mean_curvature"] == result.mean_curvature
    assert payload["max_curvature"] == result.max_curvature
    assert payload["curvature_squared_integral"] == result.curvature_squared_integral
```

- [ ] **Step 2: Verify it fails**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py::test_trajectory_optimizer_reports_curvature_metrics -q
```

Expected: `AttributeError` or missing key for `mean_curvature`.

- [ ] **Step 3: Implement minimal curvature metrics**

In `TrajectoryResult`, add:

```python
mean_curvature: float
max_curvature: float
curvature_squared_integral: float
```

In `to_dict()`, add:

```python
"mean_curvature": self.mean_curvature,
"max_curvature": self.max_curvature,
"curvature_squared_integral": self.curvature_squared_integral,
```

Add helper:

```python
@staticmethod
def _curvature_metrics(positions: np.ndarray, timestamps: np.ndarray) -> tuple[float, float, float]:
    if len(positions) < 3:
        return 0.0, 0.0, 0.0
    curvatures: list[float] = []
    integral = 0.0
    for i in range(1, len(positions) - 1):
        a = positions[i] - positions[i - 1]
        b = positions[i + 1] - positions[i]
        la = float(np.linalg.norm(a))
        lb = float(np.linalg.norm(b))
        chord = float(np.linalg.norm(positions[i + 1] - positions[i - 1]))
        if la <= 1e-9 or lb <= 1e-9 or chord <= 1e-9:
            curvature = 0.0
        else:
            cross_norm = float(np.linalg.norm(np.cross(a, b)))
            curvature = 2.0 * cross_norm / max(la * lb * chord, 1e-9)
        curvatures.append(curvature)
        dt = max(float(timestamps[min(i + 1, len(timestamps) - 1)] - timestamps[i]), 1e-6)
        integral += curvature * curvature * dt
    if not curvatures:
        return 0.0, 0.0, 0.0
    return float(np.mean(curvatures)), float(np.max(curvatures)), float(integral)
```

Call it in `optimize()` after timestamps are built and pass values into every `TrajectoryResult` constructor. Empty result constructors should use `0.0` for all three fields.

- [ ] **Step 4: Verify pass**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py::test_trajectory_optimizer_reports_curvature_metrics -q
```

Expected: `1 passed`.

- [ ] **Step 5: Add metrics extractor coverage**

In `tests/test_metrics_extractor.py`, extend the `planned_trajectory` payload in `test_extract_metrics_from_standard_sim_result`:

```python
"mean_curvature": 0.11,
"max_curvature": 0.22,
"curvature_squared_integral": 0.33,
```

Add assertions:

```python
assert metrics["trajectory_mean_curvature"] == 0.11
assert metrics["trajectory_max_curvature"] == 0.22
assert metrics["trajectory_curvature_squared_integral"] == 0.33
```

- [ ] **Step 6: Verify extractor test fails**

Run:

```powershell
python -m pytest tests/test_metrics_extractor.py::test_extract_metrics_from_standard_sim_result -q
```

Expected: failure for missing `trajectory_mean_curvature`.

- [ ] **Step 7: Implement extractor fields**

In `experiments/metrics_extractor.py`, beside existing planned trajectory metric extraction, add:

```python
metrics["trajectory_mean_curvature"] = _first_float(planned_trajectory.get("mean_curvature"))
metrics["trajectory_max_curvature"] = _first_float(planned_trajectory.get("max_curvature"))
metrics["trajectory_curvature_squared_integral"] = _first_float(
    planned_trajectory.get("curvature_squared_integral")
)
```

Also add the same keys in `_derive_trajectory_metrics()` using the discrete helper or zero-safe defaults if no curvature can be derived.

- [ ] **Step 8: Run focused tests**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py tests/test_metrics_extractor.py -q
```

Expected: all tests pass.

---

### Task 2: Python FIRI Corridor Gate

**Files:**
- Modify: `tests/test_trajectory_optimizer.py`
- Modify: `core/planning/firi.py`
- Modify: `core/planning/trajectory_optimizer.py`
- Modify: `simulations/obstacle_scenario.py`

- [ ] **Step 1: Write failing corridor acceptance test**

Add to `tests/test_trajectory_optimizer.py`:

```python
def test_trajectory_optimizer_accepts_candidate_inside_corridor():
    from core.planning.firi import FIRICorridor

    corridor = FIRICorridor(
        A=np.array([[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]], dtype=float),
        b=np.array([1.0, 1.0], dtype=float),
        start=np.array([0.0, 0.0, 1.0], dtype=float),
        goal=np.array([2.0, 0.0, 1.0], dtype=float),
        min_clearance=0.0,
    )
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.2, smoothing_window=3)
    path = np.array([[0.0, 0.0, 1.0], [1.0, 0.5, 1.0], [2.0, 0.0, 1.0]], dtype=float)

    result = optimizer.optimize(path, corridors=[corridor], method="moving_average")

    assert result.accepted is True
    assert result.fallback_reason is None
```

- [ ] **Step 2: Write failing corridor rejection test**

Add:

```python
def test_trajectory_optimizer_falls_back_when_corridor_gate_rejects_candidate():
    from core.planning.firi import FIRICorridor

    corridor = FIRICorridor(
        A=np.array([[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]], dtype=float),
        b=np.array([0.05, 0.05], dtype=float),
        start=np.array([0.0, 0.0, 1.0], dtype=float),
        goal=np.array([2.0, 0.0, 1.0], dtype=float),
        min_clearance=0.0,
    )
    optimizer = TrajectoryOptimizer(nominal_speed=1.0, sample_dt=0.2, smoothing_window=5)
    path = np.array([[0.0, 0.0, 1.0], [1.0, 0.4, 1.0], [2.0, 0.0, 1.0]], dtype=float)

    result = optimizer.optimize(path, corridors=[corridor], method="moving_average", fallback_to_raw=True)

    assert result.accepted is False
    assert result.fallback_reason == "optimized_path_failed_corridor_gate"
    assert np.allclose(result.positions[0], path[0])
    assert np.allclose(result.positions[-1], path[-1])
```

- [ ] **Step 3: Verify tests fail**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py::test_trajectory_optimizer_accepts_candidate_inside_corridor tests/test_trajectory_optimizer.py::test_trajectory_optimizer_falls_back_when_corridor_gate_rejects_candidate -q
```

Expected: `TypeError` for unexpected keyword `corridors`.

- [ ] **Step 4: Expose FIRI corridor builder**

In `core/planning/firi.py`, add:

```python
def build_corridors_for_path(self, seeds: np.ndarray) -> list[FIRICorridor]:
    seeds = np.asarray(seeds, dtype=float)
    if len(seeds) < 2:
        return []
    safe_seeds = self._prepare_seeds(seeds)
    return self._build_corridors(safe_seeds)
```

- [ ] **Step 5: Implement corridor gate in optimizer**

Add `corridors=None` to `optimize()` and `_select_by_smoothness_cost()`. Implement:

```python
def _passes_corridor_gate(self, positions: np.ndarray, corridors) -> bool:
    if not corridors:
        return True
    for point in positions:
        if not any(corridor.contains(point, tol=1e-6) for corridor in corridors):
            return False
    return True
```

After candidate optimization and before clearance gate:

```python
if corridors and len(optimized) > 1 and not self._passes_corridor_gate(optimized, corridors):
    if fallback_to_raw:
        optimized = resampled
        accepted = False
        fallback_reason = "optimized_path_failed_corridor_gate"
    else:
        raise ValueError("optimized path failed corridor gate")
```

Use `fallback_reason` only if it is not already set by the corridor gate before running the clearance gate.

- [ ] **Step 6: Verify focused tests pass**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py -q
```

Expected: all trajectory optimizer tests pass.

- [ ] **Step 7: Wire corridors into obstacle scenario**

In `simulations/obstacle_scenario.py`, find the `self.trajectory_optimizer.optimize(` call inside the planned trajectory block and compute:

```python
corridors = None
if getattr(self.config, "firi_enabled", False) and hasattr(self, "firi_refiner"):
    corridors = self.firi_refiner.build_corridors_for_path(path)
```

Pass `corridors=corridors` into the optimizer call.

- [ ] **Step 8: Run scenario integration focus**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py tests/test_obstacle_scenario.py -q -k "firi or planned_trajectory"
```

Expected: selected tests pass.

---

### Task 3: Python MPC Feasibility Evaluator

**Files:**
- Create: `tests/test_mpc_tracker.py`
- Create: `core/planning/mpc_tracker.py`
- Modify: `core/planning/__init__.py`

- [ ] **Step 1: Write failing feasible test**

Create `tests/test_mpc_tracker.py`:

```python
from __future__ import annotations

import numpy as np

from core.planning.mpc_tracker import MPCFeasibilityEvaluator


def test_mpc_feasibility_evaluator_accepts_within_limits():
    evaluator = MPCFeasibilityEvaluator(max_speed=2.0, max_acceleration=3.0)
    positions = np.array([[0.0, 0.0, 1.0], [0.5, 0.0, 1.0], [1.0, 0.0, 1.0]], dtype=float)
    timestamps = np.array([0.0, 0.5, 1.0], dtype=float)

    result = evaluator.evaluate_arrays(positions=positions, timestamps=timestamps)

    assert result.evaluated is True
    assert result.feasible is True
    assert result.max_velocity_violation == 0.0
    assert result.max_acceleration_violation == 0.0
    assert result.saturation_ratio == 0.0
    assert result.recommendation == "continue_mpc_prototype"
```

- [ ] **Step 2: Write failing infeasible test**

Add:

```python
def test_mpc_feasibility_evaluator_flags_velocity_and_acceleration_violations():
    evaluator = MPCFeasibilityEvaluator(max_speed=1.0, max_acceleration=1.0)
    positions = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0], [3.0, 3.0, 1.0]], dtype=float)
    timestamps = np.array([0.0, 0.5, 1.0], dtype=float)

    result = evaluator.evaluate_arrays(positions=positions, timestamps=timestamps)

    assert result.evaluated is True
    assert result.feasible is False
    assert result.max_velocity_violation > 0.0
    assert result.max_acceleration_violation > 0.0
    assert result.saturation_ratio > 0.0
    assert result.recommendation == "defer_online_mpc"
    assert result.to_dict()["feasible"] is False
```

- [ ] **Step 3: Verify tests fail**

Run:

```powershell
python -m pytest tests/test_mpc_tracker.py -q
```

Expected: import failure for `core.planning.mpc_tracker`.

- [ ] **Step 4: Implement evaluator**

Create `core/planning/mpc_tracker.py`:

```python
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class MPCFeasibilityResult:
    evaluated: bool
    feasible: bool
    tracking_rms_proxy: float
    max_velocity_violation: float
    max_acceleration_violation: float
    saturation_ratio: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


class MPCFeasibilityEvaluator:
    def __init__(self, *, max_speed: float, max_acceleration: float, rms_limit: float = 0.75):
        self.max_speed = max(float(max_speed), 1e-6)
        self.max_acceleration = max(float(max_acceleration), 1e-6)
        self.rms_limit = max(float(rms_limit), 0.0)

    def evaluate_trajectory(self, trajectory) -> MPCFeasibilityResult:
        return self.evaluate_arrays(
            positions=np.asarray(trajectory.positions, dtype=float),
            timestamps=np.asarray(trajectory.timestamps, dtype=float),
        )

    def evaluate_arrays(self, *, positions: np.ndarray, timestamps: np.ndarray) -> MPCFeasibilityResult:
        positions = np.asarray(positions, dtype=float)
        timestamps = np.asarray(timestamps, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 2:
            return MPCFeasibilityResult(False, False, 0.0, 0.0, 0.0, 0.0, "missing_trajectory")
        velocities, accelerations = self._differentiate(positions, timestamps)
        speed_norms = np.linalg.norm(velocities, axis=1)
        acc_norms = np.linalg.norm(accelerations, axis=1)
        velocity_violation = float(max(0.0, np.max(speed_norms) - self.max_speed))
        acceleration_violation = float(max(0.0, np.max(acc_norms) - self.max_acceleration))
        tracking_rms_proxy = float(np.sqrt(np.mean(np.minimum(acc_norms / self.max_acceleration, 2.0) ** 2)))
        saturated = (speed_norms > self.max_speed) | (acc_norms > self.max_acceleration)
        saturation_ratio = float(np.mean(saturated))
        feasible = velocity_violation == 0.0 and acceleration_violation == 0.0 and tracking_rms_proxy <= self.rms_limit
        return MPCFeasibilityResult(
            evaluated=True,
            feasible=bool(feasible),
            tracking_rms_proxy=tracking_rms_proxy,
            max_velocity_violation=velocity_violation,
            max_acceleration_violation=acceleration_violation,
            saturation_ratio=saturation_ratio,
            recommendation="continue_mpc_prototype" if feasible else "defer_online_mpc",
        )

    @staticmethod
    def _differentiate(positions: np.ndarray, timestamps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if len(timestamps) != len(positions):
            timestamps = np.arange(len(positions), dtype=float)
        velocities = np.zeros_like(positions)
        accelerations = np.zeros_like(positions)
        for i in range(1, len(positions)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            velocities[i] = (positions[i] - positions[i - 1]) / dt
        velocities[0] = velocities[1]
        for i in range(1, len(positions)):
            dt = max(float(timestamps[i] - timestamps[i - 1]), 1e-6)
            accelerations[i] = (velocities[i] - velocities[i - 1]) / dt
        accelerations[0] = accelerations[1]
        return velocities, accelerations
```

- [ ] **Step 5: Export evaluator**

In `core/planning/__init__.py`, import and add to `__all__`:

```python
from .mpc_tracker import MPCFeasibilityEvaluator, MPCFeasibilityResult
```

```python
"MPCFeasibilityEvaluator",
"MPCFeasibilityResult",
```

- [ ] **Step 6: Run focused tests**

Run:

```powershell
python -m pytest tests/test_mpc_tracker.py -q
```

Expected: all tests pass.

---

### Task 4: Python Simulation And Metrics Integration

**Files:**
- Modify: `tests/test_trajectory_optimizer.py`
- Modify: `tests/test_metrics_extractor.py`
- Modify: `simulations/formation_simulation.py`
- Modify: `simulations/obstacle_scenario.py`
- Modify: `experiments/metrics_extractor.py`

- [ ] **Step 1: Extend scenario integration test**

In `test_obstacle_scenario_emits_planned_trajectory_when_enabled`, add:

```python
mpc_feasibility = result.get("mpc_feasibility")
assert mpc_feasibility is not None
assert mpc_feasibility["evaluated"] is True
assert "tracking_rms_proxy" in mpc_feasibility
assert "recommendation" in mpc_feasibility
```

- [ ] **Step 2: Verify scenario test fails**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py::test_obstacle_scenario_emits_planned_trajectory_when_enabled -q
```

Expected: assertion failure because `mpc_feasibility` is missing.

- [ ] **Step 3: Add config fields**

In `SimulationConfig` in `simulations/formation_simulation.py`, add near trajectory optimizer fields:

```python
mpc_feasibility_enabled: bool = True
mpc_feasibility_rms_limit: float = 0.75
```

- [ ] **Step 4: Emit MPC feasibility**

In `simulations/obstacle_scenario.py`, import:

```python
from core.planning.mpc_tracker import MPCFeasibilityEvaluator
```

After planned trajectory is computed, set:

```python
self.mpc_feasibility = None
if self.planned_trajectory is not None and getattr(self.config, "mpc_feasibility_enabled", True):
    evaluator = MPCFeasibilityEvaluator(
        max_speed=getattr(self.config, "leader_max_vel", 4.0),
        max_acceleration=getattr(self.config, "leader_max_acc", 5.0),
        rms_limit=getattr(self.config, "mpc_feasibility_rms_limit", 0.75),
    )
    self.mpc_feasibility = evaluator.evaluate_trajectory(self.planned_trajectory)
```

In the returned result dict, add:

```python
"mpc_feasibility": None if self.mpc_feasibility is None else self.mpc_feasibility.to_dict(),
```

- [ ] **Step 5: Add metric extraction test**

In `tests/test_metrics_extractor.py`, extend payload:

```python
"mpc_feasibility": {
    "evaluated": True,
    "feasible": False,
    "tracking_rms_proxy": 0.8,
    "max_velocity_violation": 0.2,
    "max_acceleration_violation": 0.1,
    "saturation_ratio": 0.25,
    "recommendation": "defer_online_mpc",
},
```

Add assertions:

```python
assert metrics["mpc_feasible"] == 0
assert metrics["mpc_tracking_rms_proxy"] == 0.8
assert metrics["mpc_saturation_ratio"] == 0.25
assert metrics["mpc_recommendation"] == "defer_online_mpc"
```

- [ ] **Step 6: Implement extractor fields**

In `experiments/metrics_extractor.py`, add:

```python
mpc = sim_result.get("mpc_feasibility") or {}
metrics["mpc_feasible"] = int(bool(mpc.get("feasible"))) if mpc.get("evaluated") is not None else 0
metrics["mpc_tracking_rms_proxy"] = _first_float(mpc.get("tracking_rms_proxy"))
metrics["mpc_max_velocity_violation"] = _first_float(mpc.get("max_velocity_violation"))
metrics["mpc_max_acceleration_violation"] = _first_float(mpc.get("max_acceleration_violation"))
metrics["mpc_saturation_ratio"] = _first_float(mpc.get("saturation_ratio"))
metrics["mpc_recommendation"] = str(mpc.get("recommendation", "not_evaluated"))
```

- [ ] **Step 7: Run focused Python integration**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py tests/test_mpc_tracker.py tests/test_metrics_extractor.py -q
```

Expected: all tests pass.

---

### Task 5: C++ Synchronization Tests

**Files:**
- Modify: `tests/test_cpp_sync_static.py`

- [ ] **Step 1: Add failing static sync test**

Add to `tests/test_cpp_sync_static.py`:

```python
def test_cpp_a1_a2_trajectory_mpc_sync_fields_exist():
    cmake = read("cpp/CMakeLists.txt")
    scenario_header = read("cpp/include/obstacle_scenario.hpp")
    scenario_source = read("cpp/src/obstacle_scenario.cpp")
    formation_header = read("cpp/include/formation_simulation.hpp")
    result_writer = read("cpp/include/result_writer.hpp")

    assert "src/trajectory_metrics.cpp" in cmake
    assert "src/mpc_tracker.cpp" in cmake
    assert "trajectory_metrics.hpp" in scenario_source
    assert "mpc_tracker.hpp" in scenario_source
    assert "struct TrajectoryMetrics" in read("cpp/include/trajectory_metrics.hpp")
    assert "struct MPCFeasibilityResult" in read("cpp/include/mpc_tracker.hpp")
    assert "mpc_feasibility_enabled" in scenario_header or "mpc_feasibility_enabled" in formation_header
    assert "planned_trajectory" in result_writer
    assert "mpc_feasibility" in result_writer
    assert "mean_curvature" in result_writer
    assert "curvature_squared_integral" in result_writer
    assert "tracking_rms_proxy" in result_writer
    assert "recommendation" in result_writer
```

- [ ] **Step 2: Verify it fails**

Run:

```powershell
python -m pytest tests/test_cpp_sync_static.py::test_cpp_a1_a2_trajectory_mpc_sync_fields_exist -q
```

Expected: failure because new files/strings do not exist.

---

### Task 6: C++ Metrics, MPC Result, And JSON Writer

**Files:**
- Create: `cpp/include/trajectory_metrics.hpp`
- Create: `cpp/src/trajectory_metrics.cpp`
- Create: `cpp/include/mpc_tracker.hpp`
- Create: `cpp/src/mpc_tracker.cpp`
- Modify: `cpp/include/formation_simulation.hpp`
- Modify: `cpp/include/obstacle_scenario.hpp`
- Modify: `cpp/src/obstacle_scenario.cpp`
- Modify: `cpp/include/result_writer.hpp`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: Add C++ trajectory metrics header**

Create `cpp/include/trajectory_metrics.hpp`:

```cpp
#pragma once

#include <vector>

#include "math_utils.hpp"

namespace sim {

struct TrajectoryMetrics {
    bool available = false;
    double path_length = 0.0;
    double mean_curvature = 0.0;
    double max_curvature = 0.0;
    double curvature_squared_integral = 0.0;
    double mean_jerk = 0.0;
    double max_jerk = 0.0;
    double jerk_squared_integral = 0.0;
    double snap_squared_integral = 0.0;
};

TrajectoryMetrics compute_trajectory_metrics(const std::vector<Vec3>& path, double sample_dt);

}  // namespace sim
```

- [ ] **Step 2: Add C++ trajectory metrics source**

Create `cpp/src/trajectory_metrics.cpp` with deterministic discrete path metrics:

```cpp
#include "trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

TrajectoryMetrics compute_trajectory_metrics(const std::vector<Vec3>& path, double sample_dt) {
    TrajectoryMetrics out;
    if (path.empty()) return out;
    out.available = true;
    const double dt = std::max(sample_dt, 1e-6);
    for (std::size_t i = 1; i < path.size(); ++i) {
        out.path_length += norm(path[i] - path[i - 1]);
    }
    if (path.size() >= 3) {
        double curvature_sum = 0.0;
        int curvature_count = 0;
        for (std::size_t i = 1; i + 1 < path.size(); ++i) {
            Vec3 a = path[i] - path[i - 1];
            Vec3 b = path[i + 1] - path[i];
            double la = norm(a);
            double lb = norm(b);
            double chord = norm(path[i + 1] - path[i - 1]);
            double curvature = 0.0;
            if (la > 1e-9 && lb > 1e-9 && chord > 1e-9) {
                curvature = 2.0 * norm(cross(a, b)) / std::max(la * lb * chord, 1e-9);
            }
            curvature_sum += curvature;
            out.max_curvature = std::max(out.max_curvature, curvature);
            out.curvature_squared_integral += curvature * curvature * dt;
            ++curvature_count;
        }
        out.mean_curvature = curvature_count > 0 ? curvature_sum / static_cast<double>(curvature_count) : 0.0;
    }
    return out;
}

}  // namespace sim
```

- [ ] **Step 3: Add C++ MPC header/source**

Create `cpp/include/mpc_tracker.hpp`:

```cpp
#pragma once

#include <string>
#include <vector>

#include "math_utils.hpp"

namespace sim {

struct MPCFeasibilityResult {
    bool evaluated = false;
    bool feasible = false;
    double tracking_rms_proxy = 0.0;
    double max_velocity_violation = 0.0;
    double max_acceleration_violation = 0.0;
    double saturation_ratio = 0.0;
    std::string recommendation = "missing_trajectory";
};

MPCFeasibilityResult evaluate_mpc_feasibility(const std::vector<Vec3>& path,
                                              double sample_dt,
                                              double max_speed,
                                              double max_acceleration,
                                              double rms_limit);

}  // namespace sim
```

Create `cpp/src/mpc_tracker.cpp` using finite differences over the path and the same recommendations as Python.

- [ ] **Step 4: Add result/config fields**

In `SimulationResult` in `cpp/include/formation_simulation.hpp`, add:

```cpp
#include "trajectory_metrics.hpp"
#include "mpc_tracker.hpp"
```

and fields:

```cpp
TrajectoryMetrics planned_trajectory;
MPCFeasibilityResult mpc_feasibility;
```

In `ObstacleConfig` in `cpp/include/obstacle_scenario.hpp`, add:

```cpp
bool trajectory_optimizer_enabled = true;
std::string trajectory_optimizer_method = "cpp_firi_metrics";
double trajectory_optimizer_sample_dt = 0.2;
bool mpc_feasibility_enabled = true;
double mpc_feasibility_rms_limit = 0.75;
```

- [ ] **Step 5: Populate C++ result**

In `cpp/src/obstacle_scenario.cpp`, include:

```cpp
#include "trajectory_metrics.hpp"
#include "mpc_tracker.hpp"
```

Before returning `result`, after `result.planned_path = planned_path_;`, add:

```cpp
if (!planned_path_.empty() && config_.trajectory_optimizer_enabled) {
    result.planned_trajectory = compute_trajectory_metrics(
        planned_path_,
        config_.trajectory_optimizer_sample_dt
    );
}
if (!planned_path_.empty() && config_.mpc_feasibility_enabled) {
    result.mpc_feasibility = evaluate_mpc_feasibility(
        planned_path_,
        config_.trajectory_optimizer_sample_dt,
        config_.leader_max_vel,
        config_.leader_max_acc,
        config_.mpc_feasibility_rms_limit
    );
}
```

- [ ] **Step 6: Emit JSON fields**

In `cpp/include/result_writer.hpp`, emit:

```cpp
w.key("planned_trajectory").begin_object();
w.key("path_length").value(result.planned_trajectory.path_length);
w.key("mean_curvature").value(result.planned_trajectory.mean_curvature);
w.key("max_curvature").value(result.planned_trajectory.max_curvature);
w.key("curvature_squared_integral").value(result.planned_trajectory.curvature_squared_integral);
w.key("mean_jerk").value(result.planned_trajectory.mean_jerk);
w.key("max_jerk").value(result.planned_trajectory.max_jerk);
w.key("jerk_squared_integral").value(result.planned_trajectory.jerk_squared_integral);
w.key("snap_squared_integral").value(result.planned_trajectory.snap_squared_integral);
w.end_object();

w.key("mpc_feasibility").begin_object();
w.key("evaluated").value(result.mpc_feasibility.evaluated);
w.key("feasible").value(result.mpc_feasibility.feasible);
w.key("tracking_rms_proxy").value(result.mpc_feasibility.tracking_rms_proxy);
w.key("max_velocity_violation").value(result.mpc_feasibility.max_velocity_violation);
w.key("max_acceleration_violation").value(result.mpc_feasibility.max_acceleration_violation);
w.key("saturation_ratio").value(result.mpc_feasibility.saturation_ratio);
w.key("recommendation").value(result.mpc_feasibility.recommendation);
w.end_object();
```

- [ ] **Step 7: Update CMake**

In `cpp/CMakeLists.txt`, add to `CORE_SOURCES`:

```cmake
    src/trajectory_metrics.cpp
    src/mpc_tracker.cpp
```

- [ ] **Step 8: Run C++ sync static test**

Run:

```powershell
python -m pytest tests/test_cpp_sync_static.py::test_cpp_a1_a2_trajectory_mpc_sync_fields_exist -q
```

Expected: test passes.

---

### Task 7: Verification And A-Line Regression

**Files:**
- No new implementation files unless verification exposes failures.

- [ ] **Step 1: Run focused Python tests**

Run:

```powershell
python -m pytest tests/test_trajectory_optimizer.py tests/test_mpc_tracker.py tests/test_metrics_extractor.py -q
```

Expected: all pass.

- [ ] **Step 2: Run A-line FIRI/obstacle regression**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py -q -k "firi or obstacle_simulation_zero_collision or hybrid_astar_obstacle"
```

Expected: selected tests pass.

- [ ] **Step 3: Run C++ sync checks**

Run:

```powershell
python -m pytest tests/test_cpp_sync_static.py -q
```

Expected: all C++ static sync checks pass.

- [ ] **Step 4: Run broader non-slow suite**

Run:

```powershell
python -m pytest -m "not slow" -q
```

Expected: all non-slow tests pass, or record exact unrelated pre-existing failures.

- [ ] **Step 5: Try C++ build**

Run:

```powershell
cmake --build cpp/build --config Release
```

Expected: build passes if `cpp/build` is configured. If not configured, record the exact CMake error and run the static sync tests as the minimum C++ gate.

- [ ] **Step 6: Inspect git diff**

Run:

```powershell
git diff --stat
git status --short
```

Expected: changes are limited to A1/A2 implementation, tests, and the approved plan/spec docs; the pre-existing staged `docs/后续改进待办清单.md` remains untouched.

---

## Self-Review Notes

- Spec coverage: A1 curvature/smoothness evidence is covered by Tasks 1-2; A2 offline MPC evidence is covered by Tasks 3-4; C++ synchronization is covered by Tasks 5-6; verification is covered by Task 7.
- Completion-marker scan: no unresolved markers or optional file path markers remain.
- Type consistency: Python result field names match C++ JSON keys: `planned_trajectory`, `mean_curvature`, `max_curvature`, `curvature_squared_integral`, and `mpc_feasibility` fields.
