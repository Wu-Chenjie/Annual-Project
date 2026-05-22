# A1/A2 Trajectory And MPC Feasibility Design

## Scope

This spec covers the approved A-line first slice:

- A1: FIRI-aware continuous trajectory quality improvement.
- A2: offline MPC tracking feasibility evaluation.
- Python/C++ synchronization for public result fields and evidence metrics.

It does not implement A3 Voronoi partitioning, A5 dynamic-obstacle decay, or A6 topology-energy metrics.

## Baseline

The Python side already has:

- `core/planning/firi.py` with `FIRICorridor` and `FIRIRefiner`.
- `core/planning/trajectory_optimizer.py` with time-parameterized samples, moving-average/minimum-jerk candidates, jerk metrics, and snap proxy metrics.
- `simulations/obstacle_scenario.py` wiring for `planned_trajectory`.
- `experiments/metrics_extractor.py` extraction for planned trajectory metrics.

The C++ side already has:

- `cpp/include/firi.hpp` and `cpp/src/firi.cpp` with FIRI corridor projection and path refinement.
- `cpp/src/obstacle_scenario.cpp` wiring FIRI into initial planning and online replanning.
- `cpp/include/result_writer.hpp` outputting `planned_path`, config snapshots, metrics, and safety fields.
- `tests/test_cpp_sync_static.py` checking several cross-line implementation contracts.

## Design Goals

1. Make A1 measurable: optimized trajectories must report curvature and existing smoothness metrics.
2. Make A1 safe: FIRI-aware optimization must respect corridor containment or fall back to current resampled/refined paths.
3. Make A2 evidence-based: evaluate MPC feasibility offline without replacing Backstepping + SMC.
4. Keep Python and C++ result schemas aligned enough for README/docs/capability-matrix claims.
5. Preserve current online planning stability and collision red lines.

## Python Design

### FIRI-Aware Trajectory Optimization

Extend `TrajectoryOptimizer.optimize()` with an optional `corridors` parameter:

```python
def optimize(
    self,
    path: np.ndarray,
    *,
    clearance_checker=None,
    corridors: list[FIRICorridor] | None = None,
    method: str = "moving_average",
    fallback_to_raw: bool = True,
) -> TrajectoryResult:
    # Existing behavior plus optional corridor gate.
```

Corridor handling:

- A path sample is assigned to the corridor whose segment has the closest projection along the input seed path.
- Candidate trajectories are accepted only if every sample is contained by its assigned corridor.
- If the candidate fails the corridor gate and `fallback_to_raw=True`, use the resampled path and mark `accepted=False` with a specific fallback reason.
- If there are no corridors, behavior remains backward-compatible.

Add curvature metrics to `TrajectoryResult`:

- `mean_curvature`
- `max_curvature`
- `curvature_squared_integral`

Use a discrete three-point curvature proxy over positions. Degenerate short segments produce zero curvature rather than errors.

### FIRI Integration

Add a corridor-returning API to `FIRIRefiner`:

```python
def build_corridors_for_path(self, seeds: np.ndarray) -> list[FIRICorridor]:
    # Returns the same segment corridors used by refine().
```

`ObstacleScenarioSimulation` should pass these corridors into `TrajectoryOptimizer` when FIRI is enabled and trajectory optimization is enabled. The existing path projection/refinement remains the fallback path.

### MPC Feasibility Evaluation

Create `core/planning/mpc_tracker.py` with a small offline evaluator:

```python
@dataclass(frozen=True)
class MPCFeasibilityResult:
    evaluated: bool
    feasible: bool
    tracking_rms_proxy: float
    max_velocity_violation: float
    max_acceleration_violation: float
    saturation_ratio: float
    recommendation: str
```

The evaluator consumes `TrajectoryResult` or trajectory arrays and compares velocity/acceleration demand against configured limits. It does not run online MPC and does not change `Controller`, `BacksteppingController`, or `HybridAttitudeController`.

`ObstacleScenarioSimulation.run()` should include an `mpc_feasibility` result field when planned trajectory data exists.

## C++ Design

### Trajectory Metrics

Add a small C++ trajectory metrics utility:

- `cpp/include/trajectory_metrics.hpp`
- `cpp/src/trajectory_metrics.cpp`

It should compute from `std::vector<Vec3>`:

- `path_length`
- `mean_curvature`
- `max_curvature`
- `curvature_squared_integral`
- jerk/snap proxy fields if enough timing information exists, otherwise deterministic zero or path-step proxy values.

The C++ implementation does not need a full minimum-snap optimizer in this slice. It must provide synchronized evidence fields for FIRI-refined paths.

### MPC Feasibility

Add a C++ offline feasibility helper:

- `cpp/include/mpc_tracker.hpp`
- `cpp/src/mpc_tracker.cpp`

It mirrors the Python output shape:

- `evaluated`
- `feasible`
- `tracking_rms_proxy`
- `max_velocity_violation`
- `max_acceleration_violation`
- `saturation_ratio`
- `recommendation`

Like Python, it must not replace the active controller.

### Result Synchronization

Extend C++ `SimulationResult` and `result_writer.hpp` to emit:

- `planned_trajectory` with synchronized metric keys when a planned path exists.
- `mpc_feasibility` with the same public keys as Python.
- Config snapshot fields for whether evaluation was enabled and the relevant limits.

`tests/test_cpp_sync_static.py` should assert these headers/sources and JSON keys exist.

## Data Flow

1. Planner produces a discrete path.
2. FIRI refines the path and exposes corridors.
3. Trajectory optimizer builds candidate trajectories.
4. Corridor and clearance gates accept or reject candidates.
5. Accepted or fallback trajectory is emitted as `planned_trajectory`.
6. MPC feasibility evaluator scans the trajectory demands and emits `mpc_feasibility`.
7. Metrics extractor and C++ result writer expose matching evidence fields.

## Error Handling

- Invalid input shapes still raise `ValueError`.
- Empty paths return empty accepted trajectory metrics.
- Corridor rejection uses fallback when enabled.
- MPC feasibility on empty trajectories returns `evaluated=False`, `feasible=False`, and a recommendation explaining missing data.
- C++ result writing should emit absent/empty-safe objects rather than crashing on short paths.

## Testing

Python tests:

- `tests/test_trajectory_optimizer.py`
  - corridor-contained candidate is accepted.
  - corridor-violating candidate falls back with explicit reason.
  - curvature metrics are present and non-negative.
- New `tests/test_mpc_tracker.py`
  - feasible trajectory produces feasible recommendation.
  - acceleration/velocity demand violations produce infeasible recommendation.
- Existing scenario integration still emits planned trajectory and now includes feasibility when enabled.

C++ tests:

- `tests/test_cpp_sync_static.py`
  - trajectory metrics files are included in CMake.
  - result writer emits `planned_trajectory` and `mpc_feasibility`.
  - config/result structs expose synchronized fields.

Verification commands:

```powershell
python -m pytest tests/test_trajectory_optimizer.py tests/test_mpc_tracker.py -q
python -m pytest tests/test_cpp_sync_static.py tests/test_obstacle_scenario.py -q -k "firi or obstacle_simulation_zero_collision or hybrid_astar_obstacle"
python -m pytest -m "not slow" -q
cmake --build cpp/build --config Release
```

If C++ build artifacts are not configured on the local machine, record the exact failure and keep the static sync tests as the minimum gate.

## Acceptance Criteria

- A1 emits trajectory smoothness and curvature metrics.
- FIRI-aware optimization never silently accepts a path outside its corridor gate.
- A2 emits MPC feasibility evidence without changing the active controller.
- Python tests and A-line obstacle/FIRI regressions pass.
- C++ public result fields and build files are synchronized.
- Existing collision red line remains `0` in covered A-line scenarios.
