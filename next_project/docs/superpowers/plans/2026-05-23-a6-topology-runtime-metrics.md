# A6 Topology Runtime Metrics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Python/C++ runtime topology metrics to simulation result payloads.

**Architecture:** A small accumulator observes existing offsets and control vectors without changing controller behavior. Python and C++ expose the same JSON keys through `topology_metrics`.

**Tech Stack:** Python, NumPy, pytest, C++17, CMake.

---

### Task 1: Python Accumulator

**Files:**
- Create: `core/topology_metrics.py`
- Test: `tests/test_topology_metrics.py`

- [ ] **Step 1: Write failing aggregation tests**

Add tests that instantiate `TopologyMetricAccumulator(dt=0.5)`, add samples with offsets and controls, and assert `available`, `sample_count`, lambda2 fields, and energy proxies.

- [ ] **Step 2: Run the focused test**

Run: `python -m pytest tests/test_topology_metrics.py -q`

Expected: fail because `core.topology_metrics` does not exist.

- [ ] **Step 3: Implement the accumulator**

Create `TopologyRuntimeMetrics` and `TopologyMetricAccumulator`. Compute lambda2 with `TopologyGraph(offsets).algebraic_connectivity`, integrate squared control norms by `dt`, and serialize with `to_dict()`.

- [ ] **Step 4: Verify the focused test**

Run: `python -m pytest tests/test_topology_metrics.py -q`

Expected: pass.

### Task 2: Python Simulation Wiring

**Files:**
- Modify: `simulations/formation_simulation.py`
- Modify: `simulations/obstacle_scenario.py`
- Test: `tests/test_obstacle_scenario.py`

- [ ] **Step 1: Add a failing result test**

Extend an existing lightweight obstacle scenario test to assert `topology_metrics` exists and includes positive sample count plus lambda2 keys.

- [ ] **Step 2: Wire accumulator into loops**

Instantiate `TopologyMetricAccumulator(config.dt)` and feed offsets, leader control, and follower controls once per simulation step.

- [ ] **Step 3: Attach result field**

Add `"topology_metrics": topology_metrics.to_dict(self.fault_log)` for obstacle scenarios and `"topology_metrics": topology_metrics.to_dict()` for the base formation simulation.

- [ ] **Step 4: Verify Python scenario coverage**

Run: `python -m pytest tests/test_topology_metrics.py tests/test_obstacle_scenario.py -q -k "topology_metrics or formation_safety_metrics_are_reported_when_enabled"`

Expected: pass.

### Task 3: C++ Sync

**Files:**
- Modify: `cpp/include/formation_simulation.hpp`
- Modify: `cpp/include/result_writer.hpp`
- Modify: `cpp/src/formation_simulation.cpp`
- Modify: `cpp/src/obstacle_scenario.cpp`
- Test: `tests/test_cpp_sync_static.py`

- [ ] **Step 1: Add static sync assertions**

Require `TopologyRuntimeMetrics`, `SimulationResult::topology_metrics`, result writer keys, and obstacle scenario assignment.

- [ ] **Step 2: Run static test and observe failure**

Run: `python -m pytest tests/test_cpp_sync_static.py -q -k topology`

Expected: fail until C++ sync is implemented.

- [ ] **Step 3: Implement C++ fields and writer output**

Add the struct, update simulation loops to track lambda2 and energy proxies, count `fault_log_` entries, and serialize `topology_metrics`.

- [ ] **Step 4: Build C++**

Run: `cmake --build cpp/build --config Release`

Expected: build succeeds.

### Task 4: Verification and Commit

**Files:**
- All touched files.

- [ ] **Step 1: Run focused tests**

Run: `python -m pytest tests/test_topology_metrics.py tests/test_cpp_sync_static.py -q`

Expected: pass.

- [ ] **Step 2: Run non-slow suite with repo-local temp if needed**

Run: `python -m pytest -m "not slow" -q`

Expected: pass. If Windows temp permissions fail, set `TMP` and `TEMP` to `.pytest_tmp` inside the repo and rerun.

- [ ] **Step 3: Commit**

Run:

```bash
git add docs/superpowers/specs/2026-05-23-a6-topology-runtime-metrics-design.md docs/superpowers/plans/2026-05-23-a6-topology-runtime-metrics.md core/topology_metrics.py simulations/formation_simulation.py simulations/obstacle_scenario.py cpp/include/formation_simulation.hpp cpp/include/result_writer.hpp cpp/src/formation_simulation.cpp cpp/src/obstacle_scenario.cpp tests/test_topology_metrics.py tests/test_cpp_sync_static.py tests/test_obstacle_scenario.py
git commit -m "feat: add A6 topology runtime metrics"
```
