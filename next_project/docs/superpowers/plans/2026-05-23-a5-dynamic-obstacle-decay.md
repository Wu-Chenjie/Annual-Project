# A5 Dynamic Obstacle Decay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add opt-in transient/persistent decay classes for sensor-discovered obstacles.

**Architecture:** Keep classification inside `WindowReplanner` next to existing sensor TTL state. Preserve uniform TTL as the default. Mirror the configuration and state surface in C++ without changing planner search behavior.

**Tech Stack:** Python, NumPy, pytest, C++20, CMake.

---

## File Structure

- Modify: `core/planning/replanner.py`
- Modify: `tests/test_replanner_semantics.py`
- Modify: `cpp/include/replanner.hpp`
- Modify: `cpp/include/obstacle_scenario.hpp`
- Modify: `cpp/src/obstacle_scenario.cpp`
- Modify: `tests/test_cpp_sync_static.py`

---

### Task 1: Python Classified Sensor TTL

**Files:**
- Modify: `tests/test_replanner_semantics.py`
- Modify: `core/planning/replanner.py`

- [ ] **Step 1: Write failing Python tests**

Add tests proving a single transient hit expires quickly and repeated hits promote to persistent TTL:

```python
def test_classified_sensor_obstacle_promotes_to_persistent_ttl():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(16, 16, 4))
    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=4.0,
        sensor_obstacle_ttl_steps=1,
        sensor_obstacle_classification_enabled=True,
        sensor_obstacle_persistent_hits=2,
        sensor_obstacle_persistent_ttl_steps=4,
    )
    pose = np.array([2.0, 2.0, 1.0], dtype=float)
    readings = np.array([2.0, 4.0, 4.0, 4.0, 4.0, 4.0], dtype=float)
    hit_idx = grid.world_to_index(np.array([4.0, 2.0, 1.0], dtype=float))

    replanner._update_grid_from_sensor(pose, readings)
    assert replanner._sensor_obstacle_class[hit_idx] == replanner.SENSOR_OBSTACLE_TRANSIENT
    assert replanner._sensor_ttl[hit_idx] == 1

    replanner._update_grid_from_sensor(pose, readings)
    assert replanner._sensor_obstacle_class[hit_idx] == replanner.SENSOR_OBSTACLE_PERSISTENT
    assert replanner._sensor_ttl[hit_idx] == 4
```

Add a second test:

```python
def test_classified_sensor_obstacle_single_hit_expires_as_transient():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(16, 16, 4))
    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=4.0,
        sensor_obstacle_ttl_steps=1,
        sensor_obstacle_classification_enabled=True,
        sensor_obstacle_persistent_ttl_steps=4,
    )
    pose = np.array([2.0, 2.0, 1.0], dtype=float)
    readings = np.array([2.0, 4.0, 4.0, 4.0, 4.0, 4.0], dtype=float)
    hit_idx = grid.world_to_index(np.array([4.0, 2.0, 1.0], dtype=float))

    replanner._update_grid_from_sensor(pose, readings)
    replanner._decay_sensor_obstacles()

    assert not replanner._sensor_occupied[hit_idx]
    assert replanner._sensor_obstacle_class[hit_idx] == replanner.SENSOR_OBSTACLE_NONE
```

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m pytest tests/test_replanner_semantics.py::test_classified_sensor_obstacle_promotes_to_persistent_ttl -q
```

Expected: constructor or attribute failure because classification does not exist yet.

- [ ] **Step 3: Implement Python classification**

In `WindowReplanner`, add class constants, constructor parameters, `_sensor_hit_count`, `_sensor_obstacle_class`, `_sensor_ttl_for_hit()`, and reset logic when cells clear.

- [ ] **Step 4: Verify GREEN**

Run:

```powershell
python -m pytest tests/test_replanner_semantics.py -q
```

Expected: all replanner semantic tests pass.

---

### Task 2: C++ Surface Synchronization

**Files:**
- Modify: `tests/test_cpp_sync_static.py`
- Modify: `cpp/include/replanner.hpp`
- Modify: `cpp/include/obstacle_scenario.hpp`
- Modify: `cpp/src/obstacle_scenario.cpp`

- [ ] **Step 1: Write failing static sync assertions**

Extend `test_cpp_replanner_has_risk_adaptive_interval_and_sensor_ttl` to require:

```python
assert "enable_sensor_obstacle_classification" in source
assert "sensor_hit_count_" in source
assert "sensor_obstacle_class_" in source
assert "sensor_ttl_for_hit" in source
```

Add scenario assertions for:

```python
assert "bool sensor_obstacle_classification_enabled = false;" in scenario_header
assert "int sensor_obstacle_persistent_hits = 2;" in scenario_header
assert "int sensor_obstacle_persistent_ttl_steps = 6;" in scenario_header
assert "enable_sensor_obstacle_classification(" in scenario_source
```

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m pytest tests/test_cpp_sync_static.py::test_cpp_replanner_has_risk_adaptive_interval_and_sensor_ttl -q
```

Expected: static assertion failure.

- [ ] **Step 3: Implement C++ sync**

Add the same state arrays and helper methods to `cpp/include/replanner.hpp`, then expose config fields and scenario wiring.

- [ ] **Step 4: Verify C++ sync**

Run:

```powershell
python -m pytest tests/test_cpp_sync_static.py -q
cmake --build cpp/build --config Release
```

Expected: static tests and build pass.

---

### Task 3: Regression And Publish

**Files:**
- No additional files unless a regression appears.

- [ ] **Step 1: Run focused tests**

```powershell
python -m pytest tests/test_replanner_semantics.py tests/test_cpp_sync_static.py -q
```

- [ ] **Step 2: Run A-line regression**

```powershell
python -m pytest tests/test_replanner_danger_quality.py tests/test_fault_tolerance_online.py -q
python -m pytest tests/test_obstacle_scenario.py -q -k "online_mode_preserves_task_waypoint_semantics or terminal_hold_reduces_end_jitter"
python -m pytest -m "not slow" -q
git diff --check
```

- [ ] **Step 3: Commit and push**

```powershell
git add -- core/planning/replanner.py tests/test_replanner_semantics.py cpp/include/replanner.hpp cpp/include/obstacle_scenario.hpp cpp/src/obstacle_scenario.cpp tests/test_cpp_sync_static.py docs/superpowers/specs/2026-05-23-a5-dynamic-obstacle-decay-design.md docs/superpowers/plans/2026-05-23-a5-dynamic-obstacle-decay.md
git commit -m "feat: add A5 classified sensor obstacle decay"
git push
```
