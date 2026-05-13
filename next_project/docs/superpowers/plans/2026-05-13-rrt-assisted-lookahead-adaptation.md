# RRT-Assisted Lookahead Formation Adaptation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在在线仿真中用 RRT* 辅助前瞻判断，提前识别狭长通道急转弯和局部采样死路，并触发编队变换与 escape 路径规划。

**Architecture:** 保留现有 `ObstacleScenarioSimulation` 在线重规划主循环，把前瞻诊断做成独立方法：先对当前局部路径窗口做 clearance 和转角诊断，再在风险出现时执行 formation adaptation，最后用 RRT* 对当前 leader 到任务航点生成候选 escape 路径。报告层通过 `planning_events` 和 `formation_adaptation_events` 暴露 `lookahead_reference_blocked`、`rrt_escape_attempt`、`rrt_escape_accepted`、`rrt_escape_failed` 等事件，消融层新增 lookahead adaptive 变体。

**Tech Stack:** Python 3, NumPy, pytest, existing `RRTStar`, existing `SimulationConfig`, existing experiment metrics/report workflow.

---

### Task 1: Lookahead Diagnosis

**Files:**
- Modify: `next_project/tests/test_obstacle_scenario.py`
- Modify: `next_project/simulations/formation_simulation.py`
- Modify: `next_project/simulations/obstacle_scenario.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_lookahead_diagnosis_detects_sharp_corner_before_arrival():
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=3,
        enable_obstacles=True,
        planner_mode="online",
        planner_use_formation_envelope=True,
        formation_adaptation_enabled=True,
        formation_lookahead_enabled=True,
        formation_lookahead_distance=4.0,
        formation_lookahead_turn_threshold_rad=1.0,
        obstacle_field=ObstacleField(),
        waypoints=[np.array([0, 0, 1.0]), np.array([4, 0, 1.0])],
    )
    sim = ObstacleScenarioSimulation(cfg)
    path = np.array([[0, 0, 1.0], [2, 0, 1.0], [2, 2, 1.0], [4, 2, 1.0]], dtype=float)
    diagnosis = sim._lookahead_path_diagnosis(path, path[0])
    assert diagnosis["blocked"] is True
    assert diagnosis["reason"] == "lookahead_sharp_turn"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_lookahead_diagnosis_detects_sharp_corner_before_arrival -q`

Expected: FAIL because `SimulationConfig` has no `formation_lookahead_enabled` field or `_lookahead_path_diagnosis` does not exist.

- [ ] **Step 3: Add minimal config and diagnosis implementation**

Add config fields:

```python
formation_lookahead_enabled: bool = False
formation_lookahead_distance: float = 4.0
formation_lookahead_turn_threshold_rad: float = 1.05
formation_lookahead_min_interval: float = 0.8
formation_lookahead_rrt_enabled: bool = False
formation_lookahead_rrt_max_iter: int = 800
formation_lookahead_rrt_rewire_radius: float = 1.2
```

Add `_path_window_from_position`, `_path_max_turn_angle`, and `_lookahead_path_diagnosis` to `ObstacleScenarioSimulation`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_lookahead_diagnosis_detects_sharp_corner_before_arrival -q`

Expected: PASS.

### Task 2: RRT Escape Events And Acceptance

**Files:**
- Modify: `next_project/tests/test_obstacle_scenario.py`
- Modify: `next_project/simulations/obstacle_scenario.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_rrt_escape_path_records_attempt_and_acceptance():
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=1,
        enable_obstacles=True,
        planner_mode="online",
        formation_lookahead_enabled=True,
        formation_lookahead_rrt_enabled=True,
        formation_lookahead_rrt_max_iter=40,
        obstacle_field=ObstacleField(),
        waypoints=[np.array([0, 0, 1.0]), np.array([4, 0, 1.0])],
    )
    sim = ObstacleScenarioSimulation(cfg)
    path = sim._plan_rrt_escape_path(np.array([0, 0, 1.0]), np.array([4, 0, 1.0]), 0.3, 1.0)
    kinds = [event["kind"] for event in sim.formation_adaptation_events]
    assert path is not None
    assert "rrt_escape_attempt" in kinds
    assert "rrt_escape_accepted" in kinds
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_rrt_escape_path_records_attempt_and_acceptance -q`

Expected: FAIL because `_plan_rrt_escape_path` does not exist.

- [ ] **Step 3: Add RRT escape method**

Implement `_plan_rrt_escape_path` using `RRTStar(max_iter=config.formation_lookahead_rrt_max_iter, rewire_radius=config.formation_lookahead_rrt_rewire_radius, smooth_method="shortcut")`, append attempt/accepted/failed events, and return only paths passing `_segment_is_safe`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_rrt_escape_path_records_attempt_and_acceptance -q`

Expected: PASS.

### Task 3: Online Loop Integration

**Files:**
- Modify: `next_project/tests/test_obstacle_scenario.py`
- Modify: `next_project/simulations/obstacle_scenario.py`

- [ ] **Step 1: Write the failing test**

```python
def test_lookahead_escape_runs_before_regular_sensor_replan():
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=1,
        enable_obstacles=True,
        planner_mode="online",
        formation_lookahead_enabled=True,
        formation_lookahead_rrt_enabled=True,
        obstacle_field=ObstacleField(),
        waypoints=[np.array([0, 0, 1.0]), np.array([4, 0, 1.0])],
    )
    sim = ObstacleScenarioSimulation(cfg)
    local_path = np.array([[0, 0, 1.0], [1, 0, 1.0], [1, 1.5, 1.0], [4, 1.5, 1.0]], dtype=float)
    new_path = sim._maybe_rrt_lookahead_escape(1.0, local_path[0], local_path, np.array([4, 0, 1.0]))
    assert new_path is not None
    assert any(event.get("phase") == "online_lookahead_rrt_escape" for event in sim.planning_events)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_lookahead_escape_runs_before_regular_sensor_replan -q`

Expected: FAIL because `_maybe_rrt_lookahead_escape` does not exist.

- [ ] **Step 3: Add integration helper and call it in the online loop**

Add `_maybe_rrt_lookahead_escape` and invoke it before `force_replan`, `local_path_exhausted`, and `online_sensor_replan`. When it returns a candidate, set `planning_phase="online_lookahead_rrt_escape"` and use the existing `_accept_online_path` gate.

- [ ] **Step 4: Run focused tests**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_lookahead_diagnosis_detects_sharp_corner_before_arrival tests/test_obstacle_scenario.py::test_rrt_escape_path_records_attempt_and_acceptance tests/test_obstacle_scenario.py::test_lookahead_escape_runs_before_regular_sensor_replan -q`

Expected: PASS.

### Task 4: Metrics And Ablation

**Files:**
- Modify: `next_project/tests/test_experiment_workflow.py`
- Modify: `next_project/tests/test_metrics_extractor.py`
- Modify: `next_project/experiments/ablation.py`
- Modify: `next_project/experiments/metrics_extractor.py`
- Modify: `next_project/experiments/report_writer.py`

- [ ] **Step 1: Write failing tests**

```python
def test_lookahead_adaptation_ablation_variant_is_explicit():
    cfg = get_scenario_config("meeting_room", quick=True)
    apply_variant(cfg, "formation_aware_lookahead_adaptive")
    assert cfg.formation_adaptation_enabled is True
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True
```

```python
assert metrics["lookahead_reference_blocked_count"] == 1
assert metrics["rrt_escape_attempt_count"] == 1
assert metrics["rrt_escape_accepted_count"] == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_experiment_workflow.py::test_lookahead_adaptation_ablation_variant_is_explicit tests/test_metrics_extractor.py::test_extract_metrics_from_standard_sim_result -q`

Expected: FAIL because variant and metrics do not exist.

- [ ] **Step 3: Implement variant and metric extraction**

Add ablation variant `formation_aware_lookahead_adaptive`; count `formation_adaptation_events[*].kind` values in metrics extractor; add fields to preferred report columns.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_experiment_workflow.py::test_lookahead_adaptation_ablation_variant_is_explicit tests/test_metrics_extractor.py::test_extract_metrics_from_standard_sim_result -q`

Expected: PASS.

### Task 5: Verification

**Files:**
- No code edits.

- [ ] **Step 1: Run focused regression**

Run: `python -m pytest tests/test_obstacle_scenario.py tests/test_experiment_workflow.py tests/test_metrics_extractor.py -q`

Expected: PASS.

- [ ] **Step 2: Run full project tests**

Run: `python -m pytest tests -q`

Expected: PASS or report the exact remaining failures with file and assertion names.

