# RRT Formation Test Maps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two staged UAV formation test maps: first a dual-channel RRT escape map, then a harder maze stress map, both usable by scenario presets, ablation runs, and markdown/CSV reports.

**Architecture:** Store the maps as JSON obstacle primitives under `next_project/maps/`, expose each through `config.py` and `experiments/scenario_registry.py`, and verify behavior through pytest plus quick ablation smoke runs. Stage B is the correctness gate for RRT-assisted lookahead escape; Stage C is a stress scenario that reuses the same workflow after B is stable.

**Tech Stack:** Python 3, NumPy, pytest, existing `SimulationConfig`, existing JSON map loader, existing `experiments.run_ablation` reporting flow.

---

### Task 1: Dual-Channel Map JSON

**Files:**
- Create: `next_project/maps/rrt_dual_channel_escape.json`
- Modify: `next_project/tests/test_obstacle_scenario.py`

- [ ] **Step 1: Write the failing map-loader test**

Add this test near the existing map-loader tests in `tests/test_obstacle_scenario.py`:

```python
def test_rrt_dual_channel_escape_map_loads():
    map_path = _project / "maps" / "rrt_dual_channel_escape.json"

    field, bounds = load_from_json(str(map_path))

    assert bounds.shape == (2, 3)
    assert bounds[0].tolist() == [-2.0, -4.0, 0.0]
    assert bounds[1].tolist() == [24.0, 12.0, 4.0]
    assert len(field) >= 16
    assert field.signed_distance(np.array([0.0, 0.0, 1.8], dtype=float)) > 0.2
    assert field.signed_distance(np.array([20.5, 4.5, 1.8], dtype=float)) > 0.2
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py::test_rrt_dual_channel_escape_map_loads -q
```

Expected: FAIL with `FileNotFoundError` for `rrt_dual_channel_escape.json`.

- [ ] **Step 3: Add the dual-channel map**

Create `maps/rrt_dual_channel_escape.json` with this structure:

```json
{
  "bounds": [[-2, -4, 0], [24, 12, 4]],
  "description": "RRT双通道绕行对照图：主通道局部前瞻呈现假死路，侧向旁路可由RRT escape发现，同时包含狭长通道与90度转弯，用于验证编队提前变换和从机安全裕度。",
  "obstacles": [
    {"type": "aabb", "min": [-2, -4, 0], "max": [24, -2.0, 4]},
    {"type": "aabb", "min": [-2, 8.0, 0], "max": [24, 12, 4]},
    {"type": "aabb", "min": [-2, -4, 0], "max": [-0.7, 12, 4]},
    {"type": "aabb", "min": [23.0, -4, 0], "max": [24, 12, 4]},

    {"type": "aabb", "min": [2.0, 1.15, 0], "max": [8.0, 8.0, 4]},
    {"type": "aabb", "min": [2.0, -2.0, 0], "max": [8.0, -1.15, 4]},

    {"type": "aabb", "min": [8.0, -2.0, 0], "max": [10.2, 1.0, 4]},
    {"type": "aabb", "min": [10.2, -2.0, 0], "max": [11.2, 2.7, 4]},
    {"type": "aabb", "min": [11.2, -2.0, 0], "max": [12.2, 3.5, 4]},

    {"type": "aabb", "min": [8.0, 2.15, 0], "max": [10.4, 8.0, 4]},
    {"type": "aabb", "min": [10.4, 3.35, 0], "max": [13.8, 8.0, 4]},
    {"type": "aabb", "min": [13.8, 5.85, 0], "max": [20.0, 8.0, 4]},

    {"type": "aabb", "min": [13.8, -2.0, 0], "max": [20.0, 3.25, 4]},
    {"type": "aabb", "min": [20.0, -2.0, 0], "max": [23.0, 3.15, 4]},
    {"type": "aabb", "min": [20.0, 5.85, 0], "max": [23.0, 8.0, 4]},

    {"type": "cylinder", "center_xy": [4.0, 0.0], "radius": 0.18, "z_range": [0, 3.6]},
    {"type": "cylinder", "center_xy": [6.3, 0.0], "radius": 0.18, "z_range": [0, 3.6]},
    {"type": "cylinder", "center_xy": [13.2, 4.5], "radius": 0.20, "z_range": [0, 3.6]},
    {"type": "cylinder", "center_xy": [17.2, 4.5], "radius": 0.20, "z_range": [0, 3.6]}
  ]
}
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py::test_rrt_dual_channel_escape_map_loads -q
```

Expected: PASS.

### Task 2: Dual-Channel Preset And Registry

**Files:**
- Modify: `next_project/config.py`
- Modify: `next_project/experiments/scenario_registry.py`
- Modify: `next_project/tests/test_experiment_workflow.py`

- [ ] **Step 1: Write the failing preset test**

Add this test to `tests/test_experiment_workflow.py`:

```python
def test_rrt_dual_channel_online_scenario_is_registered():
    scenarios = list_scenarios("online")
    assert any(spec.name == "rrt_dual_channel_online" for spec in scenarios)

    cfg = get_scenario_config("rrt_dual_channel_online", quick=True)

    assert cfg.enable_obstacles is True
    assert cfg.planner_mode == "online"
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True
    assert cfg.map_file is not None
    assert cfg.map_file.endswith("rrt_dual_channel_escape.json")
    assert cfg.max_sim_time <= 5.0
    assert cfg.planner_horizon <= 3.0
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_rrt_dual_channel_online_scenario_is_registered -q
```

Expected: FAIL because the scenario is not registered.

- [ ] **Step 3: Add `rrt_dual_channel_online` config**

In `config.py`:

1. Add a branch to `get_config`:

```python
elif preset == "rrt_dual_channel_online":
    return _config_rrt_dual_channel_online()
```

2. Add this function after `_config_meeting_room_online`:

```python
def _config_rrt_dual_channel_online() -> SimulationConfig:
    """RRT双通道绕行对照图：局部前瞻假死路 + RRT旁路escape + 编队提前变换。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=28.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.55,
        initial_formation="diamond",
        wp_radius=0.45,
        wp_radius_final=0.25,
        leader_max_vel=1.0,
        leader_max_acc=1.4,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "rrt_dual_channel_escape.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_z_bounds=(1.4, 2.4),
        sensor_enabled=True,
        planner_replan_interval=1.0,
        planner_horizon=4.0,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=True,
        formation_lookahead_enabled=True,
        formation_lookahead_rrt_enabled=True,
        formation_lookahead_distance=4.2,
        formation_lookahead_turn_threshold_rad=1.0,
        formation_lookahead_min_interval=0.8,
        formation_lookahead_rrt_max_iter=900,
        formation_lookahead_rrt_rewire_radius=1.2,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([6.5, 0.0, 1.8], dtype=float),
            np.array([11.5, 4.5, 1.8], dtype=float),
            np.array([20.5, 4.5, 1.8], dtype=float),
        ],
    )
```

In `experiments/scenario_registry.py`, add:

```python
"rrt_dual_channel_online": ScenarioSpec(
    name="rrt_dual_channel_online",
    category="online",
    description="Dual-channel RRT lookahead escape and formation adaptation scenario.",
),
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_rrt_dual_channel_online_scenario_is_registered -q
```

Expected: PASS.

### Task 3: Dual-Channel Behavioral Smoke Test

**Files:**
- Modify: `next_project/tests/test_experiment_workflow.py`

- [ ] **Step 1: Write the failing smoke test**

Add this test:

```python
def test_rrt_dual_channel_quick_run_records_rrt_escape():
    cfg = get_scenario_config("rrt_dual_channel_online", quick=True)
    apply_variant(cfg, "formation_aware_lookahead_adaptive")
    cfg.max_sim_time = min(cfg.max_sim_time, 5.0)

    from experiments.metrics_extractor import extract_metrics
    from simulations.obstacle_scenario import ObstacleScenarioSimulation

    result = ObstacleScenarioSimulation(cfg).run()
    metrics = extract_metrics(result)

    assert metrics["lookahead_reference_blocked_count"] > 0
    assert metrics["rrt_escape_attempt_count"] > 0
    assert metrics["rrt_escape_accepted_count"] > 0
    assert metrics["collision_count"] == 0
```

- [ ] **Step 2: Run test to verify it fails if behavior is not yet tuned**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_rrt_dual_channel_quick_run_records_rrt_escape -q
```

Expected: FAIL if the map or config does not yet trigger accepted RRT escape.

- [ ] **Step 3: Tune only map/config values needed to satisfy behavior**

Allowed tuning:

- Corridor widths in `maps/rrt_dual_channel_escape.json`
- `planner_horizon`
- `formation_lookahead_distance`
- `formation_lookahead_rrt_max_iter`
- `formation_lookahead_rrt_rewire_radius`
- waypoints in `_config_rrt_dual_channel_online`

Do not change core planner logic for this task unless the failure reveals a real bug.

- [ ] **Step 4: Run test to verify it passes**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_rrt_dual_channel_quick_run_records_rrt_escape -q
```

Expected: PASS.

### Task 4: Dual-Channel Ablation Report

**Files:**
- Generate: `next_project/outputs/ablation_rrt_dual_channel/summary.csv`
- Generate: `next_project/outputs/ablation_rrt_dual_channel/report.md`

- [ ] **Step 1: Run quick ablation**

Run:

```powershell
python -m experiments.run_ablation --quick --scenario rrt_dual_channel_online --variant leader_only_planner --variant formation_aware_adaptive --variant formation_aware_lookahead_adaptive --output-dir outputs/ablation_rrt_dual_channel
```

Expected: command exits 0 and writes `summary.csv` plus `report.md`.

- [ ] **Step 2: Inspect output**

Run:

```powershell
Get-Content outputs/ablation_rrt_dual_channel/summary.csv
```

Expected: `formation_aware_lookahead_adaptive` row has `rrt_escape_accepted_count > 0`.

### Task 5: Maze Stress Map JSON And Preset

**Files:**
- Create: `next_project/maps/formation_maze_stress.json`
- Modify: `next_project/config.py`
- Modify: `next_project/experiments/scenario_registry.py`
- Modify: `next_project/tests/test_experiment_workflow.py`

- [ ] **Step 1: Write failing tests**

Add:

```python
def test_formation_maze_stress_online_scenario_is_registered():
    scenarios = list_scenarios("online")
    assert any(spec.name == "formation_maze_stress_online" for spec in scenarios)

    cfg = get_scenario_config("formation_maze_stress_online", quick=True)

    assert cfg.enable_obstacles is True
    assert cfg.planner_mode == "online"
    assert cfg.formation_lookahead_enabled is True
    assert cfg.formation_lookahead_rrt_enabled is True
    assert cfg.map_file is not None
    assert cfg.map_file.endswith("formation_maze_stress.json")
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_formation_maze_stress_online_scenario_is_registered -q
```

Expected: FAIL because the scenario is not registered.

- [ ] **Step 3: Add map and preset**

Create `maps/formation_maze_stress.json` with AABB walls that create: initial narrow gate, 90-degree corner, false branch, RRT bypass, U-turn, wide recovery area, final narrow exit.

Add a `get_config` branch:

```python
elif preset == "formation_maze_stress_online":
    return _config_formation_maze_stress_online()
```

Add `_config_formation_maze_stress_online` with online A* settings similar to `rrt_dual_channel_online`, longer `max_sim_time=38.0`, `planner_horizon=4.5`, and waypoints:

```python
[
    np.array([0.0, 0.0, 1.8], dtype=float),
    np.array([4.5, 3.5, 1.8], dtype=float),
    np.array([10.0, 3.5, 1.8], dtype=float),
    np.array([10.0, -2.5, 1.8], dtype=float),
    np.array([17.0, -2.5, 1.8], dtype=float),
    np.array([20.5, 5.0, 1.8], dtype=float),
]
```

Add a scenario registry entry:

```python
"formation_maze_stress_online": ScenarioSpec(
    name="formation_maze_stress_online",
    category="online",
    description="Maze-like formation stress test with false branches, narrow gates, and RRT escape.",
),
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```powershell
python -m pytest tests/test_experiment_workflow.py::test_formation_maze_stress_online_scenario_is_registered -q
```

Expected: PASS.

### Task 6: Maze Stress Report Smoke

**Files:**
- Generate: `next_project/outputs/ablation_formation_maze_stress/summary.csv`
- Generate: `next_project/outputs/ablation_formation_maze_stress/report.md`

- [ ] **Step 1: Run focused tests**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py::test_rrt_dual_channel_escape_map_loads tests/test_experiment_workflow.py::test_rrt_dual_channel_online_scenario_is_registered tests/test_experiment_workflow.py::test_rrt_dual_channel_quick_run_records_rrt_escape tests/test_experiment_workflow.py::test_formation_maze_stress_online_scenario_is_registered -q
```

Expected: PASS.

- [ ] **Step 2: Run maze quick ablation**

Run:

```powershell
python -m experiments.run_ablation --quick --scenario formation_maze_stress_online --variant leader_only_planner --variant formation_aware_adaptive --variant formation_aware_lookahead_adaptive --output-dir outputs/ablation_formation_maze_stress
```

Expected: command exits 0 and writes summary/report.

- [ ] **Step 3: Run broader verification**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py tests/test_experiment_workflow.py tests/test_metrics_extractor.py -q
```

Expected: PASS.

- [ ] **Step 4: Run full verification**

Run:

```powershell
python -m pytest tests -q
```

Expected: PASS or report exact remaining failures.

