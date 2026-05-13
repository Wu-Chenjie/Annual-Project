# RRT Formation Test Maps Design

## Purpose

Build two staged test maps for the UAV formation planning workflow:

1. `rrt_dual_channel_escape`: a readable dual-channel map for validating RRT-assisted lookahead escape against a local-window dead-end false positive.
2. `formation_maze_stress`: a harder maze-like map for second-stage stress testing formation-aware clearance, adaptive formation changes, RRT escape, and reporting.

The first map is the implementation priority. The second map should reuse the same data model and reporting workflow after the first one is stable.

## Existing Context

Maps are JSON files under `next_project/maps/` with:

- `bounds`: world-space lower and upper corners.
- `description`: Chinese scenario description.
- `obstacles`: AABB, sphere, or cylinder obstacle primitives.

Scenario presets live in `next_project/config.py`, and experiment discovery uses `next_project/experiments/scenario_registry.py`. Ablation runs use `experiments/run_ablation.py`; generated metrics already include lookahead and RRT escape counts:

- `lookahead_reference_blocked_count`
- `rrt_escape_attempt_count`
- `rrt_escape_accepted_count`
- `rrt_escape_failed_count`

## Stage 1 Map: B Dual-Channel Escape

### File And Preset

- Map file: `next_project/maps/rrt_dual_channel_escape.json`
- Config preset: `rrt_dual_channel_online`
- Scenario registry entry: `rrt_dual_channel_online`
- Suggested report output directory: `outputs/ablation_rrt_dual_channel`

### Geometry

The map is a medium-size indoor corridor environment with one main route and one RRT-discoverable bypass.

Recommended bounds:

```json
[[-2, -4, 0], [24, 12, 4]]
```

Recommended flight altitude:

```python
planner_z_bounds=(1.4, 2.4)
```

Core regions:

1. Start staging area: a wider entry pocket so the initial formation can be stable.
2. Narrow approach corridor: wide enough for compact `triangle` or `line`, too narrow for the default wider formation.
3. False dead-end pocket: the local lookahead window sees a blocked continuation.
4. Side bypass: an offset corridor reachable by RRT, not obvious from a short forward-only window.
5. Corner segment: a 90-degree turn after the bypass, requiring formation adaptation before entering.
6. Finish area: a wider area where the formation can recover if clearance becomes safe.

### Waypoints

Use waypoints that naturally lead the leader into the main corridor and then to the final goal:

```python
[
    np.array([0.0, 0.0, 1.8], dtype=float),
    np.array([6.5, 0.0, 1.8], dtype=float),
    np.array([11.5, 4.5, 1.8], dtype=float),
    np.array([20.5, 4.5, 1.8], dtype=float),
]
```

These points intentionally do not over-specify the bypass. The lookahead/RRT module should discover the local escape route when the main forward window becomes blocked or sharply turning.

### Recommended Config

```python
SimulationConfig(
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
)
```

### Success Criteria

For quick ablation on `rrt_dual_channel_online`:

- `leader_only_planner` should provide a baseline path and expose posthoc follower clearance risk.
- `formation_aware_adaptive` should adapt formation but should not record RRT escape events.
- `formation_aware_lookahead_adaptive` should record:
  - `lookahead_reference_blocked_count > 0`
  - `rrt_escape_attempt_count > 0`
  - `rrt_escape_accepted_count > 0`
  - `collision_count == 0` or no worse than `formation_aware_adaptive`
- Report output should include the four lookahead/RRT count columns.

### Tests

Add tests that verify:

1. The JSON map loads through `load_from_json`.
2. `get_config("rrt_dual_channel_online")` returns an online obstacle scenario with the new map file.
3. `apply_variant(..., "formation_aware_lookahead_adaptive")` keeps lookahead and RRT enabled on this preset.
4. A short smoke simulation records at least one lookahead blocked event and one accepted RRT escape event.

## Stage 2 Map: C Maze Stress

### File And Preset

- Map file: `next_project/maps/formation_maze_stress.json`
- Config preset: `formation_maze_stress_online`
- Scenario registry entry: `formation_maze_stress_online`
- Suggested report output directory: `outputs/ablation_formation_maze_stress`

### Geometry

The maze map should increase difficulty without making failure causes unreadable.

Required regions:

1. Initial narrow gate for early formation contraction.
2. First 90-degree corner.
3. False branch dead-end.
4. RRT-discoverable bypass.
5. U-shaped turn with limited local visibility.
6. Wide recovery area.
7. Final narrow exit.

### Success Criteria

This map is a stress scenario, so it should not be used as the first correctness gate. It should be accepted when:

- It produces multiple formation adaptation events.
- It produces at least one lookahead/RRT event pair.
- It produces a readable report with planning events, waypoint arrivals, and path/trajectory metrics.
- It remains reproducible under the quick-mode ablation runtime.

## Reporting

Both maps should be usable with:

```powershell
python -m experiments.run_ablation --quick --scenario rrt_dual_channel_online --variant leader_only_planner --variant formation_aware_adaptive --variant formation_aware_lookahead_adaptive --output-dir outputs/ablation_rrt_dual_channel
```

Then use the second-stage map with:

```powershell
python -m experiments.run_ablation --quick --scenario formation_maze_stress_online --variant leader_only_planner --variant formation_aware_adaptive --variant formation_aware_lookahead_adaptive --output-dir outputs/ablation_formation_maze_stress
```

The generated `summary.csv` and `report.md` are the primary artifacts for comparing modes.

## Out Of Scope

- C++ parity for the new maps is not part of this first design unless the Python map workflow is stable.
- New planner algorithms are not required.
- New visualization UI is not required; existing markdown/CSV report output is sufficient.
