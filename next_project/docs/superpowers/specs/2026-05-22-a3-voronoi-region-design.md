# A3 Voronoi Region Prototype Design

## Scope

This spec covers A3 only: a lightweight local Voronoi-region prototype for stabilizing online subgoal side selection in dense multi-obstacle scenes.

It does not implement full 3D Voronoi diagrams, NMPC control, dynamic obstacle class decay, or fault-topology metrics.

## Baseline

`WindowReplanner` already chooses subgoals through:

- global reference points inside the local horizon,
- straight-line samples toward the task goal,
- free-frontier BFS candidates near local occupied cells,
- `_score_subgoal()` using progress, clearance, global-reference distance, and distance penalty.

The current scoring has no memory of which obstacle side was previously chosen. In symmetric or near-symmetric obstacle layouts, small score changes can flip the selected candidate from one side to the other.

## Design Goals

1. Reduce side-switching jitter in multi-obstacle online replanning.
2. Keep the feature experimental and easy to disable.
3. Use 2D horizontal projection only, matching the A3 fallback requirement.
4. Avoid new heavy dependencies such as SciPy Voronoi construction.
5. Preserve existing candidate filtering and safety gates.

## Approach

Add `core/planning/voronoi_region.py` with a small helper that derives a stable region signature from local obstacle centers:

- Project pose, goal, candidate, and obstacle centers to XY.
- Build an obstacle-center list from occupied grid cells or simple obstacle objects when available.
- Compute the nearest obstacle pair around the candidate and derive which side of the pose-to-goal axis the candidate occupies.
- Return a `VoronoiRegionScore` with:
  - `enabled`
  - `region_id`
  - `side`
  - `stability_bonus`
  - `obstacle_count`

The term "Voronoi" is used as a local nearest-obstacle partition proxy, not a full diagram. The public API should make this clear.

## Replanner Integration

Extend `WindowReplanner.__init__()` with:

```python
voronoi_region_enabled: bool = False
voronoi_region_weight: float = 0.25
```

Runtime state:

```python
self._last_voronoi_region_id: str | None = None
self._last_voronoi_side: int = 0
```

Scoring:

- `_score_subgoal()` keeps its current base score.
- If enabled, add `voronoi_region_weight * stability_bonus`.
- A candidate gets positive bonus when it stays in the same region/side as the previous accepted subgoal.
- A candidate gets a small penalty when it crosses to the opposite side without a progress or clearance reason.
- If fewer than two local obstacle centers are available, the score contribution is zero.

Selection memory:

- `_compute_subgoal()` updates the stored region after it selects a candidate.
- Direct goal returns do not force a region update.

## Error Handling

- Missing obstacle field, empty grid, or out-of-bounds cells returns a disabled score.
- Degenerate pose-to-goal vectors use the previous side if present, otherwise zero.
- Any extraction failure falls back to zero contribution and does not fail replanning.

## Testing

Python unit tests:

- `tests/test_voronoi_region.py`
  - same-side candidates receive a larger stability bonus than opposite-side candidates.
  - insufficient obstacles disables the score.
- `tests/test_replanner_subgoal.py`
  - with `voronoi_region_enabled=True`, repeated near-symmetric candidate scoring keeps the same side.
  - with the feature disabled, existing subgoal tests keep current behavior.

Regression commands:

```powershell
python -m pytest tests/test_voronoi_region.py tests/test_replanner_subgoal.py -q
python -m pytest tests/test_replanner_semantics.py tests/test_replanner_danger_quality.py -q
python -m pytest tests/test_obstacle_scenario.py -q -k "online_mode_preserves_task_waypoint_semantics or terminal_hold_reduces_end_jitter"
```

## Acceptance Criteria

- A3 can be enabled through `WindowReplanner` without changing planner APIs.
- Stable-side scoring is deterministic and unit-tested.
- Existing subgoal behavior remains unchanged when disabled.
- No new non-standard dependencies are introduced.
