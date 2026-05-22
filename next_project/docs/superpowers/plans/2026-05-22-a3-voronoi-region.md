# A3 Voronoi Region Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a lightweight 2D Voronoi-region stability score to online subgoal selection.

**Architecture:** Keep the Voronoi proxy in a standalone `core/planning/voronoi_region.py` helper with no new dependencies. `WindowReplanner` remains responsible for candidate filtering and only adds the helper's stability score when explicitly enabled.

**Tech Stack:** Python, NumPy, pytest, existing `WindowReplanner` and `OccupancyGrid`.

---

## File Structure

- Create: `core/planning/voronoi_region.py`
  - Defines `VoronoiRegionScore` and `VoronoiRegionSelector`.
- Modify: `core/planning/replanner.py`
  - Adds optional config, stores last selected region, and applies stability bonus.
- Modify: `core/planning/__init__.py`
  - Exports helper classes.
- Create: `tests/test_voronoi_region.py`
  - Unit tests for score behavior.
- Modify: `tests/test_replanner_subgoal.py`
  - Integration-style unit tests for enabled and disabled replanner behavior.

---

### Task 1: Voronoi Region Helper

**Files:**
- Create: `tests/test_voronoi_region.py`
- Create: `core/planning/voronoi_region.py`

- [ ] **Step 1: Write failing helper tests**

Create `tests/test_voronoi_region.py`:

```python
from __future__ import annotations

import numpy as np

from core.planning.voronoi_region import VoronoiRegionSelector


def test_voronoi_region_prefers_previous_side_for_same_region():
    selector = VoronoiRegionSelector(weight=0.5)
    pose = np.array([0.0, 0.0, 1.0])
    goal = np.array([8.0, 0.0, 1.0])
    obstacles = np.array([[3.0, -1.0, 1.0], [3.0, 1.0, 1.0], [5.0, -1.0, 1.0], [5.0, 1.0, 1.0]])

    upper = selector.score(pose, goal, np.array([4.0, 1.5, 1.0]), obstacles, previous_side=1)
    lower = selector.score(pose, goal, np.array([4.0, -1.5, 1.0]), obstacles, previous_side=1)

    assert upper.enabled is True
    assert upper.region_id == lower.region_id
    assert upper.stability_bonus > lower.stability_bonus
    assert upper.side == 1
    assert lower.side == -1
```

Add a second test:

```python
def test_voronoi_region_disables_when_obstacles_are_insufficient():
    selector = VoronoiRegionSelector()
    result = selector.score(
        np.array([0.0, 0.0, 1.0]),
        np.array([8.0, 0.0, 1.0]),
        np.array([4.0, 1.0, 1.0]),
        np.array([[3.0, 0.0, 1.0]]),
    )

    assert result.enabled is False
    assert result.stability_bonus == 0.0
```

- [ ] **Step 2: Verify tests fail**

Run:

```powershell
python -m pytest tests/test_voronoi_region.py -q
```

Expected: import failure for `core.planning.voronoi_region`.

- [ ] **Step 3: Implement helper**

Create `core/planning/voronoi_region.py` with:

```python
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VoronoiRegionScore:
    enabled: bool
    region_id: str
    side: int
    stability_bonus: float
    obstacle_count: int


class VoronoiRegionSelector:
    def __init__(self, *, weight: float = 0.25):
        self.weight = max(float(weight), 0.0)

    def score(
        self,
        pose: np.ndarray,
        goal: np.ndarray,
        candidate: np.ndarray,
        obstacle_centers: np.ndarray,
        *,
        previous_region_id: str | None = None,
        previous_side: int = 0,
    ) -> VoronoiRegionScore:
        centers = np.asarray(obstacle_centers, dtype=float)
        if centers.ndim != 2 or centers.shape[1] < 2 or len(centers) < 2:
            return VoronoiRegionScore(False, "", 0, 0.0, int(len(centers)) if centers.ndim else 0)
        pose_xy = np.asarray(pose, dtype=float)[:2]
        goal_xy = np.asarray(goal, dtype=float)[:2]
        cand_xy = np.asarray(candidate, dtype=float)[:2]
        centers_xy = centers[:, :2]
        distances = np.linalg.norm(centers_xy - cand_xy, axis=1)
        nearest = np.argsort(distances)[:2]
        pair = tuple(sorted(int(i) for i in nearest))
        region_id = f"{pair[0]}:{pair[1]}"
        axis = goal_xy - pose_xy
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1e-9:
            side = int(np.sign(previous_side))
        else:
            rel = cand_xy - pose_xy
            cross_z = axis[0] * rel[1] - axis[1] * rel[0]
            side = 1 if cross_z > 1e-9 else -1 if cross_z < -1e-9 else 0
        bonus = 0.0
        if previous_region_id and previous_region_id == region_id and previous_side and side:
            bonus = self.weight if side == previous_side else -self.weight
        elif previous_side and side:
            bonus = 0.5 * self.weight if side == previous_side else -0.5 * self.weight
        return VoronoiRegionScore(True, region_id, side, float(bonus), len(centers))
```

- [ ] **Step 4: Verify helper tests pass**

Run:

```powershell
python -m pytest tests/test_voronoi_region.py -q
```

Expected: all pass.

---

### Task 2: WindowReplanner Integration

**Files:**
- Modify: `tests/test_replanner_subgoal.py`
- Modify: `core/planning/replanner.py`
- Modify: `core/planning/__init__.py`

- [ ] **Step 1: Write failing replanner tests**

Add to `tests/test_replanner_subgoal.py`:

```python
def test_voronoi_region_score_biases_replanner_toward_previous_side():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(12, 12, 4))
    grid.data[3, 1, 0] = 1
    grid.data[3, 3, 0] = 1
    replanner = WindowReplanner(
        planner=DummyPlanner(),
        grid=grid,
        horizon=6.0,
        voronoi_region_enabled=True,
        voronoi_region_weight=0.5,
    )
    replanner._last_voronoi_side = 1
    pose = np.array([0.0, 2.0, 0.0], dtype=float)
    goal = np.array([8.0, 2.0, 0.0], dtype=float)
    upper = np.array([4.0, 3.0, 0.0], dtype=float)
    lower = np.array([4.0, 1.0, 0.0], dtype=float)

    assert replanner._score_subgoal(pose, goal, upper) > replanner._score_subgoal(pose, goal, lower)
```

Add:

```python
def test_voronoi_region_disabled_keeps_existing_score_shape():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(12, 12, 4))
    replanner = WindowReplanner(planner=DummyPlanner(), grid=grid, horizon=6.0)
    pose = np.array([0.0, 2.0, 0.0], dtype=float)
    goal = np.array([8.0, 2.0, 0.0], dtype=float)
    candidate = np.array([4.0, 3.0, 0.0], dtype=float)

    assert isinstance(replanner._score_subgoal(pose, goal, candidate), float)
```

- [ ] **Step 2: Verify replanner test fails**

Run:

```powershell
python -m pytest tests/test_replanner_subgoal.py::test_voronoi_region_score_biases_replanner_toward_previous_side -q
```

Expected: `TypeError` for unexpected `voronoi_region_enabled`.

- [ ] **Step 3: Integrate helper**

In `core/planning/replanner.py`:

- Import `VoronoiRegionSelector`.
- Add `voronoi_region_enabled` and `voronoi_region_weight` parameters to `__init__`.
- Initialize:

```python
self.voronoi_region_enabled = bool(voronoi_region_enabled)
self.voronoi_region_selector = VoronoiRegionSelector(weight=voronoi_region_weight)
self._last_voronoi_region_id: str | None = None
self._last_voronoi_side: int = 0
```

- Add `_local_obstacle_centers()` that returns occupied grid cell centers within horizon.
- In `_score_subgoal()`, add the helper's `stability_bonus` when enabled.
- In `_compute_subgoal()`, after choosing `best[1]`, update last region from the selected candidate.

- [ ] **Step 4: Export helper**

In `core/planning/__init__.py`, add:

```python
from .voronoi_region import VoronoiRegionScore, VoronoiRegionSelector
```

and add both names to `__all__`.

- [ ] **Step 5: Verify focused tests pass**

Run:

```powershell
python -m pytest tests/test_voronoi_region.py tests/test_replanner_subgoal.py -q
```

Expected: all pass.

---

### Task 3: Regression

**Files:**
- No new files unless a regression appears.

- [ ] **Step 1: Run replanner regression**

Run:

```powershell
python -m pytest tests/test_replanner_semantics.py tests/test_replanner_danger_quality.py -q
```

Expected: all pass.

- [ ] **Step 2: Run A-line online behavior regression**

Run:

```powershell
python -m pytest tests/test_obstacle_scenario.py -q -k "online_mode_preserves_task_waypoint_semantics or terminal_hold_reduces_end_jitter"
```

Expected: selected tests pass.

- [ ] **Step 3: Run non-slow suite**

Run:

```powershell
python -m pytest -m "not slow" -q
```

Expected: all non-slow tests pass, or record exact pre-existing skips.

---

## Self-Review Notes

- Spec coverage: helper, integration, disabled fallback, and regressions are all covered.
- Type consistency: public names are `VoronoiRegionScore` and `VoronoiRegionSelector`.
- Scope: no C++ work is included because A3 is an experimental Python online-replanner scoring prototype.
