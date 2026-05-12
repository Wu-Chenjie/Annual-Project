# Formation Adaptation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicit formation adaptation module so strict formation-aware clearance can switch to a smaller safe formation in narrow scenes and report those decisions.

**Architecture:** Introduce `core/formation_adaptation.py` as a pure policy module. `ObstacleScenarioSimulation` feeds it channel width and clearance status, applies decisions through `FormationTopology`, rebuilds the planning grid, and records `formation_adaptation_events`. Experiments add static vs adaptive formation-aware variants.

**Tech Stack:** Python 3.14, NumPy, existing `FormationTopology`, `FormationClearancePolicy`, pytest, existing ablation/report writer pipeline.

---

### Task 1: Policy Module

**Files:**
- Create: `next_project/core/formation_adaptation.py`
- Modify: `next_project/core/__init__.py`
- Test: `next_project/tests/test_formation_adaptation.py`

- [ ] **Step 1: Write the failing test**

```python
def test_policy_selects_line_when_current_formation_is_too_wide():
    policy = FormationAdaptationPolicy(
        formations=("diamond", "line"),
        default_formation="diamond",
        min_hold_time_s=0.0,
    )
    decision = policy.decide(
        time_now=1.0,
        current_formation="diamond",
        channel_width=(0.8, 10.0, 3.0),
        envelope_by_formation={
            "diamond": (0.9, 1.2, 0.2),
            "line": (0.2, 1.2, 0.2),
        },
        clearance_margin=-0.1,
    )
    assert decision.should_switch
    assert decision.target_formation == "line"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_formation_adaptation.py -q`
Expected: import failure for `core.formation_adaptation`.

- [ ] **Step 3: Implement minimal policy**

Create dataclasses `FormationAdaptationDecision` and `FormationAdaptationPolicy`. Choose the first candidate whose per-axis envelope fits `channel_width`; prefer the current formation when it is safe; return `line` when no candidate fits.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_formation_adaptation.py -q`
Expected: all tests pass.

### Task 2: Simulation Integration

**Files:**
- Modify: `next_project/simulations/obstacle_scenario.py`
- Test: `next_project/tests/test_obstacle_scenario.py`

- [ ] **Step 1: Write the failing test**

```python
def test_formation_adaptation_event_is_reported_when_channel_is_narrow():
    cfg = SimulationConfig(..., formation_adaptation_enabled=True, planner_use_formation_envelope=True)
    sim = ObstacleScenarioSimulation(cfg)
    event = sim._apply_formation_adaptation(1.0, (0.3, 40.0, 5.0), -0.2)
    assert event is not None
    assert event["to"] == "line"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_formation_adaptation_event_is_reported_when_channel_is_narrow -q`
Expected: attribute failure for `_apply_formation_adaptation`.

- [ ] **Step 3: Implement integration**

Add config fields, instantiate policy in `ObstacleScenarioSimulation`, replace direct `topology.auto_shrink()` with `_apply_formation_adaptation()`, rebuild grid on switch, and return `formation_adaptation_events` in `run()`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_obstacle_scenario.py::test_formation_adaptation_event_is_reported_when_channel_is_narrow -q`
Expected: pass.

### Task 3: Experiment and Reporting

**Files:**
- Modify: `next_project/experiments/ablation.py`
- Modify: `next_project/experiments/metrics_extractor.py`
- Modify: `next_project/experiments/report_writer.py`
- Test: `next_project/tests/test_experiment_workflow.py`
- Test: `next_project/tests/test_metrics_extractor.py`

- [ ] **Step 1: Write failing tests**

Assert `formation_aware_static` disables adaptation and `formation_aware_adaptive` enables it. Assert metrics include `formation_adaptation_count` and `formation_adaptation_last`.

- [ ] **Step 2: Run tests to verify failure**

Run: `python -m pytest tests/test_experiment_workflow.py tests/test_metrics_extractor.py -q`
Expected: missing variants or missing metrics.

- [ ] **Step 3: Implement experiment/report fields**

Add variants and extract event counts into CSV/Markdown.

- [ ] **Step 4: Run tests to verify pass**

Run: `python -m pytest tests/test_experiment_workflow.py tests/test_metrics_extractor.py -q`
Expected: pass.

### Task 4: Verification and Ablation

**Files:**
- Create: `next_project/outputs/ablation_formation_adaptation/formation_adaptation_ablation_zh.md`

- [ ] **Step 1: Run focused tests**

Run: `python -m pytest tests/test_formation_adaptation.py tests/test_obstacle_scenario.py::test_formation_adaptation_event_is_reported_when_channel_is_narrow tests/test_experiment_workflow.py tests/test_metrics_extractor.py -q`
Expected: pass.

- [ ] **Step 2: Run ablation**

Run: `python -m experiments.run_ablation --scenario meeting_room --scenario warehouse --variant formation_aware_static --variant formation_aware_adaptive --quick --output-dir outputs/ablation_formation_adaptation`
Expected: writes `summary.csv` and `report.md`.

- [ ] **Step 3: Run full tests**

Run: `python -m pytest -q`
Expected: all non-skipped tests pass.
