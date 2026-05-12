# Chinese Result Reporting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Chinese reporting pipeline that automatically writes Markdown and PNG result figures after each simulation, and can compare planner modes.

**Architecture:** Keep reporting under `experiments/` so simulation code only emits data and calls one reporting entry point. Add lightweight planning telemetry to simulation results, pass it through `sim_result.json`, then render Chinese figures and Markdown from that payload.

**Tech Stack:** Python, matplotlib, JSON, pytest, existing `experiments.metrics_extractor`, existing `main.run_with_config`.

---

### Task 1: Result Reporter

**Files:**
- Create: `experiments/result_reporter.py`
- Create: `experiments/report_results.py`
- Test: `tests/test_result_reporter.py`

- [x] Write tests requiring Chinese Markdown and PNG figures from a synthetic `sim_result.json`.
- [x] Implement `generate_result_report(input_json, output_dir=None, title=None)`.
- [x] Render route, 3D route, flight parameters, tracking errors, event timeline, and planner telemetry figures.
- [x] Add CLI `python -m experiments.report_results <sim_result.json>`.

### Task 2: Simulation Telemetry

**Files:**
- Modify: `simulations/formation_simulation.py`
- Modify: `simulations/obstacle_scenario.py`
- Modify: `core/result_schema.py`
- Test: `tests/test_result_schema.py`

- [x] Add `waypoint_events` to formation and obstacle simulations.
- [x] Add `planning_events` to obstacle simulations with planner, phase, sim time, wall time, point count, accepted status, and fallback reason.
- [x] Pass `planned_trajectory`, `planning_events`, `waypoint_events`, `task_waypoints`, and `replanned_waypoints` through standardized payloads.

### Task 3: Automatic Report Output

**Files:**
- Modify: `main.py`
- Test: `tests/test_result_reporter.py`

- [x] Add CLI flags `--no-report` and `--report-title`.
- [x] Generate `report.md` and `report_figures/*.png` after `sim_result.json` is written.
- [x] Print the report path in the command summary.

### Task 4: Planner Comparison

**Files:**
- Create: `experiments/planner_comparison.py`
- Test: `tests/test_planner_comparison.py`

- [x] Implement a Chinese scoring explanation that treats collision-free completion as the first priority.
- [x] Add CLI for repeated planner-kind runs on one preset.
- [x] Write `summary.csv`, `report.md`, and `规划模式对比.png`.

### Task 5: Verification

- [x] Run focused tests for result reporting and planner comparison.
- [x] Run existing experiment/report tests.
- [x] Run one short simulation and verify automatic report output exists.
