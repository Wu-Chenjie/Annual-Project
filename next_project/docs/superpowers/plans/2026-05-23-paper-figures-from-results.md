# Paper Figures From Results Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate traceable paper figures from existing reproducible result artifacts and insert them into the IEEE paper.

**Architecture:** Use existing CSV/JSON outputs as the only quantitative source. Generate English PNG/PDF figures into `next_project/论文/figures/`, then reference them from `next_project/论文/uav_formation_simulation_paper.tex`.

**Tech Stack:** Python, matplotlib, pandas/csv/json, IEEEtran LaTeX.

---

### Figure Requirements

- Every generated figure must expose traceable data, not decorative shapes.
- Quantitative panels must include x-axis labels, y-axis labels, tick marks, units where applicable, and value labels or legends that identify the plotted data.
- Use IEEE-style figure formatting:
  - Prefer vector PDF for LaTeX inclusion, with matching PNG copies for quick inspection.
  - Use single-column width for compact figures, about 3.45 in wide, and double-column width for dense comparison figures, about 7.16 in wide.
  - Keep plot fonts readable at IEEE scale: roughly 8--10 pt labels, 7--8 pt tick labels and legends.
  - Use captions and labels in LaTeX, not large title text embedded inside the image.
  - Avoid heavy grid backgrounds; use light y-axis grid lines only where they help read values.
  - Keep line widths and marker sizes visible after PDF scaling.
- The architecture figure should be a two-dimensional pipeline matrix rather than a free-form flowchart:
  - x-axis: pipeline stage, such as Runtime, Result Schema, Report, Benchmark.
  - y-axis: execution line, such as Python, C++, Web.
  - cells: concrete artifact names or field groups, for example `sim_result.json`, `benchmark_results.json`, `report.md`, `summary.csv`.
- Do not invent values. If a metric is unavailable or null in the source artifact, either omit that bar/point or annotate it as unavailable.

### Lightweight Checklist

- [x] **Step 1: Inspect available CSV/JSON metrics**

Available direct sources:

- `next_project/outputs/ablation_m2_trajectory_snap/summary.csv`
  - Rows: 2
  - Fields: scenario, variant, completed_waypoint_count, collision_count, replan_count, planned_path_length, executed_path_length, trajectory_path_length, trajectory_max_speed, trajectory_max_acceleration, trajectory_mean_jerk, trajectory_max_jerk, trajectory_jerk_squared_integral, trajectory_snap_squared_integral, mean_error_overall, max_error_overall, final_error_overall, min_inter_drone_distance, downwash_hits.
- `next_project/outputs/ablation_formation_clearance/summary.csv`
  - Rows: 4
  - Adds formation clearance fields: mode, required clearance, min margins, violation counts, posthoc margins.
- `next_project/outputs/ablation_rrt_dual_channel/summary.csv`
  - Rows: 3
  - Adds adaptation/lookahead fields: formation_adaptation_count, formation_adaptation_last, lookahead_reference_blocked_count, rrt_escape_attempt_count, rrt_escape_accepted_count, rrt_escape_failed_count.
- `next_project/outputs/ablation_formation_maze_stress/summary.csv`
  - Rows: 3
  - Same method-comparison field family as RRT dual-channel, with a maze-stress scenario.
- `next_project/outputs/planner_compare_smoke/summary.csv`
  - Rows: 2
  - Adds planner and planning_wall_time_s for A*/Dijkstra smoke comparison.
- `next_project/outputs/benchmark_default/20260513-204859/benchmark_results.json`
  - Schema-bearing benchmark output with runtime_mean_s, runtime_std_s, mean/max/final error arrays, completed_waypoint_count per record, and seed.
- `next_project/outputs/unknown_map_online/unknown_map_verify/metrics.json`
  - Representative run result: python, runtime_s, completed_waypoint_count, collision_count, replan_count, mean_error_overall, path lengths, jerk/snap proxies.
- `next_project/outputs/rrt_dual_channel_online_unknown/debug_py_rrt/metrics.json`
  - Stress/debug run result with nonzero collision_count and replan_count; useful as a contrast case, not as a success claim.
- `next_project/outputs/obstacle/report-check/metrics.json`
  - Small report-check run with runtime, waypoint completion, collision count, and tracking error; trajectory jerk/snap fields are null.

- [x] **Step 2: Generate English figures**

Generate:

- `architecture_pipeline.png` and `.pdf`: a 2D pipeline matrix with x/y axes for Python, C++, Web execution lines plus schema/report/benchmark outputs.
- `method_comparison.png` and `.pdf`: axis-labeled bar/line panels for baseline/trajectory optimizer/formation-aware/lookahead adaptive comparisons using the available ablation summary CSVs.
- `run_result.png` and `.pdf`: axis-labeled bar/line panels for waypoint completion, collision count, baseline tracking error, benchmark runtime/error from metrics JSON and benchmark JSON.

- [x] **Step 3: Insert figures into IEEE TeX**

Modify `next_project/论文/uav_formation_simulation_paper.tex`:

- Add `figure*` near Overall Architecture for the pipeline diagram.
- Add `figure*` in Results near Scenario-Level Results for method comparison.
- Add `figure` or `figure*` in Results for run result metrics.

- [x] **Step 4: Compile and inspect PDF**

Run LaTeX from `next_project/论文` and check that the generated PDF includes all figures without missing-file or overfull layout problems.

Verification performed:

- `python -m experiments.generate_paper_figures`
- `pdflatex -interaction=nonstopmode -halt-on-error uav_formation_simulation_paper.tex` twice from `next_project/论文`
- Poppler `pdftoppm` rendered pages 4, 6, and 7 to `next_project/tmp/pdfs/pdftoppm-page-04.png`, `pdftoppm-page-06.png`, and `pdftoppm-page-07.png`
