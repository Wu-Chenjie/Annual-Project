# A5 Dynamic Obstacle Decay Design

## Goal

Add an optional classified decay policy for sensor-discovered obstacles so short-lived noise clears quickly while repeatedly observed dynamic obstacles persist long enough for online replanning to react.

## Scope

This is an A-line replanner robustness increment. It touches the online sensor grid update path in Python and mirrors the public/config surface in C++. It does not change obstacle geometry, planner search, or default behavior.

## Design

`WindowReplanner` keeps the existing uniform TTL behavior by default. A new opt-in classifier tracks per-cell sensor hit counts and a compact class code:

- `0`: none
- `1`: transient, assigned to newly observed sensor obstacles
- `2`: persistent, assigned after a cell is observed at least `sensor_obstacle_persistent_hits` times

When classification is enabled, transient cells use `sensor_obstacle_ttl_steps`; persistent cells use `sensor_obstacle_persistent_ttl_steps`. Clearing or static-cell reconciliation resets class and hit count. The policy is intentionally simple because the current sensor abstraction only reports six range rays, not object IDs or velocities.

## Python Integration

`core/planning/replanner.py` owns the classifier arrays beside `_sensor_ttl` and `_sensor_occupied`. Existing tests must keep passing with classification disabled. New tests verify:

- a single transient hit expires with the base TTL
- repeated hits promote a cell to persistent and use the longer TTL
- static cells still never get cleared by sensor decay

## C++ Synchronization

The C++ `WindowReplanner` exposes the same opt-in surface and internal arrays. `ObstacleConfig` gets matching fields so the scenario layer can enable the policy. Static sync tests cover the C++ surface and CMake build verifies the header-only replanner changes.

## Acceptance

- Focused Python replanner tests pass.
- C++ static sync tests pass.
- C++ Release build passes.
- A-line non-slow regression passes.
