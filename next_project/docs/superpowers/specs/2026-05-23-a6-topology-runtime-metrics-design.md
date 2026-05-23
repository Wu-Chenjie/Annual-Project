# A6 Topology Runtime Metrics Design

## Context

A1/A2, A3, and A5 now expose trajectory quality, Voronoi stability, and dynamic obstacle decay behavior across Python and C++. The remaining A-line item is a low-risk topology observability enhancement: report finer fault-tolerance topology metrics without changing controller behavior.

## Goal

Add runtime topology metrics to Python and C++ simulation outputs so runs can report:

- algebraic connectivity (`lambda2`) over time;
- control energy proxies for leader and followers;
- fault and reconfiguration event counts;
- sample count and availability state.

## Non-Goals

- No topology controller changes.
- No new fault detector behavior.
- No expensive graph history serialization.
- No dependency on plotting or benchmark post-processing.

## Design

Create a small Python accumulator in `core/topology_metrics.py`. Each simulation step can pass current formation offsets and optional control vectors. The accumulator computes `TopologyGraph(offsets).algebraic_connectivity`, integrates squared control norms by `dt`, and emits a JSON-safe dictionary.

For C++, add a matching `TopologyRuntimeMetrics` struct and small helper behavior near simulation loops. Keep the contract aligned with Python keys:

- `available`
- `sample_count`
- `mean_algebraic_connectivity`
- `min_algebraic_connectivity`
- `final_algebraic_connectivity`
- `leader_control_energy_proxy`
- `follower_control_energy_proxy`
- `fault_event_count`
- `reconfiguration_event_count`

Obstacle scenarios should count fault/reconfiguration events from the existing fault log. Base formation simulations should report zero fault/reconfiguration counts.

## Error Handling

If offsets are unavailable or `lambda2` cannot be computed, the accumulator skips that sample and leaves `available` false until a valid sample arrives. Non-finite controls are ignored for energy contribution.

## Testing

Add focused Python tests for:

- lambda2 aggregation and energy proxy integration;
- fault and reconfiguration count extraction from dict and string log entries.

Extend existing static C++ sync tests to require the struct, result field, result writer keys, and scenario assignment. Run a C++ build after sync changes.
