from __future__ import annotations

from dataclasses import dataclass

from config import get_config
from simulations.formation_simulation import SimulationConfig


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    category: str
    description: str
    default_max_time: float | None = None


SCENARIOS: dict[str, ScenarioSpec] = {
    "basic": ScenarioSpec(
        name="basic",
        category="formation",
        description="Obstacle-free formation control baseline.",
    ),
    "obstacle": ScenarioSpec(
        name="obstacle",
        category="offline",
        description="Simple three-pillar obstacle avoidance baseline.",
    ),
    "obstacle_unknown": ScenarioSpec(
        name="obstacle_unknown",
        category="unknown",
        description="Simple obstacle fully unknown — sensor-discovered pillars at cm precision.",
    ),
    "warehouse": ScenarioSpec(
        name="warehouse",
        category="offline",
        description="Warehouse offline/standard obstacle scenario for trajectory metrics.",
    ),
    "warehouse_a": ScenarioSpec(
        name="warehouse_a",
        category="offline",
        description="Warehouse A* variant with ESDF and danger mode.",
    ),
    "warehouse_online": ScenarioSpec(
        name="warehouse_online",
        category="online",
        description="Warehouse online replanning baseline.",
    ),
    "warehouse_danger": ScenarioSpec(
        name="warehouse_danger",
        category="online",
        description="Warehouse online replanning with GNN danger mode and conservative APF.",
    ),
    "warehouse_unknown": ScenarioSpec(
        name="warehouse_unknown",
        category="unknown",
        description="Warehouse fully unknown — empty initial map, sensor-discovered shelves and aisles.",
    ),
    "warehouse_a_unknown": ScenarioSpec(
        name="warehouse_a_unknown",
        category="unknown",
        description="Warehouse A* fully unknown variant.",
    ),
    "warehouse_online_unknown": ScenarioSpec(
        name="warehouse_online_unknown",
        category="unknown",
        description="Warehouse online fully unknown — 3-waypoint unknown exploration.",
    ),
    "warehouse_danger_unknown": ScenarioSpec(
        name="warehouse_danger_unknown",
        category="unknown",
        description="Warehouse danger fully unknown — sensor discovery without GNN dual mode.",
    ),
    "fault_tolerance": ScenarioSpec(
        name="fault_tolerance",
        category="offline",
        description="Fault tolerance offline with fault injection and topology reconfiguration.",
    ),
    "fault_tolerance_online": ScenarioSpec(
        name="fault_tolerance_online",
        category="online",
        description="Fault tolerance online with fault injection and topology reconfiguration.",
    ),
    "fault_tolerance_unknown": ScenarioSpec(
        name="fault_tolerance_unknown",
        category="unknown",
        description="Fault tolerance fully unknown — sensor discovery + fault injection + topology reconfig.",
    ),
    "fault_tolerance_online_unknown": ScenarioSpec(
        name="fault_tolerance_online_unknown",
        category="unknown",
        description="Fault tolerance online fully unknown with adaptive replan and fault injection.",
    ),
    "school_corridor": ScenarioSpec(
        name="school_corridor",
        category="offline",
        description="School corridor offline A* with GNN danger mode.",
    ),
    "school_corridor_online": ScenarioSpec(
        name="school_corridor_online",
        category="online",
        description="Narrow school corridor online replanning scenario.",
    ),
    "school_corridor_unknown": ScenarioSpec(
        name="school_corridor_unknown",
        category="unknown",
        description="School corridor fully unknown — sensor-discovered narrow passage and L-turn.",
    ),
    "school_corridor_online_unknown": ScenarioSpec(
        name="school_corridor_online_unknown",
        category="unknown",
        description="School corridor online fully unknown with adaptive replan interval.",
    ),
    "company_cubicles": ScenarioSpec(
        name="company_cubicles",
        category="offline",
        description="Company cubicles offline Hybrid A* with partition matrix.",
    ),
    "company_cubicles_online": ScenarioSpec(
        name="company_cubicles_online",
        category="online",
        description="Company cubicles online Hybrid A* scenario with partition overflight.",
    ),
    "company_cubicles_unknown": ScenarioSpec(
        name="company_cubicles_unknown",
        category="unknown",
        description="Company cubicles fully unknown — sensor-discovered partition matrix and overflight.",
    ),
    "company_cubicles_online_unknown": ScenarioSpec(
        name="company_cubicles_online_unknown",
        category="unknown",
        description="Company cubicles online fully unknown with Hybrid A*.",
    ),
    "meeting_room": ScenarioSpec(
        name="meeting_room",
        category="offline",
        description="Meeting room offline obstacle scenario for trajectory metrics.",
    ),
    "meeting_room_online": ScenarioSpec(
        name="meeting_room_online",
        category="online",
        description="Meeting room online obstacle avoidance scenario.",
    ),
    "meeting_room_unknown": ScenarioSpec(
        name="meeting_room_unknown",
        category="unknown",
        description="Meeting room fully unknown — sensor-discovered oval table and chairs at cm precision.",
    ),
    "meeting_room_online_unknown": ScenarioSpec(
        name="meeting_room_online_unknown",
        category="unknown",
        description="Meeting room online fully unknown with cm-level sensor discovery.",
    ),
    "rrt_dual_channel_online": ScenarioSpec(
        name="rrt_dual_channel_online",
        category="online",
        description="Dual-channel RRT lookahead escape and formation adaptation scenario.",
    ),
    "rrt_dual_channel_online_unknown": ScenarioSpec(
        name="rrt_dual_channel_online_unknown",
        category="unknown",
        description="RRT dual-channel fully unknown — sensor-discovered channels without formation adaptation.",
    ),
    "formation_maze_stress_online": ScenarioSpec(
        name="formation_maze_stress_online",
        category="online",
        description="Maze-like formation stress test with false branches, narrow gates, and RRT escape.",
    ),
    "formation_maze_stress_online_unknown": ScenarioSpec(
        name="formation_maze_stress_online_unknown",
        category="unknown",
        description="Maze stress fully unknown — sensor-discovered narrow gates and false branches.",
    ),
    "unknown_map_online": ScenarioSpec(
        name="unknown_map_online",
        category="unknown",
        description="Unknown-map online exploration with an empty initial planner map and sensor-discovered obstacles.",
    ),
    "laboratory": ScenarioSpec(
        name="laboratory",
        category="offline",
        description="Laboratory offline A* with multi-height obstacles.",
    ),
    "laboratory_online": ScenarioSpec(
        name="laboratory_online",
        category="online",
        description="Laboratory online Hybrid A* scenario.",
    ),
    "laboratory_unknown": ScenarioSpec(
        name="laboratory_unknown",
        category="unknown",
        description="Laboratory fully unknown — sensor-discovered benches, fume hoods, and reagent shelves.",
    ),
    "laboratory_online_unknown": ScenarioSpec(
        name="laboratory_online_unknown",
        category="unknown",
        description="Laboratory online fully unknown with Hybrid A*.",
    ),
}


def list_scenarios(category: str | None = None) -> list[ScenarioSpec]:
    specs = sorted(SCENARIOS.values(), key=lambda spec: spec.name)
    if category is None:
        return specs
    return [spec for spec in specs if spec.category == category]


def get_scenario_config(name: str, *, quick: bool = False) -> SimulationConfig:
    if name not in SCENARIOS:
        available = ", ".join(sorted(SCENARIOS))
        raise ValueError(f"unknown scenario {name!r}; available: {available}")
    cfg = get_config(name)
    if quick:
        cfg.max_sim_time = min(float(cfg.max_sim_time), 5.0)
        cfg.planner_horizon = min(float(cfg.planner_horizon), 3.0)
    return cfg


__all__ = ["SCENARIOS", "ScenarioSpec", "get_scenario_config", "list_scenarios"]
