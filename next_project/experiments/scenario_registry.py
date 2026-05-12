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
    "warehouse_online": ScenarioSpec(
        name="warehouse_online",
        category="online",
        description="Warehouse online replanning baseline.",
    ),
    "warehouse": ScenarioSpec(
        name="warehouse",
        category="offline",
        description="Warehouse offline/standard obstacle scenario for trajectory metrics.",
    ),
    "warehouse_danger": ScenarioSpec(
        name="warehouse_danger",
        category="online",
        description="Warehouse online replanning with GNN danger mode and conservative APF.",
    ),
    "school_corridor_online": ScenarioSpec(
        name="school_corridor_online",
        category="online",
        description="Narrow school corridor online replanning scenario.",
    ),
    "meeting_room_online": ScenarioSpec(
        name="meeting_room_online",
        category="online",
        description="Meeting room online obstacle avoidance scenario.",
    ),
    "meeting_room": ScenarioSpec(
        name="meeting_room",
        category="offline",
        description="Meeting room offline obstacle scenario for trajectory metrics.",
    ),
    "laboratory_online": ScenarioSpec(
        name="laboratory_online",
        category="online",
        description="Laboratory online Hybrid A* scenario.",
    ),
    "basic": ScenarioSpec(
        name="basic",
        category="formation",
        description="Obstacle-free formation control baseline.",
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
