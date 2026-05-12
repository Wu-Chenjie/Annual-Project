from __future__ import annotations

from dataclasses import dataclass

from simulations.formation_simulation import SimulationConfig


@dataclass(frozen=True)
class AblationVariant:
    name: str
    description: str
    overrides: dict[str, object]


ABLATION_VARIANTS: dict[str, AblationVariant] = {
    "baseline": AblationVariant(
        name="baseline",
        description="Use scenario defaults.",
        overrides={},
    ),
    "no_esdf": AblationVariant(
        name="no_esdf",
        description="Disable ESDF-aware soft cost while preserving hard collision checks.",
        overrides={"planner_esdf_aware": False},
    ),
    "no_gnn": AblationVariant(
        name="no_gnn",
        description="Disable GNN danger-mode routing.",
        overrides={"danger_mode_enabled": False},
    ),
    "no_apf": AblationVariant(
        name="no_apf",
        description="Disable enhanced APF profile.",
        overrides={"apf_paper1_profile": "off"},
    ),
    "no_formation_envelope": AblationVariant(
        name="no_formation_envelope",
        description="Disable formation envelope expansion in planning.",
        overrides={"planner_use_formation_envelope": False},
    ),
    "leader_only_planner": AblationVariant(
        name="leader_only_planner",
        description="Plan and gate only the leader centerline; follower obstacle clearance is evaluated but not used as a planning constraint.",
        overrides={
            "planner_use_formation_envelope": False,
            "formation_safety_enabled": False,
            "plan_clearance_extra": 0.0,
        },
    ),
    "formation_aware_clearance": AblationVariant(
        name="formation_aware_clearance",
        description="Use the explicit formation-aware clearance policy for leader and follower-offset safety margins.",
        overrides={
            "planner_use_formation_envelope": True,
            "formation_safety_enabled": True,
            "plan_clearance_extra": 0.25,
        },
    ),
    "formation_aware_static": AblationVariant(
        name="formation_aware_static",
        description="Use formation-aware clearance with a fixed formation.",
        overrides={
            "planner_use_formation_envelope": True,
            "formation_safety_enabled": True,
            "formation_adaptation_enabled": False,
            "plan_clearance_extra": 0.25,
        },
    ),
    "formation_aware_adaptive": AblationVariant(
        name="formation_aware_adaptive",
        description="Use formation-aware clearance plus adaptive formation switching in narrow or unsafe regions.",
        overrides={
            "planner_use_formation_envelope": True,
            "formation_safety_enabled": True,
            "formation_adaptation_enabled": True,
            "plan_clearance_extra": 0.25,
        },
    ),
    "with_trajectory_optimizer": AblationVariant(
        name="with_trajectory_optimizer",
        description="Enable trajectory post-processing and select the accepted candidate with the lowest snap-squared proxy cost.",
        overrides={
            "trajectory_optimizer_enabled": True,
            "trajectory_optimizer_method": "min_snap_proxy",
        },
    ),
}


def list_variants() -> list[AblationVariant]:
    return [ABLATION_VARIANTS[name] for name in sorted(ABLATION_VARIANTS)]


def apply_variant(config: SimulationConfig, variant_name: str) -> SimulationConfig:
    if variant_name not in ABLATION_VARIANTS:
        available = ", ".join(sorted(ABLATION_VARIANTS))
        raise ValueError(f"unknown ablation variant {variant_name!r}; available: {available}")
    variant = ABLATION_VARIANTS[variant_name]
    for key, value in variant.overrides.items():
        if not hasattr(config, key):
            raise AttributeError(f"SimulationConfig has no field {key!r}")
        setattr(config, key, value)
    return config


__all__ = ["ABLATION_VARIANTS", "AblationVariant", "apply_variant", "list_variants"]
