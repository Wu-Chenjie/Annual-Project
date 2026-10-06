from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class SafetyProfile:
    """Safety profile converted from indoor demo or low-altitude standards."""

    name: str
    horizontal_separation: float
    vertical_separation: float
    horizontal_position_error: float
    vertical_position_error: float
    surveillance_period: float
    decision_time: float
    communication_delay: float
    navigation_source_redundancy: int
    scenario_scale: str
    note: str

    def to_dict(self) -> dict:
        return asdict(self)


SAFETY_PROFILES: dict[str, SafetyProfile] = {
    "indoor_demo": SafetyProfile(
        name="indoor_demo",
        horizontal_separation=0.8,
        vertical_separation=0.5,
        horizontal_position_error=0.03,
        vertical_position_error=0.02,
        surveillance_period=0.1,
        decision_time=0.2,
        communication_delay=0.05,
        navigation_source_redundancy=1,
        scenario_scale="indoor",
        note="Small indoor demo profile that preserves current map scale.",
    ),
    "indoor_research": SafetyProfile(
        name="indoor_research",
        horizontal_separation=1.2,
        vertical_separation=0.8,
        horizontal_position_error=0.05,
        vertical_position_error=0.03,
        surveillance_period=0.1,
        decision_time=0.3,
        communication_delay=0.08,
        navigation_source_redundancy=1,
        scenario_scale="indoor",
        note="Indoor research profile with slightly stricter separation and error budget.",
    ),
    "low_altitude_regulatory": SafetyProfile(
        name="low_altitude_regulatory",
        horizontal_separation=30.0,
        vertical_separation=15.0,
        horizontal_position_error=30.0,
        vertical_position_error=15.0,
        surveillance_period=1.0,
        decision_time=5.0,
        communication_delay=0.5,
        navigation_source_redundancy=2,
        scenario_scale="low_altitude",
        note="Regulatory low-altitude profile; not suitable for current indoor map scale.",
    ),
}


def get_safety_profile(name: str = "indoor_demo") -> SafetyProfile:
    try:
        return SAFETY_PROFILES[name]
    except KeyError as exc:
        raise ValueError(f"Unknown safety_profile: {name}") from exc
