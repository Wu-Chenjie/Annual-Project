from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.safety_profiles import get_safety_profile, SAFETY_PROFILES


def test_safety_profiles_registry_contains_expected_profiles():
    assert {"indoor_demo", "indoor_research", "low_altitude_regulatory"} <= set(SAFETY_PROFILES)


def test_indoor_profile_preserves_indoor_scale():
    profile = get_safety_profile("indoor_demo")
    assert profile.scenario_scale == "indoor"
    assert profile.horizontal_separation < 5.0


def test_regulatory_profile_is_not_indoor_scale():
    profile = get_safety_profile("low_altitude_regulatory")
    assert profile.scenario_scale == "low_altitude"
    assert profile.horizontal_separation >= 30.0
