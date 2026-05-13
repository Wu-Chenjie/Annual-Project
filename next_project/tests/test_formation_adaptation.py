from __future__ import annotations

from core.formation_adaptation import FormationAdaptationPolicy


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
        last_switch_time_s=-10.0,
    )

    assert decision.should_switch
    assert decision.target_formation == "line"
    assert decision.reason in {"channel_too_narrow", "clearance_violation"}


def test_policy_recovers_default_when_roomy_and_clearance_is_safe():
    policy = FormationAdaptationPolicy(
        formations=("diamond", "line"),
        default_formation="diamond",
        min_hold_time_s=0.0,
        recovery_clearance_margin=0.2,
    )

    decision = policy.decide(
        time_now=3.0,
        current_formation="line",
        channel_width=(6.0, 6.0, 3.0),
        envelope_by_formation={
            "diamond": (0.9, 1.2, 0.2),
            "line": (0.2, 1.2, 0.2),
        },
        clearance_margin=0.4,
        last_switch_time_s=-10.0,
    )

    assert decision.should_switch
    assert decision.target_formation == "diamond"
    assert decision.reason == "recover_default"


def test_policy_uses_smallest_overall_envelope_when_channel_is_unknown_and_clearance_violates():
    policy = FormationAdaptationPolicy(
        formations=("diamond", "line", "triangle"),
        default_formation="diamond",
        min_hold_time_s=0.0,
    )

    decision = policy.decide(
        time_now=1.0,
        current_formation="diamond",
        channel_width=None,
        envelope_by_formation={
            "diamond": (0.7, 1.2, 0.2),
            "line": (0.2, 1.7, 0.2),
            "triangle": (0.5, 0.8, 0.2),
        },
        clearance_margin=-0.1,
        last_switch_time_s=None,
    )

    assert decision.should_switch
    assert decision.target_formation == "triangle"


def test_policy_does_not_recover_default_without_clearance_evidence():
    policy = FormationAdaptationPolicy(
        formations=("diamond", "line"),
        default_formation="diamond",
        min_hold_time_s=0.0,
    )

    decision = policy.decide(
        time_now=3.0,
        current_formation="line",
        channel_width=(10.0, 10.0, 3.0),
        envelope_by_formation={
            "diamond": (0.7, 1.2, 0.2),
            "line": (0.2, 1.7, 0.2),
        },
        clearance_margin=None,
        last_switch_time_s=-10.0,
    )

    assert not decision.should_switch
    assert decision.target_formation == "line"


def test_policy_returns_spacing_for_selected_safe_option():
    policy = FormationAdaptationPolicy(
        formations=("line", "triangle"),
        default_formation="triangle",
        min_hold_time_s=0.0,
    )

    decision = policy.decide(
        time_now=1.0,
        current_formation="triangle",
        channel_width=None,
        envelope_by_formation={
            "line": (0.2, 1.0, 0.2),
            "triangle": (0.4, 0.7, 0.2),
        },
        spacing_by_formation={"line": 0.4, "triangle": 0.3},
        min_pair_distance_by_formation={"line": 0.4, "triangle": 0.3},
        min_inter_drone_distance=0.35,
        clearance_margin=-0.1,
        last_switch_time_s=None,
    )

    assert decision.should_switch
    assert decision.target_formation == "line"
    assert decision.target_spacing == 0.4
    assert decision.selected_min_pair_distance == 0.4
