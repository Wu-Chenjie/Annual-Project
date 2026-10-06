"""Latency variation, clock slowdown and censored requests affect future boundaries."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.planning.handoff_timing import HandoffTiming
from core.planning.handoff import future_boundary_time


def test_rolling_p95_includes_slow_requests_and_converts_wall_to_simulation():
    timing = HandoffTiming(minimum_samples=5, margin_wall_s=1.)
    for sample in [1., 1., 1., 1., 12.]:
        timing.record_planning(sample)
    for token in range(5):
        timing.proposed(token, 100.); timing.authorized(token, 102.)
    for wall in range(10):
        timing.observe_clock(wall*.5, wall)
    result = timing.estimate()
    assert result['planning_wall_p95_s'] == pytest.approx(9.8)
    assert result['authorization_wall_p95_s'] == 2.
    assert result['lead_sim_s'] == pytest.approx((9.8+2.+1.)*.5)
    assert result['planning_source'] == 'rolling_p95'
    assert timing.estimate()['clock_source'] == 'measured_p95'


def test_short_history_and_long_latency_do_not_clip_to_an_unsafe_boundary():
    timing = HandoffTiming()
    timing.record_planning(20.)
    estimate = timing.estimate()
    assert estimate['planning_wall_p95_s'] == 20.
    assert estimate['planning_source'] == 'conservative_default'
    assert future_boundary_time(type('Curve', (), {'duration': 15.})(), 1., estimate['lead_sim_s']) is None


def test_clock_pause_reset_and_cancelled_lease_do_not_fabricate_samples():
    timing = HandoffTiming(window=5)
    timing.observe_clock(0., 0.); timing.observe_clock(2., 1.)
    timing.observe_clock(2., 2.)
    assert timing.estimate()['real_time_factor'] == 2.
    timing.observe_clock(0., 3.)
    assert timing.estimate()['clock_source'] == 'conservative_default'
    timing.proposed('cancelled', 5.); timing.cancelled('cancelled'); timing.authorized('cancelled', 10.)
    assert not timing.authorization
    for value in range(10): timing.record_planning(float(value))
    assert len(timing.planning) == 5
