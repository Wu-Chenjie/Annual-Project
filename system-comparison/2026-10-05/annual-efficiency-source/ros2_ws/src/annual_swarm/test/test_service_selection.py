"""Spatial tour order, fresh tail information, and collapsed gain regressions."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.service_selection import choose_view_window, view_gain_retained


def test_window_skips_low_value_stop_but_keeps_near_ties_in_tour_order():
    leftover = (7, dict(objective=.01))
    useful = (8, dict(objective=.4))
    assert choose_view_window([leftover, useful]) is useful
    close = (9, dict(objective=.011))
    assert choose_view_window([leftover, close]) is leftover
    assert choose_view_window([]) is None


def test_peer_traffic_is_a_soft_cost_and_transit_keeps_its_role():
    first = (7, dict(objective=.3))
    blocked = (8, dict(objective=.4, traffic_delay_s=12.))
    assert choose_view_window([first, blocked]) is first
    transit = (9, dict(objective=0., transit=True))
    assert choose_view_window([transit, first]) is transit
    assert choose_view_window([first, transit]) is first


@pytest.mark.parametrize('remaining,planned,accept', [
    (0,100,False), (1,100,False), (19,100,False), (20,100,True),
    (1,1,True), (1,0,True), (5,21,True), (4,21,False),
])
def test_collapsed_cached_gain_replans_without_absolute_tail_floor(remaining,planned,accept):
    assert view_gain_retained(remaining,planned) is accept
