import pytest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.timing_evidence import parent_geometry_timings


def packet(time, wall, count, session='first', drone=0):
    return dict(time=time, drone=drone, session=session,
                fusion=dict(parent_geometry_wall_s=wall, parent_geometry_count=count))


def test_window_differences_exclude_startup_and_keep_restart_work():
    states = [packet(2., 5., 10), packet(10., 7., 12), packet(15., 1., 2, 'restart'),
              packet(20., 3., 4, 'restart'), packet(25., 9., 10, 'restart'),
              packet(12., 2., 3, drone=1)]
    result = parent_geometry_timings(states, 5., 20.)
    assert result['wall_s'] == 7. and result['rebuilds'] == 9
    assert len(result['sessions']) == 3


def test_legacy_missing_counters_are_not_zero():
    result = parent_geometry_timings([dict(time=12., drone=0, fusion={})], 5., 20.)
    assert result['status'] == 'UNAVAILABLE' and result['wall_s'] is None


def test_counters_cannot_reset_without_a_new_session():
    with pytest.raises(ValueError, match='decreased'):
        parent_geometry_timings([packet(10., 7., 12), packet(15., 1., 2)], 5., 20.)


def test_missing_one_drone_is_partial_not_a_fleet_total():
    states = [packet(2., 0., 0), packet(10., 7., 12), dict(time=12., drone=1, fusion={})]
    result = parent_geometry_timings(states, 5., 20.)
    assert result['status'] == 'PARTIAL' and result['wall_s'] is None


def test_proposal_counters_use_the_same_window_and_do_not_become_geometry():
    state=dict(time=10.,drone=0,session='first',fusion=dict(parent_proposal_wall_s=2.,parent_proposal_count=3))
    result=parent_geometry_timings([state],5.,20.,'parent_proposal')
    assert result['calls']==3 and result['wall_s']==2. and 'rebuilds' not in result
