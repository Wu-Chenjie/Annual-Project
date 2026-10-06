"""Spatial tour order, fresh tail information, and collapsed gain regressions."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.service_selection import choose_view_window, view_gain_retained, materialize_view_window


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


def test_only_winner_builds_reserves_and_other_previews_are_not_rejected():
    options=[(7,dict(objective=.01)),(8,dict(objective=.4)),(9,dict(objective=.3))]
    calls=[]
    def factory(rid,view):
        calls.append(rid);return dict(ready=True)
    chosen,rejected=materialize_view_window(options,factory)
    assert chosen==(8,dict(ready=True)) and calls==[8] and rejected==[]
    assert [r for r,_ in options]==[7,8,9]


def test_failed_reserve_retries_other_view_without_accepting_empty_pool():
    options=[(7,dict(objective=.01)),(8,dict(objective=.4)),(9,dict(objective=.3))]
    calls=[]
    def factory(rid,view):
        calls.append(rid);return dict(ready=True) if rid==9 else None
    chosen,rejected=materialize_view_window(options,factory)
    assert chosen[0]==9 and calls==[8,9] and rejected==[8]
    chosen,rejected=materialize_view_window(options,lambda rid,view:None)
    assert chosen is None and set(rejected)=={7,8,9}


def test_real_preview_does_no_reserve_search_then_builds_safe_complete_pool(monkeypatch):
    import numpy as np
    from types import SimpleNamespace
    from core.exploration import regions
    from core.exploration.voxel_mapping import VoxelMap,VoxelRouter
    m=VoxelMap([[0,0,0],[9,9,4]]);m.state[:]=0;m.state[20:,:,:]=-1;m.rebuild()
    start=np.array([2.25,3.15,1.65]);points=[np.array([3.15,3.15,1.65]),np.array([4.05,3.15,1.65])]
    task=SimpleNamespace(id=7,viewpoints=points);planner=regions.ObservationPlanner();calls=[]
    build=regions.route_pool
    def counted(*args,**kwargs):
        calls.append(args[3]);return build(*args,**kwargs)
    monkeypatch.setattr(regions,'route_pool',counted)
    before=m.state.copy();known=(m.state!=-1).ravel()
    choice=planner.preview(m,VoxelRouter(m),start,0.,task,observed_mask=known,history_weight=0.)
    assert choice is not None and not calls and choice['cells']
    assert all(not known[c] for c in choice['cells']) and m.safe_path(choice['path'])
    selection=planner.materialize(m,start,0.,task,2,choice,known,history_weight=0.)
    assert selection is not None and calls==[7]
    assert selection['planned_team_cells']==len(choice['cells'])
    assert selection['pool'].active is not None and len(selection['pool'].backups)<=5
    assert all(m.safe_path(c.path) for c in [selection['pool'].active]+selection['pool'].backups)
    assert np.array_equal(m.state,before)
    assert planner.preview(m,VoxelRouter(m),start,0.,task,observed_mask=np.ones(m.state.size,bool),history_weight=0.) is None
    assert calls==[7]


def test_fusion_compares_multiple_previews_but_builds_one_winning_local_pool(monkeypatch):
    import numpy as np
    from core.exploration import regions
    from core.exploration.fusion import FusionPlanner
    from core.exploration.voxel_mapping import VoxelMap
    m=VoxelMap([[0,0,0],[9,9,4]]);m.state[:]=0;m.state[20:,:,:]=-1;m.rebuild()
    start=np.array([2.25,3.15,1.65]);calls=[];build=regions.route_pool
    def counted(*args,**kwargs):
        calls.append(args[3]);return build(*args,**kwargs)
    monkeypatch.setattr(regions,'route_pool',counted)
    planner=FusionPlanner(0,m.bounds)
    result=planner.compute(m,start,0.,None,[],{},[],1,10.,True,{}, {},{})
    selection=result['selection']
    assert selection is not None and len(selection['view_window_regions'])>=2
    assert calls==[result['selected']]
    assert m.safe_path(selection['trajectory'].path())
    assert selection['trajectory'].limits()['speed']<=.60001
