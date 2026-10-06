import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from candidate_runtime import generate_candidates,make_runtime,apply_obstacles,PathQualityEvaluator,RankedPathPool
from core.planning.path_quality import Candidate,resample
MAP=str(PACKAGE.parents[2]/'next_project/maps/sample_simple.json')

@pytest.fixture(scope='module')
def generated():
    return generate_candidates(MAP,[2,3,1.5],[18,16,1.5],['astar','hybrid_astar','rrt_star'],4,0,[])[0]

def test_multiple_planners_keep_five_distinct_ranked_backups(generated):
    items=[generated.active]+generated.backups
    assert len(items)==6
    assert len({x.planner for x in items})>1
    assert [x.quality['score'] for x in items]==sorted(x.quality['score'] for x in items)
    for i,a in enumerate(items):
        for b in items[i+1:]:assert np.mean(np.linalg.norm(resample(a.path)-resample(b.path),axis=1))>=.25

def test_one_planner_multiple_variants():
    pool,_,_=generate_candidates(MAP,[2,3,1.5],[18,16,1.5],['astar'],12,0,[])
    assert len(pool.backups)==5
    assert all(x.planner=='astar' for x in [pool.active]+pool.backups)

def test_duplicate_paths_do_not_fill_reserves(generated):
    item=generated.active
    duplicate=Candidate('duplicate',item.planner,999,resample(item.path,200),item.quality,0)
    pool=RankedPathPool();pool.rank([item,duplicate])
    assert pool.active is not None and pool.backups==[]

def test_obstacle_invalidates_active_and_switches_to_cached_backup(generated):
    pool=copy.deepcopy(generated);runtime=make_runtime(MAP)
    old=pool.active.id;backup_ids={c.id for c in pool.backups}
    point=next(p for p in pool.active.path[len(pool.active.path)//3:2*len(pool.active.path)//3]
        if any(np.min(np.linalg.norm(c.path-p,axis=1))>3. for c in pool.backups))
    apply_obstacles(runtime,[dict(center_xy=point[:2],radius=.55,z_range=[0,3])])
    changed,rejected=pool.revalidate([2,3,1.5],runtime,PathQualityEvaluator(),1)
    assert changed and old in rejected
    assert pool.active.id in backup_ids
    assert pool.active.map_version==1 and runtime.safe_path(pool.active.path)
    assert all(runtime.safe_path(c.path) and c.map_version==1 for c in pool.backups)

def test_no_safe_backup_returns_none(generated):
    pool=copy.deepcopy(generated);runtime=make_runtime(MAP)
    apply_obstacles(runtime,[dict(center_xy=[10,10],radius=30,z_range=[0,3])])
    changed,rejected=pool.revalidate([2,3,1.5],runtime,PathQualityEvaluator(),2)
    assert changed and len(rejected)==6 and pool.active is None and pool.backups==[]

def test_blocked_connector_is_not_accepted():
    runtime=make_runtime(MAP)
    # Both endpoints are free, but their segment crosses the column at (7, 7).
    assert runtime.safe_point([3,7,1.5]) and runtime.safe_point([11,7,1.5])
    assert RankedPathPool.connect([3,7,1.5],np.array([[11,7,1.5],[12,7,1.5]]),runtime) is None

def test_invalid_weights_and_unsafe_paths_rejected():
    with pytest.raises(ValueError):PathQualityEvaluator({'length':1})
    with pytest.raises(ValueError):PathQualityEvaluator().evaluate([[2,3,1.5],[7,7,1.5]],make_runtime(MAP))

def test_density_does_not_change_straight_path_quality():
    runtime=make_runtime(MAP);evaluator=PathQualityEvaluator();path=np.array([[2,3,1.5],[18,3,1.5]])
    a=evaluator.evaluate(path,runtime);b=evaluator.evaluate(resample(path,150),runtime)
    assert a['score']==pytest.approx(b['score'],abs=.001)

def test_stale_async_generation_is_discarded():
    from candidate_pool_node import CandidatePlanning
    class Future:
        def done(self):return True
        def result(self):raise AssertionError('Stale result must not be consumed')
    fake=SimpleNamespace(future=Future(),job_version=(0,0),map_version=1,goal_revision=0,
        events=[],pending=False,position=None,status='PAUSED',diagnostics=lambda:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)),
        status_pub=SimpleNamespace(publish=lambda _:None))
    CandidatePlanning.tick(fake)
    assert fake.pending and fake.future is None
    assert fake.events[-1]['type']=='discard_stale_candidates'

def test_dynamic_grid_vectorization_matches_analytic_clearance():
    runtime=make_runtime(MAP)
    apply_obstacles(runtime,[dict(center_xy=[10,5],radius=.55,z_range=[0,3])])
    for idx in list(np.ndindex(runtime.grid.shape))[::19]:
        assert bool(runtime.grid.data[idx]) == (not runtime.safe_point(runtime.grid.index_to_world(idx),margin=.18))


def test_removing_obstacle_restores_static_grid():
    runtime=make_runtime(MAP);original=runtime.grid.data.copy()
    apply_obstacles(runtime,[dict(center_xy=[10,5],radius=2,z_range=[0,3])])
    assert np.any(runtime.grid.data!=original)
    apply_obstacles(runtime,[])
    assert np.array_equal(runtime.grid.data,original)


def test_single_replan_reuses_updated_grid_and_checks_obstacle():
    from candidate_runtime import regenerate_single
    runtime=make_runtime(MAP)
    apply_obstacles(runtime,[dict(center_xy=[9.5,5],radius=.55,z_range=[0,3])])
    before=runtime.grid.data.copy()
    pool,failures,count=regenerate_single(runtime,[3,3,1.5],[18,16,1.5],1,None,2)
    assert count==1 and failures==[] and pool.backups==[]
    assert pool.active.planner=='astar' and pool.active.map_version==1
    assert runtime.safe_path(pool.active.path)
    assert np.array_equal(before,runtime.grid.data)
