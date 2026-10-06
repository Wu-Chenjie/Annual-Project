"""Route tradeoffs, decentralized observed receipts, and bounded preview work."""
import copy
import itertools
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.regions import optimize_tour, information_count
from core.exploration.pairwise import solve_pair, subset_tours
from core.exploration.priority import ExplorationPriority, PriorityConfig
from core.exploration.mrdtg import MultiRobotGraph, observation_grid
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.hierarchy import ExplorationRegion


def test_joint_route_keeps_spatial_order_instead_of_following_individual_scores():
    tasks={i:SimpleNamespace(entry=float(i),unknown=5) for i in (2,3,12,13)}
    graph=SimpleNamespace(distance=lambda a,b:abs(a-b))
    reward={2:1.,3:2.,12:20.,13:30.}
    route,work=optimize_tour(0.,list(tasks),tasks,graph,rewards=reward)
    sorted_scores=sorted(tasks,key=lambda r:-reward[r]/(1.5+r/.6))
    def evaluate(order):
        t=0.; weighted=0.
        for a,b in zip([0]+order,order):
            t+=abs(b-a)/.6+1+tasks[b].unknown*.012
            weighted+=reward[b]*t
        return t+.5*weighted/sum(reward.values())
    assert evaluate(route)<evaluate(sorted_scores)
    assert route==[2,3,12,13]
    assert work==pytest.approx(13/.6+4*1.06)


@pytest.mark.parametrize('seed',range(8))
def test_incremental_route_matches_full_objective_on_asymmetric_costs(seed):
    rng=np.random.default_rng(seed);n=9
    distances=rng.uniform(.1,20,(n+1,n+1));np.fill_diagonal(distances,0.)
    if seed%2:distances[1,4]=np.inf
    tasks={i:SimpleNamespace(entry=i,unknown=int(rng.integers(1,300))) for i in range(n)}
    rewards={i:float(rng.uniform(.01,20)) for i in tasks}
    pinned=0 if seed%3==0 else None
    graph=SimpleNamespace(distance=lambda a,b:distances[a,b])
    def objective(order):
        t=0.;weighted=0.
        for a,b in zip([n]+order,order):
            t+=distances[a,b]/.6+1.+tasks[b].unknown*.012
            weighted+=rewards[b]*t
        return t+.5*weighted/sum(rewards.values()),t
    reference=[pinned] if pinned is not None else []
    remaining=set(tasks)-set(reference);first=1 if reference else 0
    while remaining:
        _,r,k=min((objective(reference[:k]+[r]+reference[k:])[0],r,k)
                  for r in remaining for k in range(first,len(reference)+1))
        reference.insert(k,r);remaining.remove(r)
    for _ in range(4):
        best=reference;value=objective(best)[0]
        for i in range(first,n):
            for j in range(i+2,n+1):
                candidate=reference[:i]+reference[i:j][::-1]+reference[j:]
                score=objective(candidate)[0]
                if score+1e-6<value:best=candidate;value=score
        if best==reference:break
        reference=best
    actual,work=optimize_tour(n,list(tasks),tasks,graph,pinned,rewards=rewards)
    assert actual==reference
    assert work==pytest.approx(objective(actual)[1])


@pytest.mark.parametrize('seed',[0,1,2])
def test_information_subset_routes_match_exhaustive_directed_oracle(seed):
    rng=np.random.default_rng(seed);n=5
    start=rng.uniform(1,10,n);between=rng.uniform(1,10,(n,n));np.fill_diagonal(between,0)
    weights=rng.uniform(.01,3,n);alpha=.5
    costs,route=subset_tours(start,between,weights,alpha)
    def evaluate(order):
        t=0.; reward_time=0.
        for j,r in enumerate(order):
            t+=(start[r] if j==0 else between[order[j-1],r])+1.5
            reward_time+=weights[r]*t
        return t+alpha*reward_time/weights.sum()
    for mask in range(1<<n):
        members=[j for j in range(n) if mask & (1<<j)]
        expected=min((evaluate(p) for p in itertools.permutations(members)),default=0.)
        assert costs[mask]==pytest.approx(expected)
        assert evaluate(route(mask))==pytest.approx(expected)
        assert set(route(mask))==set(members)


def test_information_assignment_preserves_capacity_pinning_and_exact_objective():
    start=np.array([[2.,4.,12.,14.],[14.,12.,4.,2.]])
    between=np.abs(np.array([0.,2.,10.,12.])[:,None]-np.array([0.,2.,10.,12.])[None,:])
    weights=np.array([1.,3.,2.,5.]);owners={i:0 for i in range(4)}
    result=solve_pair(list(owners),start,between,[1]*4,owners,(0,1),{0:0},reward_weights=weights)
    def cost(drone,order):
        t=0.; weighted=0.
        for k,r in enumerate(order):
            t+=(start[drone,r] if k==0 else between[order[k-1],r])+1.5;weighted+=weights[r]*t
        return t+.5*weighted/weights.sum()
    best=np.inf
    for assignment in itertools.product((0,1),repeat=4):
        if assignment[0]!=0 or max(assignment.count(0),assignment.count(1))>result['capacity']:continue
        values=[min(cost(i,p) for p in itertools.permutations([r for r in owners if assignment[r]==i])) for i in (0,1)]
        best=min(best,sum(values)+.25*max(values))
    assert result['after']==pytest.approx(best)
    assert result['assignments'][0]==0 and max(result['loads'])<=result['capacity']
    assert sorted(result['routes']['0']+result['routes']['1'])==list(owners)
    assert solve_pair([0],[[np.inf],[np.inf]],[[0]],[1],{0:0},(0,1),reward_weights=[1])['status']=='infeasible'


def test_observed_receipts_are_actual_union_idempotent_and_never_open_unknown_space():
    m=VoxelMap([[0,0,0],[8,8,4]]);other=copy.deepcopy(m)
    m.state.ravel()[10:300]=0;m.state.ravel()[310:320]=1;m.rebuild()
    a=MultiRobotGraph(0,m.bounds);b=MultiRobotGraph(1,m.bounds)
    a.record_observation(m);seq=a.replica.sequence;a.record_observation(m)
    assert a.replica.sequence==seq
    packet=a.replica.packet(full=True);assert b.replica.merge(packet);assert not b.replica.merge(packet)
    b.rebuild();before=other.state.copy();safe=other.safe.copy();mask=b.observed_mask(other)
    assert np.array_equal(mask,(m.state!=-1).ravel())
    assert np.array_equal(other.state,before) and np.array_equal(other.safe,safe)
    assert not other.safe_path([[2,2,1.5],[3,2,1.5]])
    assert all(set(v)=={'kind','grid','block','bits'} for v in packet['records'].values())
    unknown={11,12,400};assert information_count(unknown,mask,.1)==pytest.approx(1.2)
    other.state.ravel()[400]=0;b.record_observation(other);b.rebuild()
    assert b.observed_mask(other)[400] and b.observed_mask(other).sum()==mask.sum()+1
    # Matching indices in another resolution are not matching observed cells.
    different=VoxelMap(m.bounds,resolution=.5)
    assert not b.observed_mask(different).any()


def test_observed_receipts_rejoin_rejects_old_incarnation_and_restores_from_sensor_map():
    m=VoxelMap([[0,0,0],[4,4,4]]);m.state.ravel()[:20]=0
    a=MultiRobotGraph(0,m.bounds);b=MultiRobotGraph(1,m.bounds)
    a.record_observation(m);old=a.replica.packet(full=True);b.replica.merge(old)
    restarted=MultiRobotGraph(0,m.bounds);restarted.record_observation(m)
    b.replica.rejoin(0,restarted.replica.session)
    assert not b.replica.merge(old)
    assert b.replica.merge(restarted.replica.packet(full=True));b.rebuild()
    assert b.observed_mask(m).sum()==20


@pytest.mark.parametrize('bits',['f'*63,'-'+'1'*63,'g'*64])
def test_malformed_receipt_does_not_advance_replication(bits):
    m=VoxelMap([[0,0,0],[4,4,4]]);a=MultiRobotGraph(0,m.bounds);b=MultiRobotGraph(1,m.bounds)
    a.replica.put('bad',dict(kind='observed_cells',grid=observation_grid(m),block=0,bits=bits))
    with pytest.raises(ValueError):b.replica.merge(a.replica.packet(full=True))
    assert not b.replica.received and not b.replica.remote


def preview_scene():
    m=VoxelMap([[0,0,0],[14,12,4]]);m.state[:]=0;m.state[28:,:,:]=-1;m.rebuild()
    tasks={}
    for i in range(12):
        p=np.array([6.15,1.35+i*.75,1.65])
        tasks[i]=ExplorationRegion(i,[[5.,0.,0.],[10.,12.,4.]],100,[p],p)
    return m,tasks


def test_preview_budget_rotates_and_cache_reuses_only_unchanged_ray_windows():
    m,tasks=preview_scene();p=ExplorationPriority(PriorityConfig(preview_regions=4,refresh_regions=2))
    costs=SimpleNamespace(distance=lambda a,b:float(np.linalg.norm(a-b)))
    for k in range(6):
        p.rank(m,tasks,tasks,costs,np.array([2.,2.,1.5]),k,{})
        assert p.diagnostics['raycasts']<=4*2*4
    assert set(p._refreshed)==set(tasks)
    p.rank(m,tasks,tasks,costs,np.array([2.,2.,1.5]),10.,{})
    assert p.diagnostics['raycasts']==0
    # No version bump: a new wall in the ray windows must still invalidate them.
    m.state[25,:,:]=1
    rows=p.rank(m,tasks,tasks,costs,np.array([2.,2.,1.5]),11.,{})
    assert p.diagnostics['raycasts']>0
    assert all(row['raw_gain']==0 for row in rows.values() if row['gain_source']=='local_rays')


def test_history_discounts_ray_gain_without_erasing_small_remnant_or_map():
    m,tasks=preview_scene();tasks={0:tasks[0]};p=ExplorationPriority();costs=SimpleNamespace(distance=lambda a,b:1.)
    before=m.state.copy();first=p.rank(m,tasks,tasks,costs,tasks[0].entry,0.,{})[0]
    mask=np.ones(m.state.size,bool)
    after=p.rank(m,tasks,tasks,costs,tasks[0].entry,0.,{},observed_mask=mask)[0]
    assert after['predicted_gain']==pytest.approx(first['predicted_gain']*.1)
    assert 0<after['score']<first['score']
    assert np.array_equal(before,m.state)


@pytest.mark.parametrize('config',[dict(history_weight=0.),dict(history_weight=1.1),dict(aging_bonus_max=-1.),dict(preview_regions=2,refresh_regions=3)])
def test_invalid_integrated_config_rejected(config):
    with pytest.raises(ValueError):PriorityConfig(**config)
