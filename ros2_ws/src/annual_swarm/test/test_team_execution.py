"""Team-accounting, continuous boundary and reservation fault properties."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.mapping import ObservedMap
from core.exploration.team_evidence import known_mask, service_accounting, regional_evidence, may_reactivate
from core.exploration.evidence_audit import ObservationAudit
from core.exploration.planning_budget import PlanningRequest
from core.exploration.decentralized import PeerLedger, fused_execution_lease
from core.exploration.hierarchy import AdaptiveRegions
from core.exploration.priority import ExplorationPriority
from core.planning.continuous_trajectory import interpolate, optimize_trajectory, ContinuousTrajectory
from core.planning.handoff import validate_handoff
from core.planning.path_quality import Candidate, RankedPathPool, PathQualityEvaluator


def world():
    m=ObservedMap([[0,0,0],[12,12,4]]); m.state[:]=0;m.state[20:,:]=-1;m.rebuild();return m


def test_team_union_removes_frontier_reward_without_opening_private_map():
    m=world(); before=m.state.copy(); h=AdaptiveRegions(m.bounds)
    assert h.update(m)
    assert not h.update(m, observed_mask=np.ones(m.state.size,bool))
    assert np.array_equal(m.state,before)
    assert np.any(~m.safe & (m.state == -1))


def test_team_known_local_new_does_not_reset_zero_gain():
    m=world(); ids=np.flatnonzero(m.state.ravel()==-1)[:40]
    prior=np.ones(m.state.size,bool);m.state.ravel()[ids]=0
    result=service_accounting(m,ids,prior)
    assert result['local_new_cells']==40 and result['team_new_cells']==0
    before=np.zeros(m.state.size,bool); late=np.zeros(m.state.size,bool);late[ids]=True
    assert service_accounting(m,ids,before,late)['team_new_cells']==0


def test_expiry_requires_changed_evidence_or_proven_gain():
    m=world(); evidence=regional_evidence(m,[[0,0],[12,12]])
    feedback=dict(low_yield_streak=2,defer_until=20.,evidence_signature=evidence['signature'],expected_team_cells=20)
    assert may_reactivate(feedback,evidence,30.,20)[0] is False
    assert may_reactivate(feedback,evidence,30.,26)[0]
    assert not may_reactivate(feedback,evidence,19.,100)[0]
    m.state[22,22]=0
    assert may_reactivate(feedback,regional_evidence(m,[[0,0],[12,12]]),30.)[0]


def packet(source,time,sequence,cells,values=None,**extra):
    return dict(source=source,time=time,sequence=sequence,sensor_session='s',shape=[3,3],origin=[0,0],resolution=1.,
                measured_indices=cells,measured_values=values if values is not None else [0]*len(cells),**extra)


def test_late_reordered_receipts_use_actual_time_not_arrival():
    a=ObservationAudit([3,3],[0,0],1.,2)
    assert a.ingest(packet(1,12.,2,[[0,0],[1,1]]))
    assert a.ingest(packet(0,10.,1,[[0,0]]))
    assert not a.ingest(packet(0,10.,1,[[0,0]]))
    assert a.window(0,9.,13.)['actual_team_new_known']==1
    assert a.window(1,9.,13.)['actual_team_new_known']==1
    assert a.window(1,9.,13.)['duplicate_local_new_known']==1
    # Snapshots and forecasts are not measured evidence.
    snapshot=packet(0,14.,2,[],full_snapshot=True,indices=[[2,2]])
    snapshot['values']=[0];a.ingest(snapshot)
    assert not np.isfinite(a.first[8])


def test_same_timestamp_tie_and_known_vs_free_are_deterministic():
    a=ObservationAudit([3,3],[0,0],1.,2)
    a.ingest(packet(1,5.,1,[[0,0]],values=[1]))
    a.ingest(packet(0,5.,1,[[0,0]],values=[1]))
    a.ingest(packet(1,6.,2,[[0,0]]))
    assert a.owner[0]==0 and a.free_owner[0]==1
    assert a.window(0,0,7)['actual_team_new_free']==0
    assert a.window(1,0,7)['actual_team_new_free']==1


def test_wrong_grid_is_rejected_before_dedup_fence():
    a=ObservationAudit([3,3],[0,0],1.,2); p=packet(0,5.,1,[[0,0]])
    p['resolution']=.5
    with pytest.raises(ValueError):a.ingest(p)
    p['resolution']=1.;assert a.ingest(p)


def test_wall_and_simulation_deadlines_and_incarnation_fences():
    r=PlanningRequest('x','s',4,7,10.,100.)
    assert r.rejection('s',4,11.,111.) is None
    assert r.rejection('s',4,11.,113.)=='wall_deadline'
    assert r.rejection('s',4,26.,101.)=='simulation_age'
    assert r.rejection('new',4,11.,101.)=='incarnation_changed'
    assert r.rejection('s',5,11.,101.)=='epoch_changed'


def test_moving_curve_preserves_c2_and_yaw_boundary_after_time_optimization():
    m=world();m.state[:]=0;m.rebuild()
    old=interpolate(np.array([[2.,2.,1.5],[4.,2.,1.5]]),[12.],0.,.4)
    p,v,a,h,rate=old.sample(8.)
    new=optimize_trajectory([p,[6.,3.,1.5]],m,h,.8,start_velocity=v,start_acceleration=a,
                            start_yaw_rate=rate,start_yaw_acceleration=old.yaw_acceleration(8.))
    assert max(validate_handoff(old,new,8.,6.).values())<1e-6
    assert np.linalg.norm(new.sample(0.)[1])>.05
    restored=ContinuousTrajectory.from_dict(new.to_dict())
    assert restored.yaw_rate_limit()<=.651 and m.safe_path(restored.path())
    bad=interpolate(np.array([p,[6.,3.,1.5]]),[12.],h,.8-h)
    with pytest.raises(ValueError):validate_handoff(old,bad,8.,6.)


def test_expired_curve_budget_cannot_produce_an_unvalidated_curve():
    import time
    m=world();m.state[:]=0;m.rebuild()
    with pytest.raises(TimeoutError):
        optimize_trajectory([[2.,2.,1.5],[4.,2.,1.5]],m,0.,0.,deadline_wall=time.monotonic()-1.)


def test_pending_and_current_reservations_share_source_but_both_exclude_other_uavs():
    ledger=PeerLedger(1,members=(0,1,2))
    def intent(token,region,path):return dict(token=token,region=region,path=path,created=1.,committed=True)
    old=intent('old',1,[[2,2,1.5],[4,2,1.5]])
    new=intent('new',2,[[4,2,1.5],[6,2,1.5]])
    ledger.states={0:dict(drone=0,time=2.,position=[2,2,1.5],intent=old,pending_intent=new),
                   2:dict(drone=2,time=2.,position=[7,2,1.5],intent=intent('third',3,[[5,2,1.5],[7,2,1.5]]))}
    own=dict(drone=1,time=2.,intent=None,position=[2,9,1.5])
    assert ledger.acknowledge(own,2.)==['new','old']
    assert not ledger.can_propose([[5,2,1.5],[8,2,1.5]],4,2.)
    ledger.acknowledge(own,100.)
    assert set(ledger.grants)=={'new','old'} # Silence cannot release either.
    ledger.states[0]['intent']=None
    ledger.acknowledge(own,101.)
    assert 'old' not in ledger.grants and 'new' in ledger.grants


def test_pending_lease_never_authorizes_missing_ack_or_retiring_curve():
    old=dict(token='old',committed=True,path=[[2,2,1.5],[3,2,1.5]],voters=[1])
    new=dict(token='new',committed=True,path=[[3,2,1.5],[4,2,1.5]],voters=[1])
    states={0:dict(time=1.,intent=old,pending_intent=new),1:dict(acks=['old'])}
    seeds=[[2,2,1.5],[2,8,1.5]]
    assert fused_execution_lease(states,0,'old',1.,seeds)
    assert not fused_execution_lease(states,0,'new',1.,seeds)
    states[1]['acks'].append('new');assert fused_execution_lease(states,0,'new',1.,seeds)
    old['retiring']=True;assert not fused_execution_lease(states,0,'old',1.,seeds)
    assert not fused_execution_lease(states,0,'new',10.,seeds)


def test_cached_reserve_expires_and_joins_actual_position():
    m=world();m.state[:]=0;m.rebuild();p=np.array([[2.,2.,1.5],[5.,2.,1.5]])
    evaluator=PathQualityEvaluator();pool=RankedPathPool()
    pool.rank([Candidate('x','test',0,p,evaluator.evaluate(p,m),0,0.,20.)])
    pool.revalidate([3.,2.,1.5],m,evaluator,2,now=10.)
    assert np.allclose(pool.active.path[0],[3.,2.,1.5])
    pool.revalidate([3.,2.,1.5],m,evaluator,3,now=21.)
    assert pool.active is None


def test_regional_grid_mismatch_cannot_retire_local_frontier():
    from core.exploration.mrdtg import MultiRobotGraph
    m=world();g=MultiRobotGraph(0,m.bounds);g.runtime=m
    g.replica.remote[1]={'r:7':dict(kind='region',id=7,unknown=0,status='deadR',stamp=9.,grid='a'*64)}
    g.rebuild();assert 7 not in g.regions


def test_pending_arrival_forecast_includes_unconsumed_old_curve():
    from core.exploration.team_evidence import predicted_peer_completion
    peer=dict(execution=dict(token='old',trajectory_time=4.,trajectory_duration=12.))
    new=dict(token='new',created=100.,duration=10.,handoff=dict(from_token='old',trajectory_time=8.))
    assert predicted_peer_completion(peer,new,100.)==pytest.approx(114.65)


def test_worker_result_excludes_private_geometry_and_rebuilds_after_map_change():
    import pickle
    from core.exploration.fusion import FusionPlanner,compute_worker
    m=world();position=np.array([2.,2.,1.5]);planner=FusionPlanner(0,m.bounds)
    def run(parent,now):
        return compute_worker(parent,m,position,0.,None,[],{},[],1,now,False,{}, {},{})
    first=pickle.loads(pickle.dumps(run(planner,1.)))
    assert first['graph'].runtime is None and first['graph'].trees=={}
    assert first['graph'].local_router is None and first['fusion'].priority._views=={}
    assert first['bids'] and first['graph'].nodes
    m.update(np.array([[10,10]]),np.array([1]));m.rebuild()
    second=pickle.loads(pickle.dumps(run(first['fusion'],2.)))
    assert second['graph'].version==m.version
    assert all(m.safe_path(edge['points']) for edge in second['graph'].edges)


def test_late_sensor_window_and_bootstrap_history_do_not_create_new_credit():
    from core.exploration.decentralized import MapReplica
    replica=MapReplica([[0,0,0],[12,12,4]],0)
    replica.merge(dict(source=0,sequence=1,time=10.,sensor_session='s',full_snapshot=True,
                       indices=[[20,20],[21,21]],values=[0,0],measured_indices=[[20,20]]))
    assert replica.first_observed[20,20]==10. and replica.first_observed[21,21]==-np.inf
    replica.merge(dict(source=0,sequence=2,time=13.,sensor_session='s',indices=[[22,22]],values=[0]))
    cells=np.ravel_multi_index(([20,21,22],[20,21,22]),replica.map.shape)
    result=service_accounting(replica.map,cells,np.zeros(replica.map.state.size,bool),
        first_observed=replica.first_observed,start=11.,end=14.)
    assert result['local_new_cells']==1 and result['team_new_cells']==1
