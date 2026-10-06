"""A future-boundary reserve remains useful after the old view reaches rest."""
import sys
from pathlib import Path
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'next_project'));sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from decentralized_agent_node import ExplorationAgent,fit_selection
from core.planning.continuous_trajectory import interpolate
from core.planning.path_quality import Candidate,RankedPathPool
from core.exploration.regions import RegionTask


def test_cached_endpoint_view_can_refit_from_future_motion_without_executing_before_observation(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from core.planning.handoff import validate_handoff
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.now=lambda:10.;node.epoch=1
        node.replica.map.state[:]=0;node.replica.map.state[24:]=-1;node.replica.map.rebuild()
        old=interpolate(np.array([[2.25,2.25,1.65],[4.05,2.25,1.65]]),[14.],0.,0.)
        node.intent=dict(token='old',epoch=1,committed=True,purpose='explore',trajectory=old.to_dict())
        node.execution=dict(epoch=1,trajectory_time=1.);node.position=old.sample(1.)[0];node.yaw=0.
        goal=np.array([6.15,3.15,1.65]);node.owners={8:0}
        node.tasks[8]=RegionTask(8,[[6.,0.,0.],[8.,8.,4.]],100,[goal],goal)
        node.fusion.graph.regions[8]=dict(status='activeR')
        cached=interpolate(np.array([[4.05,2.25,1.65],goal]),[16.],0.,0.)
        pool=RankedPathPool();pool.rank([Candidate('cached','test',0,cached.path(.15),{'score':0.},0,5.,65.)])
        selection=dict(pool=pool,yaw=0.,gain=1.,lookahead=None,objective=1.,trajectory=cached,
                       trajectory_candidate='cached',transit=True,purpose='transit_reobserve')
        node.preplanned=dict(epoch=1,time=9.,region=8,selection=selection,position=[4.05,2.25,1.65],request_id='cached',boundary=None)
        jobs=[]
        def submit(function,*args,**kwargs):
            jobs.append((function,args,kwargs));return SimpleNamespace(done=lambda:False)
        monkeypatch.setattr(node.worker,'submit',submit)
        node.adopt_moving_preplan(10.)
        assert node.pending_intent is None and node.intent['token']=='old' and node.epoch==1
        assert node.future_kind=='moving_fit' and len(jobs)==1 and node.preplanned is None
        function,args,kwargs=jobs[0];result=function(*args,**kwargs)
        curve=result['selection']['trajectory'];boundary=node.future_boundary
        assert max(validate_handoff(old,curve,boundary['trajectory_time'],1.).values())<1e-5
        assert curve.limits()['speed']<=.601 and node.replica.map.safe_path(curve.path())
        assert len(result['selection']['curve_alternatives'])==1
    finally:
        node.worker.shutdown();node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('now',[20.,35.])
@pytest.mark.parametrize('after_fit_change',['none','cooldown','split','completed','owner'])
def test_live_reserve_is_refitted_at_rest_with_fresh_request_and_new_authorization(tmp_path,now,after_fit_change):
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.now=lambda:now;node.epoch=2;node.position=np.array([5.25,2.25,1.65]);node.yaw=0.
        node.replica.map.state[:]=0;node.replica.map.state[24:]=-1;node.replica.map.rebuild()
        start=np.array([3.15,2.25,1.65]);goal=np.array([6.15,3.15,1.65])
        curve=interpolate(np.array([start,goal]),[16.],0.,0.)
        pool=RankedPathPool();pool.rank([Candidate('cached','test',0,curve.path(.15),{'score':0.},0,5.,65.)])
        selection=dict(pool=pool,yaw=0.,gain=1.,objective=1.,lookahead=None,purpose='explore',
                       trajectory=curve,trajectory_candidate='cached',curve_alternatives={'cached':curve})
        node.tasks[8]=RegionTask(8,[[6.,0.,0.],[8.,8.,4.]],100,[goal],goal)
        node.fusion.graph.regions[8]=dict(status='activeR');node.owners={8:0}
        node.preplanned=dict(epoch=2,time=5.,position=start,region=8,selection=selection,
                             request_id='old-result',anticipatory=True)
        arrivals=[]
        def promises(runtime,peers,time,arrival=None):
            arrivals.append(arrival)
            assert arrival is not None  # A later peer promise must not erase this earlier service.
            return frozenset()
        node.fusion.priority.committed_cells=promises
        node.adopt_preplan(now)
        assert node.preplanned is None and node.intent is None
        assert node.fit_pending and node.fit_pending[0]==8 and selection['trajectory'] is None
        assert node.events[-1]['type']=='cached_preplan_refit_queued'
        result=fit_selection(selection,node.reserved_map(),node.position,node.yaw,8,now)
        if after_fit_change=='cooldown':node.cooldown[8]=now+1.
        elif after_fit_change=='split':node.fusion.graph.regions[8]['status']='splitR'
        elif after_fit_change=='completed':node.fusion.graph.regions[8]['status']='deadR'
        elif after_fit_change=='owner':node.owners[8]=1
        node.propose(8,result['selection'])
        assert node.parent_proposal_count>=2 and node.parent_proposal_wall_s>0
        if after_fit_change!='none':
            assert node.intent is None and node.events[-1]['type']=='task_lifecycle_invalidated'
            return
        assert node.intent and not node.intent['committed'] and node.intent['epoch']==3
        np.testing.assert_allclose(node.intent['path'][0],node.position,atol=1e-9)
        assert node.replica.map.safe_path(node.intent['path'])
        assert len(arrivals)>=2 and all(t>now for t in arrivals)
    finally:
        node.worker.shutdown();node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('fault',['epoch','expiry','split','completed'])
def test_expired_or_invalid_task_reserve_is_not_refreshed(tmp_path,fault):
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.now=lambda:35.;node.epoch=2;node.position=np.array([5.25,2.25,1.65]);node.yaw=0.
        node.replica.map.state[:]=0;node.replica.map.state[24:]=-1;node.replica.map.rebuild()
        goal=np.array([6.15,3.15,1.65]);pool=RankedPathPool()
        pool.rank([Candidate('cached','test',0,np.array([[3.15,2.25,1.65],goal]),{'score':0.},0,5.,10. if fault=='expiry' else 65.)])
        node.fusion.graph.regions[8]=dict(status='splitR' if fault=='split' else 'deadR' if fault=='completed' else 'activeR')
        node.owners={8:0}
        node.preplanned=dict(epoch=1 if fault=='epoch' else 2,time=5.,position=[3.15,2.25,1.65],region=8,
                             request_id='old',anticipatory=True,selection=dict(pool=pool,yaw=0.,gain=1.))
        node.adopt_preplan(35.)
        assert node.intent is None and node.fit_pending is None and node.preplanned is None
    finally:
        node.worker.shutdown();node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()
