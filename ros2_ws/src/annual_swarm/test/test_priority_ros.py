"""ROS message construction, configuration and service-feedback callback checks."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'next_project'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_visualization_node import Visualization
from exploration_palette import PRIORITY_HIGH
from decentralized_agent_node import ExplorationAgent
from core.exploration.priority import ExplorationPriority
from core.exploration.voxel_mapping import VoxelMap


def test_priority_markers_contain_unknown_voxels_size_and_score():
    m = VoxelMap([[0, 0, 0], [8, 8, 4]])
    m.state[:] = 0; m.state[20:, :, :] = -1
    priority = ExplorationPriority(); priority.update_map(m)
    priority.scores = {7: dict(region=7, components=[1], score=.7, deferred=False, wait_s=90.)}
    layer = priority.snapshot(m, 10., {7: 0}, 0)
    rclpy.init()
    node = Visualization()
    try:
        packets = []; node.pub = SimpleNamespace(publish=packets.append)
        node.graphs[0] = dict(exploration_priority=layer)
        node.tick()
        markers = packets[-1].markers
        heat = next(x for x in markers if x.ns == 'unknown_region_priority')
        assert len(heat.points) == np.count_nonzero(np.array(layer['slice_labels']) == 1)
        assert (heat.color.r, heat.color.g, heat.color.b) == pytest.approx(PRIORITY_HIGH)
        assert heat.color.a > 0
        text = next(x.text for x in markers if x.ns == 'unknown_region_labels')
        assert 'm3' in text and '0.700' in text
        # A peer observation changes the display category, never the private map.
        observed = m.state.copy(); observed[20:22, :, :] = 0
        node.map = dict(state=observed.ravel().tolist(), shape=list(m.shape),
                        resolution=m.resolution, origin=m.origin.tolist())
        node.tick(); markers = packets[-1].markers
        heat = next(x for x in markers if x.ns == 'unknown_region_priority')
        peer = next(x for x in markers if x.ns == 'peer_observed_local_unknown')
        assert len(peer.points) == 2*m.shape[1]
        assert len(heat.points)+len(peer.points) == np.count_nonzero(np.array(layer['slice_labels']) == 1)
        assert np.all(m.state[20:, :, :] == -1)
        # A completed component disappears on the next published DELETEALL frame.
        node.graphs[0] = {}
        node.tick()
        assert not any(x.ns == 'unknown_region_priority' for x in packets[-1].markers)
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_agent_feedback_persists_streak_and_success_across_restart(tmp_path):
    config = ROOT/'ros2_ws/src/annual_swarm/config/exploration_priority.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config),
                    '-p', 'bounds:="[[0,0,0],[8,8,4]]"',
                    '-p', 'fleet_starts:="[[2,2,1.5]]"', '-p', f'output_dir:={tmp_path}'])
    node = ExplorationAgent()
    try:
        assert node.fusion.priority.config.retry_base_s == 15.
        node.now = lambda: 100.
        node.position = np.array([2.25, 2.25, 1.65])
        node.bootstrapped = node.rejoin_ready = True
        node.replica.map.state[:] = 0
        node.replica.map.state.ravel()[100:200] = -1
        node.replica.map.rebuild()
        def observe(gain):
            node.intent = dict(committed=True, path=[node.position.tolist()], yaw=0., token='test')
            node.selection = dict(transit=False); node.active = 7
            node.view_start_cells = np.arange(100, 200)
            node.view_start_known = int(np.count_nonzero(node.replica.map.state != -1))
            node.replica.map.state.ravel()[100:100+gain] = 0
            node.execution = dict(epoch=node.epoch, arrived=True, ready=True)
            node.heartbeat()
        observe(0)
        assert node.service_feedback[7]['low_yield_streak'] == 1
        assert node.cooldown[7] == 115.
        node.now = lambda: 120.
        observe(40)
        assert node.service_feedback[7]['low_yield_streak'] == 0
        assert node.cooldown[7] == 120.
    finally:
        node.worker.shutdown(); node.log.close(); node.paths.close(); node.destroy_node()
    restored = ExplorationAgent()
    try:
        assert restored.service_feedback[7]['low_yield_streak'] == 0
        assert restored.service_feedback[7]['observed_new_cells'] == 40
    finally:
        restored.worker.shutdown(); restored.log.close(); restored.paths.close(); restored.destroy_node()
        rclpy.shutdown()


def test_moving_service_feedback_records_consumed_pose_and_not_unvisited_goal(tmp_path):
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        node.now=lambda:20.;node.position=np.array([3.1,3.,1.5]);node.yaw=.25
        node.replica.map.state[:]=0;node.replica.map.state.ravel()[100:200]=-1;node.replica.map.rebuild()
        node.service_start=0.;node.view_start_cells=np.arange(100,200)
        node.view_team_before=np.zeros(node.replica.map.state.size,bool)
        node.view_start_known=int(np.count_nonzero(node.replica.map.state!=-1))
        node.intent=dict(token='old',path=[[2.,2.,1.5],[6.,2.,1.5]],yaw=.8,purpose='explore',epoch=1)
        node.selection=dict(transit=False);node.active=7
        node.finish_service('moving_sensor_service_complete',15.,([3.,3.,1.5],.2))
        assert node.service_feedback[7]['failed_view']==dict(position=[3.,3.,1.5],yaw=.2)
        np.testing.assert_allclose(node.recent[-1][1],[3.,3.,1.5]);assert node.recent[-1][2]==.2
        event=next(e for e in node.events if e['type']=='view_observed')
        assert event['service_end']==15. and event['service_pose_source']=='consumed_curve_boundary'
    finally:
        node.worker.shutdown();node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('blocked',[False,True])
def test_refitted_reserves_are_ranked_by_actual_continuous_curves(blocked):
    from decentralized_agent_node import fit_selection
    from core.planning.path_quality import Candidate,RankedPathPool,resample
    m=VoxelMap([[0.,0.,0.],[12.,12.,4.]]);m.state[:]=0
    if blocked:m.state[13:16,7:9,:]=1
    m.rebuild()
    start=np.array([2.25,2.25,1.65]);goal=np.array([6.25,2.25,1.65])
    pool=RankedPathPool()
    pool.rank([Candidate('long','test',0,np.array([start,[3.25,5.25,1.65],goal]),{'score':-100.},0),
               Candidate('short','test',1,np.array([start,[4.25,1.05,1.65],goal]),{'score':100.},0)])
    assert pool.active.id=='long'
    selection=dict(pool=pool,yaw=0.)
    fit_selection(selection,m,start,0.,7,0.)
    if blocked:
        assert pool.active.id=='short' and selection['trajectory_candidate']=='short'
        assert selection['trajectory'].duration < selection['curve_alternatives']['long'].duration
        assert pool.active.quality['curve_quality_score'] < pool.backups[0].quality['curve_quality_score']
    else:
        # The certified reserve keeps a genuine bypass even in open space.
        assert pool.active.id=='long' and [c.id for c in pool.backups]==['short']
        assert len(selection['curve_alternatives'])==2
        active=selection['curve_alternatives']['long'].path(.15)
        reserve=selection['curve_alternatives']['short'].path(.15)
        assert np.mean(np.linalg.norm(resample(active)-resample(reserve),axis=1))>=pool.diversity_m
        assert pool.active.quality['curve_quality_score'] < pool.backups[0].quality['curve_quality_score']
    assert pool.active.quality['executed_curve']['duration_s']==selection['trajectory'].duration
    before=selection['curve_qualities'][pool.active.id]
    pool.revalidate(start,m,__import__('core.planning.path_quality',fromlist=['PathQualityEvaluator']).PathQualityEvaluator(),m.version,now=0.)
    assert 'executed_curve' not in pool.active.quality
    assert before['quality']['executed_curve']['duration_s']==selection['trajectory'].duration
    assert before['map_version']==m.version


@pytest.mark.parametrize('peer_case',['transit','earlier','later','changes_before_authorization','split_before_authorization'])
def test_early_handoff_cannot_leave_until_actual_old_service_gain_is_sufficient(tmp_path,monkeypatch,peer_case):
    from core.planning.continuous_trajectory import interpolate
    from core.planning.path_quality import Candidate,RankedPathPool
    rclpy.init(args=['--ros-args','-p','bounds:="[[0,0,0],[8,8,4]]"',
                    '-p','fleet_starts:="[[2,2,1.5]]"','-p',f'output_dir:={tmp_path}'])
    node=ExplorationAgent()
    try:
        old=interpolate(np.array([[2.,2.,1.5],[6.,2.,1.5]]),[24.],0.,.4)
        p,v,a,h,rate=old.sample(12.)
        new=interpolate(np.array([p,[7.,3.,1.5]]),[16.],h,.4-h,start_velocity=v,start_acceleration=a,
                        start_yaw_rate=rate,start_yaw_acceleration=old.yaw_acceleration(12.))
        node.position=old.sample(4.)[0];node.yaw=old.sample(4.)[3]
        node.epoch=1;node.execution=dict(trajectory_time=4.,epoch=1);node.now=lambda:5.
        node.intent=dict(token='old',purpose='explore',recovery=False)
        node.selection=dict(gain=100*node.replica.map.resolution**3);node.owners={8:node.id}
        node.replica.map.state[:]=0;node.replica.map.state.ravel()[100:200]=-1;node.replica.map.rebuild()
        node.view_start_cells=np.arange(100,200);node.view_target_cells=np.arange(100,200)
        node.view_team_before=np.zeros(node.replica.map.state.size,bool)
        boundary=dict(from_token='old',from_epoch=1,trajectory_time=12.,position=p.tolist(),yaw=h)
        pool=RankedPathPool();pool.rank([Candidate('reserve','test',0,new.path(.15),{'score':0.},0)])
        node.preplanned=dict(epoch=1,time=5.,region=8,boundary=boundary,request_id='early',
                             selection=dict(trajectory=new,pool=pool,gain=0.,transit=True,purpose='transit_reobserve',yaw=.4))
        if peer_case!='transit':
            import decentralized_agent_node as agent_module
            monkeypatch.setattr(agent_module,'visible_cells',lambda *args,**kwargs:frozenset(range(180,200)))
            node.preplanned['selection'].update(transit=False,purpose='explore')
            def promises(runtime,peers,now,arrival=None):
                assert arrival==pytest.approx(29.)  # Includes the remaining old prefix and the new curve.
                return frozenset(range(180,200)) if peer_case=='earlier' else frozenset()
            node.fusion.priority.committed_cells=promises
        node.adopt_moving_preplan(5.)
        assert node.pending_intent is None and node.epoch==1 and node.preplanned is not None
        node.replica.map.state.ravel()[100:161]=0;node.replica.map.rebuild()
        node.adopt_moving_preplan(5.)
        if peer_case=='earlier':
            assert node.pending_intent is None and node.preplanned is None and node.intent['token']=='old'
            return
        assert node.pending_intent and not node.pending_intent['committed']
        assert node.intent['token']=='old' and node.epoch==2
        recorded=__import__('json').loads((tmp_path/'drone_0/candidates.jsonl').read_text())
        assert recorded['phase']=='handoff_proposed' and recorded['active_candidate']=='reserve'
        assert recorded['paths'][0]['trajectory']==new.to_dict()
        if peer_case in ('changes_before_authorization','split_before_authorization'):
            if peer_case=='changes_before_authorization':
                node.fusion.priority.committed_cells=lambda *args,**kwargs:frozenset(range(180,200))
            else:node.fusion.graph.regions[8]=dict(status='splitR')
            node.manage_pending(5.)
            assert node.pending_intent is None and node.intent['token']=='old'
            assert node.events[-1]['type']=='handoff_cancelled'
            assert node.replica.map.state.ravel()[180:200].tolist()==[-1]*20
    finally:
        node.worker.shutdown();node.log.close();node.paths.close();node.destroy_node();rclpy.shutdown()
