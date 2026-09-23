"""Decision safety and assignment invariants independent of Gazebo dynamics."""
from pathlib import Path
import sys
import json
import numpy as np
import pytest
PACKAGE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PACKAGE/'scripts'))
import planning_runtime
from core.exploration.mapping import RaySensorWorld,ObservedMap
from core.exploration.graph import TopologyGraph,route_pool
from core.exploration.allocation import allocate
from core.exploration.coordinator import SearchCoordinator
MAPS=PACKAGE.parents[2]/'next_project/maps'

def test_ray_sensor_does_not_observe_through_wall(tmp_path):
    path=tmp_path/'wall.json';path.write_text(json.dumps(dict(bounds=[[0,0,0],[10,10,4]],obstacles=[dict(type='aabb',min=[4,0,0],max=[4.5,10,4])])))
    world=RaySensorWorld(path,radius=8);idx,values=world.observe([2,5,1.5]);xy=world.origin+(idx+.5)*world.resolution
    assert np.max(xy[:,0])<=4.25 and (values==1).any()
    assert np.linalg.norm(xy-np.array([2,5]),axis=1).max()<8.2

def test_unknown_cells_block_paths():
    m=ObservedMap([[0,0,0],[10,10,4]]);assert not m.safe_path([[2,2,1.5],[8,8,1.5]])
    m.state[3:16,3:16]=0;m.rebuild()
    assert m.safe_path([[2,2,1.5],[3,3,1.5]])
    assert not m.safe_path([[2,2,1.5],[5,2,1.5]])

def test_graph_uses_door_detour_not_euclidean_distance():
    m=ObservedMap([[0,0,0],[10,10,4]],clearance=.4);m.state[:]=0;m.state[19:21,:30]=1;m.rebuild();g=TopologyGraph(m)
    a=np.array([4,2,1.5]);b=np.array([6,2,1.5]);d=g.distances([a])[0,g.node(b)]
    assert d>10 and np.linalg.norm(a-b)==2

def test_disconnected_task_is_not_assigned():
    routes,report=allocate([[1,np.inf],[2,np.inf]],[[0,10],[10,0]],[10,11],[0,1])
    assert report['unreachable']==[11] and sum(10 in x for x in routes.values())==1

def test_pairwise_balances_routes_and_preserves_pinned_task():
    d=np.array([[4,4,4,4],[4.5,4.5,4.5,4.5]],float);tc=(np.ones((4,4))-np.eye(4))*8
    r,s=allocate(d,tc,[10,11,12,13],[0,1],pinned={10:0})
    assert s['objective']<=s['initial_objective'] and r[0][0]==10
    assert sorted(sum(r.values(),[]))==[10,11,12,13] and r[1]

def test_invalid_policy_rejected():
    with pytest.raises(ValueError):allocate([[1]],[[0]],[1],[0],policy='racer')

@pytest.mark.parametrize('name',['search_office','search_maze'])
def test_complex_maps_are_connected_and_initially_occluded(name):
    data=json.loads((MAPS/(name+'.json')).read_text());w=RaySensorWorld(MAPS/(name+'.json'));m=ObservedMap(w.bounds)
    assert len(data['obstacles'])>=28
    for p in data['search_starts']:m.update(*w.observe(p))
    assert .1<w.coverage(m)<.3
    m.state[:]=w.occupied;m.rebuild();g=TopologyGraph(m);d=g.distances(data['search_starts'])
    assert np.isfinite(d).all(),'Inflated free space must be connected, including the narrow doorways'

def test_reserved_path_blocks_other_vehicle_crossing():
    m=ObservedMap([[0,0,0],[10,10,4]]);m.state[:]=0;m.rebuild();m.block_paths([[[5,1,1.5],[5,9,1.5]]])
    assert not m.safe_path([[2,5,1.5],[8,5,1.5]])

def test_search_pool_reuses_quality_gate_and_task_identity():
    m=ObservedMap([[0,0,0],[10,10,4]]);m.state[:]=0;m.rebuild();p=route_pool(m,[2,2,1.5],[7,7,1.5],123,4)
    assert p.active and p.active.id.startswith('123:4:') and len(p.backups)<=5
    assert all(m.safe_path(c.path) for c in [p.active]+p.backups)

def test_release_retains_interrupted_task_for_transfer():
    c=SearchCoordinator([[0,0,0],[10,10,4]]);c.active[1]=12;c.task_positions[12]=np.array([5,5,1.5]);c.release(1,'unavailable')
    assert 1 not in c.active and c.pending_transfers[12]==1

def test_off_center_pose_uses_same_occupancy_cell_as_sensor():
    m=ObservedMap([[0,0,0],[10,10,4]]);m.state[:]=0;m.rebuild()
    m.safe[7,:]=False;m.distance[7,:]=0;m.grid.data[:,:,0]=~m.safe
    # Floor from cell-centre origin would incorrectly put x=2.01 into blocked cell 7.
    p=route_pool(m,[2.01,2.01,1.5],[6.125,2.125,1.5],99,1)
    assert p.active and m.safe_path(p.active.path)

def prepared_pool():
    from core.planning.path_quality import Candidate,PathQualityEvaluator,RankedPathPool
    c=SearchCoordinator([[0,0,0],[10,10,4]]);c.map.state[:]=0;c.map.rebuild()
    a=np.array([[2.,5,1.5],[8,5,1.5]]);b=np.array([[2.,5,1.5],[2,8,1.5],[8,8,1.5],[8,5,1.5]])
    ev=PathQualityEvaluator();pool=RankedPathPool();pool.rank([Candidate('old','astar',0,a,ev.evaluate(a,c.map),0),Candidate('reserve','astar',1,b,ev.evaluate(b,c.map),0)])
    assert pool.active.id=='old'
    c.active[0]=123;c.pools[0]=pool;c.task_positions[123]=a[-1];c.epochs[0]=1
    return c

def test_new_observation_switches_safe_cached_path_for_same_task():
    c=prepared_pool();cells=np.array([(x,y) for x in range(19,22) for y in range(17,24)])
    commands=c.step({0:np.array([2.,5,1.5])},{0},{},[(cells,np.ones(len(cells),np.int8))],2.)
    assert commands[0]['task']==123 and commands[0]['epoch']==2
    assert c.pools[0].active.id=='reserve' and c.map.safe_path(commands[0]['path'])
    assert any(e['type']=='backup_switch' for e in c.events)

def test_all_routes_blocked_stop_and_queue_task_instead_of_flying_old_route():
    c=prepared_pool();cells=np.array([(x,y) for x in range(19,22) for y in range(40)])
    commands=c.step({0:np.array([2.,5,1.5])},{0},{},[(cells,np.ones(len(cells),np.int8))],2.)
    assert commands[0] is None and 0 not in c.active and c.pending_transfers[123]==0

def test_dynamic_snapshot_remains_hidden_until_sensed():
    w=RaySensorWorld(MAPS/'search_office.json');m=ObservedMap(w.bounds);m.update(*w.observe([2,2,1.5]));before=m.state.copy()
    w.dynamic_snapshot(2,[dict(center_xy=[20,17],radius=.55,z_range=[0,3])])
    m.update(*w.observe([2,2,1.5]));assert np.array_equal(before,m.state)
    w.dynamic_snapshot(1,[]);assert w.last_obstacle_version==2

def test_nearest_baseline_preserves_active_task_owner():
    routes,_=allocate([[1,2],[.1,3]],[[0,1],[1,0]],[10,11],[0,1],policy='nearest_frontier',pinned={10:0})
    assert routes=={0:[10],1:[11]}

def test_graph_delta_removes_blocked_cells_and_keeps_stable_ids():
    c=SearchCoordinator([[0,0,0],[10,10,4]]);c.map.state[:]=0;c.map.rebuild()
    positions={0:np.array([2.,2,1.5])};c.step(positions,{0},{},[],0.)
    ids=set(c.graph_nodes);assert c.topology_delta['nodes_added']
    cells=np.array([[20,20]]);c.step(positions,{0},{},[(cells,np.ones(1,np.int8))],1.)
    assert c.topology_delta['nodes_removed'] and c.graph_nodes<ids
