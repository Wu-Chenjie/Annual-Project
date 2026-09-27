"""Completed regions can host useful views of adjacent unfinished regions."""
import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.fusion import FusionPlanner
from core.exploration.mrdtg import DeltaGraph, observation_grid
from core.exploration.priority import ExplorationPriority
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.voxel_mapping import VoxelRouter
from core.exploration.hierarchy import AdaptiveRegions,ExplorationRegion
from core.exploration.regions import ObservationPlanner,visible_cells


def test_one_observable_voxel_is_not_permanently_discarded_as_too_small():
    m=VoxelMap([[0,0,0],[12,12,4]]);m.state[:]=0;m.state[20,20,6]=-1;m.rebuild()
    point=np.array([4.65,6.15,1.65]);start=np.array([3.15,6.15,1.65])
    assert len(visible_cells(m,point,0.))==1
    hierarchy=AdaptiveRegions(m.bounds);assert hierarchy.update(m)
    task=ExplorationRegion(1,m.bounds.tolist(),1,[point],point,0,-1,'activeR')
    selection=ObservationPlanner().plan(m,VoxelRouter(m),start,0.,task,1,history_weight=0.)
    assert selection and selection['gain']==pytest.approx(m.resolution**3)
    assert m.safe_path(selection['pool'].active.path)
    assert not hierarchy.update(m,observed_mask=np.ones(m.state.size,bool))


def support_case():
    bounds=[[0.,0.,0.],[16.,8.,4.]]
    a=VoxelMap(bounds);a.state[:]=0;a.state[42:]=-1;a.rebuild()
    pa=FusionPlanner(0,bounds)
    for t,x in enumerate((2.25,5.25,8.25,11.25),1):
        pa.compute(a,np.array([x,3.75,1.65]),0.,None,[],{},[],0,float(t),False,{}, {},{})
    b=VoxelMap(bounds);b.state[:16]=0;b.rebuild()
    pb=FusionPlanner(1,bounds)
    assert pb.graph.replica.merge(pa.graph.replica.packet(full=True))
    peers={0:dict(position=[11.25,3.75,1.65],time=4.,available=False,
                  graph_connections=pa.connections)}
    return a,pa,b,pb,peers


def test_team_known_corridor_support_survives_completed_lifecycle_without_private_map_copy():
    _,_,b,pb,peers=support_case();before=b.state.copy()
    result=pb.compute(b,np.array([2.25,3.75,1.65]),0.,None,[],{},[[peers[0]['position']]],
                      0,4.,True,peers,{}, {})
    selection=result['selection']
    assert selection and selection['purpose']=='transit_reobserve'
    assert selection['team_gain']==0.
    assert b.safe_path(selection['trajectory'].path())
    assert np.linalg.norm(selection['trajectory'].path()[-1]-[2.25,3.75,1.65])>.4
    assert np.array_equal(before,b.state)
    assert pb.graph.regions[result['selected']]['status']=='deadR'
    assert pb.graph.regions[result['selected']]['unknown']==0


def test_actual_neighbor_receipts_retire_even_stale_scoped_service_without_opening_geometry():
    _,_,b,pb,_=support_case();before=b.state.copy();pb.graph.runtime=b;pb.graph.rebuild()
    mask=pb.graph.observed_mask(b)
    services=pb.graph.planning_regions(b,100.,mask)
    scoped={r:v for r,v in services.items() if v.get('forecast_scope_kind')=='stale_footprint_bound'}
    assert scoped
    rid,service=next(iter(scoped.items()))
    mask=mask.copy()
    for anchor in pb.graph.service_anchors[rid]:mask[anchor.get('forecast_cells',[])]=True
    retired=pb.graph.planning_regions(b,100.,mask)
    assert retired[rid].get('service_role') is None
    assert np.array_equal(before,b.state)
    pb.graph.regions[rid]=dict(pb.graph.regions[rid],status='splitR')
    assert pb.graph.planning_regions(b,4.,np.zeros(b.state.size,bool))[rid]['status']=='splitR'


@pytest.mark.parametrize('bad',[[-1],[10**9],[True],[1,1],[[1]],'bad'])
def test_malformed_forecast_cannot_advance_replica_fence(bad):
    _,pa,b,_,_=support_case();packet=copy.deepcopy(pa.graph.replica.packet(full=True))
    region=next(v for v in packet['records'].values() if v and v.get('forecast_cells'))
    region['forecast_cells']=bad;receiver=DeltaGraph(2)
    with pytest.raises(ValueError):receiver.merge(packet)
    assert not receiver.received and not receiver.remote
    region['forecast_cells']=[0];region['forecast_grid_shape']=list(map(int,b.shape))
    assert receiver.merge(packet)


def test_peer_view_footprint_discount_is_cross_region_arrival_scoped_and_never_an_observation():
    b=VoxelMap([[0.,0.,0.],[8.,8.,4.]]);before=b.state.copy();priority=ExplorationPriority()
    intent=dict(region=999,token='view',committed=True,duration=10.,committed_at=0.,
                observation_grid=observation_grid(b),expected_observation_cells=[10,20])
    peer=dict(time=2.,intent=intent,execution=dict(token='view',trajectory_time=2.,trajectory_duration=10.))
    assert not priority.committed_cells(b,{0:peer},2.,arrival=5.)
    assert priority.committed_cells(b,{0:peer},2.,arrival=12.)==frozenset([10,20])
    assert not priority.committed_cells(b,{0:dict(peer,execution={})},2.,arrival=5.)
    assert not priority.committed_cells(b,{0:dict(peer,time=-10.)},2.,arrival=12.)
    intent['committed']=False
    assert not priority.committed_cells(b,{0:peer},2.,arrival=12.)
    assert np.array_equal(b.state,before)
