import hashlib,json
from pathlib import Path
import numpy as np
from adapter import FrontierVoxelMap,GridTopology,central_execution_lease,target_heading,plan_step

def test_vendor_exact_historical_hashes():
    root=Path(__file__).parent/'vendor';p=json.loads((root/'provenance.json').read_text())
    assert all(hashlib.sha256((root/Path(name).name).read_bytes()).hexdigest()==digest for name,digest in p['files'].items())

def test_grid_routes_in_three_axes_without_diagonal_edges():
    m=FrontierVoxelMap([[0,0,0],[5,5,4]]);m.state[:]=0;m.rebuild();g=GridTopology(m)
    rows,cols=g.matrix.nonzero()
    assert np.all(np.sum(np.abs(g.cells[rows]-g.cells[cols]),axis=1)==1)
    a=m.points([[5,5,5]])[0];b=m.points([[7,8,7]])[0]
    assert np.isclose(g.distances([a])[0,g.node(b)],7*m.resolution)

def test_unknown_space_and_vertical_limits_reject():
    m=FrontierVoxelMap([[0,0,0],[5,5,4]]);m.state[:]=0;m.state[8,8,5]=-1;m.rebuild()
    assert not m.safe_path([[1.5,2.55,1.65],[3.5,2.55,1.65]])
    assert not m.safe_path([[2,2,3.5]])

def test_frontiers_have_unique_3d_ids_and_safe_positions():
    m=FrontierVoxelMap([[0,0,0],[8,8,4]]);m.state[:13,:,:]=0;m.rebuild();goals=m.frontier_targets()
    assert goals
    assert all(k==np.ravel_multi_index(v['cell'],m.shape) and m.safe_path([v['position']]) for k,v in goals.items())
    assert len(goals)<=24

def test_central_authorization_requires_live_matching_client_and_planner():
    s={0:dict(time=10.,central_time=10.,available=True,ready=True,intent=dict(token='a',committed=True))}
    assert central_execution_lease(s,0,'a',10.1,[])
    assert not central_execution_lease(s,0,'b',10.1,[])
    assert not central_execution_lease(s,0,'a',14.,[])
    s[0]['intent']['retiring']=True
    assert not central_execution_lease(s,0,'a',10.1,[])

def test_heading_uses_unknown_frontier_direction():
    m=FrontierVoxelMap([[0,0,0],[8,8,4]]);m.state[:]=0;m.state[16:,:,:]=-1
    angle=target_heading(m,np.array([4.,4.,1.5]),np.pi)
    assert abs(np.arctan2(np.sin(angle),np.cos(angle)))<=np.pi/3

def test_persistent_completion_survives_skipped_transient_reason():
    from adapter import view_finished
    state=dict(epoch=7,arrived=True,reason='idle',trajectory_time=4.)
    assert view_finished(state,7,4.)
    assert not view_finished(state,8,4.)
    assert not view_finished(state,7,5.)
    state['arrived']=False
    assert not view_finished(state,7,4.)
