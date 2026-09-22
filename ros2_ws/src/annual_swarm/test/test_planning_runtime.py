import sys
from pathlib import Path
import numpy as np
import pytest

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from planning_runtime import Runtime, PLANNERS, PlanarGraph, CheckedGNN
MAP=PACKAGE.parents[2]/'next_project/maps/sample_simple.json'

@pytest.mark.parametrize('algorithm',PLANNERS)
def test_obstacle_route(algorithm):
    runtime=Runtime(str(MAP),algorithm)
    path,report=runtime.plan([2,3,1.5],[18,16,1.5])
    assert runtime.safe_path(path)
    assert np.allclose(path[0],[2,3,1.5])
    assert np.allclose(path[-1],[18,16,1.5])
    assert report['algorithm']==algorithm
    assert report['plan_count']==1


def test_dstar_preserves_search_state():
    runtime=Runtime(str(MAP),'dstar_lite')
    path,_=runtime.plan([2,3,1.5],[18,16,1.5])
    search=runtime.dstar
    _,report=runtime.plan(path[10],[18,16,1.5])
    assert runtime.dstar is search
    assert report['dstar_state_reused']


def test_unsafe_endpoint_and_configuration_rejected():
    runtime=Runtime(str(MAP))
    with pytest.raises(ValueError): runtime.plan([2,3,1.5],[7,7,1.5])
    with pytest.raises(ValueError): Runtime(str(MAP),'made_up')
    with pytest.raises(ValueError): Runtime(str(MAP),'rrt_star',esdf=True)


def test_gnn_edges_remain_valid_after_endpoint_insertion():
    runtime=Runtime(str(MAP),'gnn'); graph=PlanarGraph(runtime)
    graph.build(np.array([2.,3.,1.5]),np.array([18.,16.,1.5]),runtime.field,100)
    for i,neighbors in enumerate(graph.adjacency):
        for j in neighbors:
            assert i in graph.adjacency[j]
            assert runtime.safe_path([graph.vertices[i],graph.vertices[j]])


def test_disconnected_gnn_must_not_append_unsafe_goal():
    graph=type('Graph',(),{'vertices':[np.zeros(3),np.ones(3)],'adjacency':[set(),set()]})()
    with pytest.raises(Exception,match='no route'):
        CheckedGNN()._extract_path_from_activities(np.ones(2),graph,np.zeros(3),np.ones(3))

@pytest.mark.parametrize('method',['none','moving_average','minimum_jerk','min_snap_proxy','min_jerk_cost'])
def test_checked_postprocessing(method):
    runtime=Runtime(str(MAP),'astar',esdf=True,firi=True,trajectory=method)
    path,report=runtime.plan([2,3,1.5],[18,16,1.5])
    assert runtime.safe_path(path)
    assert report['mpc_feasibility_only']['evaluated']
