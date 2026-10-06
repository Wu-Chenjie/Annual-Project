"""Physical derivative envelopes, corridor predictions and exact pair oracles."""
import itertools
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.service_cost import stopped_motion_time, view_service_cost
from core.exploration.team_evidence import expected_traffic_delay
from core.exploration.pairwise import solve_pair
from core.planning.continuous_trajectory import interpolate
from core.planning.handoff import moving_boundary_time, validate_handoff


@pytest.mark.parametrize('distance', [.001, .03, .1, .5, 1., 4., 10.])
def test_rest_to_rest_proxy_satisfies_exact_quintic_derivative_limits(distance):
    duration = stopped_motion_time(distance)
    curve = interpolate(np.array([[0.,0.,0.],[distance,0.,0.]]), [duration], 0., 0.)
    limits = curve.limits()
    assert limits['speed'] <= .600001
    assert limits['acceleration'] <= .800001
    assert limits['jerk'] <= 2.000001
    assert duration > distance/.6


def test_cost_charges_corners_repeated_travel_and_concurrent_yaw_without_filtering_tails():
    direct = view_service_cost([[0.,0.,0.],[2.,0.,0.]], 0., 0.)
    corner = view_service_cost([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.]], 0., 0.)
    repeat = view_service_cost([[0.,0.,0.],[2.,0.,0.]], 0., 0., [([1.,0.,0.],0.)])
    yaw = view_service_cost([[0.,0.,0.],[2.,0.,0.]], 0., np.pi)
    assert corner['total_s'] > direct['total_s'] and corner['path_turn_s'] > 0.
    assert repeat['repeated_travel_s'] > 0. and repeat['total_s'] > direct['total_s']
    assert yaw['motion_s'] == max(yaw['yaw_s'], direct['motion_s'])
    assert np.isfinite(view_service_cost([[0.,0.,0.],[.01,0.,0.]],0.,0.)['total_s'])


def peer(path, duration=20., position=None):
    return dict(time=10., position=path[0] if position is None else position,
                execution=dict(token='peer', trajectory_time=0., trajectory_duration=duration),
                intent=dict(token='peer', committed=True, path=path, duration=duration))


def test_corridor_cost_is_vertex_density_invariant_and_not_double_counted_per_peer():
    path = [[0.,0.,1.5],[6.,0.,1.5]]
    other = [[3.,-3.,1.5],[3.,3.,1.5]]
    sparse = expected_traffic_delay(path, {1:peer(other)}, 10.)
    dense = expected_traffic_delay(np.linspace(path[0],path[1],21), {1:peer(np.linspace(other[0],other[1],21).tolist())}, 10.)
    assert sparse > 0. and sparse == pytest.approx(dense)
    p = peer(other); p['pending_intent'] = dict(p['intent'],token='pending')
    assert expected_traffic_delay(path,{1:p},10.) == pytest.approx(sparse)
    assert expected_traffic_delay(path,{1:dict(p,time=0.)},10.) == 0.
    assert expected_traffic_delay([[0.,8.,1.5],[6.,8.,1.5]],{1:p},10.) == 0.


def test_finished_corridor_prefix_and_late_arrivals_do_not_manufacture_waits():
    own = [[0.,0.,1.5],[6.,0.,1.5]]
    consumed = peer([[3.,-3.,1.5],[3.,0.,1.5],[3.,6.,1.5]],position=[3.,6.,1.5])
    assert expected_traffic_delay(own,{1:consumed},10.) == 0.
    p = peer([[5.,-3.,1.5],[5.,3.,1.5]],duration=1.)
    assert expected_traffic_delay(own,{1:p},10.) == 0.


def test_wait_changes_priority_score_without_double_discounting_route_information(monkeypatch):
    from types import SimpleNamespace
    from core.exploration.priority import ExplorationPriority
    from core.exploration.hierarchy import ExplorationRegion
    from core.exploration.voxel_mapping import VoxelMap
    from core.exploration import team_evidence
    m=VoxelMap([[0,0,0],[9,9,4]]);m.state[:]=0;m.state[20:,:,:]=-1;m.rebuild()
    point=np.array([4.05,3.15,1.65]);start=np.array([2.25,3.15,1.65])
    task=ExplorationRegion(7,m.bounds,100,[point],point)
    costs=SimpleNamespace(distance=lambda a,b:1.,graph=object(),route_to_region=lambda rid:[start,point])
    args=(m,{7:task},{7},costs,start,10.,{})
    plain=ExplorationPriority().rank(*args,motion_time=stopped_motion_time)[7]
    monkeypatch.setattr(team_evidence,'expected_traffic_delay',lambda *args:5.)
    traffic=ExplorationPriority().rank(*args,peers={1:peer([[3.,0.,1.5],[3.,6.,1.5]])},motion_time=stopped_motion_time)[7]
    assert plain['predicted_gain']>0 and traffic['score']<plain['score']
    assert traffic['information_reward']==pytest.approx(plain['information_reward'])
    assert traffic['travel_s']==pytest.approx(stopped_motion_time(1.))


@pytest.mark.parametrize('seed', [0,1,2])
def test_vehicle_specific_wait_costs_match_exhaustive_assignment_oracle(seed):
    rng = np.random.default_rng(seed); n=4
    starts = rng.uniform(0.,10.,(2,n)); between = rng.uniform(0.,10.,(2,n,n))
    weights = rng.uniform(.1,3.,n); owners = {r:0 for r in range(n)}
    result = solve_pair(list(owners),starts,between,[1]*n,owners,(0,1),{0:0},reward_weights=weights)
    def cost(i, order):
        time=0.; weighted=0.
        for k,r in enumerate(order):
            time += (starts[i,r] if k==0 else between[i,order[k-1],r])+1.5
            weighted += weights[r]*time
        return time+.5*weighted/weights.sum()
    best=np.inf
    for assignment in itertools.product((0,1),repeat=n):
        if assignment[0]!=0 or max(assignment.count(0),assignment.count(1))>result['capacity']:continue
        values=[min(cost(i,p) for p in itertools.permutations([r for r in owners if assignment[r]==i])) for i in (0,1)]
        best=min(best,sum(values)+.25*max(values))
    assert result['after']==pytest.approx(best)
    assert result['assignments'][0]==0 and sorted(result['routes']['0']+result['routes']['1'])==list(owners)


def test_moving_boundary_preserves_measured_lead_and_c2_before_deceleration():
    old = interpolate(np.array([[0.,0.,1.5],[4.,0.,1.5]]),[20.],0.,.4)
    boundary = moving_boundary_time(old,1.,2.)
    assert boundary >= 3. and boundary == pytest.approx(10.,abs=.06)
    p,v,a,h,rate = old.sample(boundary)
    assert np.linalg.norm(v) >= .1 and np.dot(v,a) >= -1e-4
    new = interpolate(np.array([p,[6.,1.,1.5]]),[16.],h,.8-h,start_velocity=v,start_acceleration=a,
                      start_yaw_rate=rate,start_yaw_acceleration=old.yaw_acceleration(boundary))
    assert max(validate_handoff(old,new,boundary,1.).values()) < 1e-5
    assert moving_boundary_time(old,11.,2.) == 13.  # Preserve valid late handoffs.
    assert moving_boundary_time(old,1.,30.) is None
