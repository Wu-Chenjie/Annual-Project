"""Batched computation must preserve polynomial and collision-distance semantics."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'next_project'))
from core.planning.continuous_trajectory import interpolate
from core.exploration.voxel_mapping import VoxelMap


def test_batched_position_derivatives_and_yaw_match_scalar_at_all_segment_boundaries():
    curve=interpolate(np.array([[2.,2.,1.5],[3.,4.,2.1],[5.,3.,1.8],[7.,6.,2.4]]),
                      [4.,7.,5.],-.4,1.1,start_velocity=[.1,0.,.03],
                      start_acceleration=[0.,.01,0.],start_yaw_rate=.02,start_yaw_acceleration=-.01)
    times=np.r_[-1.,curve.knots,curve.knots[1:-1]-1e-9,curve.knots[1:-1]+1e-9,
                np.linspace(0.,curve.duration,501),curve.duration+1.]
    expected=list(zip(*(curve.sample(t) for t in times)))
    for batch,scalar in zip(curve.sample_many(times),expected):
        np.testing.assert_allclose(batch,np.array(scalar),rtol=1e-12,atol=1e-12)
    dt=.13;sample_times=np.linspace(0.,curve.duration,max(2,int(np.ceil(curve.duration/dt))+1))
    np.testing.assert_allclose(curve.path(dt),np.array([curve.sample(t)[0] for t in sample_times]),atol=1e-12)


def test_batched_obstacle_box_distances_match_scalar_including_unknown_and_bounds():
    m=VoxelMap([[0.,0.,0.],[8.,8.,4.]]);m.state[:]=0;m.state[12:14,8:20]=1;m.state[22:]=-1;m.rebuild()
    points=np.random.default_rng(47).uniform([-.3,-.3,-.3],[8.3,8.3,4.3],(300,3))
    points=np.vstack([points,[[3.7,2.8,1.5],[4.2,5.9,3.1],[0.,0.,0.],[8.,8.,4.]]])
    np.testing.assert_allclose(m.signed_distances(points),[m.signed_distance(p) for p in points],atol=1e-12)
