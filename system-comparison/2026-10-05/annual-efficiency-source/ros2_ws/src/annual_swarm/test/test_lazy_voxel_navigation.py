"""Differential checks against the exact frozen eager candidate8 implementation."""
import copy
import importlib.util
from pathlib import Path
import pickle
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.voxel_mapping import VoxelMap,VoxelRouter
from core.planning.path_quality import PathQualityEvaluator

file=Path(__file__).with_name('fixtures')/'voxel_mapping_candidate8.py'
spec=importlib.util.spec_from_file_location('frozen_eager_voxels',file)
reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)


def maps():
    old=reference.VoxelMap([[0.,0.,0.],[8.,8.,4.]])
    new=VoxelMap(old.bounds)
    old.state[:]=0;old.state[12:14,8:20]=1;old.state[22:]=-1
    new.state[:]=old.state
    old.rebuild();new.rebuild()
    return old,new


@pytest.mark.parametrize('intervention',['none','reservation','two_reservations','direct_state_edit','clearance','flight_limits'])
def test_navigation_and_envelope_predicates_equal_the_frozen_eager_reference(intervention):
    old,new=maps()
    for m in (old,new):
        if intervention in ('reservation','two_reservations'):
            m.block_paths([[[2.7,1.8,1.65],[2.7,4.8,1.65]]])
            if intervention=='two_reservations':m.block_paths([[[5.4,5.7,1.65]]],radius=1.1)
        elif intervention=='direct_state_edit':m.state[4:6,3:5]=1;m.rebuild()
        elif intervention=='clearance':m.clearance=.5;m.rebuild()
        elif intervention=='flight_limits':m.flight_limits=(1.2,2.7);m.rebuild()
    assert new._physical_distance is None
    points=np.random.default_rng(41).uniform([-.2,-.2,-.2],[8.2,8.2,4.2],(120,3))
    np.testing.assert_allclose(new.signed_distances(points),[old.signed_distance(p) for p in points],atol=1e-12)
    assert [new.safe_path([p]) for p in points]==[old.safe_path([p]) for p in points]
    assert new._physical_distance is None  # Safety checking does not build a navigation grid.
    np.testing.assert_array_equal(new.safe,old.safe)
    np.testing.assert_allclose(new.distance,old.distance,atol=1e-12)
    np.testing.assert_array_equal(new.grid.data,old.grid.data)


def test_lazy_reserved_snapshot_survives_worker_copy_and_serialization():
    old,new=maps()
    for m in (old,new):m.block_paths([[[2.7,1.8,1.65],[2.7,4.8,1.65]]])
    for transported in [copy.deepcopy(new),pickle.loads(pickle.dumps(new))]:
        assert transported.field is transported and transported._physical_distance is None
        np.testing.assert_array_equal(transported.safe,old.safe)
        np.testing.assert_allclose(transported.distance,old.distance,atol=1e-12)
    # A subsequent rebuild clears reservations exactly as the old API does.
    old.rebuild();new.rebuild()
    np.testing.assert_array_equal(new.safe,old.safe)
    assert new.reserved_points is None


def test_route_and_quality_equal_eager_reference_after_lazy_materialization():
    old,new=maps();start=[1.35,1.35,1.65];goal=[5.85,6.75,1.65]
    left=reference.VoxelRouter(old).route(start,goal);right=VoxelRouter(new).route(start,goal)
    assert left is not None and right is not None
    np.testing.assert_allclose(right,left,atol=1e-12)
    a=PathQualityEvaluator().evaluate(left,old);b=PathQualityEvaluator().evaluate(right,new)
    assert a['score']==pytest.approx(b['score'],abs=1e-12)
    assert a['min_clearance_m']==pytest.approx(b['min_clearance_m'],abs=1e-12)
