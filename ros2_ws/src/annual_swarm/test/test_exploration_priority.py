"""Behavioral regressions for unknown-region utility and feedback."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.mapping import ObservedMap
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.hierarchy import ExplorationRegion
from core.exploration.priority import ExplorationPriority, PriorityConfig, service_result, service_cells
from core.exploration.fusion import FusionPlanner
from core.exploration.mrdtg import MultiRobotGraph
from core.exploration.regions import visible_cells


def scene():
    m = ObservedMap([[0, 0, 0], [16, 12, 4]])
    m.state[:] = 0; m.state[32:, :] = -1; m.rebuild()
    return m


def task(ident=1, parent=-1):
    p = np.array([6., 4., 1.5])
    return ExplorationRegion(ident, [[4., 2.], [10., 6.]], 128, [p], p, parent=parent)


@pytest.mark.parametrize('ndim', [2, 3])
def test_unknown_components_respect_walls_and_update_without_version_bump(ndim):
    m = scene() if ndim == 2 else VoxelMap([[0, 0, 0], [6, 6, 4]])
    m.state[:] = 0
    m.state[2:5, 2:5] = -1; m.state[6:9, 2:5] = -1
    p = ExplorationPriority(); p.update_map(m)
    assert len(p.components) == 2
    assert sum(c['cells'] for c in p.components.values()) == np.count_nonzero(m.state == -1)
    assert all(p.labels[m.state != -1] == 0)
    m.state[5, 2:5] = -1
    p.update_map(m)
    assert len(p.components) == 1
    m.state[5, 2:5] = 1
    p.update_map(m)
    assert len(p.components) == 2


def test_diagonal_unknown_pockets_are_not_connected():
    m = scene(); m.state[:] = 0; m.state[3, 3] = m.state[4, 4] = -1
    p = ExplorationPriority(); p.update_map(m)
    assert len(p.components) == 2


def test_more_information_and_large_region_can_outweigh_longer_travel():
    p = ExplorationPriority()
    nearby, _ = p.utility(.4, 1., 4., 0., {}, 0.)
    large, _ = p.utility(2., 80., 12., 0., {}, 0.)
    assert large > nearby
    # Size never manufactures a benefit without observable information.
    assert p.utility(0., 10000., 1., 10000., {}, 0.)[0] == 0
    assert p.utility(10., 10000., np.inf, 10000., {}, 0.)[0] == 0


def test_waiting_is_bounded_and_cannot_override_information_per_time():
    p = ExplorationPriority()
    small, _ = p.utility(.2, 1., 10., 0., {}, 0.)
    large, _ = p.utility(2., 80., 12., 0., {}, 0.)
    aged, _ = p.utility(.2, 1., 10., 600., {}, 600.)
    assert small < aged < large
    assert aged == pytest.approx(small*(1+p.config.aging_bonus_max))
    assert p.utility(.2, 1., 10., 1e9, {}, 1e9)[0] == aged


def test_repeated_low_yield_backoff_and_success_reset():
    feedback = {}
    delays = []
    for k in range(6):
        feedback = service_result(feedback, 200.*k, 0, 80)
        delays.append(feedback['defer_until']-feedback['stamp'])
    assert delays == [15., 30., 60., 120., 120., 120.]
    p = ExplorationPriority()
    normal = p.utility(2., 80., 12., 0., {}, 1000.)[0]
    assert p.utility(2., 80., 12., 0., feedback, 1000.)[0] < normal
    # Clearing a tiny residual pocket is successful, even below 30 voxels.
    success = service_result(feedback, 1100., 4, 4)
    assert success['low_yield_streak'] == 0 and success['defer_until'] == 1100.
    assert p.utility(2., 80., 12., 0., success, 1100.)[0] == normal


def test_latest_shared_success_clears_older_failure_and_survives_duplicate():
    a = MultiRobotGraph(0, [[0, 0, 0], [16, 12, 4]])
    b = MultiRobotGraph(1, a.bounds)
    a.replica.put('s:7', dict(kind='region_service', id=7, **service_result({}, 10., 0, 100)))
    b.replica.merge(a.replica.packet(full=True)); b.rebuild()
    b.replica.put('s:7', dict(kind='region_service', id=7, **service_result(b.services[7], 12., 80, 100)))
    b.rebuild()
    assert b.services[7]['defer_until'] == 12. and b.services[7]['low_yield_streak'] == 0
    assert not b.replica.merge(a.replica.packet(full=True))
    b.rebuild(); assert b.services[7]['low_yield_streak'] == 0


def test_peer_commitments_are_deduplicated_and_expire_without_touching_map():
    m = scene(); p = ExplorationPriority(); point = [6., 4., 1.5]
    peer = dict(time=10., intent=dict(committed=True, path=[point], yaw=0.))
    before = m.state.copy(); safe = m.safe.copy()
    cells = p.committed_cells(m, {1: peer, 2: peer}, 11.)
    assert cells and cells == p.view(m, point, 0.)
    assert not p.committed_cells(m, {1: peer}, 13.)
    for flag in ('retiring', 'recovery'):
        inactive = dict(peer, intent=dict(peer['intent'], **{flag: True}))
        assert not p.committed_cells(m, {1: inactive}, 11.)
    peer['intent']['committed'] = False
    assert not p.committed_cells(m, {1: peer}, 11.)
    assert np.array_equal(before, m.state) and np.array_equal(safe, m.safe)


def test_priority_deducts_overlapping_gain_and_keeps_unreserved_gain():
    m = scene(); p = ExplorationPriority(); t = task()
    costs = SimpleNamespace(distance=lambda a, b: float(np.linalg.norm(a-b)))
    original = p.rank(m, {1: t}, {1}, costs, t.entry, 1., {})[1]
    assert original['score'] > 0 and original['components']
    cells = set()
    for yaw in np.linspace(-np.pi, np.pi, p.config.preview_headings, endpoint=False):
        cells.update(p.view(m, t.entry, yaw))
    partial = p.rank(m, {1: t}, {1}, costs, t.entry, 1., {}, frozenset(sorted(cells)[::2]))[1]
    excluded = p.rank(m, {1: t}, {1}, costs, t.entry, 1000., {}, frozenset(cells))[1]
    assert 0 < partial['predicted_gain'] < original['predicted_gain']
    assert excluded['predicted_gain'] == excluded['score'] == 0
    assert excluded['raw_gain'] > 0


def test_view_cache_rechecks_new_occluder_and_does_not_see_through_wall():
    m = scene(); p = ExplorationPriority(); point = [6., 4., 1.5]
    assert p.view(m, point, 0.)
    m.state[28, :] = 1  # Occupied wall between view and all unknown cells.
    assert not p.view(m, point, 0.)
    m.state[28, :] = 0
    assert p.view(m, point, 0.)


def test_coarse_rays_preserve_occlusion_and_default_view_resolution():
    m = VoxelMap([[0, 0, 0], [8, 8, 4]])
    m.state[:] = 0; m.state[20:, :, :] = -1
    point = [4., 4., 1.5]; p = ExplorationPriority()
    exact = visible_cells(m, point, 0.)
    coarse = p.view(m, point, 0., coarse=True)
    assert coarse and coarse <= exact
    assert exact == visible_cells(m, point, 0., azimuth_samples=61, pitch_samples=13)
    m.state[18, :, :] = 1
    assert not p.view(m, point, 0., coarse=True)


def test_service_gain_excludes_unrelated_new_observations():
    m = scene(); t = task()
    ids = service_cells(m, t, t.entry, 0.)
    assert len(ids)
    elsewhere = np.flatnonzero((m.state == -1).ravel())
    elsewhere = np.setdiff1d(elsewhere, ids)[:100]
    m.state.ravel()[elsewhere] = 0
    assert np.count_nonzero(m.state.ravel()[ids] != -1) == 0
    m.state.ravel()[ids[:4]] = 0
    assert np.count_nonzero(m.state.ravel()[ids] != -1) == 4


def test_wait_age_inherits_parent_and_resets_on_service():
    m = scene(); p = ExplorationPriority(); t = task(2, parent=1)
    p.first_seen[1] = 10.
    costs = SimpleNamespace(distance=lambda a, b: 1.)
    row = p.rank(m, {2: t}, {2}, costs, t.entry, 100., {})[2]
    assert row['wait_s'] == 90.
    row = p.rank(m, {2: t}, {2}, costs, t.entry, 100., {2: service_result({}, 95., 80, 100)})[2]
    assert row['wait_s'] == 5.


def test_remote_proxy_and_diagnostic_snapshot_are_explicit_and_json_safe():
    m = scene(); p = ExplorationPriority(); t = task()
    costs = SimpleNamespace(distance=lambda a, b: np.inf)
    row = p.rank(m, {1: t}, set(), costs, t.entry, 10., {})[1]
    assert row['gain_source'] == 'remote_proxy' and row['travel_s'] is None and row['score'] == 0
    packet = p.snapshot(m, 10., {1: 0}, 0)
    json.dumps(packet, allow_nan=False)
    layer = np.array(packet['slice_labels']).reshape(packet['shape'])
    assert np.array_equal(layer > 0, m.state == -1)
    assert packet['source'] == 0 and packet['unit'] == 'm2'


def test_fusion_joint_tour_uses_feedback_and_preserves_known_free_paths():
    m = scene(); planner = FusionPlanner(0, m.bounds)
    result = planner.compute(m, np.array([2., 4., 1.5]), 0., None, [], {}, [], 1, 1., False, {}, {}, {})
    tour = result['tour']; assert len(tour) > 1
    assert len(tour) == len(set(tour))
    assert planner.diagnostics['route_objective'] == 'travel_plus_information_latency'
    # Giving another region a continuous low-yield history changes the next task.
    former = tour[0]
    feedback = {former: dict(stamp=1., defer_until=1., observed_new_cells=0, low_yield_streak=100)}
    result = planner.compute(m, np.array([2., 4., 1.5]), 0., former, [], {}, [], 2, 2., True, {}, {}, {}, feedback)
    assert result['tour'][0] != former
    if result['selection']:
        assert m.safe_path(result['selection']['pool'].active.path)
        assert result['selection']['priority']['region'] == result['selected']
    assert 'priority' in planner.diagnostics['stage_wall_s']
