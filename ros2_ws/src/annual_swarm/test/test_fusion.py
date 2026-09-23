"""Independent properties of the fused algorithms and wire protocols."""
import itertools
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.mapping import ObservedMap
from core.exploration.hierarchy import AdaptiveRegions, FrontierIndex
from core.exploration.mrdtg import DeltaGraph, MultiRobotGraph, graph_voronoi
from core.exploration.pairwise import solve_pair, PairExchange
from core.planning.continuous_trajectory import optimize_trajectory, ContinuousTrajectory, interpolate
from core.exploration.fusion import FusionPlanner


def opened():
    m = ObservedMap([[0, 0, 0], [16, 12, 4]])
    m.state[:] = 0; m.rebuild(); return m


def test_hgrid_refines_and_pins_service():
    m = opened(); m.state[16:, :] = -1; m.rebuild()
    h = AdaptiveRegions(m.bounds)
    h.update(m, pinned=[0]); assert 0 not in h.split
    h.update(m); assert 0 in h.split and len(h.leaves) > 4
    leaves = set(h.leaves); h.update(m); assert set(h.leaves) == leaves
    assert len(set(h.leaves)) == len(h.leaves)


def test_frontiers_update_and_reuse_unchanged_clusters():
    m = opened(); m.state[24:, :] = -1; m.rebuild()
    f = FrontierIndex().update(m); keys = set(f.clusters)
    f.update(m); assert f.changed_cells == 0 and f.reused_clusters == len(keys)
    m.state[24, 4:9] = 0; f.update(m)
    assert not np.any(f.frontier & (m.state != 0))
    assert f.changed_cells == 5


def test_bootstrap_splits_reach_peer_without_previous_parent_records():
    m = opened(); m.state[16:, :] = -1; m.rebuild()
    h = AdaptiveRegions(m.bounds); h.update(m)
    a, b = MultiRobotGraph(0, m.bounds), MultiRobotGraph(1, m.bounds)
    assert h.split and not a.replica.records
    a.update(m, np.array([2., 2., 1.5]), h, 1.)
    assert b.replica.merge(a.replica.packet(full=True))
    b.rebuild()
    assert all(b.regions[r]['status'] == 'splitR' for r in h.split)
    # Even a peer with no local observations must reproduce the same hierarchy.
    unknown = opened(); unknown.state[:] = -1; unknown.rebuild()
    remote = AdaptiveRegions(m.bounds)
    remote.split.update(r for r, v in b.regions.items() if v['status'] == 'splitR')
    remote.update(unknown)
    assert set(remote.leaves) == set(h.leaves)


def test_delta_gap_repair_duplicate_and_tombstone():
    a, b = DeltaGraph(0, 'a'), DeltaGraph(1, 'b')
    a.put('h:0:1', dict(kind='history', id='0:1', position=[2., 2., 1.5]))
    a.put('h:0:2', dict(kind='history', id='0:2', position=[5., 2., 1.5]))
    assert not b.merge(a.packet(after=1)) and 0 in b.needs_snapshot
    assert b.merge(a.packet(full=True))
    assert not b.merge(a.packet(full=True))
    a.delete('h:0:1'); assert b.merge(a.packet(after=2))
    assert b.remote[0]['h:0:1'] is None
    old = a.packet(full=True); old['session'] = 'old'
    old['sequence'] += 1
    assert not b.merge(old) and b.received[0] == 3


def test_invalid_graph_does_not_advance_source_fence():
    a, b = DeltaGraph(0), DeltaGraph(1)
    a.put('h:bad', dict(kind='history', id='bad', position=[float('nan'), 0, 0]))
    with pytest.raises(ValueError):
        b.merge(a.packet(full=True))
    assert 0 not in b.received


def test_history_nodes_cross_source_handshake_and_sparse_routes():
    m = opened(); h = AdaptiveRegions(m.bounds); h.update(m)
    a, b = MultiRobotGraph(0, m.bounds), MultiRobotGraph(1, m.bounds)
    a.update(m, np.array([2., 2., 1.5]), h, 1.)
    b.update(m, np.array([7., 2., 1.5]), h, 1.)
    a.replica.merge(b.replica.packet(full=True))
    a.update(m, np.array([4., 2., 1.5]), h, 2.)
    assert len(a.nodes) == 2 and a.handshakes > 0
    assert any(e['u'].split(':')[0] != e['v'].split(':')[0] for e in a.edges)
    assert all(m.safe_path(e['points']) for e in a.edges)


def test_graph_voronoi_uses_traversable_cost_not_euclidean():
    g = MultiRobotGraph(0, [[0, 0, 0], [16, 12, 4]])
    g.nodes = {'a': np.zeros(3), 'b': np.ones(3)}
    g.adj = {'a': [('b', 20., 0, False)], 'b': [('a', 20., 0, True)]}
    g.regions = {7: dict(status='activeR', node='b', length=1.)}
    owners, global_owners, tiers, _ = graph_voronoi(g, {0: {'a': 0.}, 1: {'b': 3.}}, {0, 1})
    assert owners[7] == 1 and global_owners['b'] == 1 and tiers[7] == 'local'
    owners, _, _, _ = graph_voronoi(g, {0: {'a': 0.}, 1: {'b': 3.}}, {0})
    assert owners[7] == 0


def test_deferred_regions_do_not_enter_bilateral_exchange_window():
    m = opened(); m.state[24:, :] = -1; m.rebuild()
    planner = FusionPlanner(0, m.bounds); position = np.array([2., 2., 1.5])
    first = planner.compute(m, position, 0., None, [], {}, [], 1, 1., False, {}, {}, {})
    assert len(first['tasks']) >= 2
    peers = {1: dict(available=True, time=2., graph_connections=dict(planner.connections))}
    cooldown = {r: 100. for r in first['tasks']}
    second = planner.compute(m, position, 0., None, [], cooldown, [], 1, 2., False, peers, {}, {})
    assert second['offer'] is None and not second['bids']


def test_pair_cvrp_matches_brute_force_and_capacity():
    xy = np.array([[1., 0.], [2., 0.], [8., 0.], [9., 0.]])
    start = np.linalg.norm(np.array([[0., 0.], [10., 0.]])[:, None]-xy[None], axis=2)
    between = np.linalg.norm(xy[:, None]-xy[None], axis=2)
    r = solve_pair([0, 1, 2, 3], start, between, [1, 1, 1, 1], {i: 0 for i in range(4)}, (0, 1))
    assert r['assignments'] == {0: 0, 1: 0, 2: 1, 3: 1}
    best = np.inf
    for assigned in itertools.product([0, 1], repeat=4):
        if max(assigned.count(0), assigned.count(1)) > r['capacity']:
            continue
        costs = []
        for drone in range(2):
            ids = [j for j in range(4) if assigned[j] == drone]
            costs.append(min((start[drone, order[0]]+sum(between[a, b] for a, b in zip(order, order[1:]))
                              for order in itertools.permutations(ids)), default=0.))
        best = min(best, sum(costs)+.25*max(costs))
    assert r['after'] == pytest.approx(best)
    assert max(r['loads']) <= r['capacity']


def test_pair_cvrp_respects_committed_task_and_unreachable():
    r = solve_pair([1, 2], [[1, 3], [3, 1]], [[0, 2], [2, 0]], [1, 1], {1: 0, 2: 0}, (0, 1), {1: 1})
    assert r['assignments'][1] == 1
    r = solve_pair([1], [[np.inf], [np.inf]], [[0]], [1], {1: 0}, (0, 1))
    assert r['status'] == 'infeasible'


def test_pair_handshake_survives_repeated_delivery():
    owners = {1: 0, 2: 0}; a, b = PairExchange(0), PairExchange(1)
    result = dict(status='optimal_window', before=10., after=4., capacity=2., loads=[1., 1.],
                  assignments={1: 0, 2: 1}, routes={'0': [1], '1': [2]})
    assert a.offer(1, result, owners, 1.)
    b.tick({0: dict(pair_transaction=a.transaction)}, owners, 1.1)
    assert b.transaction['phase'] == 'accept'
    a.tick({1: dict(pair_transaction=b.transaction)}, owners, 1.2)
    assert a.transaction['phase'] == 'commit'
    b.tick({0: dict(pair_transaction=a.transaction)}, owners, 1.3)
    a.tick({1: dict(pair_transaction=b.transaction)}, owners, 1.4)
    assert a.overrides == b.overrides == {1: 0, 2: 1}
    assert a.commits == b.commits == 1
    for _ in range(3):
        b.apply(b.transaction)
    assert b.commits == 1



@pytest.mark.parametrize('commit_before_outage', [False, True])
def test_pair_accept_lock_survives_partition_until_leader_resolution(commit_before_outage):
    owners = {1: 0, 2: 0}; a, b = PairExchange(0), PairExchange(1)
    result = dict(status='optimal_window', before=10., after=4., capacity=2., loads=[1., 1.],
                  assignments={1: 0, 2: 1}, routes={'0': [1], '1': [2]})
    a.offer(1, result, owners, 1.)
    old_leader = dict(time=1., pair_transaction=dict(a.transaction))
    b.tick({0: old_leader}, owners, 1.1)
    if commit_before_outage:
        a.tick({1: dict(time=1.1, pair_transaction=b.transaction)}, owners, 1.2)
    # Eighteen seconds without a fresh leader must not discard an accepted vote.
    b.tick({0: old_leader}, owners, 19.)
    assert b.transaction['phase'] == 'accept'
    a.tick({}, owners, 19.)
    b.tick({0: dict(time=19., pair_transaction=a.transaction)}, owners, 19.1)
    if commit_before_outage:
        # The acknowledgement can also be delayed beyond the original expiry.
        b.tick({0: dict(time=19., pair_transaction=a.transaction)}, owners, 37.)
        assert b.transaction['phase'] == 'applied'
        a.tick({1: dict(time=37., pair_transaction=b.transaction)}, owners, 37.2)
        assert a.overrides == b.overrides == {1: 0, 2: 1}
        assert a.commits == b.commits == 1
    else:
        assert a.transaction is None and b.transaction is None
        assert a.commits == b.commits == 0


def test_pair_rejects_stale_owner_revision():
    a, b = PairExchange(0), PairExchange(1)
    a.offer(1, dict(status='optimal_window', before=10., after=4., capacity=2., loads=[1., 1.],
                   assignments={1: 0, 2: 1}, routes={}), {1: 0, 2: 0}, 1.)
    b.tick({0: dict(pair_transaction=a.transaction)}, {1: 0, 2: 1}, 1.1)
    assert b.transaction is None and b.rejections == 1


def test_continuous_trajectory_endpoint_dynamics_and_roundtrip():
    m = opened(); path = np.array([[2., 2., 1.5], [6., 2., 1.5], [8., 5., 1.5]])
    t = optimize_trajectory(path, m, 3., -3.)
    assert np.allclose(t.sample(0)[0], path[0]) and np.allclose(t.sample(t.duration)[0], path[-1])
    assert np.linalg.norm(t.sample(0)[1]) < 1e-8 and np.linalg.norm(t.sample(t.duration)[2]) < 1e-8
    assert t.limits()['speed'] <= .6 and t.limits()['acceleration'] <= .8 and t.limits()['jerk'] <= 2.
    assert 1.875*abs(t.yaw_delta)/t.duration <= .65
    assert m.safe_path(t.path())
    copy = ContinuousTrajectory.from_dict(t.to_dict())
    assert np.allclose(copy.sample(1.5)[0], t.sample(1.5)[0])


def test_minimum_control_spline_is_continuous_through_waypoints():
    p = np.array([[2., 2., 1.5], [4., 3., 1.5], [7., 2., 1.5]])
    t = interpolate(p, [8., 9.], 0., 1.)
    assert np.allclose(t.sample(8.)[0], p[1])
    for k in (0, 1, 2):
        assert np.linalg.norm(t.sample(8.-1e-6)[k]-t.sample(8.+1e-6)[k]) < 1e-5


def test_continuous_curve_cannot_cut_obstacle_corner():
    m = opened(); m.state[24:32, 8:32] = 1; m.rebuild()
    path = np.array([[3., 3., 1.5], [4.5, 9., 1.5], [10., 9., 1.5], [11., 3., 1.5]])
    assert m.safe_path(path)
    t = optimize_trajectory(path, m, 0, 1)
    assert m.safe_path(t.path(.03)) and t.limits()['speed'] <= .6


def test_short_axis_aligned_recovery_polynomial_degree():
    m = opened()
    t = optimize_trajectory(np.array([[2., 2., 1.5], [2.025, 2., 1.5]]), m, 0., .2)
    assert t.limits()['speed'] <= .6 and np.isfinite(list(t.limits().values())).all()


def test_voxel_rays_do_not_mark_behind_returns_free():
    from core.exploration.voxel_mapping import VoxelMap
    m = VoxelMap([[0, 0, 0], [6, 6, 4]])
    m.integrate([1., 1., 1.5], [[3., 1., 1.5]], [True])
    assert m.state[tuple(m.indices([3., 1., 1.5]))] == 1
    assert m.state[tuple(m.indices([4., 1., 1.5]))] == -1
    m.integrate([1., 1., 1.5], [[4.5, 1., 1.5]], [False])
    assert m.state[tuple(m.indices([3., 1., 1.5]))] == 0


def test_voxel_route_really_changes_height_to_cross_overhang():
    from core.exploration.voxel_mapping import VoxelMap, VoxelRouter
    m = VoxelMap([[0, 0, 0], [8, 6, 6]])
    m.state[:] = 0; m.state[12:15, :, :9] = 1; m.rebuild()
    path = VoxelRouter(m).route(np.array([2., 3., 1.5]), np.array([6., 3., 1.5]))
    assert path is not None and path[:, 2].max() > 3. and m.safe_path(path)


def test_3d_adaptive_regions_and_information_gain():
    from core.exploration.voxel_mapping import VoxelMap
    from core.exploration.regions import visible_cells
    m = VoxelMap([[0, 0, 0], [8, 8, 6]])
    m.state[:14] = 0; m.rebuild(); h = AdaptiveRegions(m.bounds); h.update(m)
    assert h.ndim == 3 and h.split and h.tasks
    assert all(len(t.bounds[0]) == 3 for t in h.tasks.values())
    p = np.array([3., 3., 2.5])
    assert len(visible_cells(m, p, 0)) > len(visible_cells(m, p, np.pi))


def test_contingency_cells_are_disjoint_and_do_not_discard_old_grants():
    from core.exploration.decentralized import PeerLedger, inside_contingency_cell
    seeds = [[2., 2., 1.5], [2., 8.5, 1.5], [2., 18., 1.5]]
    path = [[3., 2., 1.5], [7., 3., 2.]]
    assert inside_contingency_cell(path, 0, seeds)
    assert not inside_contingency_cell(path, 1, seeds)
    assert not inside_contingency_cell([[2., 5.25, 1.5]], 0, seeds)
    ledger = PeerLedger(0); ledger.seeds = seeds
    intent = dict(token='1:1', region=7, path=[[2., 8.5, 1.5]], committed=True, created=1.)
    ledger.receive(dict(drone=1, sequence=1, time=1., position=seeds[1], intent=intent, acks=[]))
    ledger.receive(dict(drone=2, sequence=1, time=1., position=seeds[2], intent=None, acks=[]))
    own = dict(position=seeds[0], intent=None)
    assert '1:1' in ledger.acknowledge(own, 1.1)
    assert '1:1' in ledger.acknowledge(own, 10.)
    assert ledger.can_propose(path, 9, 10.)
    assert not ledger.can_propose([[2., 8.5, 1.5]], 9, 10.)


def test_committed_lease_survives_outage_but_not_explicit_revocation():
    from core.exploration.decentralized import fused_execution_lease
    seeds = [[2., 2., 1.5], [2., 8.5, 1.5], [2., 18., 1.5]]
    states = {0: dict(time=10., intent=dict(token='0:1', committed=True, voters=[1, 2], path=[seeds[0]])),
              1: dict(time=1., acks=['0:1']), 2: dict(time=1., acks=['0:1'])}
    assert fused_execution_lease(states, 0, '0:1', 10., seeds)
    states[1]['acks'] = []
    assert not fused_execution_lease(states, 0, '0:1', 10., seeds)


def test_inertial_filter_rejects_outlier_and_stays_covariance_positive():
    from core.exploration.state_estimation import InertialOdometryFilter
    f = InertialOdometryFilter(); R = np.diag([.01]*6)
    assert f.correct([0, 0, 1], [1, 0, 0], R)
    for k in range(101):
        f.predict(k*.01, [0, 0, 0])
    assert f.x[0] == pytest.approx(1.)
    assert not f.correct([100, 100, 100], [0, 0, 0], R)
    assert f.correct([1.01, .01, 1.01], [1, 0, 0], R)
    assert np.linalg.eigvalsh(f.P).min() > 0


def test_stopped_incarnation_rejoins_without_old_packet_resurrection():
    from core.exploration.decentralized import PeerLedger
    l = PeerLedger(0)
    old = dict(drone=1, sequence=100, time=1., session='old', epoch=7)
    assert l.receive(old)
    new = dict(drone=1, sequence=1, time=2., session='new', epoch=8, stopped=False, rejoin_ready=True)
    assert not l.receive(new)
    new['stopped'] = True
    assert l.receive(new)
    assert not l.receive(dict(old, sequence=999, time=4.))
    assert l.states[1]['session'] == 'new'


def test_nested_hgrid_service_leases_conflict():
    from core.exploration.decentralized import regions_conflict
    parent = dict(region=1, bounds=[[0, 0, 0], [4, 4, 4]])
    child = dict(region=12, bounds=[[0, 0, 0], [2, 2, 2]])
    neighbour = dict(region=2, bounds=[[4, 0, 0], [8, 4, 4]])
    assert regions_conflict(parent, child)
    assert not regions_conflict(parent, neighbour)


def test_invalid_sensor_restart_does_not_retire_valid_stream():
    from core.exploration.decentralized import MapReplica
    r = MapReplica([[0, 0, 0], [8, 8, 4]], 0)
    old = dict(source=0, sensor_session='old', sequence=9, time=1., indices=[[2, 2]], values=[0])
    r.merge(old)
    with pytest.raises(ValueError):
        r.merge(dict(old, sensor_session='new', sequence=1, time=2., indices=[[999, 999]]))
    assert r.sensor_sessions[0] == 'old' and r.sequences[0] == 9
    r.merge(dict(old, sequence=10, time=3.))
    assert r.sequences[0] == 10


def test_oriented_hull_retreat_keeps_unknown_boxes_blocking():
    from core.exploration.voxel_mapping import VoxelMap
    from core.exploration.decentralized import tracking_recovery
    m = VoxelMap([[0, 0, 0], [6, 6, 4]])
    m.state[:] = 0; m.state[:, :7, :] = 1; m.rebuild()
    p = np.array([3., 2.56, 1.5])
    assert not m.safe_path([p])
    path = tracking_recovery(m, p, yaw=0.)
    assert path is not None and m.safe_path([path[-1]])
    m.recovery_yaw = 0.; m.clearance = .5
    assert m.safe_path(path)
    assert not m.safe_path([[3., 2.41, 1.5]])
    m.state[:, :7, :] = -1; m.rebuild()
    assert not m.safe_path([[3., 2.41, 1.5]])


def test_source_veto_blocks_stale_foreign_edge_until_released():
    g = MultiRobotGraph(0, [[0, 0, 0], [10, 10, 4]])
    for n,x in [('a',2.), ('b',5.)]:
        g.replica.put('h:'+n, dict(kind='history', id=n, position=[x, 2., 1.5]))
    edge = dict(kind='edge', u='a', v='b', length=3., points=[[2., 2., 1.5], [5., 2., 1.5]])
    g.replica.put('e:a|b', edge)
    peer = DeltaGraph(1); peer.put('b:a|b', dict(kind='edge_block', u='a', v='b', blocked=True))
    g.replica.merge(peer.packet(full=True)); g.rebuild(); assert not g.edges
    g.replica.put('e:a|b', dict(edge, length=2.99)); g.rebuild(); assert not g.edges
    peer.put('b:a|b', dict(kind='edge_block', u='a', v='b', blocked=False))
    g.replica.merge(peer.packet(full=True)); g.rebuild(); assert len(g.edges) == 1


def test_persistent_eroi_view_states_follow_new_observations():
    m = opened(); m.state[24:] = -1; m.rebuild()
    h = AdaptiveRegions(m.bounds, levels=1); h.update(m)
    ids = {(rid, cell) for rid, views in h.view_catalog.items() for cell in views}
    assert ids and any(v['status'] == 'activeV' for views in h.view_catalog.values() for v in views.values())
    m.state[:] = 0; m.rebuild(); h.update(m)
    assert all(h.view_catalog[rid][cell]['status'] == 'deadV' for rid, cell in ids)
    m.state[:] = -1; m.rebuild(); h.update(m)
    assert all(h.view_catalog[rid][cell]['status'] == 'inactiveV' for rid, cell in ids)


def test_pair_capacity_includes_tasks_outside_exchange_window():
    r = solve_pair([1, 2], [[1, 2], [2, 1]], [[0, 1], [1, 0]], [1, 1],
                   {1: 0, 2: 0}, (0, 1), fixed_loads=[9, 0])
    assert r['assignments'] == {1: 1, 2: 1}
    assert r['loads'] == [9., 2.] and max(r['loads']) <= r['capacity']
    assert not r['before_feasible']


def test_graph_does_not_reopen_exhausted_region_from_less_informed_peer():
    g = MultiRobotGraph(0, [[0, 0, 0], [8, 8, 4]])
    g.replica.put('r:7', dict(kind='region', id=7, status='deadR', unknown=2, stamp=1.))
    peer = DeltaGraph(1); peer.put('r:7', dict(kind='region', id=7, status='activeR', unknown=50, stamp=10.))
    g.replica.merge(peer.packet(full=True)); g.rebuild()
    assert g.regions[7]['status'] == 'deadR' and g.regions[7]['unknown'] == 2
    peer.put('r:7', dict(kind='region', id=7, status='splitR', unknown=2, stamp=11.))
    g.replica.merge(peer.packet(full=True)); g.rebuild(); assert g.regions[7]['status'] == 'splitR'


def test_observed_service_feedback_crosses_graph_delta_without_raw_map():
    a, b = MultiRobotGraph(0, [[0, 0, 0], [8, 8, 4]]), MultiRobotGraph(1, [[0, 0, 0], [8, 8, 4]])
    a.replica.put('s:7', dict(kind='region_service', id=7, stamp=20., defer_until=620., observed_new_cells=4))
    b.replica.merge(a.replica.packet()); b.rebuild()
    assert b.services[7]['defer_until'] == 620.
    b.replica.put('s:7', dict(kind='region_service', id=7, stamp=10., defer_until=610., observed_new_cells=8))
    b.rebuild(); assert b.services[7]['defer_until'] == 620.


def test_lidar_self_filter_retains_close_external_obstacles():
    from core.exploration.voxel_mapping import lidar_return_mask
    points = np.array([[.6, 0, 0], [.2, 0, -.32], [0, 0, -.6], [4.5, 0, 0], [np.inf, 0, 0]])
    assert lidar_return_mask(points).tolist() == [True, False, True, False, False]


@pytest.mark.parametrize('geometry', [
    dict(type='aabb', min=[.9, .9, 1.1], max=[1.1, 1.1, 1.9]),
    dict(type='cylinder', center_xy=[1., 1.], radius=.2, z_range=[1.1, 1.9])])
def test_truth_uses_entire_voxel_not_only_center(tmp_path, geometry):
    import json
    from core.exploration.voxel_mapping import VoxelTruth, VoxelMap
    scene = tmp_path/'map.json'
    scene.write_text(json.dumps(dict(bounds=[[0, 0, 0], [3, 3, 3]], obstacles=[geometry])))
    truth = VoxelTruth(scene, resolution=1.)
    # A small obstacle straddles four cells, but contains none of their centers.
    assert np.count_nonzero(truth.occupied[:, :, 1]) == 4
    assert not truth.center_occupied[:, :, 1].any()
    observed = VoxelMap(truth.bounds, resolution=1.)
    observed.state[:] = truth.occupied.astype(np.int8)
    metrics = truth.coverage_metrics(observed)
    assert metrics['coverage'] == 1.
    assert metrics['legacy_center_free_coverage'] == pytest.approx(14/18)
    assert metrics['truth_free_voxels'] == metrics['observed_free_voxels'] == 14


def test_truth_dynamic_voxel_intersections_clear_with_obstacle(tmp_path):
    import json
    from core.exploration.voxel_mapping import VoxelTruth
    scene = tmp_path/'map.json'
    scene.write_text(json.dumps(dict(bounds=[[0, 0, 0], [3, 3, 3]], obstacles=[])))
    truth = VoxelTruth(scene, resolution=1.)
    obstacle = dict(type='cylinder', center_xy=[1., 1.], radius=.2, z_range=[1.1, 1.9])
    truth.dynamic_snapshot(1, [obstacle]); assert truth.occupied.sum() == 13
    truth.dynamic_snapshot(2, []); assert truth.occupied.sum() == 9
    truth.dynamic_snapshot(1, [obstacle]); assert truth.occupied.sum() == 9



def test_delayed_localization_rewinds_imu_and_keeps_outlier_gate():
    from core.exploration.state_estimation import InertialOdometryFilter
    f = InertialOdometryFilter(); R = np.diag([.007**2]*3+[.015**2]*3)
    p = np.zeros(3); v = np.zeros(3); truth = [(p.copy(), v.copy())]
    f.predict(0., np.zeros(3)); assert f.correct_at(0., p, v, R)
    accepted = 0
    for k in range(1, 251):
        a = np.array([0., 0., 5. if k < 40 else -2. if k < 90 else .5])
        p += v*.01+.5*a*.01**2; v += a*.01; truth.append((p.copy(), v.copy()))
        f.predict(k*.01, a)
        if k % 5 == 0 and k >= 10:
            past_p, past_v = truth[k-7]
            result = f.correct_at((k-7)*.01, past_p+(1000. if k == 100 else 0.), past_v, R)
            assert result == (k != 100)
            accepted += result
        assert np.allclose(f.x[:3], p, atol=1e-8)
        assert np.allclose(f.x[3:6], v, atol=1e-8)
    assert accepted > 40 and f.rejected == 1


def test_filter_lag_buffer_is_bounded_and_rejects_stale_measurement():
    from core.exploration.state_estimation import InertialOdometryFilter
    f = InertialOdometryFilter(); R = np.eye(6)*.001
    f.predict(0., np.zeros(3)); f.correct_at(0., np.zeros(3), np.zeros(3), R)
    for k in range(1, 1001):f.predict(k*.01, np.zeros(3))
    before = f.x.copy(); assert len(f.history) <= 200
    assert not f.correct_at(7., np.ones(3), np.zeros(3), R)
    assert np.array_equal(f.x, before) and f.time == 10.
    assert f.correct_at(9.9, np.zeros(3), np.zeros(3), R)
    assert f.time == 10. and len(f.history) <= 200


def test_unresolved_ground_contact_impulse_does_not_latch_innovation_rejection():
    from core.exploration.state_estimation import InertialOdometryFilter
    f = InertialOdometryFilter(); R = np.diag([.007**2]*3+[.015**2]*3)
    # Spawn drops 2 cm onto its hull: contact removes downward velocity, but
    # sampled Gazebo IMU goes directly from free fall to gravity support.
    f.predict(.05, [0., 0., -9.81])
    assert f.correct_at(.05, [2., 2., .087], [0., 0., -.5], R)
    for k in range(6, 11):f.predict(k*.01, [0., 0., -9.81 if k == 6 else 0.])
    assert f.correct_at(.10, [2., 2., .080], [0., 0., 0.], R)
    for k in range(11, 51):
        f.predict(k*.01, [0., 0., 0.])
        if k % 5 == 0:assert f.correct_at(k*.01, [2., 2., .080], [0., 0., 0.], R)
    assert abs(f.x[2]-.08) < .002 and abs(f.x[5]) < .002
    f.predict(.51, [0., 0., 0.])
    assert not f.correct_at(.51, [2., 2., 1000.], [0., 0., 0.], R)
