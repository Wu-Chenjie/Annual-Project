import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'next_project'))
from core.exploration.mapping import ObservedMap, RaySensorWorld
from core.exploration.sparse_graph import SparseTopology, length
from core.exploration.decentralized import MapReplica, PeerLedger, paths_conflict
from core.exploration.regions import RegionTasks, optimize_tour, visible_cells, ObservationPlanner


def open_map():
    m = ObservedMap([[0, 0, 0], [16, 12, 4]])
    m.state[:] = 0; m.rebuild(); return m


def test_sparse_compression_and_obstacle_detour():
    m = open_map(); m.state[28:32, :30] = 1; m.rebuild(); graph = SparseTopology(m)
    route = graph.route([3, 3, 1.5], [12, 3, 1.5])
    assert route is not None and m.safe_path(route)
    assert length(route) > 12
    assert len(graph.nodes) < graph.free_cells/8
    assert all(m.safe_path(e.path) for e in graph.edges)


def test_sparse_graph_keeps_loop_and_disconnected_components():
    m = open_map(); m.state[20:38, 16:28] = 1; m.rebuild(); g = SparseTopology(m)
    assert g.route([2, 2, 1.5], [13, 9, 1.5]) is not None
    m.state[30:34, :] = 1; m.rebuild(); g = SparseTopology(m)
    assert g.route([2, 2, 1.5], [13, 9, 1.5]) is None


def observation(source, sequence, time, value):
    return dict(source=source, sequence=sequence, time=time, indices=[[5, 5]], values=[value])


def test_replica_reordering_duplicate_and_dynamic_freeing():
    a = MapReplica([[0, 0, 0], [8, 8, 4]], 0)
    assert a.merge(observation(1, 3, 3., 1))
    assert not a.merge(observation(1, 2, 2., 0))
    assert not a.merge(observation(1, 3, 3., 1))
    assert a.merge(observation(2, 1, 4., 0))
    assert not a.merge(observation(1, 4, 3.5, 1))
    assert a.map.state[5, 5] == 0


def state(i, seq=1, time=1., intent=None):
    return dict(drone=i, sequence=seq, time=time, position=[2, 2+i*4, 1.5], available=True, bids={'7': 10.+i}, intent=intent, acks=[])


def test_auction_tie_and_unavailable_redistribution():
    ledger = PeerLedger(0); own = state(0)
    a, b = state(1), state(2); a['bids']['7'] = 1.
    ledger.receive(a); ledger.receive(b)
    assert ledger.owners(own, 1.1)[7] == 1
    a.update(sequence=2, available=False); ledger.receive(a)
    assert ledger.owners(own, 1.1)[7] == 0
    assert not ledger.receive(state(1))


def test_no_quorum_no_execution_and_partition_freeze():
    ledger = PeerLedger(0); a, b = state(1), state(2)
    a['acks'] = ['0:1']; ledger.receive(a); ledger.receive(b)
    assert not ledger.quorum('0:1', 1.1)
    b.update(sequence=2, acks=['0:1']); ledger.receive(b)
    assert ledger.quorum('0:1', 1.1)
    assert not ledger.quorum('0:1', 5.)


def test_conflicting_intents_cannot_both_receive_reservation():
    ledger = PeerLedger(2)
    path = [[3, 4, 1.5], [7, 4, 1.5]]
    a, b = state(0), state(1)
    a['intent'] = dict(token='0:1', path=path, region=2, created=1., committed=False)
    b['intent'] = dict(token='1:1', path=path[::-1], region=3, created=1., committed=False)
    ledger.receive(a); ledger.receive(b)
    grants = ledger.acknowledge(state(2), 1.1)
    assert grants == ['0:1']
    assert not ledger.can_propose(path, 4, 1.1)
    assert paths_conflict(path, path[::-1])


def test_region_tasks_stable_and_multiple_views():
    m = open_map(); m.state[25:, :] = -1; m.rebuild()
    a = RegionTasks(m); m.state[25:27, 12:30] = 0; m.rebuild(); b = RegionTasks(m)
    assert set(a.tasks) & set(b.tasks)
    assert any(len(t.viewpoints) > 1 for t in a.tasks.values())
    assert all(len(t.bounds) == 2 for t in a.tasks.values())


def test_view_gain_uses_yaw_and_known_wall_occlusion():
    m = open_map(); m.state[24:, :] = -1; m.rebuild()
    forward = visible_cells(m, [5, 6, 1.5], 0)
    backward = visible_cells(m, [5, 6, 1.5], np.pi)
    assert len(forward) > len(backward)
    m.state[23, :] = 1; m.rebuild()
    assert len(visible_cells(m, [5, 6, 1.5], 0)) == 0


def test_actual_sensor_fov_and_wall_do_not_reveal_hidden_truth():
    file = Path(__file__).resolve().parents[4]/'next_project/maps/search_maze.json'
    w = RaySensorWorld(file)
    front, _ = w.observe([2, 2, 1.5], 0)
    back, _ = w.observe([2, 2, 1.5], np.pi)
    assert len(set(map(tuple, front))-set(map(tuple, back))) > 20
    assert len(front) < len(w.observe([2, 2, 1.5])[0])


def test_observation_motion_returns_checked_pool_and_yaw():
    m = open_map(); m.state[25:, :] = -1; m.rebuild(); g = SparseTopology(m)
    tasks = RegionTasks(m).tasks
    task = min(tasks.values(), key=lambda t: np.linalg.norm(t.entry-[3, 5, 1.5]))
    result = ObservationPlanner().plan(m, g, [3, 5, 1.5], 0., task, 1)
    assert result and result['gain'] > 0
    assert abs(result['yaw']) < np.pi/2
    pool = result['pool']; assert len(pool.backups) <= 5
    assert all(m.safe_path(c.path) for c in [pool.active]+pool.backups)


def test_agents_have_no_truth_map_or_central_search_subscription():
    code = (Path(__file__).resolve().parents[1]/'scripts/decentralized_agent_node.py').read_text()
    assert 'RaySensorWorld' not in code and "declare_parameter('map_file'" not in code
    assert '/search/control' not in code and 'SearchCoordinator' not in code


def test_bad_observation_does_not_poison_sequence_fence():
    import pytest
    replica = MapReplica([[0, 0, 0], [8, 8, 4]], 0)
    bad = observation(1, 1, 1., 7)
    with pytest.raises(ValueError):
        replica.merge(bad)
    assert replica.merge(observation(1, 1, 1., 0))


def test_clock_transport_skew_is_bounded():
    ledger = PeerLedger(0)
    ledger.receive(state(1, time=1.01)); ledger.receive(state(2, time=1.01))
    assert ledger.fresh(1.)
    assert not ledger.fresh(.8)


def test_maze_graph_can_route_through_narrow_portal():
    file = Path(__file__).resolve().parents[4]/'next_project/maps/search_maze.json'
    world = RaySensorWorld(file); m = ObservedMap(world.bounds)
    m.state[:] = world.occupied; m.rebuild(); graph = SparseTopology(m)
    path = graph.route([14.625, 11.125, 1.5], [17.125, 11.125, 1.5])
    assert path is not None and m.safe_path(path)
    assert len(graph.nodes) < graph.free_cells/8


def test_tour_optimization_is_obstacle_aware_and_pins_active_region():
    from core.exploration.regions import RegionTask, insertion_bids
    m = open_map(); m.state[28:32, :30] = 1; m.rebuild(); graph = SparseTopology(m)
    tasks = {i: RegionTask(i, [], 30, [np.array(p)], np.array(p)) for i, p in enumerate([[3, 8, 1.5], [12, 3, 1.5], [12, 9, 1.5]])}
    tour, cost = optimize_tour([3, 3, 1.5], list(tasks), tasks, graph, pinned=0)
    assert tour[0] == 0 and set(tour) == set(tasks)
    assert cost >= graph.distance([3, 3, 1.5], tasks[0].entry)/.6
    bids, route, workload, proposals = insertion_bids([3, 3, 1.5], [0, 2], tasks, graph, active=0)
    assert bids[0] == -1e6 and proposals[1][0] == 0
    assert set(proposals[1]) == set(tasks)


def test_tracking_recovery_cannot_enter_unknown_or_cross_wall():
    from core.exploration.decentralized import tracking_recovery
    m = open_map(); m.state[28:, :] = -1; m.rebuild()
    p = np.array([6.3, 5, 1.5])
    assert not m.safe_path([p])
    path = tracking_recovery(m, p)
    assert path is not None and m.safe_path([path[-1]])
    assert tracking_recovery(m, np.array([7.5, 5, 1.5])) is None


def test_executor_rechecks_grant_and_cancellation_even_after_commit():
    from core.exploration.decentralized import execution_lease
    states = {i: state(i) for i in range(3)}
    states[0]['intent'] = dict(token='0:8', committed=True)
    for i in [1, 2]:
        states[i]['acks'] = ['0:8']
    assert execution_lease(states, 0, '0:8', 1.1)
    states[2]['acks'] = []
    assert not execution_lease(states, 0, '0:8', 1.1)
    states[2]['acks'] = ['0:8']; states[0]['intent']['retiring'] = True
    assert not execution_lease(states, 0, '0:8', 1.1)
    states[0]['intent'] = dict(token='0:10', committed=True)
    assert not execution_lease(states, 0, '0:8', 1.1)
    assert not execution_lease(states, 0, '0:10', 5.)
