"""Dimension/transport adaptation of the archived 7136591 centralized policy.

The vendored coordinator and allocator are byte-for-byte historical sources.
Their dependencies are injected only in this benchmark process, never production.
"""
import copy
import math
import time
import numpy as np
from scipy.ndimage import maximum_filter, distance_transform_edt
from scipy.signal import fftconvolve
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from core.exploration.voxel_mapping import VoxelMap
from core.exploration.graph import route_pool
from core.exploration.decentralized import tracking_recovery
from core.planning.continuous_trajectory import optimize_trajectory
from vendor import coordinator as archived


class FrontierVoxelMap(VoxelMap):
    def __init__(self, bounds):
        super().__init__(bounds, flight_limits=(.7, 3.1))

    def frontier_targets(self, spacing=2., limit=24):
        # Historical geometric heuristic extended from a disk to a 2 m sphere.
        # No Hgrid, MR-DTG or fused observation-position search is used here.
        front = (self.state == 0) & maximum_filter(self.state == -1, size=3)
        if not np.any(front) or not np.any(self.safe):
            return {}
        distance = distance_transform_edt(~front)*self.resolution
        cells = np.argwhere(self.safe & (distance < 1.6))
        radius = int(np.ceil(2./self.resolution))
        xyz = np.indices((2*radius+1,)*3)-radius
        kernel = np.sum(xyz*xyz, axis=0)*self.resolution**2 <= 4.+1e-12
        gains = np.rint(fftconvolve((self.state == -1).astype(float), kernel, mode='same'))
        chosen = {}
        for cell in sorted(cells, key=lambda c: (-gains[tuple(c)], *map(int, c))):
            gain = float(gains[tuple(cell)])
            if gain < 3:
                continue
            point = self.points([cell])[0]
            if any(np.linalg.norm(point-v['position']) < spacing for v in chosen.values()):
                continue
            key = int(np.ravel_multi_index(tuple(cell), self.shape))
            chosen[key] = dict(position=point, gain=gain, cell=tuple(cell))
            if len(chosen) >= limit:
                break
        return chosen


class GridTopology:
    """The old full free-cell graph, generalized from 4 to 6 neighbors."""
    def __init__(self, runtime):
        self.runtime = runtime; self.cells = np.argwhere(runtime.safe)
        self.positions = runtime.points(self.cells)
        self.ids = np.ravel_multi_index(tuple(self.cells.T), runtime.shape)
        self.lookup = np.full(runtime.shape, -1, int)
        self.lookup[tuple(self.cells.T)] = np.arange(len(self.cells))
        rows = []; cols = []
        for axis in range(3):
            for sign in (-1, 1):
                shifted = self.cells.copy(); shifted[:, axis] += sign
                valid = np.all((shifted >= 0) & (shifted < runtime.shape), axis=1)
                a = np.flatnonzero(valid); b = self.lookup[tuple(shifted[valid].T)]; ok = b >= 0
                rows.extend(a[ok]); cols.extend(b[ok])
        self.matrix = csr_matrix((np.full(len(rows), runtime.resolution), (rows, cols)), shape=(len(self.cells),)*2)
        self.version = runtime.version

    def node(self, position):
        idx = self.runtime.indices(position)
        if np.all(idx >= 0) and np.all(idx < self.runtime.shape):
            node = self.lookup[tuple(idx)]
            if node >= 0:
                return int(node)
        order = np.argsort(np.linalg.norm(self.positions-position, axis=1))[:12]
        return next((int(i) for i in order if self.runtime.safe_path([position, self.positions[i]])), None)

    def distances(self, positions):
        return np.array([np.full(len(self.cells), np.inf) if (i := self.node(p)) is None
                         else dijkstra(self.matrix, indices=i) for p in positions])


def target_heading(runtime, position, current_yaw):
    """Translate the old omnidirectional gain proxy to one finite-FOV yaw.

Only orientation is chosen; the old frontier position and assignment are fixed.
"""
    cells = np.argwhere(runtime.state == -1); delta = runtime.points(cells)-position
    delta = delta[np.linalg.norm(delta, axis=1) <= 2.]
    if not len(delta):
        return float(current_yaw)
    bearings = np.arctan2(delta[:, 1], delta[:, 0])
    angles = np.arange(12)*math.pi/6
    scores = [np.count_nonzero(np.abs(np.arctan2(np.sin(bearings-a), np.cos(bearings-a))) <= math.pi/3) for a in angles]
    return float(angles[int(np.argmax(scores))])


def central_execution_lease(states, drone, token, now, seeds, timeout=3.):
    """An actual centralized authorization, not a fabricated peer quorum."""
    state = states.get(drone, {}); intent = state.get('intent') or {}
    return bool(token and state.get('available') and state.get('ready')
                and 0 <= now-state.get('time', -1e9) <= timeout
                and 0 <= now-state.get('central_time', -1e9) <= timeout
                and intent.get('token') == token and intent.get('committed')
                and not intent.get('retiring'))


archived.ObservedMap = FrontierVoxelMap
archived.TopologyGraph = GridTopology
archived.route_pool = route_pool
_worker = None


def plan_step(payload):
    global _worker
    if _worker is None:
        _worker = archived.SearchCoordinator(payload['bounds'], 'gvp_pairwise')
    core = _worker; start = time.monotonic()
    for i in payload['rejected']:
        core.release(i, 'blocked')
    positions = {int(i): np.array(p) for i, p in payload['positions'].items()}
    for cells, values in payload['observations']:
        core.map.update(cells, values)
    core.map.rebuild()
    available = set(payload['available']); recovery = {}
    for i in sorted(available):
        if not core.map.safe_path([positions[i]]):
            available.remove(i)
            snapshot = core.reserved_runtime(i, positions)
            path = tracking_recovery(snapshot, positions[i], yaw=payload['yaws'][i])
            if path is not None:
                snapshot.clearance = .5; snapshot.recovery_yaw = payload['yaws'][i]
                recovery[i] = (path, snapshot)
    commands = core.step(positions, available, payload['arrived'], [], payload['time'])
    candidates = {}; failures = []
    for i, packet in list(commands.items()):
        if packet is None:
            continue
        pool = core.pools[i]
        candidates[i] = [dict(**c.metadata(), points=c.path.tolist()) for c in [pool.active]+pool.backups]
        snapshot = core.reserved_runtime(i, positions)
        yaw = target_heading(core.map, np.array(packet['path'][-1]), payload['yaws'][i])
        try:
            curve = optimize_trajectory(packet['path'], snapshot, payload['yaws'][i], yaw)
            packet.update(trajectory=curve.to_dict(), yaw=yaw, recovery=False)
        except ValueError as exc:
            failures.append(dict(drone=i, reason=str(exc))); core.release(i, 'blocked'); commands[i] = None
    # Same short, certified body-envelope retreat used by the current runtime.
    for i, (path, snapshot) in recovery.items():
        if i in payload['busy']:
            continue
        try:
            curve = optimize_trajectory(path, snapshot, payload['yaws'][i], payload['yaws'][i], speed_limit=.15, acceleration_limit=.2)
            commands[i] = dict(task=-1, epoch=0, map_version=core.map.version, path=path.tolist(), yaw=payload['yaws'][i], trajectory=curve.to_dict(), recovery=True)
        except ValueError as exc:
            failures.append(dict(drone=i, reason=str(exc)))
    events = core.events; core.events = []
    stats = dict(core.stats)
    return dict(commands=commands, candidates=candidates, events=events, stats=stats, failures=failures,
                compute_wall_s=time.monotonic()-start, snapshot_time=payload['time'])


def view_finished(execution, epoch, duration):
    # 'view_observed' reason lasts one 50 Hz tick; 5 Hz reports may skip it.
    # Completion is the persistent epoch/arrival/trajectory-time contract.
    return bool(execution.get('epoch') == epoch and execution.get('arrived')
                and execution.get('trajectory_time', -1.) >= duration-1e-6)
