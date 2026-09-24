"""History / EROI graph with per-source delta replication and Dijkstra handshakes.

Edges contain traversable polylines, not straight-line visibility assumptions.
Occupancy arrays are never serialized by this transport. Received edges guide
travel; the executing UAV still checks each local segment against its own sensor.
"""
import copy
import hashlib
import heapq
import json
import math
import uuid
import numpy as np
from .sparse_graph import length
from .hierarchy import ExplorationRegion


def observation_grid(runtime):
    description = [list(map(int, runtime.shape)), runtime.origin.tolist(), float(runtime.resolution)]
    return hashlib.sha256(json.dumps(description, separators=(',', ':')).encode()).hexdigest()


def grid_tree(runtime, start, radius=6., router=None):
    root = tuple(runtime.indices(start))
    if not runtime.safe_path([start]):
        return {}, {}
    if router is not None and router.node(start) >= 0:
        distances, previous = router.search(start, limit=radius+1e-8)
        reached = np.flatnonzero(np.isfinite(distances))
        cells = [tuple(c) for c in router.cells[reached]]
        return (dict(zip(cells, distances[reached])),
                {c: tuple(router.cells[previous[i]]) for c, i in zip(cells, reached) if previous[i] >= 0})
    distances = {root: 0.}; previous = {}; queue = [(0., root)]
    dimensions = runtime.state.ndim
    shifts = [tuple(sign if j == axis else 0 for j in range(dimensions)) for axis in range(dimensions) for sign in (-1, 1)]
    while queue:
        cost, u = heapq.heappop(queue)
        if cost > distances[u]+1e-8:
            continue
        for shift in shifts:
            v = tuple(u[j]+shift[j] for j in range(dimensions))
            if any(v[j] < 0 or v[j] >= runtime.shape[j] for j in range(dimensions)) or not runtime.safe[v]:
                continue
            trial = cost+runtime.resolution
            if trial <= radius and trial < distances.get(v, np.inf)-1e-8:
                distances[v] = trial; previous[v] = u; heapq.heappush(queue, (trial, v))
    return distances, previous


def tree_path(runtime, previous, cell):
    cells = [cell]
    while cell in previous:
        cell = previous[cell]; cells.append(cell)
    return runtime.points(cells[::-1])


def simplify(path, runtime):
    p = np.asarray(path); result = [p[0]]; k = 0
    while k < len(p)-1:
        j = len(p)-1
        while j > k+1 and not runtime.safe_path([p[k], p[j]]):
            j -= 1
        result.append(p[j]); k = j
    return np.array(result)


class DeltaGraph:
    """Ordered, idempotent source streams; snapshots repair missed prefixes.

    A source incarnation is pinned on first reception. Restarted sources require
    explicit rejoin/reset rather than letting delayed old sessions resurrect data.
    """
    def __init__(self, source, session=None):
        self.source = int(source); self.session = session or uuid.uuid4().hex
        self.sequence = 0; self.records = {}; self.journal = []
        self.received = {}; self.sessions = {}; self.remote = {}; self.needs_snapshot = set()
        self.rejected = 0; self.applied = 0

    def put(self, key, value):
        if self.records.get(key) == value:
            return
        self.sequence += 1
        self.records[key] = copy.deepcopy(value)
        self.journal.append(dict(sequence=self.sequence, key=key, value=copy.deepcopy(value)))

    def delete(self, key):
        if key in self.records:
            self.put(key, None)

    def packet(self, after=0, full=False):
        if self.journal and after < self.journal[0]['sequence']-1:
            full = True
        packet = dict(schema='annual.mrdtg/1', source=self.source, session=self.session,
                    base=0 if full else after, sequence=self.sequence, full=full,
                    records=self.records if full else None,
                    changes=[] if full else [e for e in self.journal if e['sequence'] > after])
        self.journal = [e for e in self.journal if e['sequence'] > after][-5000:]
        return packet

    def merge(self, packet):
        if packet.get('schema') != 'annual.mrdtg/1':
            raise ValueError('Unsupported MR-DTG schema')
        src = int(packet['source']); seq = int(packet['sequence']); session = packet['session']
        if src == self.source:
            return False
        if src in self.sessions and self.sessions[src] != session:
            self.rejected += 1; return False
        old = self.received.get(src, 0)
        if seq <= old:
            return False
        if not packet['full'] and packet['base'] > old:
            self.needs_snapshot.add(src); return False
        target = copy.deepcopy(self.remote.get(src, {}))
        if packet['full']:
            target = copy.deepcopy(packet['records'])
        else:
            changes = [e for e in packet['changes'] if e['sequence'] > old]
            if [e['sequence'] for e in changes] != list(range(old+1, seq+1)):
                self.needs_snapshot.add(src); return False
            for e in changes:
                target[e['key']] = copy.deepcopy(e['value'])
        # Validate before advancing the source fence.
        for value in target.values():
            if value is None:
                continue
            if value['kind'] == 'history' and not np.isfinite(value['position']).all():
                raise ValueError('Nonfinite history node')
            if value['kind'] == 'edge':
                p = np.asarray(value['points'], float)
                if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all():
                    raise ValueError('Invalid graph edge')
            if value['kind'] == 'observed_cells':
                if (not isinstance(value.get('block'), int) or not 0 <= value['block'] < 1000000 or
                        not isinstance(value.get('grid'), str) or len(value['grid']) != 64 or
                        not isinstance(value.get('bits'), str) or len(value['bits']) != 64 or
                        any(c not in '0123456789abcdef' for c in value['bits']+value['grid'])):
                    raise ValueError('Invalid observed-cell receipt')
                int(value['bits'], 16)
        self.remote[src] = target; self.received[src] = seq; self.sessions[src] = session
        self.needs_snapshot.discard(src); self.applied += 1
        return True

    def rejoin(self, source, session):
        """Called only after the peer lease protocol accepted a stopped incarnation."""
        if self.sessions.get(source) == session:
            return
        self.remote.pop(source, None); self.received.pop(source, None)
        self.sessions[source] = session; self.needs_snapshot.add(source)

    def values(self):
        for source, records in [(self.source, self.records), *sorted(self.remote.items())]:
            for value in records.values():
                if value is not None:
                    yield source, value


class MultiRobotGraph:
    def __init__(self, source, bounds):
        self.source = source; self.replica = DeltaGraph(source)
        self.bounds = np.asarray(bounds); self.counter = 0; self.own_nodes = {}
        self.nodes = {}; self.edges = []; self.adj = {}; self.regions = {}; self.region_stamps = {}
        self.trees = {}; self.last_version = -1; self.free_cells = 0; self.version = 0
        self.handshakes = 0; self.attachments = {}; self.runtime = None
        self.services = {}
        self.coverage = {}; self.coverage_cache = None
        self.handshake_cache = {}

    def record_observation(self, runtime):
        """Actual locally sensed cells, in 256-bit receipts; no occupancy values.

        These records only discount redundant information gain. They never
        authorize traversal, update a voxel map or predict an unseen ray endpoint.
        A restarted source reconstructs them from its persisted local sensor map.
        """
        grid = observation_grid(runtime)
        packed = np.packbits((runtime.state != -1).ravel(), bitorder='little').tobytes()
        for offset in range(0, len(packed), 32):
            block = offset//32
            bits = int.from_bytes(packed[offset:offset+32], 'little')
            key = f'c:{grid}:{block}'
            old = self.replica.records.get(key)
            if old:
                bits |= int(old['bits'], 16)
            if bits:
                self.replica.put(key, dict(kind='observed_cells', grid=grid, block=block, bits=f'{bits:064x}'))

    def observed_mask(self, runtime):
        grid = observation_grid(runtime); blocks = self.coverage.get(grid, {})
        key = (grid, tuple(sorted(blocks.items())))
        if self.coverage_cache is None or self.coverage_cache[0] != key:
            mask = np.zeros(runtime.state.size, dtype=bool)
            for block, bits in blocks.items():
                start = block*256
                if start >= len(mask):
                    continue
                values = np.unpackbits(np.frombuffer(bits.to_bytes(32, 'little'), dtype=np.uint8), bitorder='little')
                mask[start:start+256] = values[:len(mask[start:start+256])]
            self.coverage_cache = (key, mask)
        return self.coverage_cache[1]

    def rebuild(self):
        nodes = {}; edge_records = {}; regions = {}; stamps = {}; blocked = set(); services = {}; coverage = {}
        for source, v in self.replica.values():
            if v['kind'] == 'history':
                nodes[v['id']] = np.array(v['position'])
            elif v['kind'] == 'edge':
                key = tuple(sorted((v['u'], v['v'])))
                if key not in edge_records or v['length'] < edge_records[key]['length']:
                    edge_records[key] = v
            elif v['kind'] == 'edge_block' and v['blocked']:
                blocked.add(tuple(sorted((v['u'], v['v']))))
            elif v['kind'] == 'region':
                rid = int(v['id'])
                # Unknown volume only decreases. A less-informed peer cannot
                # reopen a completed/split region by publishing a later stamp.
                stamp = (v['status'] == 'splitR', v['status'] == 'deadR', -v['unknown'], v['stamp'], source)
                if rid not in stamps or stamp > stamps[rid]:
                    regions[rid] = v; stamps[rid] = stamp
            elif v['kind'] == 'region_service':
                rid = int(v['id'])
                # New successful sensing clears an older deferral. Selecting by
                # retry deadline would make a stale failure override that success.
                if (v['stamp'], source) > services.get(rid, {}).get('_order', (-1., -1)):
                    services[rid] = dict(v, _order=(v['stamp'], source))
            elif v['kind'] == 'observed_cells':
                blocks = coverage.setdefault(v['grid'], {})
                blocks[v['block']] = blocks.get(v['block'], 0) | int(v['bits'], 16)
        self.nodes = nodes; self.edges = []; self.adj = {n: [] for n in nodes}
        for key, edge in edge_records.items():
            if key in blocked:
                continue
            if edge['u'] in nodes and edge['v'] in nodes:
                index = len(self.edges); self.edges.append(edge)
                self.adj[edge['u']].append((edge['v'], edge['length'], index, False))
                self.adj[edge['v']].append((edge['u'], edge['length'], index, True))
        self.regions = regions; self.region_stamps = stamps; self.services = services; self.coverage = coverage

    def update(self, runtime, position, hierarchy, now):
        self.runtime = runtime; self.free_cells = int(runtime.safe.sum()); self.version = runtime.version
        from .voxel_mapping import VoxelRouter
        self.local_router = VoxelRouter(runtime) if runtime.state.ndim == 3 else None
        self.rebuild()
        # A source observing an occupied edge vetoes all stale copies of that
        # edge. Its veto is cleared only after the entire corridor is locally
        # safe again, never merely because a peer repeats its old edge record.
        from scipy.spatial import cKDTree
        metric = getattr(runtime, 'metric', np.ones(3))
        occupied = runtime.points(np.argwhere(runtime.state == 1))
        tree = cKDTree(occupied*metric) if len(occupied) else None
        records = {tuple(sorted((v['u'], v['v']))): v for _, v in self.replica.values() if v['kind'] == 'edge'}
        for key, edge in records.items():
            p = np.asarray(edge['points'])
            samples = np.vstack([p]+[np.linspace(a, b, max(2, int(np.linalg.norm(b-a)/.15)+1)) for a, b in zip(p[:-1], p[1:])])
            veto_key = 'b:'+'|'.join(key)
            if tree is not None and np.any(tree.query(samples*metric)[0] < runtime.clearance):
                self.replica.put(veto_key, dict(kind='edge_block', u=key[0], v=key[1], blocked=True))
            elif self.replica.records.get(veto_key) and runtime.safe_path(p):
                self.replica.put(veto_key, dict(kind='edge_block', u=key[0], v=key[1], blocked=False))
        here, prev = grid_tree(runtime, position, router=self.local_router)
        nearby = [(here.get(tuple(runtime.indices(p)), np.inf), n) for n, p in self.nodes.items()]
        best = min(nearby, default=(np.inf, None))
        if here and best[0] > 2.5:
            self.counter += 1; nid = f'{self.source}:{self.replica.session[:8]}:{self.counter}'
            # Grid centers guarantee that tree paths connect their exact endpoints.
            point = runtime.points([runtime.indices(position)])[0]
            self.own_nodes[nid] = point
            self.replica.put('h:'+nid, dict(kind='history', id=nid, position=point.tolist()))
            self.rebuild()
        # Maintain local trees for all locally visible history nodes, including
        # foreign nodes. Overlapping trees create cross-robot handshake edges.
        trees = {}
        for nid, point in self.nodes.items():
            if np.linalg.norm(point-position) > 8. or not runtime.safe_path([point]):
                continue
            cached = self.trees.get(nid)
            # A distant observation must not invalidate every history tree.
            # Include occupied/unknown geometry and the inflated safe mask in
            # the entire search envelope, including the clearance border.
            root = runtime.indices(point); radius = int(np.ceil((6.+runtime.clearance)/runtime.resolution))+2
            area = tuple(slice(max(0, int(c)-radius), min(s, int(c)+radius+1)) for c, s in zip(root, runtime.shape))
            signature = (runtime.state[area].tobytes(), runtime.safe[area].tobytes())
            trees[nid] = cached if cached and cached[0] == signature else (signature, *grid_tree(runtime, point, router=self.local_router))
        self.trees = trees
        ids = sorted(trees)
        handshake_cache = {}
        for i, u in enumerate(ids):
            _, du, pu = trees[u]
            for v in ids[i+1:]:
                _, dv, pv = trees[v]
                key = 'e:'+'|'.join(sorted((u, v)))
                signature = (trees[u][0], trees[v][0])
                handshake_cache[key] = signature
                if self.handshake_cache.get(key) == signature:
                    continue
                common = du.keys() & dv.keys()
                if not common:
                    self.replica.delete(key); continue
                cell = min(common, key=lambda c: (du[c]+dv[c], c))
                a = tree_path(runtime, pu, cell); b = tree_path(runtime, pv, cell)
                points = simplify(np.vstack([a, b[-2::-1]]), runtime)
                if len(points) < 2 or not runtime.safe_path(points):
                    continue
                record = dict(kind='edge', u=u, v=v, length=round(length(points), 4),
                              points=points.round(5).tolist())
                if self.replica.records.get(key) != record:
                    self.handshakes += 1
                self.replica.put(key, record)
        self.handshake_cache = handshake_cache
        # Each active EROI attaches to exactly one history node by its best view.
        for rid, state in hierarchy.states.items():
            if rid in hierarchy.split:
                continue
            task = hierarchy.tasks.get(rid)
            options = []
            if task:
                for j, point in enumerate(task.viewpoints):
                    cell = tuple(runtime.indices(point))
                    for nid, (_, distances, previous) in trees.items():
                        if cell in distances:
                            options.append((distances[cell], nid, j, previous, cell))
            record = dict(kind='region', id=rid, **state)
            if options:
                cost, nid, j, previous, cell = min(options, key=lambda x: x[:3])
                p = simplify(tree_path(runtime, previous, cell), runtime)
                record.update(task.descriptor(), node=nid, points=p.tolist(), length=length(p),
                              entry=task.viewpoints[j].tolist())
            old = self.replica.records.get(f'r:{rid}')
            if old and {k: v for k, v in old.items() if k != 'stamp'} == record:
                continue
            record['stamp'] = now
            self.replica.put(f'r:{rid}', record)
        # Superseded parents must not remain explorable in other replicas.
        for rid in hierarchy.split:
            key = f'r:{rid}'; old = self.replica.records.get(key)
            if not old or old.get('status') != 'splitR':
                # A first map / restored map can immediately split a parent
                # before it ever had a leaf record. Publish that tombstone too.
                record = old or dict(kind='region', id=rid, unknown=0)
                self.replica.put(key, dict(record, status='splitR', stamp=now))
        self.rebuild()
        self.attachments = self.connect(position, runtime, tree=(here, prev))

    def connect(self, position, runtime=None, tree=None):
        runtime = runtime or self.runtime
        d, prev = tree if tree is not None else grid_tree(runtime, position, router=getattr(self, 'local_router', None))
        result = {}
        for nid, point in self.nodes.items():
            cell = tuple(runtime.indices(point))
            if cell in d:
                p = tree_path(runtime, prev, cell)
                p = np.vstack([position, p])
                result[nid] = (length(p), p)
        return result

    def search(self, attachments):
        costs = {}; parents = {}; roots = {}; queue = []
        for nid, item in attachments.items():
            if nid not in self.nodes:
                continue
            cost = float(item[0] if isinstance(item, tuple) else item)
            costs[nid] = cost; roots[nid] = nid; heapq.heappush(queue, (cost, nid))
        while queue:
            cost, u = heapq.heappop(queue)
            if cost != costs[u]:
                continue
            for v, weight, idx, reverse in self.adj.get(u, []):
                new = cost+weight
                if new < costs.get(v, np.inf)-1e-8:
                    costs[v] = new; parents[v] = (u, idx, reverse); roots[v] = roots[u]
                    heapq.heappush(queue, (new, v))
        return costs, parents, roots

    def route_to_region(self, rid, attachments=None):
        task = self.regions.get(rid)
        if not task or task.get('node') not in self.nodes:
            return None
        attachments = self.attachments if attachments is None else attachments
        costs, previous, roots = self.search(attachments); node = task['node']
        if node not in costs:
            return None
        parts = [np.array(task['points'])]
        while node in previous:
            parent, idx, reverse = previous[node]; p = np.array(self.edges[idx]['points'])
            parts.append(p[::-1] if reverse else p); node = parent
        parts.append(attachments[node][1])
        return np.vstack(parts[::-1])

    def snapshot(self):
        return dict(version=self.version, free_cells=self.free_cells, type='MR-DTG',
                    nodes=[dict(id=n, position=p.tolist()) for n, p in sorted(self.nodes.items())],
                    edges=self.edges, regions=list(self.regions.values()), handshakes=self.handshakes)


def graph_voronoi(graph, connections, available, local_radius=6.):
    """Two graph Voronoi partitions. Costs include actual traversable edges."""
    costs = {i: graph.search(c)[0] for i, c in connections.items() if i in available}
    global_owner = {}
    for node in graph.nodes:
        options = [(d.get(node, np.inf), i) for i, d in costs.items()]
        if options and np.isfinite(min(options)[0]):
            global_owner[node] = min(options)[1]
    owners = {}; tiers = {}; region_costs = {}
    for rid, task in graph.regions.items():
        if task.get('status') != 'activeR' or 'node' not in task:
            continue
        values = {i: d.get(task['node'], np.inf)+task['length'] for i, d in costs.items()}
        region_costs[rid] = values
        local = [(v, i) for i, v in values.items() if v <= local_radius]
        if local:
            owners[rid] = min(local)[1]; tiers[rid] = 'local'
        elif task['node'] in global_owner:
            owners[rid] = global_owner[task['node']]; tiers[rid] = 'global'
    return owners, global_owner, tiers, region_costs
