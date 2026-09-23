"""Single fused Hgrid -> MR-DTG -> Voronoi -> bilateral CVRP -> view pipeline."""
import copy
import math
import time
import numpy as np
from .hierarchy import AdaptiveRegions, ExplorationRegion
from .mrdtg import MultiRobotGraph, graph_voronoi
from .sparse_graph import SparseTopology, length
from .voxel_mapping import VoxelRouter
from .regions import ObservationPlanner, optimize_tour, visible_cells
from .pairwise import solve_pair
from core.planning.path_quality import Candidate, RankedPathPool, PathQualityEvaluator


class GraphCosts:
    def __init__(self, graph, sparse, tasks, position):
        self.graph = graph; self.sparse = sparse; self.tasks = tasks; self.position = position
        self.cache = {}; self.searches = {}

    def attachment(self, point):
        if np.linalg.norm(point-self.position) < 1e-5:
            return {n: c[0] for n, c in self.graph.attachments.items()}
        for rid, task in self.tasks.items():
            if np.linalg.norm(point-task.entry) < 1e-5:
                r = self.graph.regions.get(rid, {})
                if 'node' in r:
                    return {r['node']: r['length']}
        return {n: p[0] for n, p in self.graph.connect(point).items()}

    def distance(self, a, b):
        key = (tuple(np.round(a, 4)), tuple(np.round(b, 4)))
        if key in self.cache:
            return self.cache[key]
        if np.linalg.norm(np.asarray(a)-b) < 1e-6:
            return 0.
        local = self.sparse.distance(a, b)
        if np.isfinite(local):
            value = local
        else:
            if key[0] not in self.searches:
                self.searches[key[0]] = self.graph.search(self.attachment(a))[0]
            distances = self.searches[key[0]]
            value = min((distances.get(n, np.inf)+d for n, d in self.attachment(b).items()), default=np.inf)
        self.cache[key] = value; self.cache[key[::-1]] = value
        return value


class FusionPlanner:
    def __init__(self, drone, bounds):
        self.drone = drone; self.hierarchy = AdaptiveRegions(bounds); self.graph = MultiRobotGraph(drone, bounds)
        self.partition = {}; self.global_partition = {}; self.tiers = {}; self.tasks = {}
        self.diagnostics = {}; self.connections = {}; self.ownership = {}

    def compute(self, runtime, position, yaw, active, recent, cooldown, reservations, epoch, now, plan_view,
                peers, overrides, last_success, service_feedback=None):
        begin = time.monotonic(); runtime.rebuild()
        for rid, value in (service_feedback or {}).items():
            self.graph.replica.put(f's:{rid}', dict(kind='region_service', id=rid, **value))
        self.graph.rebuild()
        pinned = {int(p['active']): int(i) for i, p in peers.items() if p.get('active') is not None and p.get('intent')}
        if active is not None:
            pinned[active] = self.drone
        self.hierarchy.split.update(r for r, v in self.graph.regions.items() if v.get('status') == 'splitR')
        local_tasks = self.hierarchy.update(runtime, pinned)
        self.graph.update(runtime, position, self.hierarchy, now)
        connections = {self.drone: {n: value[0] for n, value in self.graph.attachments.items()}}
        available = {self.drone}
        for i, peer in peers.items():
            connections[i] = peer.get('graph_connections', {})
            if peer.get('available') and now-peer['time'] < 3.:
                available.add(i)
        self.connections = connections[self.drone]
        owners, global_owners, tiers, region_costs = graph_voronoi(self.graph, connections, available)
        self.partition = dict(owners); self.global_partition = global_owners; self.tiers = tiers
        # Committed pair decisions refine (rather than replace) the graph partition.
        for rid, owner in overrides.items():
            if rid in owners and owner in available and np.isfinite(region_costs.get(rid, {}).get(owner, np.inf)):
                owners[rid] = owner
        owners.update({r: i for r, i in pinned.items() if r in owners})
        tasks = dict(local_tasks)
        for rid in list(tasks):
            record = self.graph.regions.get(rid, {})
            if record.get('status') in ('deadR', 'splitR'):
                del tasks[rid]
            elif 'unknown' in record:
                tasks[rid].unknown = min(tasks[rid].unknown, record['unknown'])
        for rid, record in self.graph.regions.items():
            if rid not in tasks and record.get('status') == 'activeR' and record.get('viewpoints'):
                tasks[rid] = ExplorationRegion(rid, record['bounds'], record['unknown'],
                    [np.array(p) for p in record['viewpoints']], np.array(record['entry']),
                    record['level'], record['parent'], record['status'], tuple(record.get('view_states', [])))
        # Newly active local EROIs without an H-node connection still receive a
        # safe local view, so a narrow doorway cannot prevent graph bootstrapping.
        for rid in local_tasks:
            if rid not in owners:
                owners[rid] = self.drone
        self.tasks = tasks; self.ownership = owners
        router = VoxelRouter if runtime.state.ndim == 3 else SparseTopology
        sparse = router(runtime); costs = GraphCosts(self.graph, sparse, tasks, position)
        feasible = {r: task for r, task in tasks.items()
                    if max(cooldown.get(r, 0), self.graph.services.get(r, {}).get('defer_until', 0)) <= now}
        owned = [r for r, owner in owners.items() if owner == self.drone and r in feasible]
        tour, workload = optimize_tour(position, owned, feasible, costs, active)
        bids = {}
        for rid, task in feasible.items():
            distance = costs.distance(position, task.entry)
            if np.isfinite(distance):
                bids[rid] = float(distance/.6+(0 if owners.get(rid) == self.drone else 10000))
        if active in bids:
            bids[active] = -1e6
        # Select a fair interaction partner and solve a bounded exact two-vehicle
        # subproblem. Already executing regional services stay pinned.
        offer = None
        partners = [i for i in available if i != self.drone and self.drone < i]
        partners.sort(key=lambda i: (last_success.get(i, -1), i))
        for other in partners:
            # Deferred services have no live bids. Including them manufactures
            # owners absent from the peer's auction revision, so prepare can
            # never be accepted after the fleet has serviced boundary regions.
            ids = [r for r, owner in owners.items() if owner in (self.drone, other) and r in feasible and r in region_costs]
            ids.sort(key=lambda r: (abs(region_costs[r].get(self.drone, np.inf)-region_costs[r].get(other, np.inf)), r))
            ids = ids[:10]
            if len(ids) < 2:
                continue
            starts = [[region_costs[r].get(i, np.inf)/.6 for r in ids] for i in (self.drone, other)]
            between = [[costs.distance(tasks[a].entry, tasks[b].entry)/.6 for b in ids] for a in ids]
            demands = [1+tasks[r].unknown*runtime.resolution**runtime.state.ndim for r in ids]
            fixed = [sum(1+tasks[r].unknown*runtime.resolution**runtime.state.ndim
                         for r, owner in owners.items() if owner == i and r in tasks and r not in ids)
                     for i in (self.drone, other)]
            result = solve_pair(ids, starts, between, demands, owners, (self.drone, other), pinned, fixed_loads=fixed)
            offer = dict(other=other, result=result, owners={r: owners[r] for r in ids})
            if (result['status'] == 'optimal_window' and
                    (not result.get('before_feasible', True) or result['after'] < result['before']-.2)):
                break
        local = copy.deepcopy(runtime); local.block_paths(reservations, radius=1.25)
        local_graph = router(local) if plan_view else None
        choices = ([active] if active in feasible else [])+[r for r in tour if r != active]
        # Global burden sharing: prioritize useful, nearby history regions when
        # all locally partitioned work has disappeared. The region lease still
        # arbitrates exclusivity before any motion starts.
        if not choices:
            fallback = []
            for rid, task in feasible.items():
                d = costs.distance(position, task.entry)/.6
                users = sum(p.get('active') == rid for p in peers.values())
                gain = task.unknown*runtime.resolution**runtime.state.ndim*math.exp(-.08*d)/(users+1) if np.isfinite(d) else 0
                if gain > 0 and rid not in pinned:
                    fallback.append((-gain, rid))
            choices = [r for _, r in sorted(fallback)]
            for r in choices:
                bids[r] = costs.distance(position, tasks[r].entry)/.6+50
        selection = None; selected = None; rejected = []
        if plan_view:
            planner = ObservationPlanner()
            for k, rid in enumerate(choices):
                if rid not in feasible:
                    continue
                task = feasible[rid]
                if rid in local_tasks:
                    next_goal = feasible[choices[k+1]].entry if k+1 < len(choices) and choices[k+1] in feasible else None
                    selection = planner.plan(local, local_graph, position, yaw, task, epoch+1, recent, next_goal=next_goal)
                else:
                    path = self.graph.route_to_region(rid)
                    if path is not None:
                        # Advance only the locally observed prefix of a remote
                        # corridor; new sensor frames validate the next prefix.
                        points = [position]
                        for a, b in zip(path[:-1], path[1:]):
                            for p in np.linspace(a, b, max(2, int(np.linalg.norm(b-a)/.15)+1)):
                                if not local.safe_path([points[-1], p]):
                                    break
                                points.append(p)
                            else:
                                continue
                            break
                        route = np.array(points)
                        if length(route) > .4:
                            delta = path[-1]-route[-1]; heading = float(np.arctan2(delta[1], delta[0]))
                            pool = RankedPathPool(); pool.rank([Candidate(f'{rid}:{epoch}:mrdtg', 'mrdtg_transit', 0, route,
                                PathQualityEvaluator().evaluate(route, local), runtime.version)])
                            selection = dict(pool=pool, yaw=heading, gain=len(visible_cells(local, route[-1], heading))*runtime.resolution**runtime.state.ndim,
                                             objective=0., lookahead=None, transit=True)
                if selection:
                    selected = rid; break
                rejected.append(rid)
        self.diagnostics = dict(engine='Hgrid+MR-DTG+GVP+pair-CVRP', hgrid_leaves=len(self.hierarchy.leaves),
            hgrid_splits=len(self.hierarchy.split), frontier_clusters=len(self.hierarchy.frontiers.clusters),
            reused_frontiers=self.hierarchy.frontiers.reused_clusters, history_nodes=len(self.graph.nodes),
            handshake_updates=self.graph.handshakes, delta_applied=self.graph.replica.applied,
            local_regions=sum(v == 'local' for v in tiers.values()), global_regions=sum(v == 'global' for v in tiers.values()),
            shared_deferred_regions=sum(v['defer_until'] > now for v in self.graph.services.values()),
            pair_status=offer['result']['status'] if offer else 'no_pair', compute_wall_s=time.monotonic()-begin)
        return dict(fusion=self, graph=self.graph, tasks=tasks, bids=bids, tour=tour, workload=workload,
                    selection=selection, selected=selected, rejected=rejected, offer=offer,
                    wall=time.monotonic()-begin, position=position, version=runtime.version)
