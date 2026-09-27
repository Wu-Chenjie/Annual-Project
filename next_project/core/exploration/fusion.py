"""Single fused Hgrid -> MR-DTG -> Voronoi -> bilateral CVRP -> view pipeline."""
import copy
import math
import time
import numpy as np
from .hierarchy import AdaptiveRegions, ExplorationRegion
from .mrdtg import MultiRobotGraph, graph_voronoi, observation_grid
from .sparse_graph import SparseTopology, length
from .voxel_mapping import VoxelRouter
from .regions import ObservationPlanner, optimize_tour, visible_cells
from .pairwise import solve_pair
from .priority import ExplorationPriority
from .team_evidence import known_mask, regional_evidence, intent_records, expected_traffic_delay
from core.planning.path_quality import Candidate, RankedPathPool, PathQualityEvaluator, resample
from core.planning.continuous_trajectory import optimize_trajectory


_WORKER_PLANNER = None


def compute_worker(planner, *arguments, deadline_wall=None):
    """Keep private geometry caches inside one persistent planning process.

    The ROS process owns the transport replica and sends its latest source
    fences. No geometry cache is authoritative; each validates its actual map
    dependencies. Replacing the worker rebuilds caches from sensed data.
    """
    global _WORKER_PLANNER
    worker_started = time.monotonic()
    session = planner.graph.replica.session
    if _WORKER_PLANNER is None or _WORKER_PLANNER.graph.replica.session != session:
        _WORKER_PLANNER = planner
    else:
        _WORKER_PLANNER.graph.replica = planner.graph.replica
        _WORKER_PLANNER.priority.config = planner.priority.config
    result = _WORKER_PLANNER.compute(*arguments, deadline_wall=deadline_wall)
    public = copy.copy(_WORKER_PLANNER); graph = copy.copy(public.graph)
    graph.trees = {}; graph.handshake_cache = {}; graph.local_router = None; graph.runtime = None
    public.graph = graph
    public.priority = copy.copy(public.priority)
    public.priority._views = {}; public.priority._signature = None; public.priority._geometry_cache = None
    public.hierarchy = copy.copy(public.hierarchy)
    public.hierarchy.frontiers = copy.copy(public.hierarchy.frontiers)
    public.hierarchy.frontiers.previous = None; public.hierarchy.frontiers.clusters = {}
    result['fusion'] = public; result['graph'] = graph
    result['worker_started_wall'] = worker_started; result['worker_finished_wall'] = time.monotonic()
    return result


def reusable_trajectory(selection, runtime, position, yaw):
    """A speculative curve is reusable only after current collision validation.

    Small endpoint tracking errors use the same tolerance as view completion;
    a changed active reserve, yaw, or joining segment requires fresh fitting.
    """
    trajectory = selection.get('trajectory')
    active = selection['pool'].active
    if trajectory is None or active is None or selection.get('trajectory_candidate') != active.id:
        return None
    start = trajectory.sample(0.)[0]
    # Ordinary adoption starts from a hold. Nonzero boundaries are accepted
    # only by the separately authorized moving-handoff protocol.
    if np.linalg.norm(trajectory.sample(0.)[1]) > .02 or np.linalg.norm(trajectory.sample(0.)[2]) > .02:
        return None
    if np.linalg.norm(start-position) > .08 or abs(math.atan2(math.sin(yaw-trajectory.yaw), math.cos(yaw-trajectory.yaw))) > .12:
        return None
    if not runtime.safe_path([position, start]) or not runtime.safe_path(trajectory.path()):
        return None
    return trajectory


class GraphCosts:
    def __init__(self, graph, sparse, tasks, position,regions=None):
        self.graph = graph; self.sparse = sparse; self.tasks = tasks; self.position = position
        self.cache = {}; self.searches = {}; self.attachments = {}
        self.regions=getattr(graph,'regions',{}) if regions is None else regions

    def route_to_region(self,rid):return self.graph.route_to_region(rid,regions=self.regions)

    def attachment(self, point):
        key = tuple(np.round(point, 4))
        if key not in self.attachments:
            self.attachments[key] = self._attachment(point)
        return self.attachments[key]

    def _attachment(self, point):
        if np.linalg.norm(point-self.position) < 1e-5:
            return {n: c[0] for n, c in self.graph.attachments.items()}
        for rid, task in self.tasks.items():
            if np.linalg.norm(point-task.entry) < 1e-5:
                r = self.regions.get(rid, {})
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
    def __init__(self, drone, bounds, priority_config=None):
        self.drone = drone; self.hierarchy = AdaptiveRegions(bounds); self.graph = MultiRobotGraph(drone, bounds)
        self.partition = {}; self.global_partition = {}; self.tiers = {}; self.tasks = {}
        self.diagnostics = {}; self.connections = {}; self.ownership = {}
        self.priority = ExplorationPriority(priority_config); self.priority_layer = {}

    def compute(self, runtime, position, yaw, active, recent, cooldown, reservations, epoch, now, plan_view,
                peers, overrides, last_success, service_feedback=None, anticipated_view=None, can_move=True, deadline_wall=None):
        plan_view = plan_view and can_move
        begin = time.monotonic(); stages = {}; mark = begin
        def stage(name):
            nonlocal mark
            stamp = time.monotonic(); stages[name] = stamp-mark; mark = stamp
        runtime.rebuild()
        # Charge deferred navigation materialization to the worker map stage.
        _ = runtime.grid
        stage('map')
        for rid, value in (service_feedback or {}).items():
            self.graph.replica.put(f's:{rid}', dict(kind='region_service', id=rid, **value))
        self.graph.record_observation(runtime)
        self.graph.runtime = runtime
        self.graph.rebuild()
        pinned = {int(intent['region']): int(i) for i, p in peers.items() for intent in intent_records(p)}
        if active is not None and can_move:
            pinned[active] = self.drone
        self.hierarchy.split.update(r for r, v in self.graph.regions.items() if v.get('status') == 'splitR')
        observed_mask = known_mask(runtime, self.graph.observed_mask(runtime))
        local_tasks = self.hierarchy.update(runtime, pinned, observed_mask)
        stage('hierarchy')
        self.graph.update(runtime, position, self.hierarchy, now)
        stage('topology')
        planning_regions=self.graph.planning_regions(runtime,now,observed_mask,local_tasks)
        self.graph.observation_services=[r for r in planning_regions.values() if r.get('service_role')]
        connections = {self.drone: {n: value[0] for n, value in self.graph.attachments.items()}}
        available = {self.drone} if can_move else set()
        for i, peer in peers.items():
            connections[i] = peer.get('graph_connections', {})
            if peer.get('available') and now-peer['time'] < 3.:
                available.add(i)
        self.connections = connections[self.drone]
        owners, global_owners, tiers, region_costs = graph_voronoi(self.graph, connections, available,regions=planning_regions)
        self.partition = dict(owners); self.global_partition = global_owners; self.tiers = tiers
        # Committed pair decisions refine (rather than replace) the graph partition.
        for rid, owner in overrides.items():
            if rid in owners and owner in available and np.isfinite(region_costs.get(rid, {}).get(owner, np.inf)):
                owners[rid] = owner
        owners.update({r: i for r, i in pinned.items() if r in owners})
        tasks = dict(local_tasks)
        for rid in list(tasks):
            record = self.graph.regions.get(rid, {})
            if record.get('status') == 'splitR':
                del tasks[rid]
        for rid, record in planning_regions.items():
            if rid not in tasks and record.get('status') == 'activeR' and record.get('viewpoints'):
                evidence = regional_evidence(runtime, record['bounds'], observed_mask)
                forecast = record.get('forecast_remaining_cells',record.get('forecast_gain_cells',0) if now-record.get('forecast_stamp',-np.inf) < 12. else 0)
                if not evidence['team_unknown'] and not forecast:
                    continue
                tasks[rid] = ExplorationRegion(rid, record['bounds'], evidence['team_unknown'],
                    [np.array(p) for p in record['viewpoints']], np.array(record['entry']),
                    record['level'], record['parent'], record['status'], tuple(record.get('view_states', [])),
                    remote_gain_cells=forecast)
        # Newly active local EROIs without an H-node connection still receive a
        # safe local view, so a narrow doorway cannot prevent graph bootstrapping.
        for rid in local_tasks:
            if rid not in owners and can_move:
                owners[rid] = self.drone
        self.tasks = tasks; self.ownership = owners
        router = VoxelRouter if runtime.state.ndim == 3 else SparseTopology
        sparse = self.graph.local_router if runtime.state.ndim == 3 else router(runtime)
        costs = GraphCosts(self.graph, sparse, tasks, position,planning_regions)
        view_attachments = None; excluded_cells = frozenset()
        if anticipated_view is not None:
            # History nodes still originate at the actual observed position.
            # Only the next task/view route starts at the executing endpoint.
            position = np.asarray(anticipated_view['position'], float); yaw = anticipated_view['yaw']
            excluded_cells = visible_cells(runtime, position, yaw)
            recent = list(recent)+[(position, yaw)]
            view_attachments = self.graph.connect(position)
        # Promises are arrival-dependent utility estimates, never actual receipts.
        priorities = self.priority.rank(runtime, tasks, local_tasks, costs, position, now,
                                        self.graph.services, excluded_cells, observed_mask,
                                        preferred=[r for r, owner in owners.items() if owner == self.drone], team_only=True, peers=peers)
        for rid, row in priorities.items():
            row['deferred'] |= cooldown.get(rid, 0) > now
            row['committed_by_peer'] = rid in pinned and pinned[rid] != self.drone
            record = self.graph.replica.records.get(f'r:{rid}')
            if (rid in local_tasks and record and record.get('status')=='activeR' and
                    'entry' in record and 'node' in record and row['gain_source']=='local_rays'):
                heading=row.get('preview_yaw',0.)
                footprint={c for c in self.priority.view(runtime,np.array(record['entry']),heading,coarse=True) if not observed_mask[c]}
                updated = dict(record, forecast_gain_cells=len(footprint),forecast_cells=sorted(footprint),forecast_grid_shape=list(map(int,runtime.shape)),
                               forecast_stamp=now, forecast_kind='private_ray_estimate', forecast_yaw=heading)
                if updated['forecast_cells']!=record.get('forecast_cells') or now-record.get('forecast_stamp',-np.inf)>=3.:
                    self.graph.replica.put(f'r:{rid}', updated)
        self.priority_layer = self.priority.snapshot(runtime, now, owners, self.drone)
        stage('priority')
        feasible = {r: task for r, task in tasks.items()
                    if cooldown.get(r, 0) <= now and not priorities[r]['deferred'] and priorities[r]['predicted_gain'] > 0}
        owned = [r for r, owner in owners.items() if owner == self.drone and r in feasible]
        owned_count = len(owned)
        if len(owned) > 24:
            ordered = sorted(owned,key=lambda r:(-priorities[r]['score'],r))
            # A rotating old task keeps bounded optimization from starving
            # remote remnants. All tasks still receive bids and pair demand.
            oldest = max(owned,key=lambda r:(priorities[r]['wait_s'],-r))
            owned = ordered[:23]+([oldest] if oldest not in ordered[:23] else [ordered[23]])
        rewards = {r: p['information_reward'] for r, p in priorities.items()}
        # The ledger pins execution, while this route starts at its anticipated
        # endpoint. Do not pin a finished view or re-sort the optimized route.
        tour, workload = optimize_tour(position, owned, feasible, costs, rewards=rewards,
                                      latency_weight=self.priority.config.route_latency_weight,
                                      traffic_delays={r:p.get('traffic_delay_s',0.) for r,p in priorities.items()})
        bids = {}
        for rid, task in (feasible.items() if can_move else []):
            distance = costs.distance(position, task.entry)
            if np.isfinite(distance):
                bids[rid] = float(distance/.6+priorities[rid].get('traffic_delay_s',0.)+(0 if owners.get(rid) == self.drone else 10000))
        if active in bids:
            bids[active] = -1e6
        stage('tour')
        # Select a fair interaction partner and solve a bounded exact two-vehicle
        # subproblem. Already executing regional services stay pinned.
        offer = None
        partners = [i for i in available if can_move and i != self.drone and self.drone < i]
        partners.sort(key=lambda i: (last_success.get(i, -1), i))
        for other in partners:
            # Deferred services have no live bids. Including them manufactures
            # owners absent from the peer's auction revision, so prepare can
            # never be accepted after the fleet has serviced boundary regions.
            ids = [r for r, owner in owners.items() if owner in (self.drone, other) and r in feasible and r in region_costs]
            ids.sort(key=lambda r: (-priorities[r]['score'],
                abs(region_costs[r].get(self.drone, np.inf)-region_costs[r].get(other, np.inf)), r))
            ids = ids[:10]
            if len(ids) < 2:
                continue
            starts = [[region_costs[r].get(i, np.inf)/.6 for r in ids] for i in (self.drone, other)]
            between = [[costs.distance(tasks[a].entry, tasks[b].entry)/.6 for b in ids] for a in ids]
            demands = [1+tasks[r].unknown*runtime.resolution**runtime.state.ndim for r in ids]
            fixed = [sum(1+tasks[r].unknown*runtime.resolution**runtime.state.ndim
                         for r, owner in owners.items() if owner == i and r in tasks and r not in ids)
                     for i in (self.drone, other)]
            result = solve_pair(ids, starts, between, demands, owners, (self.drone, other), pinned, fixed_loads=fixed,
                                reward_weights=[rewards[r] for r in ids], latency_weight=self.priority.config.route_latency_weight)
            offer = dict(other=other, result=result, owners={r: owners[r] for r in ids})
            if (result['status'] == 'optimal_window' and
                    (not result.get('before_feasible', True) or result['after'] < result['before']-.2)):
                break
        stage('pair')
        local = copy.deepcopy(runtime) if plan_view else None
        if plan_view:
            local.block_paths(reservations, radius=1.25)
        local_graph = router(local) if plan_view else None
        choices = list(tour)
        # Global burden sharing: prioritize useful, nearby history regions when
        # all locally partitioned work has disappeared. The region lease still
        # arbitrates exclusivity before any motion starts.
        if not choices and can_move:
            fallback = []
            for rid, task in feasible.items():
                d = costs.distance(position, task.entry)/.6
                gain = priorities[rid]['score'] if np.isfinite(d) else 0
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
                    selection = planner.plan(local, local_graph, position, yaw, task, epoch+1, recent,
                                             next_goal=next_goal, excluded_cells=excluded_cells,
                                             observed_mask=observed_mask, history_weight=0.)
                else:
                    path = self.graph.route_to_region(rid, view_attachments,planning_regions)
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
                            if np.linalg.norm(delta)<.25: heading = planning_regions[rid].get('forecast_yaw',heading)
                            pool = RankedPathPool(); pool.rank([Candidate(f'{rid}:{epoch}:mrdtg', 'mrdtg_transit', 0, route,
                                PathQualityEvaluator().evaluate(route, local), runtime.version)])
                            selection = dict(pool=pool, yaw=heading, gain=len(visible_cells(local, route[-1], heading))*runtime.resolution**runtime.state.ndim,
                                             objective=0., lookahead=None, transit=True, purpose='transit_reobserve',
                                             navigation_target=rid, navigation_reason='private_map_corridor_prefix', team_gain=0.)
                if selection:
                    selection.setdefault('purpose', 'explore')
                    selection['traffic_delay_s'] = expected_traffic_delay(selection['pool'].active.path, peers, now)
                    selection['priority'] = priorities[rid]
                    selected = rid; break
                rejected.append(rid)
        stage('view')
        if selection:
            for candidate in [selection['pool'].active]+selection['pool'].backups:
                candidate.created_at = now; candidate.expires_at = now+60.
            boundary = {k: anticipated_view[k] for k in ('start_velocity', 'start_acceleration', 'start_yaw_rate', 'start_yaw_acceleration')
                        if anticipated_view and k in anticipated_view}
            curves = {}; valid = []
            for candidate in [selection['pool'].active]+selection['pool'].backups:
                try:
                    curve = optimize_trajectory(candidate.path, local, yaw, selection['yaw'], deadline_wall=deadline_wall, **boundary)
                    points=curve.path(.15)
                    distances=(local.signed_distances(points) if hasattr(local,'signed_distances') else
                               np.array([local.signed_distance(p) for p in points]))
                    clearance = min(float(np.linalg.norm(local.bounds[1]-local.bounds[0])),float(distances.min()))
                    candidate.quality['executed_curve'] = dict(duration_s=curve.duration, limits=curve.limits(),
                        yaw_rate=curve.yaw_rate_limit(), jerk_integral=curve.jerk_cost(), method=curve.method,
                        minimum_clearance_m=clearance)
                    candidate.quality['curve_quality_score'] = curve.duration+.035*curve.jerk_cost()+.8/max(.25,clearance-local.clearance+.25)
                    if any(np.mean(np.linalg.norm(resample(curve.path(.15))-resample(c.path(.15)),axis=1)) < selection['pool'].diversity_m for c in curves.values()):
                        continue
                    curves[candidate.id] = curve; valid.append(candidate)
                except ValueError:
                    continue
                except TimeoutError:
                    selection['curve_budget_exhausted'] = True
                    break
            selection['curve_alternatives'] = curves
            selection['curve_qualities'] = {c.id:dict(quality=copy.deepcopy(c.quality),map_version=local.version,time=now)
                                           for c in valid}
            if valid:
                valid.sort(key=lambda c:(c.quality['curve_quality_score'],c.id))
                selection['pool'].active, selection['pool'].backups = valid[0], valid[1:6]
                selection['trajectory'] = curves[valid[0].id]; selection['trajectory_candidate'] = valid[0].id
            else:
                selection = None; selected = None
        stage('trajectory')
        self.diagnostics = dict(engine='Hgrid+MR-DTG+GVP+pair-CVRP', hgrid_leaves=len(self.hierarchy.leaves),
            hgrid_splits=len(self.hierarchy.split), frontier_clusters=len(self.hierarchy.frontiers.clusters),
            reused_frontiers=self.hierarchy.frontiers.reused_clusters, history_nodes=len(self.graph.nodes),
            handshake_updates=self.graph.handshakes, delta_applied=self.graph.replica.applied,
            local_regions=sum(v == 'local' for v in tiers.values()), global_regions=sum(v == 'global' for v in tiers.values()),
            shared_deferred_regions=sum(v['defer_until'] > now for v in self.graph.services.values()),
            pair_status=offer['result']['status'] if offer else 'no_pair', compute_wall_s=time.monotonic()-begin,
            stage_wall_s=stages, anticipatory_view=anticipated_view is not None, can_move=can_move,
            unknown_components=len(self.priority.components), reserved_gain_cells=len(excluded_cells),
            team_observed_cells=int(observed_mask.sum()), priority_compute=self.priority.diagnostics,
            route_objective='travel_plus_information_latency',
            route_window_regions=len(owned), all_owned_regions=owned_count,
            observation_service_entries=len(self.graph.observation_services),
            selected_priority=priorities.get(selected))
        return dict(fusion=self, graph=self.graph, tasks=tasks, bids=bids, tour=tour, workload=workload,
                    selection=selection, selected=selected, rejected=rejected, offer=offer,
                    wall=time.monotonic()-begin, position=position, version=runtime.version)
