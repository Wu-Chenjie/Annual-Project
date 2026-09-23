"""Region tasks, insertion-tour bids and finite-FOV observation/motion search."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import maximum_filter, distance_transform_edt, convolve
from .sparse_graph import length
from .graph import route_pool
from core.planning.path_quality import Candidate, PathQualityEvaluator


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def visible_cells(runtime, position, yaw, fov=2*math.pi/3, radius=4.5):
    """Predicted view: count the first unknown surface on each ray; do not see through it.

    This function never receives simulator truth. Its predicted gain is corrected
    by the next actual observation, including previously unknown occluders.
    """
    angles = np.linspace(yaw-fov/2, yaw+fov/2, 61 if runtime.state.ndim == 3 else 121)
    ranges = np.arange(0., radius+runtime.resolution/2, runtime.resolution/2)
    if runtime.state.ndim == 3:
        position = np.asarray(position)+np.array([0., 0., .4])
        headings, pitch = np.meshgrid(angles, np.linspace(-np.pi/3, np.pi/3, 13))
        directions = np.column_stack([np.cos(pitch.ravel())*np.cos(headings.ravel()),
                                      np.cos(pitch.ravel())*np.sin(headings.ravel()), np.sin(pitch.ravel())])
        coordinates = np.asarray(position)+directions[:, None, :]*ranges[None, :, None]
    else:
        coordinates = np.asarray(position)[:2]+np.stack([np.cos(angles), np.sin(angles)], axis=1)[:, None, :]*ranges[None, :, None]
    cells = np.floor((coordinates-runtime.origin)/runtime.resolution).astype(int)
    valid = np.all((cells >= 0) & (cells < runtime.shape), axis=2)
    clipped = np.clip(cells, 0, np.array(runtime.shape)-1)
    hit = (runtime.state[tuple(np.moveaxis(clipped, -1, 0))] != 0) | ~valid
    keep = valid & (np.cumsum(hit, axis=1)-hit == 0)
    ids = np.unique(np.ravel_multi_index(tuple(clipped[keep].T), runtime.shape))
    return frozenset(int(k) for k in ids if runtime.state.flat[k] == -1)


@dataclass
class RegionTask:
    id: int
    bounds: list
    unknown: int
    viewpoints: list
    entry: np.ndarray

    def descriptor(self):
        return dict(id=self.id, bounds=self.bounds, unknown=self.unknown,
                    entry=self.entry.tolist(), viewpoints=[p.tolist() for p in self.viewpoints])


class RegionTasks:
    def __init__(self, runtime, size=4.):
        self.runtime = runtime; self.size = size
        self.shape = tuple(np.ceil((runtime.bounds[1, :2]-runtime.origin)/size).astype(int))
        self.tasks = {}
        front = (runtime.state == 0) & maximum_filter(runtime.state == -1, size=3)
        if not front.any():
            return
        near = distance_transform_edt(~front)*runtime.resolution
        candidates = np.argwhere(runtime.safe & (near < 1.6))
        xx, yy = np.ogrid[-10:11, -10:11]
        gain = convolve((runtime.state == -1).astype(float), (xx*xx+yy*yy <= 100).astype(float), mode='constant')
        buckets = {}
        # Region ownership is a stable spatial task, not the moving frontier cell ID.
        for cell in sorted(candidates, key=lambda c: (-gain[tuple(c)], *c)):
            p = runtime.points([cell])[0]; rid = self.region(p)
            if gain[tuple(cell)] < 3:
                continue
            selected = buckets.setdefault(rid, [])
            if len(selected) < 5 and all(np.linalg.norm(p-q) >= .85 for q in selected):
                selected.append(p)
        for rid, points in buckets.items():
            tx, ty = divmod(rid, self.shape[1])
            low = runtime.origin+np.array([tx, ty])*size
            high = np.minimum(low+size, runtime.bounds[1, :2])
            a = np.maximum(0, runtime.indices(np.r_[low, runtime.altitude]))
            b = np.minimum(runtime.shape, np.ceil((high-runtime.origin)/runtime.resolution).astype(int))
            unknown = int(np.count_nonzero(runtime.state[a[0]:b[0], a[1]:b[1]] == -1))
            self.tasks[rid] = RegionTask(rid, [low.tolist(), high.tolist()], unknown, points, points[0])

    def region(self, position):
        tile = np.clip(np.floor((np.asarray(position)[:2]-self.runtime.origin)/self.size).astype(int), 0, np.array(self.shape)-1)
        return int(tile[0]*self.shape[1]+tile[1])


def optimize_tour(start, ids, tasks, graph, pinned=None):
    """Cheapest insertion followed by improving 2-opt, on sparse-graph costs."""
    route = [pinned] if pinned in ids else []
    remaining = sorted(set(ids)-set(route))
    cache = {}
    def distance(a, b):
        key = (a, b)
        if key not in cache:
            cache[key] = graph.distance(start if a is None else tasks[a].entry, tasks[b].entry)/.6
        return cache[key]
    def cost(order):
        return sum(distance(a, b)+1.+tasks[b].unknown*.012 for a, b in zip([None]+order, order))
    while remaining:
        choices = [(cost(route[:k]+[r]+route[k:]), r, k) for r in remaining for k in range(1 if route and route[0] == pinned else 0, len(route)+1)]
        value, r, k = min(choices)
        if not np.isfinite(value):
            break
        route.insert(k, r); remaining.remove(r)
    for _ in range(4):
        before = cost(route); best = route
        for i in range(1 if route and route[0] == pinned else 0, len(route)):
            for j in range(i+2, len(route)+1):
                candidate = route[:i]+route[i:j][::-1]+route[j:]
                if cost(candidate)+1e-6 < cost(best):
                    best = candidate
        route = best
        if cost(route) >= before-1e-6:
            break
    return route, cost(route)


def insertion_bids(position, owned, tasks, graph, active=None):
    """A bid includes route reordering, marginal service and current workload.

    Awarding a region changes the tour used by the next auction. This is a
    distributed route-insertion heuristic, not an exact CVRP optimum.
    """
    owned = [r for r in owned if r in tasks]
    bids = {}; routes = {}
    for rid, task in tasks.items():
        without = [r for r in owned if r != rid]
        base, old = optimize_tour(position, without, tasks, graph, active)
        route, new = optimize_tour(position, without+[rid], tasks, graph, active)
        if rid not in route or not np.isfinite(new):
            continue
        marginal = max(0., new-old)
        # Marginal route cost plus makespan pressure; small ownership hysteresis.
        bids[rid] = float(marginal+.28*new-(1.5 if rid in owned else 0.))
        routes[rid] = route
    if active in bids:
        bids[active] = -1e6  # A committed regional service lease is non-preemptive.
    tour, workload = optimize_tour(position, owned, tasks, graph, active)
    return bids, tour, workload, routes


class ObservationPlanner:
    """Two-step beam search over (position, yaw), scored jointly with motion.

    Ray gain is a union across views, so turning twice toward the same unknown
    cells does not count twice. Distance, yaw slew, clearance and turning enter
    the same objective. The first view is executed, then new sensing replans.
    """
    def __init__(self):
        self.evaluator = PathQualityEvaluator()

    def plan(self, runtime, graph, position, yaw, task, epoch, recent=(), next_goal=None):
        options = []
        for point in task.viewpoints:
            path = graph.route(position, point)
            if path is None or not runtime.safe_path(path):
                continue
            travel = length(path)/.6
            for heading in np.linspace(-np.pi, np.pi, 8, endpoint=False):
                if any(np.linalg.norm(point-p) < .65 and abs(angle_delta(heading, h)) < .7 for p, h in recent):
                    continue
                cells = visible_cells(runtime, point, heading)
                if len(cells) < 5:
                    continue
                rotation = abs(angle_delta(heading, yaw))/.65
                # Coupled information/time utility, penalizing needlessly long routes.
                value = len(cells)*runtime.resolution**runtime.state.ndim/(1.5+travel+rotation)
                options.append(dict(position=point, yaw=float(heading), path=path, cells=cells, value=value, travel=travel))
        if not options:
            return None
        beam = sorted(options, key=lambda o: -o['value'])[:8]
        for first in beam:
            best_second = 0.; second = None
            for other in beam:
                extra = len(other['cells']-first['cells'])*runtime.resolution**runtime.state.ndim
                if extra <= 0:
                    continue
                cost = graph.distance(first['position'], other['position'])/.6+abs(angle_delta(other['yaw'], first['yaw']))/.65+1.5
                if np.isfinite(cost) and extra/cost > best_second:
                    best_second = extra/cost; second = other
            exit_cost = 0. if next_goal is None else graph.distance(first['position'], next_goal)/.6
            first['objective'] = first['value']+.35*best_second-(.002*exit_cost if np.isfinite(exit_cost) else 0.)
            first['second'] = second
        choice = max(beam, key=lambda o: o['objective'])
        # Preserve the original multi-planner quality evaluator and five reserves.
        pool = route_pool(runtime, position, choice['position'], task.id, epoch, max_attempts=6)
        path = choice['path']
        if len(path) >= 2 and runtime.safe_path(path):
            topology = Candidate(f'{task.id}:{epoch}:sparse', 'sparse_topology', 0, path,
                                 self.evaluator.evaluate(path, runtime), runtime.version)
            pool.rank(([pool.active] if pool.active else [])+pool.backups+[topology])
        if pool.active is None:
            return None
        # The selected route's quality enters the observation-motion objective too.
        for candidate in [pool.active]+pool.backups:
            candidate.quality['observation_motion_cost'] = (candidate.quality['length_m']/.6+
                abs(angle_delta(choice['yaw'], yaw))/.65+candidate.quality['score']-len(choice['cells'])*.025)
        candidates = sorted([pool.active]+pool.backups, key=lambda c: c.quality['observation_motion_cost'])
        pool.active, pool.backups = candidates[0], candidates[1:6]
        return dict(pool=pool, yaw=choice['yaw'], gain=len(choice['cells'])*runtime.resolution**runtime.state.ndim,
                    objective=choice['objective'], lookahead=None if choice['second'] is None else
                    dict(position=choice['second']['position'].tolist(), yaw=choice['second']['yaw']))
