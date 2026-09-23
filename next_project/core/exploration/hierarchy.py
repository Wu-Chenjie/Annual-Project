"""Incremental frontiers and adaptive, stable-ID exploration regions.

The hierarchy uses the same dyadic indexing in every replica. Splits are monotone
within a mission; a committed service region is pinned until its lease retires.
"""
from dataclasses import dataclass
import hashlib
import itertools
import numpy as np
from scipy.ndimage import maximum_filter, label, distance_transform_edt, convolve, uniform_filter
from .regions import RegionTask


class FrontierIndex:
    def __init__(self):
        self.previous = None
        self.frontier = None
        self.clusters = {}
        self.changed_cells = 0
        self.reused_clusters = 0

    def update(self, runtime):
        state = runtime.state
        dirty = np.ones(state.shape, bool) if self.previous is None else state != self.previous
        self.changed_cells = int(dirty.sum())
        if self.frontier is None:
            self.frontier = np.zeros(state.shape, bool)
        affected = maximum_filter(dirty, size=3)
        fresh = (state == 0) & maximum_filter(state == -1, size=3)
        self.frontier[affected] = fresh[affected]
        self.previous = state.copy()
        groups, count = label(self.frontier, structure=np.ones((3,)*state.ndim))
        clusters = {}; self.reused_clusters = 0
        # Detection is incremental. Connected-component labels are recomputed to
        # handle merges/splits; unchanged cluster geometry and viewpoints persist.
        for k in range(1, count+1):
            cells = np.argwhere(groups == k)
            if len(cells) < 2:
                continue
            # Spatial buckets bound cluster extent without losing frontier cells.
            tiles = cells//12
            for tile in np.unique(tiles, axis=0):
                part = cells[np.all(tiles == tile, axis=1)]
                key = hashlib.sha256(part.tobytes()).hexdigest()[:16]
                if key in self.clusters:
                    clusters[key] = self.clusters[key]; self.reused_clusters += 1
                else:
                    clusters[key] = dict(cells=part, low=part.min(axis=0), high=part.max(axis=0),
                                         centroid=runtime.points(part).mean(axis=0))
        self.clusters = clusters
        return self


@dataclass
class ExplorationRegion(RegionTask):
    level: int = 0
    parent: int = -1
    status: str = 'activeR'
    view_states: tuple = ()

    def descriptor(self):
        return dict(super().descriptor(), level=self.level, parent=self.parent,
                    status=self.status, view_states=list(self.view_states))


class AdaptiveRegions:
    def __init__(self, bounds, root_size=8., levels=3):
        self.bounds = np.asarray(bounds, float)
        self.root_size = root_size; self.levels = levels
        self.ndim = 2
        self.split = set(); self.tasks = {}; self.states = {}
        self.frontiers = FrontierIndex(); self.leaves = {}
        self.view_catalog = {}

    def ident(self, level, *cell):
        extent = self.bounds[1, :self.ndim]-self.bounds[0, :self.ndim]
        shape = np.ceil(extent/(self.root_size/2**level)).astype(int)
        offset = sum(int(np.prod(np.ceil(extent/(self.root_size/2**l)))) for l in range(level))
        return offset+int(np.ravel_multi_index(tuple(cell), tuple(shape)))

    def update(self, runtime, pinned=()):
        self.ndim = runtime.state.ndim
        self.frontiers.update(runtime)
        front = self.frontiers.frontier
        near = distance_transform_edt(~front)*runtime.resolution if front.any() else np.full(runtime.shape, np.inf)
        candidates = np.argwhere(runtime.safe & (near < 1.6))
        xx, yy = np.ogrid[-10:11, -10:11]
        gains = (convolve((runtime.state == -1).astype(float), (xx*xx+yy*yy <= 100).astype(float), mode='constant')
                 if self.ndim == 2 else uniform_filter((runtime.state == -1).astype(float), size=15, mode='constant')*15**3)
        if len(candidates):
            order = np.lexsort(tuple(candidates[:, k] for k in range(self.ndim-1, -1, -1))+
                               (-gains[tuple(candidates.T)],))
            candidates = candidates[order]
        tasks = {}; leaves = {}; pinned = set(pinned)
        def visit(level, cell, parent=-1):
            size = self.root_size/2**level
            low = self.bounds[0, :self.ndim]+np.array(cell)*size
            high = np.minimum(low+size, self.bounds[1, :self.ndim])
            if np.any(low >= high):
                return
            rid = self.ident(level, *cell)
            a = np.maximum(0, runtime.indices(np.r_[low, runtime.altitude] if self.ndim == 2 else low))
            b = np.minimum(runtime.shape, np.ceil((high-runtime.origin)/runtime.resolution).astype(int))
            cells = runtime.state[tuple(slice(i, j) for i, j in zip(a, b))]
            unknown = int(np.count_nonzero(cells == -1))
            known = 1-unknown/max(cells.size, 1)
            if level < self.levels-1 and rid not in pinned and (rid in self.split or .25 <= known < .98):
                self.split.add(rid)
                for child in itertools.product(range(2), repeat=self.ndim):
                    visit(level+1, tuple(2*np.array(cell)+child), rid)
                return
            leaves[rid] = (level, low, high)
            points = []
            local_candidates = candidates[np.all((candidates >= a) & (candidates < b), axis=1)]
            for cell in local_candidates:
                if gains[tuple(cell)] < 3:
                    continue
                p = runtime.points([cell])[0]
                if all(np.linalg.norm(p-q) >= .65 for q in points):
                    points.append(p)
                    if len(points) == 5:
                        break
            catalog = self.view_catalog.setdefault(rid, {})
            for p in points:
                cell = tuple(runtime.indices(p))
                catalog.setdefault(cell, dict(id=':'.join(map(str, cell)), position=p.tolist(), status='inactiveV'))
            for cell, view in catalog.items():
                occupied = runtime.state[cell] == 1
                if occupied:
                    view['status'] = 'deadV'
                elif runtime.state[cell] == -1 or not runtime.safe[cell]:
                    view['status'] = 'inactiveV'
                else:
                    view['status'] = 'activeV' if near[cell] < 1.6 and gains[cell] >= 3 else 'deadV'
            status = 'activeR' if points else 'deadR' if known >= .98 else 'inactiveR'
            # A dead region can reopen on a changed map / newly discovered doorway.
            self.states[rid] = dict(status=status, unknown=unknown, level=level, parent=parent,
                                    bounds=[low.tolist(), high.tolist()], views=list(catalog.values()))
            if points:
                tasks[rid] = ExplorationRegion(rid, [low.tolist(), high.tolist()], unknown, points, points[0],
                                               level, parent, status, tuple('activeV' for _ in points))
        shape = np.ceil((self.bounds[1, :self.ndim]-self.bounds[0, :self.ndim])/self.root_size).astype(int)
        for cell in np.ndindex(tuple(shape)):
            visit(0, cell)
        self.tasks = tasks; self.leaves = leaves
        return tasks
