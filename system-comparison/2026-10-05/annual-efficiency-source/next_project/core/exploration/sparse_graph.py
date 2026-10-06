"""Observed-free-space skeleton compressed into junctions and corridor polylines.

The occupancy grid is used for collision checking and skeleton extraction only.
Global distance queries, region tours and route reconstruction use this sparse graph.
"""
from dataclasses import dataclass
import heapq
import numpy as np


def thin(mask):
    """Zhang-Suen thinning, preserving components and holes (8-connectivity)."""
    a = np.pad(mask.astype(np.uint8), 1)
    while True:
        removed = 0
        for phase in (0, 1):
            p = [a[:-2, 1:-1], a[:-2, 2:], a[1:-1, 2:], a[2:, 2:],
                 a[2:, 1:-1], a[2:, :-2], a[1:-1, :-2], a[:-2, :-2]]
            count = sum(p)
            transitions = sum((p[k] == 0) & (p[(k+1) % 8] == 1) for k in range(8))
            if phase == 0:
                corner = (p[0]*p[2]*p[4] == 0) & (p[2]*p[4]*p[6] == 0)
            else:
                corner = (p[0]*p[2]*p[6] == 0) & (p[0]*p[4]*p[6] == 0)
            delete = (a[1:-1, 1:-1] == 1) & (count >= 2) & (count <= 6) & (transitions == 1) & corner
            removed += int(delete.sum())
            a[1:-1, 1:-1][delete] = 0
        if not removed:
            return a[1:-1, 1:-1].astype(bool)


def length(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


@dataclass
class Corridor:
    u: int
    v: int
    path: np.ndarray
    cost: float


class SparseTopology:
    def __init__(self, runtime, region_size=4.):
        self.runtime = runtime
        self.version = runtime.version
        self.nodes = {}
        self.edges = []
        self.adj = {}
        self.attachments = {}
        self._queries = {}
        self._distances = {}
        self._attach_cache = {}
        cells = {tuple(c) for c in np.argwhere(thin(runtime.safe))}
        # Avoid redundant diagonal triangles when an orthogonal skeleton step exists.
        neighbours = {}
        for x, y in cells:
            row = []
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                v = (x+dx, y+dy)
                if v not in cells:
                    continue
                if dx and dy and ((x+dx, y) in cells or (x, y+dy) in cells):
                    continue
                row.append(v)
            neighbours[(x, y)] = sorted(row)
        anchors = {c for c in cells if len(neighbours[c]) != 2}
        # Portals explicitly split corridors at region boundaries.
        tile = lambda c: tuple(np.floor((runtime.points([c])[0][:2]-runtime.origin)/region_size).astype(int))
        anchors |= {c for c in cells if any(tile(c) != tile(v) for v in neighbours[c])}
        # Pure loops have no junction: insert one deterministic anchor per component.
        seen = set()
        for root in sorted(cells):
            if root in seen:
                continue
            stack = [root]; component = set()
            while stack:
                c = stack.pop()
                if c in component:
                    continue
                component.add(c); stack.extend(neighbours[c])
            seen |= component
            if not component & anchors:
                anchors.add(min(component))
        visited = set()
        ident = lambda c: int(c[0]*runtime.shape[1]+c[1])
        for root in sorted(anchors):
            self.nodes[ident(root)] = runtime.points([root])[0]
            for first in neighbours[root]:
                key = tuple(sorted((root, first)))
                if key in visited:
                    continue
                chain = [root, first]; visited.add(key)
                while chain[-1] not in anchors:
                    following = [v for v in neighbours[chain[-1]] if v != chain[-2]]
                    if not following:
                        break
                    v = following[0]
                    visited.add(tuple(sorted((chain[-1], v)))); chain.append(v)
                # Limit edge length to aid collision-checked off-skeleton attachment.
                for begin in range(0, len(chain)-1, 12):
                    part = chain[begin:min(begin+13, len(chain))]
                    u, v = ident(part[0]), ident(part[-1])
                    points = runtime.points(part)
                    self.nodes[u], self.nodes[v] = points[0], points[-1]
                    edge = Corridor(u, v, points, length(points))
                    index = len(self.edges); self.edges.append(edge)
                    self.adj.setdefault(u, []).append((v, edge.cost, index, False))
                    self.adj.setdefault(v, []).append((u, edge.cost, index, True))
                    prefix = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
                    for j, c in enumerate(part):
                        self.attachments.setdefault(c, []).extend([
                            (u, float(prefix[j]), points[:j+1][::-1]),
                            (v, float(prefix[-1]-prefix[j]), points[j:])])
        for c in cells:
            if ident(c) in self.nodes and c not in self.attachments:
                self.attachments[c] = [(ident(c), 0., runtime.points([c]))]
        self.cells = sorted(self.attachments)
        self.points = runtime.points(self.cells) if self.cells else np.empty((0, 3))
        self.free_cells = int(runtime.safe.sum())

    def attach(self, point):
        point = np.asarray(point, float)
        cache_key = tuple(np.round(point, 4))
        if cache_key in self._attach_cache:
            return self._attach_cache[cache_key]
        if not len(self.points) or not self.runtime.safe_path([point]):
            return []
        best = {}
        for k in np.argsort(np.linalg.norm(self.points-point, axis=1))[:40]:
            p = self.points[k]
            if not self.runtime.safe_path([point, p]):
                continue
            for node, cost, suffix in self.attachments[self.cells[k]]:
                cost += float(np.linalg.norm(p-point))
                if node not in best or cost < best[node][0]:
                    best[node] = (cost, np.vstack([point, suffix]))
            # A few nearby attachment choices avoid forcing travel back to a junction.
            if len(best) >= 6:
                break
        self._attach_cache[cache_key] = [(node, *value) for node, value in best.items()]
        return self._attach_cache[cache_key]

    def search(self, start):
        key = tuple(np.round(start, 4))
        if key in self._queries:
            return self._queries[key]
        distances, previous, roots = {}, {}, {}
        queue = []
        for node, cost, connector in self.attach(start):
            if cost < distances.get(node, np.inf):
                distances[node] = cost; roots[node] = connector
                heapq.heappush(queue, (cost, node))
        while queue:
            cost, u = heapq.heappop(queue)
            if cost > distances[u]+1e-8:
                continue
            for v, weight, index, reverse in self.adj.get(u, []):
                trial = cost+weight
                if trial < distances.get(v, np.inf):
                    distances[v] = trial; previous[v] = (u, index, reverse)
                    roots.pop(v, None); heapq.heappush(queue, (trial, v))
        self._queries[key] = distances, previous, roots
        return distances, previous, roots

    def route(self, start, goal):
        start, goal = np.asarray(start), np.asarray(goal)
        # Visibility is also a legitimate temporary graph edge.
        if self.runtime.safe_path([start, goal]):
            return np.array([start, goal])
        distances, previous, roots = self.search(start)
        options = [(distances.get(node, np.inf)+cost, node, path) for node, cost, path in self.attach(goal)]
        if not options:
            return None
        cost, node, tail = min(options, key=lambda a: a[0])
        if not np.isfinite(cost):
            return None
        segments = [tail[::-1]]
        while node in previous:
            parent, index, reverse = previous[node]
            edge = self.edges[index]
            segments.append(edge.path[::-1] if reverse else edge.path)
            node = parent
        segments.append(roots[node])
        path = np.vstack(segments[::-1])
        path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-8]]
        return path if self.runtime.safe_path(path) else None

    def distance(self, start, goal):
        key = (tuple(np.round(start, 4)), tuple(np.round(goal, 4)))
        if key not in self._distances:
            if self.runtime.safe_path([start, goal]):
                value = float(np.linalg.norm(np.asarray(start)-goal))
            else:
                distances, _, _ = self.search(start)
                value = min((distances.get(n, np.inf)+cost for n, cost, _ in self.attach(goal)), default=np.inf)
            self._distances[key] = value
            self._distances[key[::-1]] = value
        return self._distances[key]

    def snapshot(self):
        return dict(version=self.version, free_cells=self.free_cells,
                    nodes=[dict(id=k, position=p.tolist()) for k, p in sorted(self.nodes.items())],
                    edges=[dict(u=e.u, v=e.v, length=e.cost, points=e.path.tolist()) for e in self.edges])
