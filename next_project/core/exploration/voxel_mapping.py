"""Three-dimensional ray integration and known-free voxel routing.

The mapper accepts sensor-frame endpoints plus a pose, never scenario geometry.
Ray misses are integrated as free until max range; hits update occupied voxels.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree
from core.obstacles import OccupancyGrid


def lidar_return_mask(points, maximum_range=4.48):
    """Reject the modeled self hull, not every nearby obstacle return."""
    p = np.asarray(points, float)
    finite = np.isfinite(p).all(axis=1)
    ranges = np.linalg.norm(np.where(np.isfinite(p), p, 0.), axis=1)
    in_hull = np.all(np.abs(p+np.array([0., 0., .4])) <= [.33, .33, .09], axis=1)
    return finite & (ranges < maximum_range) & (ranges >= .1) & ~in_hull


class VoxelMap:
    ndim = 3
    def __init__(self, bounds, resolution=.3, clearance=.6, flight_limits=None):
        self.bounds = np.asarray(bounds, float); self.origin = self.bounds[0].copy()
        self.flight_limits = (.7, self.bounds[1, 2]-.3) if flight_limits is None else tuple(flight_limits)
        self.resolution = resolution; self.clearance = clearance; self.altitude = 1.5
        self.shape = tuple(np.ceil((self.bounds[1]-self.origin)/resolution).astype(int))
        self.state = np.full(self.shape, -1, np.int8); self.version = 0; self.field = self
        self.rebuild()

    def indices(self, points):
        return np.floor((np.asarray(points)-self.origin)/self.resolution).astype(int)

    def points(self, indices):
        return self.origin+(np.asarray(indices)+.5)*self.resolution

    def update(self, indices, values):
        indices = np.asarray(indices, int).reshape(-1, 3); key = tuple(indices.T)
        changed = bool(np.any(self.state[key] != values))
        self.state[key] = values
        if changed:
            self.version += 1
        return changed

    def integrate(self, sensor_origin, endpoints, hits):
        origin = np.asarray(sensor_origin); endpoints = np.asarray(endpoints)
        hits = np.asarray(hits, bool); finite = np.isfinite(endpoints).all(axis=1)
        endpoints = endpoints[finite]; hits = hits[finite]
        if not len(endpoints):
            return np.empty((0, 3), int), np.empty(0, np.int8)
        count = max(2, int(np.ceil(np.max(np.linalg.norm(endpoints-origin, axis=1))/(self.resolution*.4)))+1)
        rays = origin+(endpoints-origin)[:, None, :]*np.linspace(0., 1., count)[None, :, None]
        cells = self.indices(rays)
        valid = np.all((cells >= 0) & (cells < self.shape), axis=2)
        # Surface returns within one boundary voxel are clamped to that voxel.
        # This handles range noise around the floor/map boundary conservatively.
        near = np.all((cells[:, -1] >= -1) & (cells[:, -1] <= self.shape), axis=1)
        occupied = np.clip(cells[:, -1][hits & near], 0, np.array(self.shape)-1)
        ids = np.unique(np.ravel_multi_index(tuple(cells[valid].T), self.shape))
        occupied_ids = np.unique(np.ravel_multi_index(tuple(occupied.T), self.shape)) if len(occupied) else np.empty(0, int)
        values = np.isin(ids, occupied_ids).astype(np.int8)
        indices = np.column_stack(np.unravel_index(ids, self.shape))
        self.update(indices, values)
        return indices, values

    def rebuild(self):
        # Copies sent to the planner already contain a valid collision field.
        # Compare bytes as well as configuration: tests/imports may edit state
        # directly without incrementing the sensor version counter.
        signature = (self.state.tobytes(), self.clearance, self.flight_limits)
        if getattr(self, '_built_signature', None) == signature:
            return
        self._built_signature = signature
        self.reserved_points = None
        # Quadrotor collision envelope is anisotropic: 0.6m horizontally and
        # 0.2m vertically (the physical hull is only 0.16m tall). Isotropic
        # inflation incorrectly turns the lidar's nadir shadow into a deadlock.
        self.metric = np.array([1., 1., 3.])
        occupied = np.argwhere(np.pad(self.state != 0, 1, constant_values=True))-1
        self.obstacle_centers = self.points(occupied)
        self.obstacle_tree = cKDTree(self.obstacle_centers*self.metric)
        self.distance = np.full(self.shape, -1.)
        free = np.argwhere(self.state == 0)
        if len(free):
            points = self.points(free)
            _, indices = self.obstacle_tree.query(points*self.metric, k=min(16, len(occupied)))
            delta = np.maximum(np.abs(points[:, None]-self.obstacle_centers[indices])-self.resolution/2, 0.)
            self.distance[tuple(free.T)] = np.min(np.linalg.norm(delta*self.metric, axis=2), axis=1)
        self.safe = self.distance >= self.clearance
        heights = self.origin[2]+(np.arange(self.shape[2])+.5)*self.resolution
        self.safe &= (heights[None, None, :] >= self.flight_limits[0]) & (heights[None, None, :] <= self.flight_limits[1])
        self.grid = OccupancyGrid(self.origin+self.resolution/2, self.resolution, self.shape)
        self.grid.data[:] = ~self.safe

    def signed_distance(self, p):
        idx = self.indices(p)
        if np.any(idx < 0) or np.any(idx >= self.shape):
            return -1.
        _, ids = self.obstacle_tree.query(np.asarray(p)*self.metric, k=min(16, len(self.obstacle_centers)))
        delta = np.maximum(np.abs(np.asarray(p)-self.obstacle_centers[ids])-self.resolution/2, 0.)
        return float(np.min(np.linalg.norm(delta*self.metric, axis=1)))

    def safe_path(self, path):
        p = np.asarray(path, float)
        if p.ndim != 2 or p.shape[1] != 3 or len(p) == 0 or not np.isfinite(p).all():
            return False
        chunks = [p]+[np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b-a)/.06))+1)) for a, b in zip(p[:-1], p[1:])]
        points = np.vstack(chunks); idx = self.indices(points)
        if np.any(points[:, 2] < self.flight_limits[0]) or np.any(points[:, 2] > self.flight_limits[1]):
            return False
        if not (np.all(idx >= 0) and np.all(idx < self.shape)):
            return False
        # A centerline inside an occupied/unknown box cannot pass any positive
        # body-clearance check. Reject it before the costly neighborhood query;
        # free centerlines still undergo the full envelope and reservation test.
        if np.any(self.state[tuple(idx.T)] != 0):
            return False
        # Every voxel box potentially intersecting the ellipsoid is checked;
        # nearest-center interpolation alone is not a safety certificate.
        radius = self.clearance+np.linalg.norm(self.metric*self.resolution/2)
        neighbours = self.obstacle_tree.query_ball_point(points*self.metric, radius)
        counts = np.array([len(v) for v in neighbours]); total = int(counts.sum())
        if total:
            indices = np.concatenate([v for v in neighbours if len(v)]).astype(int)
            repeated = np.repeat(points, counts, axis=0)
            delta = np.maximum(np.abs(repeated-self.obstacle_centers[indices])-self.resolution/2, 0.)
            if getattr(self, 'recovery_yaw', None) is None:
                if np.any(np.linalg.norm(delta*self.metric, axis=1) < self.clearance):
                    return False
            else:
                # Short, fixed-yaw retreat uses the actual collision hull.
                # Test all separating axes of the oriented hull
                # and occupied/unknown voxel boxes; do not erase unknown space.
                offset = self.obstacle_centers[indices]-repeated
                c, s = np.cos(self.recovery_yaw), np.sin(self.recovery_yaw)
                axes = np.array([[c, s], [-s, c]])
                half = self.resolution/2; body = .32
                xy_overlap = np.all(np.abs(offset[:, :2]) < half+body*(abs(c)+abs(s)), axis=1)
                body_overlap = np.all(np.abs(offset[:, :2]@axes.T) < body+half*(abs(c)+abs(s)), axis=1)
                if np.any(xy_overlap & body_overlap & (np.abs(offset[:, 2]) < half+.08)):
                    return False
        if getattr(self, 'reserved_points', None) is not None:
            if np.any(self.reserved_points.query(points)[0] < self.reserved_radius):
                return False
        return True

    def block_paths(self, paths, radius=1.25):
        chunks = []
        for path in paths:
            p = np.asarray(path, float); chunks.append(p)
            chunks.extend(np.linspace(a, b, max(2, int(np.linalg.norm(b-a)/.15)+1)) for a, b in zip(p[:-1], p[1:]))
        if not chunks:
            return
        self._built_signature = None
        cells = np.argwhere(self.safe)
        if not len(cells):
            return
        # Full 3D reservation distance, conservative isotropic downwash margin.
        self.reserved_points = cKDTree(np.vstack(chunks)); self.reserved_radius = radius
        blocked = self.reserved_points.query(self.points(cells))[0] < radius
        key = tuple(cells[blocked].T); self.safe[key] = False; self.distance[key] = 0.; self.grid.data[key] = True


class VoxelRouter:
    """Private local collision routing. Shared global topology remains MR-DTG."""
    def __init__(self, runtime):
        self.runtime = runtime; self.cache = {}; self.cells = np.argwhere(runtime.safe)
        self.lookup = np.full(runtime.shape, -1, int)
        self.lookup[tuple(self.cells.T)] = np.arange(len(self.cells))
        rows = []; cols = []
        for axis in range(3):
            for sign in (-1, 1):
                shifted = self.cells.copy(); shifted[:, axis] += sign
                valid = np.all((shifted >= 0) & (shifted < runtime.shape), axis=1)
                a = np.flatnonzero(valid); b = self.lookup[tuple(shifted[valid].T)]; use = b >= 0
                rows.extend(a[use]); cols.extend(b[use])
        self.matrix = csr_matrix((np.full(len(rows), runtime.resolution), (rows, cols)), shape=(len(self.cells), len(self.cells)))

    def node(self, p):
        idx = self.runtime.indices(p)
        if np.any(idx < 0) or np.any(idx >= self.runtime.shape):
            return -1
        return int(self.lookup[tuple(idx)])

    def search(self, start, limit=np.inf):
        index = self.node(start)
        if index < 0:
            return None
        key = (index, limit)
        if key not in self.cache:
            self.cache[key] = dijkstra(self.matrix, directed=False, indices=index, return_predecessors=True, limit=limit)
        return self.cache[key]

    def route(self, start, goal):
        if self.runtime.safe_path([start, goal]):
            return np.array([start, goal])
        result = self.search(start); end = self.node(goal)
        if result is None or end < 0 or not np.isfinite(result[0][end]):
            return None
        cells = []; node = end
        while node >= 0:
            cells.append(self.cells[node]); node = result[1][node]
        p = np.vstack([start, self.runtime.points(cells[::-1]), goal])
        return p if self.runtime.safe_path(p) else None

    def distance(self, start, goal):
        if self.runtime.safe_path([start, goal]):
            return float(np.linalg.norm(np.asarray(start)-goal))
        result = self.search(start); end = self.node(goal)
        return float(result[0][end]) if result is not None and end >= 0 else np.inf


class VoxelTruth:
    """Read-only simulation evaluator; never passed to an exploration agent."""
    def __init__(self, file, resolution=.3):
        import json
        data = json.loads(open(file).read()); self.bounds = np.array(data['bounds'], float)
        self.origin = self.bounds[0]; self.resolution = resolution
        self.shape = tuple(np.ceil((self.bounds[1]-self.origin)/resolution).astype(int))
        xyz = self.origin+(np.indices(self.shape).transpose(1, 2, 3, 0)+.5)*resolution
        self.xyz = xyz
        self.low = np.maximum(xyz-resolution/2, self.bounds[0])
        self.high = np.minimum(xyz+resolution/2, self.bounds[1])
        self.occupied = xyz[..., 2] < resolution
        self.center_occupied = self.occupied.copy()
        for o in data['obstacles']:
            self.occupied |= self.intersects(o)
            if o['type'] == 'aabb':
                self.center_occupied |= np.all((xyz >= o['min']) & (xyz <= o['max']), axis=3)
            elif o['type'] == 'cylinder':
                self.center_occupied |= ((np.linalg.norm(xyz[..., :2]-o['center_xy'], axis=3) <= o['radius']) &
                                  (xyz[..., 2] >= o['z_range'][0]) & (xyz[..., 2] <= o['z_range'][1]))
        self.static_occupied = self.occupied.copy()
        self.static_center_occupied = self.center_occupied.copy(); self.last_obstacle_version = -1

    def intersects(self, obstacle):
        """A surface return occupies its voxel even if the center is outside geometry.

        Use positive-volume box intersection (and closest-point circle/box
        intersection for cylinders). Boundary voxels are clipped to map bounds.
        This matches the map's conservative occupied-voxel representation.
        """
        if obstacle['type'] == 'aabb':
            return np.all((self.high > np.asarray(obstacle['min'])+1e-9) &
                          (self.low < np.asarray(obstacle['max'])-1e-9), axis=3)
        if obstacle['type'] == 'cylinder':
            center = np.asarray(obstacle['center_xy'])
            nearest = np.clip(center, self.low[..., :2], self.high[..., :2])
            return ((np.linalg.norm(nearest-center, axis=3) < obstacle['radius']-1e-9) &
                    (self.high[..., 2] > obstacle['z_range'][0]+1e-9) & (self.low[..., 2] < obstacle['z_range'][1]-1e-9))
        raise ValueError('Unsupported truth geometry')

    def dynamic_snapshot(self, version, obstacles):
        if version <= self.last_obstacle_version:return
        self.occupied = self.static_occupied.copy()
        self.center_occupied = self.static_center_occupied.copy()
        for o in obstacles:
            self.occupied |= self.intersects(dict(o, type='cylinder'))
            self.center_occupied |= ((np.linalg.norm(self.xyz[..., :2]-o['center_xy'], axis=3) <= o['radius']) &
                              (self.xyz[..., 2] >= o['z_range'][0]) & (self.xyz[..., 2] <= o['z_range'][1]))
        self.last_obstacle_version = version

    def coverage(self, observed):
        return float(np.count_nonzero((observed.state == 0) & ~self.occupied)/np.count_nonzero(~self.occupied))

    def coverage_metrics(self, observed):
        free = ~self.occupied
        return dict(coverage=self.coverage(observed),
            coverage_definition='observed-free / geometrically fully-free voxels; box/cylinder intersection',
            legacy_center_free_coverage=float(np.count_nonzero((observed.state == 0) & ~self.center_occupied)/np.count_nonzero(~self.center_occupied)),
            truth_free_voxels=int(free.sum()), observed_free_voxels=int(np.count_nonzero((observed.state == 0) & free)),
            unobserved_free_voxels=int(np.count_nonzero((observed.state == -1) & free)),
            falsely_occupied_free_voxels=int(np.count_nonzero((observed.state == 1) & free)))
