"""从观测地图构造探索收益层；不改变碰撞代价或未知空间的通行条件。"""
from dataclasses import dataclass
import hashlib
import math
import numpy as np
from scipy.ndimage import label, find_objects
from .regions import visible_cells, information_count


@dataclass(frozen=True)
class PriorityConfig:
    speed: float = .6
    observation_s: float = 1.5
    region_weight: float = .35
    region_reference_volume: float = 8.
    aging_per_minute: float = .15
    aging_bonus_max: float = .5
    history_weight: float = .1
    route_latency_weight: float = .5
    repeat_decay_s: float = 600.
    low_yield_cells: int = 30
    retry_base_s: float = 15.
    retry_max_s: float = 120.
    peer_fresh_s: float = 3.
    preview_points: int = 2
    preview_headings: int = 4
    preview_azimuth_samples: int = 21
    preview_pitch_samples: int = 5
    remote_gain_cap: float = 1.
    preview_regions: int = 24
    refresh_regions: int = 4

    def __post_init__(self):
        for name in ('speed', 'observation_s', 'region_reference_volume', 'repeat_decay_s',
                     'low_yield_cells', 'retry_base_s', 'retry_max_s', 'peer_fresh_s',
                     'preview_points', 'preview_headings', 'preview_azimuth_samples', 'preview_pitch_samples', 'remote_gain_cap',
                     'preview_regions', 'refresh_regions'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('region_weight', 'aging_per_minute', 'aging_bonus_max', 'route_latency_weight'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('low_yield_cells', 'preview_points', 'preview_headings', 'preview_azimuth_samples', 'preview_pitch_samples',
                     'preview_regions', 'refresh_regions'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} must be an integer')
        if not 0 < self.history_weight <= 1 or self.refresh_regions > self.preview_regions:
            raise ValueError('Invalid history weight or preview refresh budget')


def service_result(previous, now, new_cells, expected_cells, config=None):
    """短期退避随连续低收益递增；成功观测清除旧惩罚，小残片按比例判定。"""
    config = config or PriorityConfig()
    threshold = min(config.low_yield_cells, max(1, math.ceil(expected_cells*.2)))
    low = new_cells < threshold
    streak = int((previous or {}).get('low_yield_streak', 0))+1 if low else 0
    delay = min(config.retry_max_s, config.retry_base_s*2**min(streak-1, 16)) if low else 0.
    return dict(stamp=float(now), defer_until=float(now+delay), observed_new_cells=int(new_cells),
                expected_cells=int(expected_cells), low_yield_streak=streak)


def service_cells(runtime, task, point, yaw):
    """观测前记下区域内及跨区域可见的未知格，排除途中无关区域的新信息。"""
    cells = set(visible_cells(runtime, point, yaw))
    if task is not None:
        low, high = np.asarray(task.bounds)
        a = np.maximum(0, np.floor((low-runtime.origin)/runtime.resolution).astype(int))
        b = np.minimum(runtime.shape, np.ceil((high-runtime.origin)/runtime.resolution).astype(int))
        unknown = np.argwhere(runtime.state[tuple(slice(i, j) for i, j in zip(a, b))] == -1)+a
        if len(unknown):
            cells.update(np.ravel_multi_index(tuple(unknown.T), runtime.shape).tolist())
    return np.array(sorted(cells), dtype=int)


class ExplorationPriority:
    def __init__(self, config=None):
        self.config = config or PriorityConfig()
        self.labels = None; self.sizes = None; self.components = {}
        self.first_seen = {}; self.scores = {}; self._signature = None
        self._views = {}; self._geometry_cache = None; self._refreshed = {}; self.cycle = 0
        self.diagnostics = {}

    def update_map(self, runtime):
        signature = (runtime.state.shape, runtime.resolution, tuple(runtime.origin), runtime.state.tobytes())
        if signature == self._signature:
            return
        self._signature = signature
        # Face connectivity avoids joining pockets across occupied diagonal corners.
        self.labels, _ = label(runtime.state == -1)
        self.sizes = np.bincount(self.labels.ravel())
        self.sizes[0] = 0
        unit = runtime.resolution**runtime.state.ndim
        self.components = {}
        for ident, slices in enumerate(find_objects(self.labels), 1):
            if slices is None:
                continue
            low = runtime.origin+np.array([s.start for s in slices])*runtime.resolution
            high = np.minimum(runtime.bounds[1, :runtime.state.ndim],
                              runtime.origin+np.array([s.stop for s in slices])*runtime.resolution)
            self.components[ident] = dict(id=ident, cells=int(self.sizes[ident]),
                volume=float(self.sizes[ident]*unit), bounds=[low.tolist(), high.tolist()])

    def geometry(self, runtime, point):
        # Cache against actual nearby occupancy, including the elevated 3D sensor.
        # A direct map edit (without a version bump) must also invalidate the ray gain.
        sensor = np.asarray(point)[:runtime.state.ndim].copy()
        if runtime.state.ndim == 3:
            sensor[2] += .4
        a = np.maximum(0, np.floor((sensor-4.5-runtime.origin)/runtime.resolution).astype(int))
        b = np.minimum(runtime.shape, np.ceil((sensor+4.5-runtime.origin)/runtime.resolution).astype(int)+1)
        key = tuple(np.asarray(point, float))
        if self._geometry_cache is not None and key in self._geometry_cache:
            return self._geometry_cache[key]
        geometry = (hashlib.blake2b(runtime.state[tuple(slice(i, j) for i, j in zip(a, b))].tobytes(), digest_size=16).digest(),
                    tuple(a), tuple(b), runtime.shape, runtime.resolution, tuple(runtime.origin))
        if self._geometry_cache is not None:
            self._geometry_cache[key] = geometry
        return geometry

    def view(self, runtime, point, yaw, coarse=False):
        geometry = self.geometry(runtime, point)
        key = (tuple(np.asarray(point, float)), float(yaw), coarse)
        old = self._views.get(key)
        if old is None or old[0] != geometry:
            sampling = dict(azimuth_samples=self.config.preview_azimuth_samples,
                            pitch_samples=self.config.preview_pitch_samples) if coarse else {}
            self._views[key] = (geometry, visible_cells(runtime, point, yaw, **sampling))
            self.diagnostics['raycasts'] = self.diagnostics.get('raycasts', 0)+1
        return self._views[key][1]

    def committed_cells(self, runtime, peers, now):
        cells = set()
        for peer in peers.values():
            intent = peer.get('intent') or {}
            if (not intent.get('committed') or intent.get('retiring') or intent.get('recovery') or
                    not -.1 <= now-peer.get('time', -np.inf) < self.config.peer_fresh_s):
                continue
            if intent.get('path'):
                cells.update(self.view(runtime, intent['path'][-1], intent.get('yaw', 0.)))
        return frozenset(cells)

    def utility(self, gain, volume, travel_s, wait_s, feedback, now):
        if gain <= 0 or not np.isfinite(travel_s):
            return 0., 1.
        streak = feedback.get('low_yield_streak', 1 if feedback.get('defer_until', 0) > now else 0)
        age = max(0., now-feedback.get('stamp', now))
        repeat = 1/(1+streak*math.exp(-age/self.config.repeat_decay_s))
        bonus = 1+self.config.region_weight*math.log1p(volume/self.config.region_reference_volume)
        score = repeat*gain*bonus/(self.config.observation_s+travel_s)
        # A long wait cannot manufacture information or cancel a long trip.
        # Small remnants remain in the spatial tour and preview refresh rotation.
        return score*self.aging_multiplier(wait_s), repeat

    def aging_multiplier(self, wait_s):
        return 1.+min(self.config.aging_bonus_max, self.config.aging_per_minute*max(0., wait_s)/60.)

    def rank(self, runtime, tasks, local_ids, costs, position, now, services, excluded_cells=frozenset(),
             observed_mask=None, preferred=()):
        self.update_map(runtime)
        self._geometry_cache = {}; self.cycle += 1
        self.diagnostics = dict(raycasts=0, refined_regions=0, cached_regions=0, bound_regions=0)
        unit = runtime.resolution**runtime.state.ndim
        excluded = np.array(sorted(excluded_cells), dtype=int)
        discount = np.zeros(runtime.state.size)
        if observed_mask is not None:
            discount[observed_mask] = 1.-self.config.history_weight
        discount[excluded] = 1.
        reserved = np.bincount(self.labels.ravel(), weights=discount, minlength=len(self.sizes))
        scores = {}
        used_views = set()
        travel_times = {}; waits = {}; bounds = {}; volumes = {}
        for rid, task in tasks.items():
            self.first_seen.setdefault(rid, self.first_seen.get(getattr(task, 'parent', -1), now))
            feedback = services.get(rid, {})
            wait = max(0., now-max(self.first_seen[rid], feedback.get('stamp', -np.inf)))
            travel = costs.distance(position, task.entry)/self.config.speed
            travel_times[rid] = travel; waits[rid] = wait
            volume = task.unknown*unit
            if observed_mask is not None:
                low, high = np.asarray(task.bounds)[:, :runtime.state.ndim]
                a = np.maximum(0, np.floor((low-runtime.origin)/runtime.resolution).astype(int))
                b = np.minimum(runtime.shape, np.ceil((high-runtime.origin)/runtime.resolution).astype(int))
                window = tuple(slice(i, j) for i, j in zip(a, b))
                unknown = runtime.state[window] == -1
                total = int(unknown.sum())
                if total:
                    seen = int(np.count_nonzero(observed_mask.reshape(runtime.shape)[window] & unknown))
                    volume *= 1.-(1.-self.config.history_weight)*seen/total
            volumes[rid] = volume
            bounds[rid] = self.utility(min(volume, self.config.remote_gain_cap), volume, travel, wait, feedback, now)[0]
        eligible = [r for r in local_ids if r in tasks and np.isfinite(travel_times[r]) and services.get(r, {}).get('defer_until', 0) <= now]
        preferred = set(preferred)
        rotating = sorted(eligible, key=lambda r: (self._refreshed.get(r, -1), r))[:self.config.refresh_regions]
        ranked = sorted(eligible, key=lambda r: (r not in preferred, -bounds[r], r))
        refine = set(rotating+ [r for r in ranked if r not in rotating][:max(0, self.config.preview_regions-len(rotating))])
        for rid, task in tasks.items():
            feedback = services.get(rid, {}); wait = waits[rid]; travel = travel_times[rid]
            best = None
            if rid in local_ids:
                views = []
                for point in task.viewpoints[:self.config.preview_points]:
                    cell = runtime.indices(point)
                    if np.any(cell < 0) or np.any(cell >= runtime.shape) or not runtime.safe[tuple(cell)]:
                        continue
                    for yaw in np.linspace(-np.pi, np.pi, self.config.preview_headings, endpoint=False):
                        key = (tuple(np.asarray(point, float)), float(yaw), True)
                        used_views.add(key); views.append((point, yaw, key))
                cached = bool(views) and all(k in self._views and self._views[k][0] == self.geometry(runtime, p) for p, _, k in views)
                if cached or rid in refine:
                    self.diagnostics['cached_regions' if cached else 'refined_regions'] += 1
                    self._refreshed[rid] = self.cycle
                    for point, yaw, _ in views:
                        raw = self.view(runtime, point, yaw, coarse=True)
                        net = raw-excluded_cells
                        ids = sorted(set(self.labels.ravel()[list(net)])) if net else []
                        ids = [int(k) for k in ids if k]
                        volume = float(sum(max(0, self.sizes[k]-reserved[k]) for k in ids)*unit)
                        gain = information_count(net, observed_mask, self.config.history_weight)*unit
                        value, penalty = self.utility(gain, volume, travel, wait, feedback, now)
                        row = dict(score=value, predicted_gain=gain, raw_gain=len(raw)*unit,
                                   component_volume=volume, components=ids, repeat_factor=penalty,
                                   preview_position=np.asarray(point).tolist(), preview_yaw=float(yaw))
                        if best is None or (value, gain, row['raw_gain']) > (best['score'], best['predicted_gain'], best['raw_gain']):
                            best = row
                    source = 'local_rays'
                else:
                    # An explicit bound guides the spatial tour until its budgeted
                    # preview. The executed view always recomputes real ray gain.
                    self.diagnostics['bound_regions'] += 1
                    volume = volumes[rid]; gain = min(volume, self.config.remote_gain_cap)
                    value, penalty = self.utility(gain, volume, travel, wait, feedback, now)
                    best = dict(score=value, predicted_gain=gain, raw_gain=gain, component_volume=volume,
                                components=[], repeat_factor=penalty)
                    source = 'region_bound'
            else:
                # Remote maps are not shared. This is an explicitly marked proxy
                # for safe corridor transit, never a claimed local visible volume.
                volume = volumes[rid]
                gain = min(volume, self.config.remote_gain_cap)
                value, penalty = self.utility(gain, volume, travel, wait, feedback, now)
                best = dict(score=value, predicted_gain=gain, raw_gain=gain,
                            component_volume=volume, components=[], repeat_factor=penalty)
                source = 'remote_proxy'
            if best is None:
                best = dict(score=0., predicted_gain=0., raw_gain=0., component_volume=0., components=[], repeat_factor=1.)
            scores[rid] = dict(best, region=rid, travel_s=float(travel) if np.isfinite(travel) else None,
                              wait_s=wait, aging_multiplier=self.aging_multiplier(wait), gain_source=source,
                              information_reward=best['score']*(self.config.observation_s+travel) if np.isfinite(travel) else 0.,
                              deferred=feedback.get('defer_until', 0) > now)
        self._views = {k: v for k, v in self._views.items() if k in used_views}
        self._geometry_cache = None
        self._refreshed = {r: v for r, v in self._refreshed.items() if r in tasks}
        self.scores = scores
        return scores

    def snapshot(self, runtime, now, owners, source):
        """诊断专用：单机观测切片及组件，绝不通过 graph_delta 作为全局真值共享。"""
        components = {k: dict(v, priority=0., regions=[]) for k, v in self.components.items()}
        for rid, row in self.scores.items():
            for ident in row['components']:
                components[ident]['regions'].append(rid)
                if not row['deferred']:
                    components[ident]['priority'] = max(components[ident]['priority'], row['score'])
        cells = self.labels
        if cells.ndim == 3:
            z = int(np.clip((runtime.altitude-runtime.origin[2])/runtime.resolution, 0, cells.shape[2]-1))
            cells = cells[:, :, z]
        return dict(source=source, time=now, unit='m3' if runtime.state.ndim == 3 else 'm2',
                    resolution=runtime.resolution, origin=runtime.origin[:2].tolist(), shape=list(cells.shape),
                    slice_labels=cells.ravel().tolist(), components=list(components.values()),
                    regions=[dict(row, owner=owners.get(rid)) for rid, row in self.scores.items()])
