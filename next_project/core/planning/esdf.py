"""ESDF（欧氏符号距离场）+ CostAwareGrid 软代价包装。

用途
----
在占据栅格上叠加软代价场，使所有规划器自动倾向远离障碍物的路径。
软代价形式：cost(d) = w · exp(-d / scale)，其中 d 是到最近障碍物的欧氏距离。
当 d ≥ cap_distance 时 cost = 0。

原理
----
1) compute_esdf：基于 scipy.ndimage.distance_transform_edt 计算占据栅格到
   最近占据体素的欧氏距离（单位 m）。无 scipy 时退化为 6 邻域 BFS。
2) CostAwareGrid：包装 OccupancyGrid/SDFAwareGrid，通过 duck-typing 暴露
   extra_cost(idx)，规划器自动检测并叠加到边代价上。

"""

from __future__ import annotations

import numpy as np


def compute_esdf(grid, max_dist: float | None = None) -> np.ndarray:
    """计算占据栅格的欧氏距离场。

    参数
    ----
    grid: OccupancyGrid 或 compatible（有 data/origin/resolution/shape）
    max_dist: 最大计算距离 m，None 则计算全图

    返回
    ----
    (nx, ny, nz) float64 数组，自由体素到最近占据体素的欧氏距离 m。
    """
    # 占据 → 1（障碍），自由 → 0
    occupied = (grid.data >= 1).astype(np.uint8)

    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:
        return _bfs_distance_field(occupied, grid.resolution, max_dist)

    sampling = (grid.resolution,) * 3
    # scipy 的 EDT 返回“非零元素到最近零元素”的距离。
    # 对 1 - occupied 求 EDT，非零元素正好是自由体素，零元素正好是占据体素。
    distances = distance_transform_edt(1 - occupied, sampling=sampling)
    distances[occupied >= 1] = 0.0

    if max_dist is not None:
        distances = np.minimum(distances, max_dist)

    return distances.astype(np.float64)


def _bfs_distance_field(occupied: np.ndarray, resolution: float,
                        max_dist: float | None = None) -> np.ndarray:
    """6 邻域 BFS 距离场（scipy 不可用时的退化方案）。"""
    from collections import deque
    nx, ny, nz = occupied.shape
    dist = np.full((nx, ny, nz), float("inf"), dtype=np.float64)
    q = deque()

    # 占据体素距离为 0
    occ_idxs = np.argwhere(occupied >= 1)
    for idx in occ_idxs:
        t = tuple(idx)
        dist[t] = 0.0
        q.append(t)

    max_voxels = int(max_dist / resolution) if max_dist else max(nx, ny, nz)
    dirs = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]

    while q:
        cx, cy, cz = q.popleft()
        d = dist[cx, cy, cz]
        if d >= max_voxels * resolution:
            continue
        for dx, dy, dz in dirs:
            nx_i, ny_i, nz_i = cx + dx, cy + dy, cz + dz
            if 0 <= nx_i < nx and 0 <= ny_i < ny and 0 <= nz_i < nz:
                nd = d + resolution
                if nd < dist[nx_i, ny_i, nz_i]:
                    dist[nx_i, ny_i, nz_i] = nd
                    q.append((nx_i, ny_i, nz_i))

    return np.where(np.isinf(dist), max_dist or float(dist.max()), dist)


class CostAwareGrid:
    """软代价感知栅格包装器。

    参数
    ----
    base_grid: OccupancyGrid / SDFAwareGrid
    esdf: (nx, ny, nz) 距离场数组（由 compute_esdf 生成）
    weight: 软代价权重 w
    scale: 软代价衰减尺度 d₀ (m)
    cap_distance: 超过此距离软代价为 0
    """

    def __init__(self, base_grid, esdf: np.ndarray | None = None,
                 weight: float = 2.0, scale: float = 1.5,
                 cap_distance: float = 4.0):
        self._base = base_grid
        self.weight = float(weight)
        self.scale = float(scale)
        self.cap_distance = float(cap_distance)

        if esdf is not None:
            self._esdf = esdf
        else:
            self._esdf = compute_esdf(base_grid, max_dist=cap_distance)

        # 透传基本属性
        self.origin = base_grid.origin
        self.resolution = base_grid.resolution
        self.shape = base_grid.shape
        self.data = base_grid.data

    # ------------------------------------------------------------------
    # 栅格接口（透传）
    # ------------------------------------------------------------------

    def world_to_index(self, p: np.ndarray) -> tuple[int, int, int]:
        return self._base.world_to_index(p)

    def index_to_world(self, idx: tuple[int, int, int]) -> np.ndarray:
        return self._base.index_to_world(idx)

    def is_occupied(self, idx: tuple[int, int, int]) -> bool:
        return self._base.is_occupied(idx)

    def is_truly_occupied(self, idx: tuple[int, int, int]) -> bool:
        """仅判断真实障碍物占据（不含膨胀层）。"""
        return self._base.is_truly_occupied(idx) if hasattr(self._base, 'is_truly_occupied') else self._base.data[idx] == 1

    def is_free(self, idx: tuple[int, int, int]) -> bool:
        return self._base.is_free(idx)

    def inflate(self, radius: float) -> CostAwareGrid:
        new_base = self._base.inflate(radius)
        new_esdf = compute_esdf(new_base, max_dist=self.cap_distance)
        return CostAwareGrid(new_base, new_esdf,
                             weight=self.weight, scale=self.scale,
                             cap_distance=self.cap_distance)

    # ------------------------------------------------------------------
    # 软代价接口（规划器通过 duck-typing 检测）
    # ------------------------------------------------------------------

    def extra_cost(self, idx: tuple[int, int, int]) -> float:
        """返回体素 idx 处的软代价（叠加到边代价上）。

        被 A*/JPS/HybridA* 通过 hasattr(grid, 'extra_cost') 自动检测。
        """
        if idx[0] < 0 or idx[0] >= self.shape[0]:
            return self.weight
        if idx[1] < 0 or idx[1] >= self.shape[1]:
            return self.weight
        if idx[2] < 0 or idx[2] >= self.shape[2]:
            return self.weight

        d = float(self._esdf[idx])
        if d >= self.cap_distance:
            return 0.0
        return self.weight * np.exp(-d / self.scale)

    def signed_distance(self, pos: np.ndarray) -> float:
        """世界坐标处的 ESDF 距离 m（三线性插值近似）。

        正 = 自由空间内，值越大越安全。
        """
        idx_f = (np.asarray(pos, dtype=float) - self.origin) / self.resolution
        i0 = np.floor(idx_f).astype(int)
        i1 = i0 + 1

        def _clamp(v, dim):
            return max(0, min(v, self.shape[dim] - 1))

        # 三线性插值权重
        frac = idx_f - i0.astype(float)
        w000 = (1 - frac[0]) * (1 - frac[1]) * (1 - frac[2])
        w100 = frac[0] * (1 - frac[1]) * (1 - frac[2])
        w010 = (1 - frac[0]) * frac[1] * (1 - frac[2])
        w110 = frac[0] * frac[1] * (1 - frac[2])
        w001 = (1 - frac[0]) * (1 - frac[1]) * frac[2]
        w101 = frac[0] * (1 - frac[1]) * frac[2]
        w011 = (1 - frac[0]) * frac[1] * frac[2]
        w111 = frac[0] * frac[1] * frac[2]

        d000 = self._esdf[_clamp(i0[0],0), _clamp(i0[1],1), _clamp(i0[2],2)]
        d100 = self._esdf[_clamp(i1[0],0), _clamp(i0[1],1), _clamp(i0[2],2)]
        d010 = self._esdf[_clamp(i0[0],0), _clamp(i1[1],1), _clamp(i0[2],2)]
        d110 = self._esdf[_clamp(i1[0],0), _clamp(i1[1],1), _clamp(i0[2],2)]
        d001 = self._esdf[_clamp(i0[0],0), _clamp(i0[1],1), _clamp(i1[2],2)]
        d101 = self._esdf[_clamp(i1[0],0), _clamp(i0[1],1), _clamp(i1[2],2)]
        d011 = self._esdf[_clamp(i0[0],0), _clamp(i1[1],1), _clamp(i1[2],2)]
        d111 = self._esdf[_clamp(i1[0],0), _clamp(i1[1],1), _clamp(i1[2],2)]

        return float(w000 * d000 + w100 * d100 + w010 * d010 + w110 * d110 +
                     w001 * d001 + w101 * d101 + w011 * d011 + w111 * d111)

    def gradient(self, pos: np.ndarray) -> np.ndarray:
        """ESDF 数值梯度（指向远离障碍物方向）。"""
        eps = self.resolution * 0.25
        gx = (self.signed_distance(pos + [eps, 0, 0])
              - self.signed_distance(pos - [eps, 0, 0])) / (2 * eps)
        gy = (self.signed_distance(pos + [0, eps, 0])
              - self.signed_distance(pos - [0, eps, 0])) / (2 * eps)
        gz = (self.signed_distance(pos + [0, 0, eps])
              - self.signed_distance(pos - [0, 0, eps])) / (2 * eps)
        return np.array([gx, gy, gz], dtype=float)
