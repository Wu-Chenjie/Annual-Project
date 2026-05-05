"""障碍物建模。

用途
----
定义障碍物原语（AABB / Sphere / Cylinder）与统一容器 ObstacleField，
提供符号距离查询、碰撞检测和体素化能力，供路径规划与可视化使用。

原理
----
1) 符号距离函数 (SDF)：对点 p 返回到最近障碍物表面的有符号距离，
   负值表示在障碍物内部。
2) 占据栅格：将连续空间离散为均匀体素，0=自由, 1=占据, 2=膨胀层。
3) 膨胀半径 = arm_length + formation_envelope + safety_margin，
   确保编队整体不碰撞。

"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class AABB:
    """轴对齐立方体障碍物。"""

    min_corner: np.ndarray  # (3,)
    max_corner: np.ndarray  # (3,)

    def __post_init__(self):
        self.min_corner = np.asarray(self.min_corner, dtype=float)
        self.max_corner = np.asarray(self.max_corner, dtype=float)

    def signed_distance(self, p: np.ndarray) -> float:
        """点到 AABB 的有符号距离（内部为负）。"""
        p = np.asarray(p, dtype=float)
        #SDF公式
        q = np.abs(p - (self.min_corner + self.max_corner) / 2.0) - (self.max_corner - self.min_corner) / 2.0
        outside = np.linalg.norm(np.maximum(q, 0.0))
        inside = min(np.max(q), 0.0)
        return outside + inside


@dataclass
class Sphere:
    """球体障碍物。"""

    center: np.ndarray  # (3,)
    radius: float

    def __post_init__(self):
        self.center = np.asarray(self.center, dtype=float)
        self.radius = float(self.radius)

    def signed_distance(self, p: np.ndarray) -> float:
        p = np.asarray(p, dtype=float)
        return float(np.linalg.norm(p - self.center)) - self.radius


@dataclass
class Cylinder:
    """立式圆柱障碍物（z 轴），用于建模柱子。"""

    center_xy: np.ndarray  # (2,)
    radius: float
    z_range: tuple[float, float]

    def __post_init__(self):
        self.center_xy = np.asarray(self.center_xy, dtype=float)
        self.radius = float(self.radius)
        self.z_range = (float(self.z_range[0]), float(self.z_range[1]))

    def signed_distance(self, p: np.ndarray) -> float:
        p = np.asarray(p, dtype=float)
        dx = np.linalg.norm(p[:2] - self.center_xy) - self.radius
        dz_bottom = self.z_range[0] - p[2]
        dz_top = p[2] - self.z_range[1]
        dz = max(dz_bottom, dz_top, 0.0)
        # 在圆柱侧面外侧：只考虑径向距离
        if dz <= 0 and dx > 0:
            return dx
        # 完全在圆柱内部
        if dx <= 0 and dz <= 0:
            return max(dx, dz_bottom, dz_top)
        # 在角区
        return float(np.sqrt(max(dx, 0.0) ** 2 + dz ** 2))


class ObstacleField:
    """障碍物统一容器。

    用途
    ----
    管理多类型障碍物，提供统一的距离查询、碰撞检测和体素化接口。

    原理
    ----
    对所有子障碍物取最小符号距离；碰撞判断采用安全裕度扩展。
    """

    def __init__(self):
        self._obstacles: list[AABB | Sphere | Cylinder] = []

    def add(self, obs: AABB | Sphere | Cylinder) -> None:
        self._obstacles.append(obs)

    def add_aabb(self, min_corner, max_corner) -> None:
        self.add(AABB(np.asarray(min_corner, dtype=float), np.asarray(max_corner, dtype=float)))

    def add_sphere(self, center, radius: float) -> None:
        self.add(Sphere(np.asarray(center, dtype=float), radius))

    def add_cylinder(self, center_xy, radius: float, z_min: float, z_max: float) -> None:
        self.add(Cylinder(np.asarray(center_xy, dtype=float), radius, (z_min, z_max)))

    def signed_distance(self, p: np.ndarray) -> float:
        """返回点到最近障碍物表面的有符号距离。"""
        if not self._obstacles:
            return float("inf")
        return float(min(obs.signed_distance(p) for obs in self._obstacles))

    def is_collision(self, p: np.ndarray, inflate: float = 0.0) -> bool:
        """判断点 p 是否与障碍物碰撞（可指定膨胀半径）。"""
        return self.signed_distance(p) < inflate

    def __len__(self) -> int:
        return len(self._obstacles)

    def __iter__(self):
        return iter(self._obstacles)

    def to_voxel_grid(self, bounds: np.ndarray, resolution: float) -> OccupancyGrid:
        """将障碍物场体素化为占据栅格（按障碍物包围盒向量化，避免全网格遍历）。

        bounds: (2, 3) 数组 [[xmin, ymin, zmin], [xmax, ymax, zmax]]
        """
        bounds = np.asarray(bounds, dtype=float)
        origin = bounds[0, :].copy()
        res = float(resolution)
        shape = tuple(int(np.ceil((bounds[1, i] - bounds[0, i]) / res)) + 1 for i in range(3))
        grid = OccupancyGrid(origin=origin, resolution=res, shape=shape)
        data = grid.data

        for obs in self._obstacles:
            if isinstance(obs, AABB):
                i_min = np.clip(((obs.min_corner - origin) / res).astype(int), 0, np.array(shape) - 1)
                i_max = np.clip(((obs.max_corner - origin) / res).astype(int) + 1, 0, np.array(shape) - 1)
                data[i_min[0]:i_max[0]+1, i_min[1]:i_max[1]+1, i_min[2]:i_max[2]+1] = 1
            elif isinstance(obs, Sphere):
                r_vox = int(np.ceil(obs.radius / res))
                c_idx = ((obs.center - origin) / res).astype(int)
                i_min = np.clip(c_idx - r_vox, 0, np.array(shape) - 1)
                i_max = np.clip(c_idx + r_vox, 0, np.array(shape) - 1)
                for ix in range(i_min[0], i_max[0] + 1):
                    for iy in range(i_min[1], i_max[1] + 1):
                        for iz in range(i_min[2], i_max[2] + 1):
                            p = origin + np.array([ix, iy, iz], dtype=float) * res
                            if np.linalg.norm(p - obs.center) <= obs.radius:
                                data[ix, iy, iz] = 1
            elif isinstance(obs, Cylinder):
                r_vox = int(np.ceil(obs.radius / res))
                cx, cy = obs.center_xy
                z_min, z_max = obs.z_range
                cx_idx = int((cx - origin[0]) / res)
                cy_idx = int((cy - origin[1]) / res)
                z_min_idx = int((z_min - origin[2]) / res)
                z_max_idx = int((z_max - origin[2]) / res)
                ix_min = max(0, cx_idx - r_vox)
                ix_max = min(shape[0] - 1, cx_idx + r_vox)
                iy_min = max(0, cy_idx - r_vox)
                iy_max = min(shape[1] - 1, cy_idx + r_vox)
                iz_min = max(0, z_min_idx)
                iz_max = min(shape[2] - 1, z_max_idx)
                for ix in range(ix_min, ix_max + 1):
                    for iy in range(iy_min, iy_max + 1):
                        px = origin[0] + ix * res
                        py = origin[1] + iy * res
                        if (px - cx)**2 + (py - cy)**2 > obs.radius**2:
                            continue
                        data[ix, iy, iz_min:iz_max+1] = 1
            else:
                # 退化：SDF 逐体素检查（仅限包围盒范围）
                bbox_min, bbox_max = self._obs_bbox(obs)
                i_min = np.clip(((bbox_min - origin) / res).astype(int), 0, np.array(shape) - 1)
                i_max = np.clip(((bbox_max - origin) / res).astype(int) + 1, 0, np.array(shape) - 1)
                for ix in range(i_min[0], i_max[0] + 1):
                    for iy in range(i_min[1], i_max[1] + 1):
                        for iz in range(i_min[2], i_max[2] + 1):
                            if data[ix, iy, iz]:
                                continue
                            p = origin + np.array([ix, iy, iz], dtype=float) * res
                            if obs.signed_distance(p) <= 0:
                                data[ix, iy, iz] = 1

        return grid

    @staticmethod
    def _obs_bbox(obs) -> tuple[np.ndarray, np.ndarray]:
        """返回障碍物的粗略包围盒。"""
        if isinstance(obs, AABB):
            return obs.min_corner.copy(), obs.max_corner.copy()
        if isinstance(obs, Sphere):
            r = np.array([obs.radius] * 3)
            return obs.center - r, obs.center + r
        if isinstance(obs, Cylinder):
            r = np.array([obs.radius, obs.radius, 0.0])
            center = np.array([*obs.center_xy, (obs.z_range[0] + obs.z_range[1]) / 2])
            half_z = (obs.z_range[1] - obs.z_range[0]) / 2
            return center - r - np.array([0, 0, half_z]), center + r + np.array([0, 0, half_z])
        return np.zeros(3), np.zeros(3)


@dataclass
class OccupancyGrid:
    """三维占据栅格地图。

    用途
    ----
    将连续空间离散化为均匀体素，供路径规划算法查询。

    原理
    ----
    data 为 uint8 数组：0=自由空间, 1=占据, 2=膨胀区域。
    """

    origin: np.ndarray      # (3,) 世界坐标系原点
    resolution: float       # 体素边长 m
    shape: tuple[int, int, int]
    data: np.ndarray | None = None  # uint8 (nx, ny, nz)

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float)
        self.resolution = float(self.resolution)
        if self.data is None:
            self.data = np.zeros(self.shape, dtype=np.uint8)

    def world_to_index(self, p: np.ndarray) -> tuple[int, int, int]:
        p = np.asarray(p, dtype=float)
        idx = np.floor((p - self.origin) / self.resolution + 1e-9).astype(int)
        return tuple(np.clip(idx, 0, np.array(self.shape) - 1))

    def index_to_world(self, idx: tuple[int, int, int]) -> np.ndarray:
        return self.origin + np.array(idx, dtype=float) * self.resolution

    def is_occupied(self, idx: tuple[int, int, int]) -> bool:
        if any(i < 0 or i >= s for i, s in zip(idx, self.shape)):
            return True  # 越界视为占据
        return self.data[idx] >= 1

    def is_truly_occupied(self, idx: tuple[int, int, int]) -> bool:
        """仅判断体素是否被障碍物实体占据（值=1），不含膨胀层（值=2）。"""
        if any(i < 0 or i >= s for i, s in zip(idx, self.shape)):
            return True
        return self.data[idx] == 1

    def is_free(self, idx: tuple[int, int, int]) -> bool:
        return not self.is_occupied(idx)

    def inflate(self, radius: float) -> OccupancyGrid:
        """膨胀占据区域：将所有体素周围 radius 内的体素标记为 2（numpy 切片加速）。"""
        r_voxels = max(1, int(np.ceil(radius / self.resolution)))
        new_data = self.data.copy()
        occupied_idxs = np.argwhere(self.data >= 1)
        for oi in occupied_idxs:
            i0_min = max(0, oi[0] - r_voxels)
            i0_max = min(self.shape[0], oi[0] + r_voxels + 1)
            i1_min = max(0, oi[1] - r_voxels)
            i1_max = min(self.shape[1], oi[1] + r_voxels + 1)
            i2_min = max(0, oi[2] - r_voxels)
            i2_max = min(self.shape[2], oi[2] + r_voxels + 1)
            block = new_data[i0_min:i0_max, i1_min:i1_max, i2_min:i2_max]
            block[block == 0] = 2
        result = OccupancyGrid(origin=self.origin.copy(), resolution=self.resolution, shape=self.shape)
        result.data = new_data
        return result


class SDFAwareGrid:
    """SDF 感知占据栅格包装器。

    用途
    ----
    解决薄障碍物（半径 < resolution）在标准体素化中丢失的问题。
    `is_occupied` 在标准栅格判定为自由时，再用 SDF 精判：
    若 SDF(cell_center) < clearance 也视为占据。

    原理
    ----
    标准栅格做快速预筛（O(1) 数组查询），SDF 做精判（O(M)，M 为障碍物数）。
    LRU 缓存避免重复计算。代价：A* 节点扩展时多一次 SDF 评估。
    """

    def __init__(self, base_grid: OccupancyGrid, obstacle_field, clearance: float):
        self._base = base_grid
        self._obs = obstacle_field
        self._clearance = float(clearance)
        self._sdf_cache: dict[tuple, bool] = {}

        # 透传基本属性
        self.origin = base_grid.origin
        self.resolution = base_grid.resolution
        self.shape = base_grid.shape
        self.data = base_grid.data

    def world_to_index(self, p: np.ndarray) -> tuple[int, int, int]:
        return self._base.world_to_index(p)

    def index_to_world(self, idx: tuple[int, int, int]) -> np.ndarray:
        return self._base.index_to_world(idx)

    def is_occupied(self, idx: tuple[int, int, int]) -> bool:
        if any(i < 0 or i >= s for i, s in zip(idx, self.shape)):
            return True
        if self._base.data[idx] >= 1:
            return True
        # 精判：SDF < clearance
        cached = self._sdf_cache.get(idx)
        if cached is not None:
            return cached
        world = self._base.index_to_world(idx)
        occ = self._obs.signed_distance(world) < self._clearance
        # 限制缓存规模
        if len(self._sdf_cache) > 100000:
            self._sdf_cache.clear()
        self._sdf_cache[idx] = occ
        return occ

    def is_truly_occupied(self, idx: tuple[int, int, int]) -> bool:
        """仅判断真实障碍物占据（不含膨胀层），委托给底层 OccupancyGrid。"""
        return self._base.is_truly_occupied(idx)

    def is_free(self, idx) -> bool:
        return not self.is_occupied(idx)

    def inflate(self, radius: float) -> SDFAwareGrid:
        """对底层栅格做膨胀，clearance 不变（SDF 精判仍生效）。"""
        return SDFAwareGrid(self._base.inflate(radius), self._obs, self._clearance)
