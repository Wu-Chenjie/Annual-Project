"""机载距离探测器。

用途
----
仿真六向（±x, ±y, ±z）距离传感器，为在线路径规划提供局部障碍物感知。

原理
----
采用射线-AABB 求交简化计算：对每个方向发射射线，对 ObstacleField 中
所有障碍物求最近交点。无障碍方向返回 max_range，附加高斯噪声模拟真实传感器。

"""

from __future__ import annotations

import numpy as np


class RangeSensor6:
    """六向测距传感器。

    用途
    ----
    探测 ±x, ±y, ±z 六个方向到最近障碍物的距离，为 WindowReplanner 提供局部感知。

    原理
    ----
    射线方程 p = pose[:3] + t * dir，t in [0, max_range]。
    对每个障碍物求交，取全体最小正 t。
    """

    DIRECTIONS = np.array([
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ], dtype=float)

    def __init__(self, max_range: float = 8.0, noise_std: float = 0.02, seed: int = 0):
        self.max_range = float(max_range)
        self.noise_std = float(noise_std)
        self._rng = np.random.RandomState(seed)

    def sense(self, pose: np.ndarray, field) -> np.ndarray:
        """从当前位姿探测六向距离。

        pose: (6,) 或 (3,) 向量 [x, y, z, roll, pitch, yaw] 或仅位置。
        field: ObstacleField 实例。
        """
        origin = np.asarray(pose[:3], dtype=float)
        distances = np.full(6, self.max_range, dtype=float)

        for di, direction in enumerate(self.DIRECTIONS):
            t = self._ray_cast(origin, direction, field)
            distances[di] = min(t, self.max_range)

        # 附加高斯噪声
        noise = self._rng.randn(6) * self.noise_std
        distances = np.clip(distances + noise, 0.0, self.max_range)
        return distances

    def _ray_cast(self, origin: np.ndarray, direction: np.ndarray, field) -> float:
        """解析求交：对每个障碍物直接计算射线交点，返回最近碰撞距离。"""
        t_min = self.max_range
        for obs in field._obstacles:
            t = self._ray_obs_intersect(origin, direction, obs)
            if 0.0 < t < t_min:
                t_min = t
        return t_min

    @staticmethod
    def _ray_obs_intersect(origin: np.ndarray, direction: np.ndarray, obs) -> float:
        """单射线-单障碍物解析求交，返回正 t 或 +inf。"""
        from core.obstacles import AABB, Sphere, Cylinder
        if isinstance(obs, AABB):
            return RangeSensor6._ray_aabb(origin, direction, obs.min_corner, obs.max_corner)
        if isinstance(obs, Sphere):
            return RangeSensor6._ray_sphere(origin, direction, obs.center, obs.radius)
        if isinstance(obs, Cylinder):
            return RangeSensor6._ray_cylinder(origin, direction, obs.center_xy, obs.radius, obs.z_range)
        return float("inf")

    @staticmethod
    def _ray_aabb(origin: np.ndarray, direction: np.ndarray, bmin: np.ndarray, bmax: np.ndarray) -> float:
        """slab 方法求射线与 AABB 交点。"""
        t_enter = -float("inf")
        t_exit = float("inf")
        for d in range(3):
            if abs(direction[d]) < 1e-12:
                if origin[d] < bmin[d] or origin[d] > bmax[d]:
                    return float("inf")
            else:
                inv = 1.0 / direction[d]
                t1 = (bmin[d] - origin[d]) * inv
                t2 = (bmax[d] - origin[d]) * inv
                if t1 > t2:
                    t1, t2 = t2, t1
                t_enter = max(t_enter, t1)
                t_exit = min(t_exit, t2)
        if t_enter <= t_exit and t_exit >= 0:
            return t_enter if t_enter >= 0 else t_exit
        return float("inf")

    @staticmethod
    def _ray_sphere(origin: np.ndarray, direction: np.ndarray, center: np.ndarray, radius: float) -> float:
        oc = origin - center
        a = float(np.dot(direction, direction))
        b = 2.0 * float(np.dot(oc, direction))
        c = float(np.dot(oc, oc)) - radius * radius
        disc = b * b - 4 * a * c
        if disc < 0:
            return float("inf")
        sqrt_disc = np.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        if t1 >= 0:
            return t1
        if t2 >= 0:
            return t2
        return float("inf")

    @staticmethod
    def _ray_cylinder(origin: np.ndarray, direction: np.ndarray, center_xy: np.ndarray, radius: float, z_range: tuple) -> float:
        """射线与立式圆柱求交（XY 平面圆 + Z 范围）。"""
        cx, cy = center_xy
        z_min, z_max = z_range
        # XY 平面圆求交
        a = direction[0]**2 + direction[1]**2
        b = 2.0 * (direction[0] * (origin[0] - cx) + direction[1] * (origin[1] - cy))
        c = (origin[0] - cx)**2 + (origin[1] - cy)**2 - radius**2
        if a < 1e-12:
            return float("inf")
        disc = b * b - 4 * a * c
        if disc < 0:
            return float("inf")
        sqrt_disc = np.sqrt(disc)
        t_candidates = []
        for t in [(-b - sqrt_disc) / (2 * a), (-b + sqrt_disc) / (2 * a)]:
            if t >= 0:
                z = origin[2] + t * direction[2]
                if z_min <= z <= z_max:
                    t_candidates.append(t)
        # 检查与圆柱顶面和底面的交点
        for z_plane in (z_min, z_max):
            if abs(direction[2]) > 1e-12:
                t = (z_plane - origin[2]) / direction[2]
                if t >= 0:
                    px = origin[0] + t * direction[0]
                    py = origin[1] + t * direction[1]
                    if (px - cx)**2 + (py - cy)**2 <= radius**2:
                        t_candidates.append(t)
        return min(t_candidates) if t_candidates else float("inf")
