"""障碍物顶点可见图（论文2: GNN分层架构）。

用途
----
为 GNN 规划器构建障碍物顶点的可见图：提取障碍物表面关键顶点，
判定顶点对之间的相互可见性（SDF 采样），生成稀疏图供 GNN 活动扩散。

原理
----
- 3D 障碍物顶点提取：AABB(8角点)、Cylinder(上下圆切线点)、Sphere(Fibonacci球面)
- 顶点沿障碍物外法向膨胀 buffer_zone，避免贴面路径
- 可见性判定：等距线段 SDF 采样，所有采样点 SDF > 0 则可见
- 邻接表存储，支持增量弹窗障碍物

复杂度：M 个顶点时，可见性判定 O(M²·n_samples)
"""

from __future__ import annotations

import numpy as np


def fibonacci_sphere(n: int = 12) -> np.ndarray:
    """Fibonacci 球面分布，返回 (n, 3) 单位球面上的点。"""
    points = np.zeros((n, 3), dtype=float)
    phi = np.pi * (3.0 - np.sqrt(5.0))
    for i in range(n):
        y = 1.0 - (i / float(n - 1)) * 2.0 if n > 1 else 0.0
        radius = np.sqrt(1.0 - y * y)
        theta = phi * i
        points[i, 0] = np.cos(theta) * radius
        points[i, 1] = y
        points[i, 2] = np.sin(theta) * radius
    return points


class VisibilityGraph:
    """障碍物顶点可见图。

    参数
    ----
    angular_res: 圆柱障碍物的角度分辨率（切线点数）。
    buffer_zone: 顶点外法向膨胀距离 m。
    n_sphere: 球体 Fibonacci 采样点数。
    """

    def __init__(
        self,
        angular_res: int = 8,
        buffer_zone: float = 0.3,
        n_sphere: int = 12,
    ):
        self.angular_res = int(angular_res)
        self.buffer_zone = float(buffer_zone)
        self.n_sphere = int(n_sphere)

        self.vertices: list[np.ndarray] = []
        self.adjacency: list[set[int]] = []
        self.vertex_to_obs: list[int] = []

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def build(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacle_field,
        visible_range: float = 20.0,
    ) -> None:
        """从障碍物场构建完整可见图（含起点/终点）。"""
        self._build_obstacle_graph(obstacle_field, visible_range)
        self._insert_start_goal(start, goal, obstacle_field, visible_range)

    def _build_obstacle_graph(self, obstacle_field, visible_range: float = 20.0) -> None:
        """仅构建障碍物顶点图（不含起点/终点），供后续复用。"""
        self.vertices = []
        self.adjacency = []
        self.vertex_to_obs = []

        obs_list = list(obstacle_field) if hasattr(obstacle_field, '__iter__') else getattr(obstacle_field, '_obstacles', [])
        for obs_idx, obs in enumerate(obs_list):
            obs_verts = self._extract_obstacle_vertices(obs)
            for v in obs_verts:
                self.vertices.append(v)
                self.vertex_to_obs.append(obs_idx)

        n = len(self.vertices)
        self.adjacency = [set() for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if self.is_visible(self.vertices[i], self.vertices[j], obstacle_field, visible_range):
                    self.adjacency[i].add(j)
                    self.adjacency[j].add(i)

    def set_start_goal(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacle_field,
        visible_range: float = 20.0,
    ) -> None:
        """替换起点/终点顶点并增量更新邻接（O(2N)，替代 O(N²) 全量重建）。"""
        start_arr = np.asarray(start, dtype=float)
        goal_arr = np.asarray(goal, dtype=float)

        # 移除旧的 start(0) 和 goal(1)
        if len(self.vertices) >= 2 and self.vertex_to_obs[0] == -1:
            self.vertices.pop(0)
            self.vertex_to_obs.pop(0)
            self.adjacency.pop(0)
            for adj_set in self.adjacency:
                adj_set.discard(0)
                # 降档所有索引
                new_adj = set()
                for x in adj_set:
                    new_adj.add(x - 1 if x > 0 else x)
                adj_set.clear()
                adj_set.update(new_adj)
        if len(self.vertices) >= 1 and len(self.vertex_to_obs) >= 1:
            if self.vertex_to_obs[0] == -2:
                self.vertices.pop(0)
                self.vertex_to_obs.pop(0)
                self.adjacency.pop(0)
                for adj_set in self.adjacency:
                    adj_set.discard(0)
                    new_adj = set()
                    for x in adj_set:
                        new_adj.add(x - 1)
                    adj_set.clear()
                    adj_set.update(new_adj)

        # 插入新的 start(0) 和 goal(1)
        self.vertices.insert(0, start_arr)
        self.vertex_to_obs.insert(0, -1)
        self.adjacency.insert(0, set())
        self.vertices.insert(1, goal_arr)
        self.vertex_to_obs.insert(1, -2)
        self.adjacency.insert(1, set())

        # 仅计算 start(0) 和 goal(1) 与其他顶点的可见性
        n = len(self.vertices)
        for i in (0, 1):
            for j in range(2, n):
                if self.is_visible(self.vertices[i], self.vertices[j], obstacle_field, visible_range):
                    self.adjacency[i].add(j)
                    self.adjacency[j].add(i)
        # start 与 goal 之间的可见性
        if self.is_visible(start_arr, goal_arr, obstacle_field, visible_range):
            self.adjacency[0].add(1)
            self.adjacency[1].add(0)

    def _insert_start_goal(self, start, goal, obstacle_field, visible_range):
        """内部：在已构建障碍物顶点图基础上插入起点/终点。"""
        start_arr = np.asarray(start, dtype=float)
        goal_arr = np.asarray(goal, dtype=float)
        self.vertices.insert(0, start_arr)
        self.vertex_to_obs.insert(0, -1)
        self.adjacency.insert(0, set())
        self.vertices.insert(1, goal_arr)
        self.vertex_to_obs.insert(1, -2)
        self.adjacency.insert(1, set())
        n = len(self.vertices)
        for i in (0, 1):
            for j in range(2, n):
                if self.is_visible(self.vertices[i], self.vertices[j], obstacle_field, visible_range):
                    self.adjacency[i].add(j)
                    self.adjacency[j].add(i)
        if self.is_visible(start_arr, goal_arr, obstacle_field, visible_range):
            self.adjacency[0].add(1)
            self.adjacency[1].add(0)

    def add_obstacle(self, obs, obstacle_field, visible_range: float = 20.0) -> list[int]:
        """增量添加障碍物，返回新顶点索引列表。"""
        obs_idx = len(set(self.vertex_to_obs))  # 粗糙估计
        obs_verts = self._extract_obstacle_vertices(obs)
        new_indices = []
        for v in obs_verts:
            new_indices.append(len(self.vertices))
            self.vertices.append(v)
            self.vertex_to_obs.append(obs_idx)
            self.adjacency.append(set())

        # 只为新顶点计算可见性
        for ni in new_indices:
            for j in range(len(self.vertices)):
                if j == ni:
                    continue
                if j not in new_indices or j > ni:
                    if self.is_visible(self.vertices[ni], self.vertices[j], obstacle_field, visible_range):
                        self.adjacency[ni].add(j)
                        self.adjacency[j].add(ni)
        return new_indices

    def is_visible(self, p1: np.ndarray, p2: np.ndarray, obstacle_field, visible_range: float = 20.0) -> bool:
        """判断两点间是否相互可见（SDF 线段采样无碰撞）。"""
        diff = p2 - p1
        dist = float(np.linalg.norm(diff))
        if dist < 1e-9 or dist > visible_range:
            return False
        n_samples = max(2, int(dist / 0.2))
        for k in range(n_samples + 1):
            t = k / n_samples
            pt = p1 + t * diff
            if obstacle_field.signed_distance(pt) < 0.0:
                return False
        return True

    # ------------------------------------------------------------------
    # 顶点提取
    # ------------------------------------------------------------------

    def _extract_obstacle_vertices(self, obs) -> list[np.ndarray]:
        """根据障碍物类型分发顶点提取。"""
        cls_name = type(obs).__name__
        if cls_name == 'Sphere':
            return self._sphere_vertices(obs)
        elif cls_name == 'Cylinder':
            return self._cylinder_vertices(obs)
        elif cls_name == 'AABB':
            return self._aabb_vertices(obs)
        else:
            # 回退：尝试属性检测
            if hasattr(obs, 'radius') and hasattr(obs, 'center_xy'):
                return self._cylinder_vertices(obs)
            elif hasattr(obs, 'radius'):
                return self._sphere_vertices(obs)
            else:
                return self._aabb_vertices(obs)

    def _aabb_vertices(self, obs) -> list[np.ndarray]:
        """AABB 的 8 个角点（由 min_corner/max_corner 计算中心+半尺寸）。"""
        mn = np.asarray(obs.min_corner, dtype=float)
        mx = np.asarray(obs.max_corner, dtype=float)
        c = (mn + mx) / 2.0
        h = (mx - mn) / 2.0 + self.buffer_zone
        verts = []
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    verts.append(c + np.array([dx * h[0], dy * h[1], dz * h[2]], dtype=float))
        return verts

    def _cylinder_vertices(self, obs) -> list[np.ndarray]:
        """圆柱体上下圆切线点。"""
        c_xy = np.asarray(obs.center_xy, dtype=float)
        r = float(obs.radius) + self.buffer_zone
        z_min, z_max = obs.z_range
        verts = []
        for z_offset in (z_min, z_max):
            for k in range(self.angular_res):
                angle = 2.0 * np.pi * k / self.angular_res
                verts.append(np.array([
                    c_xy[0] + r * np.cos(angle),
                    c_xy[1] + r * np.sin(angle),
                    z_offset,
                ], dtype=float))
        return verts

    def _sphere_vertices(self, obs) -> list[np.ndarray]:
        """球体 Fibonacci 分布点。"""
        c = np.asarray(obs.center, dtype=float)
        r = float(obs.radius) + self.buffer_zone
        sphere_pts = fibonacci_sphere(self.n_sphere)
        return [c + r * pt for pt in sphere_pts]

    @property
    def num_vertices(self) -> int:
        return len(self.vertices)
