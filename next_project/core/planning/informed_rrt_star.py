"""Informed RRT* 椭球约束采样规划器。

用途
----
在 RRT* 找到初始可行解后，将采样限制在以起终点为焦点的超椭球内，
显著提升收敛速度与路径质量。

原理
----
1) 首轮全局采样找到初始解，记录代价 c_best。
2) 后续采样限制在 {x: ||x - start|| + ||x - goal|| <= c_best} 椭球内。即以起点和终点为焦点，c_best/2 为半长轴的椭球。
找到第一个最优解后立刻收紧椭球
3) 每发现更优解即收紧椭球，渐进逼近最优。

"""

from __future__ import annotations

import numpy as np

from .rrt_star import RRTStar


class InformedRRTStar(RRTStar):
    """Informed RRT* — 椭球约束采样，加速收敛。

    参数继承自 RRTStar，额外参数：
    - informed_iter_start: 开始椭球采样的迭代轮次（前 N 轮全空间探索）。
    """

    def __init__(
        self,
        max_iter: int = 4000,
        rewire_radius: float = 1.5,
        goal_sample_rate: float = 0.1,
        gamma: float | None = None,
        use_kdtree: bool = True,
        smooth_method: str = "bspline",
        smooth_points: int = 200,
        informed_iter_start: int = 200,
    ):
        super().__init__(
            max_iter=max_iter,
            rewire_radius=rewire_radius,
            goal_sample_rate=goal_sample_rate,
            gamma=gamma,
            use_kdtree=use_kdtree,
            smooth_method=smooth_method,
            smooth_points=smooth_points,
        )
        self.informed_iter_start = int(informed_iter_start)
        # 椭球参数（运行时计算）
        self._c_best: float = float("inf")
        self._ell_start: np.ndarray | None = None
        self._ell_goal: np.ndarray | None = None
        self._ell_c: np.ndarray | None = None  # 中心
        self._ell_a1: np.ndarray | None = None # 主轴单位向量
        self._ell_a: float = 0.0               # 半长轴
        self._ell_b: float = 0.0               # 半短轴

    # ------------------------------------------------------------------
    # 重写 plan
    # ------------------------------------------------------------------

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        """Informed RRT* 规划，覆盖父类以追踪 c_best。"""
        from .base import PlannerError

        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        bounds = np.array([
            grid.origin,
            grid.origin + np.array(grid.shape) * grid.resolution,
        ])

        start_idx = grid.world_to_index(start)
        goal_idx = grid.world_to_index(goal)
        if grid.is_occupied(start_idx):
            raise PlannerError("起点在障碍物内")
        if grid.is_occupied(goal_idx):
            raise PlannerError("终点在障碍物内")

        if np.linalg.norm(goal - start) < 1e-6:
            return np.array([start, goal], dtype=float)

        nodes: list[np.ndarray] = [start]
        parent: dict[int, int] = {0: -1}
        cost: dict[int, float] = {0: 0.0}
        goal_node = -1
        self._c_best = float("inf")

        rng = np.random.RandomState(kw.get("seed", 42))

        # KD-Tree
        from .rrt_star import _IncrementalKDTree
        kdtree = _IncrementalKDTree(dim=3) if self.use_kdtree else None
        if kdtree is not None:
            kdtree.insert(start, 0)

        # gamma
        gamma = self.gamma_val
        if gamma is None:
            diag = float(np.linalg.norm(bounds[1] - bounds[0]))
            gamma = diag * 2.0

        # 预存空间边界球形范围用于全空间采样
        space_center = (bounds[0] + bounds[1]) / 2.0
        space_radius = float(np.linalg.norm(bounds[1] - bounds[0])) / 2.0

        for iteration in range(self.max_iter):
            n = len(nodes)
            r_n = min(
                gamma * (np.log(n + 1) / (n + 1)) ** (1.0 / 3.0),
                self.rewire_radius,
            )

            # ---- 采样策略分层 ----
            if iteration < self.informed_iter_start or not np.isfinite(self._c_best):
                # 阶段 1：全空间 + 目标偏置采样
                if rng.random() < self.goal_sample_rate:
                    sample = goal.copy()
                else:
                    sample = self._random_sample(bounds, rng)
            else:
                # 阶段 2：椭球约束采样
                if rng.random() < self.goal_sample_rate:
                    sample = goal.copy()
                elif rng.random() < 0.1:
                    # 10% 概率全空间采样（防止局部最优）
                    sample = self._random_sample(bounds, rng)
                else:
                    sample = self._informed_sample(rng, space_center, space_radius)

            # 最近邻
            if kdtree is not None:
                nearest_idx = kdtree.nearest(sample)
            else:
                nearest_idx = self._nearest_brute(nodes, sample)
            if nearest_idx < 0:
                continue

            new_node = self._steer(nodes[nearest_idx], sample, grid.resolution * 2.0)
            new_idx_cell = grid.world_to_index(new_node)
            if grid.is_occupied(new_idx_cell):
                continue
            if not self._collision_free(nodes[nearest_idx], new_node, grid):
                continue

            # choose_parent
            best_parent = nearest_idx
            best_cost = cost[nearest_idx] + np.linalg.norm(new_node - nodes[nearest_idx])
            cands = (
                kdtree.query_radius(new_node, r_n)
                if kdtree is not None
                else self._radius_search_brute(nodes, new_node, r_n)
            )
            for ci in cands:
                if ci == nearest_idx:
                    continue
                d = np.linalg.norm(new_node - nodes[ci])
                candidate_cost = cost[ci] + d
                if candidate_cost < best_cost and self._collision_free(nodes[ci], new_node, grid):
                    best_cost = candidate_cost
                    best_parent = ci

            new_idx_local = len(nodes)
            nodes.append(new_node)
            parent[new_idx_local] = best_parent
            cost[new_idx_local] = best_cost

            if kdtree is not None:
                kdtree.insert(new_node, new_idx_local)

            # rewire
            rewire_cands = (
                kdtree.query_radius(new_node, r_n)
                if kdtree is not None
                else self._radius_search_brute(nodes, new_node, r_n)
            )
            for ri in rewire_cands:
                if ri == new_idx_local:
                    continue
                d = np.linalg.norm(new_node - nodes[ri])
                if cost[new_idx_local] + d < cost[ri] and self._collision_free(new_node, nodes[ri], grid):
                    cost[ri] = cost[new_idx_local] + d
                    parent[ri] = new_idx_local
                    # 递归传播子树代价
                    from .rrt_star import _propagate_cost_subtree
                    _propagate_cost_subtree(ri, parent, cost, nodes)

            # 检查是否到达目标
            if np.linalg.norm(new_node - goal) < grid.resolution * 2.0:
                goal_node = new_idx_local
                # 更新 c_best
                path_cost = cost[goal_node] + np.linalg.norm(nodes[goal_node] - goal)
                if path_cost < self._c_best:
                    self._c_best = path_cost
                    # 更新椭球参数
                    self._update_ellipse(start, goal)

        # 主动终点连接
        if goal_node < 0:
            goal_node = self._active_goal_connect(nodes, parent, cost, goal, grid, kdtree)
        if goal_node >= 0:
            path_cost = cost[goal_node] + np.linalg.norm(nodes[goal_node] - goal)
            if path_cost < self._c_best:
                self._c_best = path_cost

        if goal_node < 0:
            raise PlannerError("Informed RRT* 未找到可行路径")

        raw_path = self._extract_path(nodes, parent, goal_node)

        if self.smooth_method == "shortcut":
            return self._shortcut_smooth(raw_path, grid)
        elif self.smooth_method == "bspline":
            return self._bspline_smooth(raw_path, grid)
        return raw_path

    # ------------------------------------------------------------------
    # 椭球采样
    # ------------------------------------------------------------------

    def _update_ellipse(self, start: np.ndarray, goal: np.ndarray) -> None:
        """根据当前 c_best 更新椭球参数。"""
        self._ell_start = start.copy()
        self._ell_goal = goal.copy()
        self._ell_c = (start + goal) / 2.0
        c_min = np.linalg.norm(goal - start)  # 两点间直线距离
        if self._c_best <= c_min + 1e-9:
            # c_best 很小时，椭球退化为线段
            self._ell_a = c_min / 2.0 + 1e-3
        else:
            self._ell_a = self._c_best / 2.0
        f = c_min / 2.0  # 半焦距
        self._ell_b = max(np.sqrt(max(self._ell_a**2 - f**2, 0.0)), 1e-3)
        # 主轴方向
        a1_raw = goal - start
        a1_norm = np.linalg.norm(a1_raw)
        self._ell_a1 = a1_raw / a1_norm if a1_norm > 1e-12 else np.array([1.0, 0.0, 0.0])

    def _informed_sample(self, rng: np.random.RandomState, space_center, space_radius) -> np.ndarray:
        """在椭球内采样：球变换法。

        先构造任意正交基 {e1, e2, e3} 其中 e1 = 主轴方向，
        在标准椭球内均匀采样后变换回世界坐标。

        球面方向均匀使用 cos(theta) ~ U(-1,1)，避免两极采样过密。
        """
        c = self._ell_c
        a1 = self._ell_a1
        a = self._ell_a
        b = self._ell_b

        # 构造正交基
        if abs(a1[0]) < 0.9:
            v = np.array([1.0, 0.0, 0.0])
        else:
            v = np.array([0.0, 1.0, 0.0])
        e2 = v - np.dot(v, a1) * a1
        e2 /= np.linalg.norm(e2)
        e3 = np.cross(a1, e2)

        R = np.column_stack([a1, e2, e3])  # (3,3)

        # 单位球内均匀采样：r^3 均匀 + cos(theta) ~ U(-1,1)
        for _attempt in range(20):
            r = rng.random() ** (1.0 / 3.0)
            u_z = rng.uniform(-1.0, 1.0)             # cos(theta) 均匀
            phi = rng.uniform(0.0, 2.0 * np.pi)
            sin_t = float(np.sqrt(max(0.0, 1.0 - u_z * u_z)))
            x = r * sin_t * np.cos(phi)
            y = r * sin_t * np.sin(phi)
            z = r * u_z

            x_ell = np.array([a * x, b * y, b * z])
            pt = c + R @ x_ell

            # 数值精度兜底：椭球边界外偏差极小则接受
            if np.linalg.norm(pt - self._ell_start) + np.linalg.norm(pt - self._ell_goal) <= self._c_best * 1.001:
                return pt

        # 兜底：退回全空间均匀采样，避免死循环
        bounds_fallback = np.array([
            c - np.array([space_radius, space_radius, space_radius]),
            c + np.array([space_radius, space_radius, space_radius]),
        ])
        return self._random_sample(bounds_fallback, rng)
