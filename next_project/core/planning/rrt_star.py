"""RRT* 渐近最优路径规划。

用途
----
在高维/大范围地图中快速找到可行路径，并随采样数增加渐近逼近最优解。

原理
----
1) 随机采样 + KD-Tree 加速最近邻与邻域查询。
2) 收缩半径策略保证 RRT* 渐近最优性。
3) 对每个新节点在半径内寻找更优父节点 (choose_parent)。
4) 对邻近节点做 rewiring 降低全局代价。
5) 主循环后主动连接终点，并做 shortcut + B 样条路径平滑。

"""

from __future__ import annotations

from collections import defaultdict, deque

import numpy as np

from .base import Planner, PlannerError


def _propagate_cost_subtree(
    root: int,
    parent: dict[int, int],
    cost: dict[int, float],
    nodes: list[np.ndarray],
    max_subtree: int = 2000,
) -> None:
    """rewire 后 BFS 向下传播子树 cost，保证渐近最优性。

    max_subtree 防止极端情况下退化为 O(N) 的硬上限。
    """
    children: dict[int, list[int]] = defaultdict(list)
    for c, p in parent.items():
        if p >= 0:
            children[p].append(c)
    if root not in children:
        return
    queue: deque[int] = deque([root])
    visited = 0
    while queue and visited < max_subtree:
        u = queue.popleft()
        visited += 1
        for v in children.get(u, ()):
            new_c = cost[u] + float(np.linalg.norm(nodes[v] - nodes[u]))
            if abs(new_c - cost[v]) > 1e-9:
                cost[v] = new_c
                queue.append(v)

# ---------------------------------------------------------------------------
# 内部工具：增量式 KD-Tree
# ---------------------------------------------------------------------------

class _IncrementalKDTree:
    """增量式 KD-Tree，支持分批重建 + 缓冲区查询。

    用途
    ----
    替换 RRT* 中的暴力 O(N) 最近邻/邻域查询，将单次查询摊还至 O(log N)。

    原理
    ----
    scipy.spatial.cKDTree 不支持增量插入，因此维护一棵已建好的主树 +
    一个待插入缓冲列表。查询时同时在主树和缓冲区中搜索取最优/并集。
    当缓冲区超过阈值时全量重建主树。
    """

    def __init__(self, dim: int, rebuild_threshold_ratio: float = 0.3):
        self._dim = int(dim)
        self._points: list[np.ndarray] = []
        self._indices: list[int] = []            # 原始索引映射
        self._buf_points: list[np.ndarray] = []
        self._buf_indices: list[int] = []
        self._kdtree = None
        # rebuild 阈值为 max(50, sqrt(N))
        self._rebuild_threshold_ratio = float(rebuild_threshold_ratio)

    def __len__(self) -> int:
        return len(self._points)

    def insert(self, point: np.ndarray, idx: int) -> None:
        """插入一个点，先进缓冲。"""
        self._buf_points.append(np.asarray(point, dtype=float))
        self._buf_indices.append(int(idx))
        self._points.append(np.asarray(point, dtype=float))
        self._indices.append(int(idx))
        n = len(self._points)
        threshold = max(50, int(np.sqrt(n)))
        if len(self._buf_points) >= threshold:
            self._rebuild()

    def _rebuild(self) -> None:
        """全量重建主 KD-Tree，清空缓冲。"""
        if len(self._points) == 0:
            return
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            return
        pts = np.array(self._points, dtype=float)
        if pts.ndim != 2 or pts.shape[0] < 1:
            return
        self._kdtree = cKDTree(pts)
        self._buf_points.clear()
        self._buf_indices.clear()

    def _ensure_tree(self) -> None:
        """仅在首次查询时构建主树。"""
        if self._kdtree is None and len(self._points) > 0:
            self._rebuild()

    def nearest(self, point: np.ndarray) -> int:
        """返回最近邻的原始索引。主树 + 缓冲双路搜索。"""
        if len(self._points) == 0:
            return -1
        pt = np.asarray(point, dtype=float)
        best = -1
        best_d = float("inf")

        # 搜索主树
        if self._kdtree is not None and len(self._buf_points) < len(self._points):
            dist_arr, idx_arr = self._kdtree.query(pt.reshape(1, -1), k=1)
            dist_val = float(np.asarray(dist_arr).flat[0])
            if np.isfinite(dist_val):
                best_d = dist_val
                best = int(np.asarray(idx_arr).flat[0])

        # 搜索缓冲区
        for bi, bp in enumerate(self._buf_points):
            d = np.sum((bp - pt) ** 2)
            if d < best_d:
                best_d = d
                best = self._buf_indices[bi]

        return best

    def query_radius(self, point: np.ndarray, radius: float) -> list[int]:
        """返回指定半径内的所有原始索引。主树 + 缓冲双路搜索。"""
        if len(self._points) == 0:
            return []
        pt = np.asarray(point, dtype=float)
        result_set: set[int] = set()

        # 搜索主树
        if self._kdtree is not None and len(self._buf_points) < len(self._points):
            idx_list = self._kdtree.query_ball_point(pt, float(radius))
            result_set.update(int(i) for i in idx_list)

        # 搜索缓冲区
        r2 = radius * radius
        for bi, bp in enumerate(self._buf_points):
            if np.sum((bp - pt) ** 2) <= r2:
                result_set.add(self._buf_indices[bi])

        return list(result_set)


# ---------------------------------------------------------------------------
# RRT* 规划器
# ---------------------------------------------------------------------------

class RRTStar(Planner):
    """RRT* 渐近最优规划器。

    参数
    ----
    max_iter: 最大采样迭代次数。
    rewire_radius: 重连半径上界 η（与收缩半径策略配合）。
    goal_sample_rate: 目标偏置采样概率。
    gamma: 收缩半径系数，None 时自动估算。
    use_kdtree: 是否启用 KD-Tree 加速。
    smooth_method: 平滑方式 "none" | "shortcut" | "bspline"。
    smooth_points: B 样条重采样点数。
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
    ):
        self.max_iter = int(max_iter)
        self.rewire_radius = float(rewire_radius)
        self.goal_sample_rate = float(goal_sample_rate)
        self.gamma_val = gamma
        self.use_kdtree = bool(use_kdtree)
        self.smooth_method = smooth_method
        self.smooth_points = int(smooth_points)

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        bounds = np.array([grid.origin, grid.origin + np.array(grid.shape) * grid.resolution])

        start_idx = grid.world_to_index(start)
        goal_idx = grid.world_to_index(goal)

        if grid.is_occupied(start_idx):
            raise PlannerError("起点在障碍物内")
        if grid.is_occupied(goal_idx):
            raise PlannerError("终点在障碍物内")

        nodes: list[np.ndarray] = [start]
        parent: dict[int, int] = {0: -1}
        cost: dict[int, float] = {0: 0.0}
        goal_node = -1

        rng = np.random.RandomState(kw.get("seed", 42))

        # 初始化 KD-Tree
        kdtree: _IncrementalKDTree | None = None
        if self.use_kdtree:
            kdtree = _IncrementalKDTree(dim=3)
            kdtree.insert(start, 0)

        # 估算 gamma（基于地图包围盒对角线）
        gamma = self.gamma_val
        if gamma is None:
            diag = float(np.linalg.norm(bounds[1] - bounds[0]))
            gamma = diag * 2.0   # 启发式：包围盒对角线的 2 倍

        for iteration in range(self.max_iter):
            # 计算收缩半径 (RRT* 理论保证)
            n = len(nodes)
            r_n = min(gamma * (np.log(n + 1) / (n + 1)) ** (1.0 / 3.0), self.rewire_radius)

            # 有偏采样
            if rng.random() < self.goal_sample_rate:
                sample = goal.copy()
            else:
                sample = self._random_sample(bounds, rng)

            # 最近邻
            if kdtree is not None:
                nearest_idx = kdtree.nearest(sample)
            else:
                nearest_idx = self._nearest_brute(nodes, sample)
            if nearest_idx < 0:
                continue

            # 转向 (steer)
            new_node = self._steer(nodes[nearest_idx], sample, grid.resolution * 2.0)
            new_idx_cell = grid.world_to_index(new_node)
            if grid.is_occupied(new_idx_cell):
                continue

            # 碰撞检查
            if not self._collision_free(nodes[nearest_idx], new_node, grid):
                continue

            # ---- 选择最优父节点 (choose_parent) ----
            best_parent = nearest_idx
            best_cost = cost[nearest_idx] + np.linalg.norm(new_node - nodes[nearest_idx])

            if kdtree is not None:
                cands = kdtree.query_radius(new_node, r_n)
            else:
                cands = self._radius_search_brute(nodes, new_node, r_n)

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

            # ---- Rewire 邻近节点 ----
            if kdtree is not None:
                rewire_cands = kdtree.query_radius(new_node, r_n)
            else:
                rewire_cands = self._radius_search_brute(nodes, new_node, r_n)

            for ri in rewire_cands:
                if ri == new_idx_local:
                    continue
                d = np.linalg.norm(new_node - nodes[ri])
                if cost[new_idx_local] + d < cost[ri] and self._collision_free(new_node, nodes[ri], grid):
                    cost[ri] = cost[new_idx_local] + d
                    parent[ri] = new_idx_local
                    # KD-Tree 中索引不变（点没变，只是父节点/代价变了）
                    # 递归更新子树代价以保证渐近最优性
                    _propagate_cost_subtree(ri, parent, cost, nodes)

            # 检查是否到达目标
            if np.linalg.norm(new_node - goal) < grid.resolution * 2.0:
                goal_node = new_idx_local

        # ---- 主动终点连接 ----
        if goal_node < 0:
            goal_node = self._active_goal_connect(nodes, parent, cost, goal, grid, kdtree)

        if goal_node < 0:
            raise PlannerError("RRT* 未找到可行路径")

        # ---- 提取并平滑路径 ----
        raw_path = self._extract_path(nodes, parent, goal_node)

        if self.smooth_method == "shortcut":
            path = self._shortcut_smooth(raw_path, grid)
        elif self.smooth_method == "bspline":
            path = self._bspline_smooth(raw_path, grid)
        else:
            path = raw_path

        return path

    # ------------------------------------------------------------------
    # 采样与几何
    # ------------------------------------------------------------------

    @staticmethod
    def _random_sample(bounds: np.ndarray, rng: np.random.RandomState) -> np.ndarray:
        return np.array([
            rng.uniform(bounds[0, i], bounds[1, i]) for i in range(3)
        ], dtype=float)

    @staticmethod
    def _steer(from_node: np.ndarray, to_node: np.ndarray, step: float) -> np.ndarray:
        vec = to_node - from_node
        dist = np.linalg.norm(vec)
        if dist <= step:
            return to_node.copy()
        return from_node + vec / dist * step

    # ------------------------------------------------------------------
    # 碰撞检查
    # ------------------------------------------------------------------

    @staticmethod
    def _collision_free(a: np.ndarray, b: np.ndarray, grid) -> bool:
        """直线段步进取样碰撞检查。"""
        dist = np.linalg.norm(b - a)
        steps = max(2, int(dist / (grid.resolution * 0.5)))
        for s in range(steps + 1):
            t = s / steps
            pt = a + (b - a) * t
            idx = grid.world_to_index(pt)
            if grid.is_occupied(idx):
                return False
        return True

    # ------------------------------------------------------------------
    # 暴力搜索备选（KD-Tree 关闭时使用）
    # ------------------------------------------------------------------

    @staticmethod
    def _nearest_brute(nodes: list[np.ndarray], sample: np.ndarray) -> int:
        best = -1
        best_d = float("inf")
        for i, node in enumerate(nodes):
            d = np.sum((node - sample) ** 2)
            if d < best_d:
                best_d = d
                best = i
        return best

    @staticmethod
    def _radius_search_brute(nodes: list[np.ndarray], center: np.ndarray, radius: float) -> list[int]:
        result = []
        r2 = radius * radius
        for i, node in enumerate(nodes):
            if np.sum((node - center) ** 2) <= r2:
                result.append(i)
        return result

    # ------------------------------------------------------------------
    # 主动终点连接
    # ------------------------------------------------------------------

    def _active_goal_connect(
        self,
        nodes: list[np.ndarray],
        parent: dict[int, int],
        cost: dict[int, float],
        goal: np.ndarray,
        grid,
        kdtree: _IncrementalKDTree | None,
    ) -> int:
        """主循环结束后显式尝试连接终点。

        在 goal 周围搜索候选节点，按代价+距离排序，
        取首个无碰撞的直线连接。
        """
        search_radius = self.rewire_radius * 2.0

        if kdtree is not None:
            cands = kdtree.query_radius(goal, search_radius)
        else:
            cands = self._radius_search_brute(nodes, goal, search_radius)

        if not cands:
            # 无候选则退回最近邻
            if kdtree is not None:
                nearest_idx = kdtree.nearest(goal)
            else:
                nearest_idx = self._nearest_brute(nodes, goal)
            if nearest_idx >= 0 and self._collision_free(nodes[nearest_idx], goal, grid):
                goal_idx = len(nodes)
                nodes.append(goal)
                parent[goal_idx] = nearest_idx
                cost[goal_idx] = cost[nearest_idx] + np.linalg.norm(goal - nodes[nearest_idx])
                return goal_idx
            return -1

        # 按 cost + distance 排序
        cands.sort(key=lambda i: cost[i] + np.linalg.norm(goal - nodes[i]))
        for cand in cands:
            if self._collision_free(nodes[cand], goal, grid):
                goal_idx = len(nodes)
                nodes.append(goal)
                parent[goal_idx] = cand
                cost[goal_idx] = cost[cand] + np.linalg.norm(goal - nodes[cand])
                return goal_idx

        return -1

    # ------------------------------------------------------------------
    # 路径提取与平滑
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_path(nodes: list, parent: dict, goal_idx: int) -> np.ndarray:
        path = []
        node = goal_idx
        while node >= 0:
            path.append(nodes[node])
            node = parent.get(node, -1)
        path.reverse()
        return np.array(path, dtype=float)

    def _shortcut_smooth(self, path: np.ndarray, grid) -> np.ndarray:
        """随机捷径裁剪：反复尝试用直线段替换中间航点。"""
        path = np.asarray(path, dtype=float)
        if len(path) < 3:
            return path

        K = len(path)
        rng = np.random.RandomState(42)
        max_attempts = K * 50

        for _ in range(max_attempts):
            i = rng.randint(0, K - 2)
            j = rng.randint(i + 2, K)
            if self._collision_free(path[i], path[j], grid):
                # 删除中间点
                path = np.concatenate([path[:i + 1], path[j:]], axis=0)
                K = len(path)
                if K < 3:
                    break

        return path

    def _bspline_smooth(self, path: np.ndarray, grid) -> np.ndarray:
        """B 样条拟合后按弧长重采样。

        先做 shortcut 消除大锯齿，再 B 样条拟合，
        最后碰撞检查确保平滑不穿过障碍物。
        """
        from scipy import interpolate

        path = np.asarray(path, dtype=float)
        if len(path) < 4:
            return path

        # 先 shortcut 减少控制点
        path = self._shortcut_smooth(path, grid)
        if len(path) < 4:
            return path

        K = len(path)
        n_points = min(self.smooth_points, K * 10)

        try:
            # B 样条拟合 (s=0 为插值，s>0 为近似)
            tck, _ = interpolate.splprep(
                [path[:, 0], path[:, 1], path[:, 2]],
                s=0.5,   # 平滑因子
                k=min(3, K - 1),
            )
            u_new = np.linspace(0, 1, n_points)
            sx, sy, sz = interpolate.splev(u_new, tck)
            smoothed = np.column_stack([sx, sy, sz])
        except Exception:
            return path

        # 碰撞检查：移除穿过障碍物的点
        safe = [smoothed[0]]
        for i in range(1, len(smoothed)):
            if self._collision_free(safe[-1], smoothed[i], grid):
                safe.append(smoothed[i])
        safe.append(smoothed[-1])

        return np.array(safe, dtype=float)
