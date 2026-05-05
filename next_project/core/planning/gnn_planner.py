"""GNN 路径规划器（论文2: 可见图变体，vertex-only diffusion）。

用途
----
在可见图顶点上运行 GNN 活动扩散，提取最小活动值路径。
这是论文2 原版"栅格神经元 GNN"的工程变体：将稠密活动场压缩到稀疏可见图节点。

原理
----
GNN 核心公式（论文2 式 19-20）：
    dx_i/dt = -A·x_i + (B - x_i)·[I_i + Σ_j(w_ij·x_j)] - (D + x_i)·J_i

其中 w_ij = 1/(1 + d_ij)^γ，I_i 为目标/路径激励，J_i 为抑制（本项目 J_i=0）。

路径提取：从起点沿最大活动值邻居贪心移动至目标。

验收基线
--------
- 路径长度 ≤ 1.05×最优（与 A* 对比）
- 活动场从目标向外单调递减（test_gnn_steady_monotone.py）
"""

from __future__ import annotations

import numpy as np

from .base import Planner


class GNNPlanner(Planner):
    """GNN 可见图变体路径规划器。

    参数
    ----
    A: 被动衰减率（默认 10）。
    B: 活动上界（默认 1）。
    D: 活动下界（默认 1）。
    gamma: 权重衰减指数（默认 2）。
    alpha: 目标距离激励衰减指数（默认 2）。
    beta: 路径距离激励衰减指数（默认 2）。
    V: 激励缩放系数（默认 100）。
    E: 目标神经元固定高激励（默认 50）。
    dt: Euler 积分步长 s。
    max_steps: 最大积分步数。
    tol: 稳态收敛阈值。
    """

    def __init__(
        self,
        A: float = 10.0,
        B: float = 1.0,
        D: float = 1.0,
        gamma: float = 2.0,
        alpha: float = 2.0,
        beta: float = 2.0,
        V: float = 100.0,
        E: float = 50.0,
        dt: float = 0.01,
        max_steps: int = 2000,
        tol: float = 1e-4,
    ):
        self.A = float(A)
        self.B = float(B)
        self.D = float(D)
        self.gamma = float(gamma)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.V = float(V)
        self.E = float(E)
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.tol = float(tol)

        self._vis_graph = None
        self._activity: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Planner 接口
    # ------------------------------------------------------------------

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        """在可见图上运行 GNN 并提取路径。

        VisibilityGraph 实例通过 kw['visibility_graph'] 传入。
        """
        vis_graph = kw.get('visibility_graph')
        if vis_graph is None:
            from .visibility_graph import VisibilityGraph
            obstacle_field = kw.get('obstacle_field')
            if obstacle_field is None:
                raise ValueError("GNNPlanner.plan() 需要 visibility_graph 或 obstacle_field")
            vis_graph = VisibilityGraph()
            visible_range = kw.get('visible_range', 20.0)
            vis_graph.build(start, goal, obstacle_field, visible_range)

        self._vis_graph = vis_graph
        n = vis_graph.num_vertices

        # 计算连接权重矩阵
        W = self._compute_weight_matrix(vis_graph)

        # 计算激励输入
        I_ext = self._compute_excitation(vis_graph.vertices, start, goal)

        # GNN 活动积分
        x = self._integrate_gnn(W, I_ext)

        # 提取路径
        path = self._extract_path_from_activities(x, vis_graph, start, goal)

        return self.smooth(path)

    # ------------------------------------------------------------------
    # GNN 核心
    # ------------------------------------------------------------------

    def _compute_weight_matrix(self, vis_graph) -> np.ndarray:
        """计算连接权重矩阵 w_ij = 1/(1 + d_ij)^γ。"""
        n = vis_graph.num_vertices
        W = np.zeros((n, n), dtype=float)
        verts = vis_graph.vertices
        for i in range(n):
            for j in vis_graph.adjacency[i]:
                if i >= j:
                    continue
                d_ij = float(np.linalg.norm(verts[i] - verts[j]))
                w = 1.0 / ((1.0 + d_ij) ** self.gamma)
                W[i, j] = w
                W[j, i] = w
        return W

    def _compute_excitation(
        self, vertices: list[np.ndarray], start: np.ndarray, goal: np.ndarray
    ) -> np.ndarray:
        """计算外部激励 I_i。

        I_i = V · 1/(1+d_goal_i)^α · 1/(1+d_path_i)^β
        """
        n = len(vertices)
        I = np.zeros(n, dtype=float)
        for i in range(n):
            d_goal = float(np.linalg.norm(vertices[i] - goal))
            d_path = float(np.linalg.norm(vertices[i] - start))
            I[i] = self.V / ((1.0 + d_goal) ** self.alpha) / ((1.0 + d_path) ** self.beta)
        # 目标顶点固定高激励
        I[1] = self.E
        return I

    def _integrate_gnn(self, W: np.ndarray, I_ext: np.ndarray) -> np.ndarray:
        """Euler 积分 GNN 动力学至稳态。

        dx_i/dt = -A·x_i + (B - x_i)·(I_i + Σ_j w_ij·x_j) - (D + x_i)·J_i
        """
        n = len(I_ext)
        x = np.zeros(n, dtype=float)
        for step in range(self.max_steps):
            lateral = W @ x
            dx = (
                -self.A * x
                + (self.B - x) * (I_ext + lateral)
                - (self.D + x) * 0.0  # J_i = 0
            )
            x_new = x + self.dt * dx
            x_new = np.clip(x_new, 0.0, self.B)
            if np.max(np.abs(x_new - x)) < self.tol:
                x = x_new
                break
            x = x_new
        self._activity = x
        return x

    def _extract_path_from_activities(
        self, x: np.ndarray, vis_graph, start: np.ndarray, goal: np.ndarray
    ) -> np.ndarray:
        """从活动场贪心提取路径：从起点沿最大活动值邻居移动至目标。"""
        verts = vis_graph.vertices
        adj = vis_graph.adjacency
        n = len(verts)

        # 找到最近的起点/终点顶点
        start_idx = 0
        goal_idx = 1

        # 贪心追踪。路径只允许沿可见图边移动，避免生成未校验直连线段。
        path = [verts[start_idx].copy()]
        current = start_idx
        visited = {current}
        max_iter = n * 2

        for _ in range(max_iter):
            if current == goal_idx:
                break
            # 在邻居中找活动值最大的
            best_idx = -1
            best_activity = -1.0
            for nb in adj[current]:
                if nb in visited:
                    continue
                if nb != goal_idx and not self._can_reach_goal(nb, goal_idx, adj, visited | {current}):
                    continue
                if x[nb] > best_activity:
                    best_activity = x[nb]
                    best_idx = nb
            if best_idx < 0:
                suffix = self._shortest_unvisited_suffix(current, goal_idx, adj, visited)
                if suffix is None:
                    break
                for idx in suffix:
                    path.append(verts[idx].copy())
                current = goal_idx
                break
            visited.add(best_idx)
            current = best_idx
            path.append(verts[current].copy())

        if current != goal_idx:
            if len(path) == 1 or np.linalg.norm(path[-1] - goal) > 1e-9:
                path.append(goal.copy())
        return np.array(path, dtype=float)

    @staticmethod
    def _shortest_unvisited_suffix(
        start_idx: int,
        goal_idx: int,
        adjacency: list[set[int]],
        visited: set[int],
    ) -> list[int] | None:
        """Return a BFS suffix to the goal using graph edges only."""
        queue: list[int] = [start_idx]
        parent: dict[int, int | None] = {start_idx: None}
        blocked = set(visited)
        blocked.discard(start_idx)

        while queue:
            node = queue.pop(0)
            if node == goal_idx:
                break
            for nb in adjacency[node]:
                if nb in blocked or nb in parent:
                    continue
                parent[nb] = node
                queue.append(nb)

        if goal_idx not in parent:
            return None

        suffix: list[int] = []
        node = goal_idx
        while node != start_idx:
            suffix.append(node)
            prev = parent[node]
            if prev is None:
                return None
            node = prev
        suffix.reverse()
        return suffix

    @staticmethod
    def _can_reach_goal(
        start_idx: int,
        goal_idx: int,
        adjacency: list[set[int]],
        visited: set[int],
    ) -> bool:
        """Return whether the goal is reachable without revisiting blocked nodes."""
        if start_idx == goal_idx:
            return True
        blocked = set(visited)
        blocked.discard(start_idx)
        queue = [start_idx]
        seen = {start_idx}
        while queue:
            node = queue.pop(0)
            for nb in adjacency[node]:
                if nb == goal_idx:
                    return True
                if nb in blocked or nb in seen:
                    continue
                seen.add(nb)
                queue.append(nb)
        return False

    @property
    def activity(self) -> np.ndarray | None:
        """最近一次积分的稳态活动场。"""
        return self._activity
