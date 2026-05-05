"""D* Lite 增量式重规划。

用途
----
在环境部分已知或动态变化时，仅更新受影响节点的代价，
避免从零重建搜索树。适用于滑动窗口在线重规划的退化层。

原理
----
基于 LPA* 的增量式图搜索：
- 维护 g(s)（当前代价）与 rhs(s)（一步前瞻代价）。
- 当 rhs(s) != g(s) 时节点「不一致」，加入优先队列。
- compute_shortest_path 增量扩展，只处理受影响区域。
- 辅以 k_m 累积启发式偏移，支持机器人移动后高效复用。

"""

from __future__ import annotations

import heapq
from collections import defaultdict

import numpy as np


class DStarLite:
    """增量式 D* Lite 路径规划器。

    参数
    ----
    grid: OccupancyGrid 占据栅格。
    start: (3,) 起点世界坐标。
    goal: (3,) 终点世界坐标。
    """

    def __init__(self, grid, start: np.ndarray, goal: np.ndarray):
        self.grid = grid
        self._start = np.asarray(start, dtype=float)
        self._goal = np.asarray(goal, dtype=float)

        # 代价与前瞻
        self.g: dict[tuple, float] = defaultdict(lambda: float("inf"))
        self.rhs: dict[tuple, float] = defaultdict(lambda: float("inf"))

        # 优先队列: (key1, key2, version, cell)
        # 版本号机制：每次 _insert 自增，pop 时与最新版本比对，过期项自动跳过
        self.U: list[tuple[float, float, int, tuple]] = []
        self._U_version: dict[tuple, int] = {}

        self.k_m: float = 0.0  # 累积启发式偏移

        self._goal_idx = grid.world_to_index(goal)
        self._start_idx = grid.world_to_index(start)

        # 初始化：goal 的 rhs = 0
        self.rhs[self._goal_idx] = 0.0
        self._insert(self._goal_idx, self._heuristic(self._start_idx, self._goal_idx))

        self._last_start_idx = self._start_idx
        self._step_counter: int = 0   # prune 节流

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def compute_shortest_path(self) -> None:
        """扩展现有搜索树直到 start 一致性满足。"""
        while self.U:
            top = self._top_key()
            if top is None:
                break
            k1, k2 = top

            start_key = self._calculate_key(self._start_idx)
            if start_key < (k1, k2) and abs(self.rhs[self._start_idx] - self.g[self._start_idx]) < 1e-9:
                break

            # 弹出顶部有效项
            u = self._pop_valid()
            if u is None:
                break

            if (k1, k2) < self._calculate_key(u):
                self._insert(u, self._heuristic(self._start_idx, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for pred in self._predecessors(u):
                    self._update_vertex(pred)
            else:
                self.g[u] = float("inf")
                for pred in self._predecessors(u):
                    self._update_vertex(pred)
                self._update_vertex(u)

        self._last_start_idx = self._start_idx
        self._step_counter += 1
        # 每 100 次清理一次过期堆条目
        if self._step_counter % 100 == 0:
            self._gc_heap()

    def update_start(self, new_start: np.ndarray) -> None:
        """移动起点，累积 km 偏移。"""
        new_start = np.asarray(new_start, dtype=float)
        new_idx = self.grid.world_to_index(new_start)
        self.k_m += self._heuristic(self._last_start_idx, new_idx)
        self._start_idx = new_idx
        self._start = new_start

    def update_cell(self, cell: tuple, occupied: bool) -> None:
        """标记一个体素的占据状态变化。"""
        idx = self.grid.world_to_index(np.array(cell, dtype=float))
        if occupied:
            if not self.grid.is_occupied(idx):
                self.grid.data[idx] = 1
        else:
            self.grid.data[idx] = 0
        self._update_vertex(idx)

    def update_cells(self, changed_cells: list[tuple[tuple, bool]]) -> None:
        """批量更新体素。changed_cells: [(idx, occupied), ...]

        边代价 c(u,v) 变化时，v 的所有前驱 u 都需重算 rhs。
        因图为无向 6-邻域，前驱=后继，故更新变化体素及其全部邻居。
        """
        for idx, occ in changed_cells:
            self.grid.data[idx] = 1 if occ else 0

        updated: set[tuple] = set()
        for idx, _ in changed_cells:
            if idx not in updated:
                self._update_vertex(idx)
                updated.add(idx)
            for neighbor in self._successors(idx):
                if neighbor not in updated:
                    self._update_vertex(neighbor)
                    updated.add(neighbor)

    def extract_path(self) -> np.ndarray | None:
        """从当前 g/rhs 回溯最优路径。"""
        if self.g[self._start_idx] >= float("inf") - 1:
            return None

        path = []
        current = self._start_idx
        max_steps = 5000
        visited: set[tuple] = set()

        for _ in range(max_steps):
            path.append(self.grid.index_to_world(current))
            if current == self._goal_idx:
                break
            if current in visited:
                break
            visited.add(current)

            # 贪心选择 g 最小的后继
            best_next = None
            best_cost = float("inf")
            for succ in self._successors(current):
                c = self._cost(current, succ) + self.g[succ]
                if c < best_cost:
                    best_cost = c
                    best_next = succ
            if best_next is None:
                break
            current = best_next

        return np.array(path, dtype=float)

    def is_consistent(self) -> bool:
        """检查当前 start 是否一致（路径是否有效）。"""
        return abs(self.rhs[self._start_idx] - self.g[self._start_idx]) < 1e-9

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _heuristic(self, a: tuple, b: tuple) -> float:
        """欧氏距离启发式。"""
        pa = self.grid.index_to_world(a)
        pb = self.grid.index_to_world(b)
        return float(np.linalg.norm(pa - pb))

    def _cost(self, a: tuple, b: tuple) -> float:
        """两相邻体素间的边代价。占据体素代价 = inf。"""
        if self.grid.is_occupied(a) or self.grid.is_occupied(b):
            return float("inf")
        return self._heuristic(a, b)

    def _calculate_key(self, s: tuple) -> tuple[float, float]:
        """计算优先队列键值。"""
        g_rhs_min = min(self.g[s], self.rhs[s])
        return (
            g_rhs_min + self._heuristic(self._start_idx, s) + self.k_m,
            g_rhs_min,
        )

    def _insert(self, s: tuple, h_start_s: float) -> None:
        """将节点插入优先队列。版本号自增，旧条目作废。"""
        g_rhs_min = min(self.g[s], self.rhs[s])
        key = (g_rhs_min + h_start_s + self.k_m, g_rhs_min)
        ver = self._U_version.get(s, 0) + 1
        self._U_version[s] = ver
        heapq.heappush(self.U, (key[0], key[1], ver, s))

    def _is_top_valid(self) -> bool:
        if not self.U:
            return False
        _, _, ver, s = self.U[0]
        return self._U_version.get(s, 0) == ver

    def _top_key(self) -> tuple[float, float] | None:
        """查看队首键值，跳过过期条目。无有效项返回 None。"""
        while self.U:
            k1, k2, ver, s = self.U[0]
            if self._U_version.get(s, 0) != ver:
                heapq.heappop(self.U)
                continue
            return (k1, k2)
        return None

    def _pop_valid(self) -> tuple | None:
        """弹出顶部有效项，过期项自动丢弃。"""
        while self.U:
            k1, k2, ver, s = heapq.heappop(self.U)
            if self._U_version.get(s, 0) == ver:
                # 弹出后该节点视为已处理，作废其版本号避免堆中其他副本被认作有效
                self._U_version[s] = ver + 1
                return s
        return None

    def _gc_heap(self) -> None:
        """清理堆中过期条目，控制内存占用。"""
        self.U = [
            (k1, k2, ver, s) for (k1, k2, ver, s) in self.U
            if self._U_version.get(s, 0) == ver
        ]
        heapq.heapify(self.U)

    def _update_vertex(self, u: tuple) -> None:
        """更新顶点的 rhs 值并在必要时重新排队。"""
        if u == self._goal_idx:
            return

        min_rhs = float("inf")
        for succ in self._successors(u):
            c = self._cost(u, succ)
            if c < float("inf"):
                min_rhs = min(min_rhs, c + self.g[succ])
        self.rhs[u] = min_rhs

        if abs(self.g[u] - self.rhs[u]) > 1e-9:
            self._insert(u, self._heuristic(self._start_idx, u))
        else:
            # 一致 → 作废堆中旧条目（版本号失效）
            if u in self._U_version:
                self._U_version[u] += 1

    def _successors(self, idx: tuple) -> list[tuple]:
        """返回 6 邻域后继。"""
        succs = []
        for d in range(3):
            for delta in (-1, 1):
                n = list(idx)
                n[d] += delta
                nt = tuple(n)
                if all(0 <= n[i] < self.grid.shape[i] for i in range(3)):
                    succs.append(nt)
        return succs

    def _predecessors(self, idx: tuple) -> list[tuple]:
        """6 邻域前驱（无向图等同于后继）。"""
        return self._successors(idx)

    def prune_horizon(self, horizon_indices: set[tuple]) -> None:
        """清理 horizon 之外的过期状态以控制内存。"""
        for idx in list(self.g.keys()):
            if idx not in horizon_indices and idx != self._goal_idx and idx != self._start_idx:
                del self.g[idx]
                self.rhs.pop(idx, None)
                # 作废其版本号
                if idx in self._U_version:
                    self._U_version[idx] += 1
        self._gc_heap()