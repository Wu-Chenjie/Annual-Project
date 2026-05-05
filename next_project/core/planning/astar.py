"""A* 与航向偏转剪枝 A* 路径规划。

用途
----
A* 为占据栅格上的最优路径搜索；HeadingConstrainedAStar 在此基础上
对扩展邻居施加最大航向偏转角剪枝，使路径形状更平直。

原理
----
A*：使用 26 邻域 + 欧氏距离启发式，保证最优性。
HeadingConstrainedAStar：扩展节点时记录上一步方向，若新方向与
上一步方向夹角大于 max_turn_rad，则该邻居被剪枝。
注意：这是"最大单步偏转角"约束，并非真正的最小转弯半径
(R = v²/a_max)。要实现真正的 Hybrid A* 需扩展状态为 (x, y, z, ψ)
并使用 motion primitives + 解析扩展，详见 new_plan.md M7 里程碑。

"""

from __future__ import annotations

import heapq

import numpy as np

from .base import Planner, PlannerError


class AStar(Planner):
    """A* 最短路径规划器（26 邻域）。"""

    NEIGHBORS_26 = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        start_idx = grid.world_to_index(start)
        goal_idx = grid.world_to_index(goal)

        if grid.is_occupied(start_idx):
            raise PlannerError("起点在障碍物内")
        if grid.is_occupied(goal_idx):
            raise PlannerError("终点在障碍物内")

        if np.linalg.norm(goal - start) < 1e-6:
            return np.array([start, goal], dtype=float)

        def heuristic(a, b):#乘以grid将栅格距离转化为世界距离
            return np.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b))) * grid.resolution

        start_f = heuristic(start_idx, goal_idx)
        pq = [(start_f, start_idx)]#存储优先队列编号
        g_score = {start_idx: 0.0}
        parent = {start_idx: None}
        visited = set()

        while pq:
            _, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_idx:
                return self._reconstruct(current, parent, grid)

            for dx, dy, dz in self.NEIGHBORS_26:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                if grid.is_occupied(neighbor):
                    continue
                move_cost = np.sqrt(dx * dx + dy * dy + dz * dz) * grid.resolution
                # ESDF 软代价（自动检测 CostAwareGrid）
                extra_fn = getattr(grid, "extra_cost", None)
                if extra_fn is not None:
                    move_cost += extra_fn(neighbor)
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    parent[neighbor] = current
                    f = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(pq, (f, neighbor))

        raise PlannerError("A* 未找到可行路径")

    def _reconstruct(self, goal_idx, parent: dict, grid) -> np.ndarray:
        path = []
        node = goal_idx
        while node is not None:
            path.append(grid.index_to_world(node))
            node = parent[node]
        path.reverse()
        return np.array(path, dtype=float)


class HeadingConstrainedAStar(AStar):
    """航向偏转剪枝 A* 路径规划器。

    用途
    ----
    在 A* 基础上对扩展邻居施加最大单步偏转角约束，
    使规划路径形状更平直，便于后续平滑与跟踪。

    原理
    ----
    扩展时对每个候选邻居检查与父节点方向的夹角：
    若 |delta_heading| > max_turn_rad (默认 π/4)，则该邻居被剪枝。
    与速度无关——这不是真正的最小转弯半径约束。
    在原本A*的基础上在队列中存储了方向，使的过于猛烈不符合运动学的转向被剔除
    """

    def __init__(self, max_turn_rad: float = 0.7854):
        self.max_turn_rad = float(max_turn_rad)

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        start_idx = grid.world_to_index(start)
        goal_idx = grid.world_to_index(goal)

        if grid.is_occupied(start_idx):
            raise PlannerError("起点在障碍物内")
        if grid.is_occupied(goal_idx):
            raise PlannerError("终点在障碍物内")

        if np.linalg.norm(goal - start) < 1e-6:
            return np.array([start, goal], dtype=float)

        def heuristic(a, b):
            return np.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b))) * grid.resolution

        start_f = heuristic(start_idx, goal_idx)
        # 优先级队列元素: (f, idx, parent_direction)
        pq = [(start_f, start_idx, None)]
        g_score = {(start_idx, None): 0.0}
        parent = {(start_idx, None): (None, None)}
        visited = set()

        while pq:
            _, current, prev_dir = heapq.heappop(pq)
            state_key = (current, prev_dir)
            if state_key in visited:
                continue
            visited.add(state_key)

            if current == goal_idx:
                return self._reconstruct_hybrid((current, prev_dir), parent, grid)

            for dx, dy, dz in self.NEIGHBORS_26:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                if grid.is_occupied(neighbor):
                    continue

                # 运动学约束：检查航向偏转
                new_dir = (dx, dy, dz)
                if prev_dir is not None and self._any_direction(prev_dir):
                    angle = self._angle_between(prev_dir, new_dir)
                    if angle > self.max_turn_rad:
                        continue

                move_cost = np.sqrt(dx * dx + dy * dy + dz * dz) * grid.resolution
                extra_fn = getattr(grid, "extra_cost", None)
                if extra_fn is not None:
                    move_cost += extra_fn(neighbor)
                tentative_g = g_score[state_key] + move_cost
                neighbor_state = (neighbor, new_dir)

                if neighbor_state not in g_score or tentative_g < g_score[neighbor_state]:
                    g_score[neighbor_state] = tentative_g
                    parent[neighbor_state] = (current, prev_dir)
                    f = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(pq, (f, neighbor, new_dir))

        raise PlannerError("HeadingConstrainedAStar 未找到可行路径")

    def _reconstruct_hybrid(self, goal_state, parent: dict, grid) -> np.ndarray:
        path = []
        node = goal_state
        while node[0] is not None:
            path.append(grid.index_to_world(node[0]))
            node = parent[node]
        path.reverse()
        return np.array(path, dtype=float)

    @staticmethod
    def _angle_between(d1, d2) -> float:
        a = np.array(d1, dtype=float)
        b = np.array(d2, dtype=float)
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            return 0.0
        cos_angle = np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)
        return float(np.arccos(cos_angle))

    @staticmethod
    def _any_direction(d) -> bool:
        return d[0] != 0 or d[1] != 0 or d[2] != 0


# 向后兼容别名 — HeadingConstrainedAStar 施加单步最大偏转角约束，
# 并非真正的 Hybrid A*（不含 motion primitives 和解析扩展）。
TurnConstrainedAStar = HeadingConstrainedAStar
