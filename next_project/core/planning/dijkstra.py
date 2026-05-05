"""Dijkstra 最短路径。

用途
----
作为基线规划方案，在占据栅格上求取全局最短路径。

原理
----
等价于 A* 启发式 h=0 的特例。使用 26 邻域探索三维栅格，
优先队列 + 距离字典，确保找到最优解。

"""

from __future__ import annotations

import heapq

import numpy as np

from .base import Planner, PlannerError


class Dijkstra(Planner):
    """Dijkstra 最短路径规划器。"""

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

        pq = [(0.0, start_idx)]
        dist = {start_idx: 0.0}
        parent = {start_idx: None}
        visited = set()

        while pq:
            d, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_idx:
                return self._reconstruct(current, parent, grid)

            for dx, dy, dz in self.NEIGHBORS_26:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                if grid.is_occupied(neighbor):
                    continue
                cost = np.sqrt(dx * dx + dy * dy + dz * dz) * grid.resolution
                extra_fn = getattr(grid, "extra_cost", None)
                if extra_fn is not None:
                    cost += extra_fn(neighbor)
                nd = d + cost
                if neighbor not in dist or nd < dist[neighbor]:
                    dist[neighbor] = nd
                    parent[neighbor] = current
                    heapq.heappush(pq, (nd, neighbor))

        raise PlannerError("Dijkstra 未找到可行路径")

    def _reconstruct(self, goal_idx, parent: dict, grid) -> np.ndarray:
        path = []
        node = goal_idx
        while node is not None:
            path.append(grid.index_to_world(node))
            node = parent[node]
        path.reverse()
        return np.array(path, dtype=float)
