"""Hybrid A* 路径规划器（四旋翼 3D + 航向）。

用途
----
为四旋翼无人机在三维占据栅格中规划满足运动学约束的平滑避障路径。

原理
----
1) 状态空间：(x, y, z, ψ) 连续，ψ ∈ [0, 2π)
2) 运动原语：体坐标系常速控制 (vx_b, vy_b, vz, ω) × Δt，
   传播采用闭环解析公式（ω=0 为直线，ω≠0 为圆弧）
3) 启发式：max(3D 反向 Dijkstra 避障代价, 无障最小时间下界)
4) 分析扩展：每 N 次扩展尝试直线+航向插值直达 goal
5) 重复检测：栅格位置 + 72 份离散化航向角

与标准 A* 的关键区别
----
- 状态包含连续航向 ψ，使路径满足转弯约束
- 采用运动原语而非 26 邻域，保证运动学可行性
- 双重可采纳启发式加速搜索

参考文献
----
Dolgov et al. "Practical Search Techniques in Path Planning for
Autonomous Driving", AAAI 2008 Workshop.

"""

from __future__ import annotations

import heapq
from math import pi as PI

import numpy as np

from .base import Planner, PlannerError


class HybridAStar(Planner):
    """真正 Hybrid A* 规划器。

    参数
    ----
    v_max:       最大水平速度 m/s
    v_z_max:     最大垂向速度 m/s
    omega_max:   最大偏航角速率 rad/s (默认 60 deg/s)
    dt_primitive: 单条运动原语时长 s
    n_heading_bins: 航向离散化份数 (默认 72 = 5°间隔)
    max_iter:     最大迭代次数
    analytic_expand_interval: 分析扩展间隔
    reverse_penalty: 后退原语额外代价
    direction_change_penalty: 方向变化额外代价
    """

    def __init__(
        self,
        v_max: float = 2.0,
        v_z_max: float = 1.0,
        omega_max: float = 1.047197551,   # 60 deg/s
        dt_primitive: float = 0.5,
        n_heading_bins: int = 72,
        max_iter: int = 10000,
        analytic_expand_interval: int = 50,
        reverse_penalty: float = 0.5,
        direction_change_penalty: float = 0.2,
    ):
        self.v_max = float(v_max)
        self.v_z_max = float(v_z_max)
        self.omega_max = float(omega_max)
        self.dt = float(dt_primitive)
        self.n_heading_bins = int(n_heading_bins)
        self.max_iter = int(max_iter)
        self.analytic_interval = int(analytic_expand_interval)
        self.reverse_penalty = float(reverse_penalty)
        self.dir_change_penalty = float(direction_change_penalty)

        self._heading_res = 2.0 * PI / self.n_heading_bins
        self._v_diag = self.v_max / np.sqrt(2.0)

        # 运行时状态（plan() 中初始化）
        self._h_holonomic: dict[tuple, float] = {}
        self._grid = None
        self._goal_pos = np.zeros(3)
        self._goal_psi = 0.0
        self._primitives: list[tuple] = []
        self._build_primitives()

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        """规划从 start 到 goal 的运动学可行路径。

        参数
        ----
        start: (3,) 起点世界坐标 [x, y, z]
        goal:  (3,) 终点世界坐标 [x, y, z]
        grid:  OccupancyGrid 或 SDFAwareGrid
        **kw:  可选 seed (int) 用于确定性

        返回
        ----
        (K, 3) 路径航点数组
        """
        start = np.asarray(start, dtype=float)
        goal_arr = np.asarray(goal, dtype=float)
        self._grid = grid
        self._goal_pos = goal_arr

        # 验证起终点
        start_idx = grid.world_to_index(start)
        goal_idx = grid.world_to_index(goal_arr)
        if grid.is_occupied(start_idx):
            raise PlannerError("起点在障碍物内")
        if grid.is_occupied(goal_idx):
            raise PlannerError("终点在障碍物内")

        # 计算起终点航向：沿起止连线方向
        direction = goal_arr - start
        dist = float(np.linalg.norm(direction))
        start_psi = float(np.arctan2(direction[1], direction[0])) if dist > 1e-6 else 0.0
        self._goal_psi = start_psi  # 目标航向与起始相同（沿运动方向）

        # 1) 预计算避障启发式（跳过 ESDF 软代价包装，避免扭曲下界）
        heuristic_grid = grid._base if hasattr(grid, '_base') else grid
        self._h_holonomic = self._precompute_holonomic_heuristic(goal_arr, heuristic_grid)

        # 2) A* 搜索
        rng = np.random.RandomState(kw.get("seed", 42))
        path_states = self._search(start, goal_arr, start_psi, rng)
        if path_states is None or len(path_states) < 2:
            raise PlannerError("Hybrid A* 未找到可行路径")

        # 3) 提取位置 → 平滑 → 降采样 → 碰撞校验
        raw_path = np.array([[s[0], s[1], s[2]] for s in path_states], dtype=float)

        if len(raw_path) < 4:
            return raw_path

        # 轻量平滑（Hybrid A* 路径已是运动学可行的）
        smoothed = self.smooth(raw_path)

        # 降采样到合理密度：间距 ≈ max(2.5×分辨率, 1.0×原语步长)
        wp_spacing = max(grid.resolution * 2.5, self.v_max * self.dt * 1.0)
        downsampled = self._downsample_path(smoothed, wp_spacing)

        # 平滑后碰撞校验：移除穿过障碍物的点
        return self._safe_path(downsampled, grid)

    # ------------------------------------------------------------------
    # 运动原语
    # ------------------------------------------------------------------

    def _build_primitives(self) -> None:
        v = self.v_max
        vz = self.v_z_max
        w = self.omega_max
        vd = self._v_diag

        self._primitives = [
            (v, 0.0, 0.0, 0.0),           # 0: 全速前进
            (v / 2.0, 0.0, 0.0, 0.0),     # 1: 半速前进
            (0.0, v, 0.0, 0.0),           # 2: 左移
            (0.0, -v, 0.0, 0.0),          # 3: 右移
            (vd, vd, 0.0, 0.0),           # 4: 前进+左移
            (vd, -vd, 0.0, 0.0),          # 5: 前进+右移
            (-v / 2.0, 0.0, 0.0, 0.0),   # 6: 半速后退
            (0.0, 0.0, vz, 0.0),          # 7: 上升
            (0.0, 0.0, -vz, 0.0),         # 8: 下降
            (v / 2.0, 0.0, 0.0, w),       # 9: 前进+左转
            (v / 2.0, 0.0, 0.0, -w),      # 10: 前进+右转
            (0.0, 0.0, 0.0, w),           # 11: 原地左转
            (0.0, 0.0, 0.0, -w),          # 12: 原地右转
        ]

    def _propagate(self, state: tuple, prim_idx: int) -> tuple:
        """对状态应用运动原语，返回 (new_state, trajectory_points)。

        state: (x, y, z, psi)
        trajectory_points: [(x, y, z), ...] 采样点列表

        闭环传播公式：
        - ω=0 且 |v_xy|>0：直线 Δpos = R(ψ)·[vx_b, vy_b]^T·dt
        - ω≠0：圆弧 Δψ=ω·dt, Δpos 用积分闭合解
        """
        x, y, z, psi = state
        vx_b, vy_b, vz, omega = self._primitives[prim_idx]
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        dt = self.dt

        # --- 垂向：直线 ---
        new_z = z + vz * dt

        if abs(omega) < 1e-9:
            # 直线运动
            dx = (vx_b * cos_psi - vy_b * sin_psi) * dt
            dy = (vx_b * sin_psi + vy_b * cos_psi) * dt
            new_x = x + dx
            new_y = y + dy
            new_psi = psi

            # 轨迹采样点
            n_samples = max(2, int(dt / 0.05))
            traj = []
            for i in range(n_samples + 1):
                t = i / n_samples * dt
                traj.append((
                    x + (vx_b * cos_psi - vy_b * sin_psi) * t,
                    y + (vx_b * sin_psi + vy_b * cos_psi) * t,
                    z + vz * t,
                ))
        else:
            # 圆弧运动
            new_psi = (psi + omega * dt) % (2.0 * PI)
            if new_psi < 0:
                new_psi += 2.0 * PI
            sin_new = np.sin(new_psi)
            cos_new = np.cos(new_psi)

            # 位置闭式积分
            inv_w = 1.0 / omega
            new_x = x + inv_w * (vx_b * (sin_new - sin_psi) + vy_b * (cos_new - cos_psi))
            new_y = y + inv_w * (vx_b * (cos_psi - cos_new) + vy_b * (sin_new - sin_psi))

            # 角度均匀采样轨迹点
            n_samples = max(2, int(abs(omega) * dt / 0.05))
            traj = []
            for i in range(n_samples + 1):
                t = i / n_samples * dt
                psi_t = psi + omega * t
                if omega > 1e-9 or omega < -1e-9:
                    inv_w_t = 1.0 / omega
                    xt = x + inv_w_t * (vx_b * (np.sin(psi_t) - sin_psi) + vy_b * (np.cos(psi_t) - cos_psi))
                    yt = y + inv_w_t * (vx_b * (cos_psi - np.cos(psi_t)) + vy_b * (np.sin(psi_t) - sin_psi))
                else:
                    xt, yt = x, y
                traj.append((xt, yt, z + vz * t))

        return (new_x, new_y, new_z, new_psi), traj

    # ------------------------------------------------------------------
    # 碰撞检查
    # ------------------------------------------------------------------

    def _collision_free_trajectory(self, traj: list[tuple], grid) -> bool:
        """检查轨迹采样点序列是否全部无碰。"""
        for pt in traj:
            idx = grid.world_to_index(np.array(pt, dtype=float))
            if grid.is_occupied(idx):
                return False
        return True

    def _is_free_position(self, pos: np.ndarray, grid) -> bool:
        idx = grid.world_to_index(pos)
        return not grid.is_occupied(idx)

    # ------------------------------------------------------------------
    # 启发式
    # ------------------------------------------------------------------

    def _precompute_holonomic_heuristic(self, goal: np.ndarray, grid) -> dict[tuple, float]:
        """3D 反向 Dijkstra：从 goal 出发，计算到所有可达栅格的最短避障路径长度。

        采用 26 邻域，欧氏边长。只访问 grid.is_free 的栅格。
        返回 dict: {index_tuple: cost_to_goal}
        """
        goal_idx = grid.world_to_index(goal)
        if grid.is_occupied(goal_idx):
            return {}

        resolution = grid.resolution
        neighbors = [
            (dx, dy, dz)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        cost_map: dict[tuple, float] = {goal_idx: 0.0}
        pq: list[tuple[float, tuple]] = [(0.0, goal_idx)]

        while pq:
            d, current = heapq.heappop(pq)
            if d > cost_map.get(current, float("inf")):
                continue

            cx, cy, cz = current
            for dx, dy, dz in neighbors:
                nx, ny, nz = cx + dx, cy + dy, cz + dz
                n_idx = (nx, ny, nz)
                if n_idx[0] < 0 or n_idx[0] >= grid.shape[0]:
                    continue
                if n_idx[1] < 0 or n_idx[1] >= grid.shape[1]:
                    continue
                if n_idx[2] < 0 or n_idx[2] >= grid.shape[2]:
                    continue
                if not grid.is_free(n_idx):
                    continue

                edge_cost = np.sqrt(dx**2 + dy**2 + dz**2) * resolution
                extra_fn = getattr(grid, "extra_cost", None)
                if extra_fn is not None:
                    edge_cost += extra_fn(n_idx)
                nd = d + edge_cost
                if nd < cost_map.get(n_idx, float("inf")):
                    cost_map[n_idx] = nd
                    heapq.heappush(pq, (nd, n_idx))

        return cost_map

    def _holonomic_h(self, pos: np.ndarray) -> float:
        """查询预计算的避障启发式。"""
        if not self._h_holonomic:
            return 0.0
        idx = self._grid.world_to_index(pos)
        val = self._h_holonomic.get(idx)
        return val if val is not None else float("inf")

    def _nonholonomic_h(self, state: tuple) -> float:
        """无障最小时间下界 = 距离/v_max + 航向差/omega_max。"""
        x, y, z, psi = state
        gx, gy, gz = self._goal_pos[0], self._goal_pos[1], self._goal_pos[2]
        dist = np.sqrt((x - gx)**2 + (y - gy)**2 + (z - gz)**2)
        dpsi = abs(self._wrap_angle(self._goal_psi - psi))
        return dist / self.v_max + dpsi / self.omega_max

    def _heuristic(self, state: tuple) -> float:
        pos = np.array(state[:3], dtype=float)
        h_holo = self._holonomic_h(pos)
        h_nonholo = self._nonholonomic_h(state)
        return max(h_holo, h_nonholo)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """将角度包裹到 [-π, π]。"""
        return (angle + PI) % (2.0 * PI) - PI

    # ------------------------------------------------------------------
    # 状态离散化
    # ------------------------------------------------------------------

    def _state_key(self, pos: np.ndarray, psi: float) -> tuple:
        idx = self._grid.world_to_index(pos)
        hbin = int(round(psi / self._heading_res)) % self.n_heading_bins
        return (idx[0], idx[1], idx[2], hbin)

    # ------------------------------------------------------------------
    # 分析扩展
    # ------------------------------------------------------------------

    def _analytic_expand(self, state: tuple, goal: np.ndarray) -> tuple | None:
        """尝试直线+航向插值连接到 goal。

        返回 goal_state 或 None（不可行）。
        """
        x, y, z, psi = state
        gx, gy, gz = goal[0], goal[1], goal[2]
        gpsi = self._goal_psi

        dist = np.sqrt((x - gx)**2 + (y - gy)**2 + (z - gz)**2)
        if dist < 1e-6:
            return (gx, gy, gz, gpsi)

        # 以 grid_resolution/2 步长采样
        step = self._grid.resolution * 0.5
        n = max(2, int(dist / step))
        for i in range(1, n + 1):
            t = i / n
            pt = np.array([
                x + (gx - x) * t,
                y + (gy - y) * t,
                z + (gz - z) * t,
            ], dtype=float)
            if not self._is_free_position(pt, self._grid):
                return None

        return (gx, gy, gz, gpsi)

    # ------------------------------------------------------------------
    # 代价
    # ------------------------------------------------------------------

    def _primitive_cost(self, prim_idx: int, prev_idx: int | None) -> float:
        """计算应用一条原语的总代价。"""
        vx_b, vy_b, vz, omega = self._primitives[prim_idx]
        cost = self.dt

        # 后退惩罚
        if vx_b < -1e-9:
            cost += self.reverse_penalty

        # 方向变化惩罚
        if prev_idx is not None:
            prev = self._primitives[prev_idx]
            v_cur = np.array([vx_b, vy_b, vz], dtype=float)
            v_prev = np.array([prev[0], prev[1], prev[2]], dtype=float)
            n_cur = np.linalg.norm(v_cur)
            n_prev = np.linalg.norm(v_prev)
            if n_cur > 1e-9 and n_prev > 1e-9:
                cos_ang = np.clip(np.dot(v_cur, v_prev) / (n_cur * n_prev), -1.0, 1.0)
                angle = float(np.arccos(cos_ang))
                if angle > PI / 4.0:
                    cost += self.dir_change_penalty

        return cost

    # ------------------------------------------------------------------
    # A* 搜索主循环
    # ------------------------------------------------------------------

    def _search(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        start_psi: float,
        rng: np.random.RandomState,
    ) -> list[tuple] | None:
        grid = self._grid
        start_state = (float(start[0]), float(start[1]), float(start[2]), start_psi)

        sk = self._state_key(start[:3], start_psi)
        g_score: dict[tuple, float] = {sk: 0.0}
        parent: dict[tuple, tuple] = {sk: (None, -1)}  # (parent_key, prim_idx)
        h_start = self._heuristic(start_state)

        # 优先队列: (f, tiebreaker, state_key)
        # tiebreaker 保证同 f 时 LIFO 避免比较 state_key
        tie = 0
        pq: list[tuple[float, int, tuple]] = [(h_start, tie, sk)]
        visited: set[tuple] = set()

        goal_pos = np.array([goal[0], goal[1], goal[2]], dtype=float)
        n_prims = len(self._primitives)
        resolution = grid.resolution

        for iteration in range(self.max_iter):
            if not pq:
                break

            f_val, _, current_key = heapq.heappop(pq)
            if current_key in visited:
                continue
            visited.add(current_key)

            # 重建当前连续状态用于扩展
            cur_state = self._reconstruct_full_state(current_key, parent, start_state)

            # ---- 终止检查 ----
            cx, cy, cz, cpsi = cur_state
            pos_err = np.sqrt((cx - goal_pos[0])**2 + (cy - goal_pos[1])**2 + (cz - goal_pos[2])**2)
            psi_err = abs(self._wrap_angle(self._goal_psi - cpsi))
            if pos_err < resolution * 1.5 and psi_err < self._heading_res:
                return self._extract_path(current_key, parent, start_state)

            # ---- 分析扩展 ----
            if iteration > 0 and iteration % self.analytic_interval == 0:
                goal_state = self._analytic_expand(cur_state, goal_pos)
                if goal_state is not None:
                    gs, gy, gz, gpsi = goal_state
                    gk = self._state_key(np.array([gs, gy, gz]), gpsi)
                    parent[gk] = (current_key, -2)  # -2 标记分析扩展
                    g_score[gk] = g_score[current_key] + pos_err / self.v_max
                    return self._extract_path(gk, parent, start_state)

            # ---- 扩展运动原语 ----
            prev_prim = parent[current_key][1] if current_key in parent else -1
            if prev_prim < 0:
                prev_prim = None

            for p_idx in range(n_prims):
                new_state, traj = self._propagate(cur_state, p_idx)

                # 检查轨迹碰撞
                if not self._collision_free_trajectory(traj, grid):
                    continue

                nx, ny, nz, npsi = new_state
                # 检查终态位置是否在界内
                npos = np.array([nx, ny, nz], dtype=float)
                nidx = grid.world_to_index(npos)
                if nidx[0] < 0 or nidx[0] >= grid.shape[0]:
                    continue
                if nidx[1] < 0 or nidx[1] >= grid.shape[1]:
                    continue
                if nidx[2] < 0 or nidx[2] >= grid.shape[2]:
                    continue

                nk = self._state_key(npos, npsi)
                if nk in visited:
                    continue

                added_cost = self._primitive_cost(p_idx, prev_prim)
                tentative_g = g_score[current_key] + added_cost

                if tentative_g < g_score.get(nk, float("inf")):
                    g_score[nk] = tentative_g
                    parent[nk] = (current_key, p_idx)
                    f_new = tentative_g + self._heuristic(new_state)
                    tie += 1
                    heapq.heappush(pq, (f_new, tie, nk))

        return None

    # ------------------------------------------------------------------
    # 路径提取与平滑
    # ------------------------------------------------------------------

    def _extract_path(
        self,
        goal_key: tuple,
        parent: dict,
        start_state: tuple,
    ) -> list[tuple]:
        """从 goal_key 沿 parent 回溯，返回 [(x,y,z,psi), ...] 状态列表。"""
        states: list[tuple] = []
        key = goal_key
        while key is not None:
            state = self._reconstruct_full_state(key, parent, start_state)
            states.append(state)
            entry = parent.get(key)
            key = entry[0] if entry is not None else None
        states.reverse()
        return states

    def _reconstruct_full_state(
        self,
        key: tuple,
        parent: dict,
        start_state: tuple,
    ) -> tuple:
        """从离散 key 重建连续状态。

        若该 key 不在 parent 中且不是 start → 从 key 的栅格索引近似重建。
        """
        entry = parent.get(key)
        if entry is None:
            # 起始状态
            return start_state

        # 从父状态传播恢复
        parent_key, prim_idx = entry
        parent_state = self._reconstruct_full_state(parent_key, parent, start_state)
        if prim_idx >= 0:
            new_state, _ = self._propagate(parent_state, prim_idx)
            return new_state
        elif prim_idx == -2:
            # 分析扩展：从父状态直线到达 goal
            px, py, pz, ppsi = parent_state
            gx, gy, gz = self._goal_pos
            return (gx, gy, gz, self._goal_psi)
        return parent_state

    def smooth(self, path: np.ndarray, num_insert: int = 1, grid=None) -> np.ndarray:
        """对 Hybrid A* 路径做稀疏平滑。

        Hybrid A* 路径由运动原语生成已是运动学可行的，不需要密集插值。
        默认 num_insert=1（每段插入 1 点），约为栅格分辨率的 2 倍间距。
        """
        return super().smooth(path, num_insert=num_insert, grid=grid)

    @staticmethod
    def _downsample_path(path: np.ndarray, spacing: float) -> np.ndarray:
        """弧长降采样到目标间距。"""
        if len(path) < 2:
            return path
        path = np.asarray(path, dtype=float)
        diffs = np.diff(path, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
        total = cum_len[-1]
        if total < spacing:
            return path
        n = max(2, int(np.ceil(total / spacing)))
        sample_lens = np.linspace(0, total, n)
        result = np.zeros((n, 3))
        for d in range(3):
            result[:, d] = np.interp(sample_lens, cum_len, path[:, d])
        return result

    @staticmethod
    def _safe_path(path: np.ndarray, grid) -> np.ndarray:
        """移除平滑后穿过障碍物的点。"""
        if len(path) < 2:
            return path
        safe = [path[0]]
        for i in range(1, len(path)):
            # 线段采样
            dist = np.linalg.norm(path[i] - safe[-1])
            if dist < grid.resolution * 0.5:
                safe.append(path[i])
                continue
            steps = max(2, int(dist / (grid.resolution * 0.5)))
            collision = False
            for s in range(1, steps):
                t = s / steps
                pt = safe[-1] + (path[i] - safe[-1]) * t
                idx = grid.world_to_index(pt)
                if grid.is_occupied(idx):
                    collision = True
                    break
            if not collision:
                safe.append(path[i])
        return np.array(safe, dtype=float)
