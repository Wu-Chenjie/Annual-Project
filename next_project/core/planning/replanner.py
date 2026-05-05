"""在线重规划调度器。

用途
----
在在线模式下，按固定周期利用传感器数据局部更新障碍地图并重新规划。
使用滑动窗口策略 + D* Lite 增量层 + Informed RRT* 全局层三级分层架构，
保证每次重规划在相应层级上执行，平衡响应速度与路径质量。

新增（论文2+3）
--------------
- Safe/Danger 双模式集成：Danger 模式切换到 GNN+可见图规划
- RiskAdaptiveReplanInterval：风险驱动的自适应重规划间隔
- 编队自动收缩检测

原理
----
分层调度策略：
local (滑动窗口) → incremental (D* Lite) → global (Informed RRT*)

1) 每 interval 秒从传感器拉取最近一帧，局部更新 OccupancyGrid。
2) Danger 模式下切换 GNN 可见图规划。
3) 风险驱动自适应间隔：高风险→缩短间隔，低风险→延长间隔。
4) D* Lite 代价显著退化 → Informed RRT* 重建全局参考。
5) 用 Hausdorff/DTW 全面度量新旧路径偏差。

"""

from __future__ import annotations

import numpy as np
from collections import deque

from .base import Planner


# 调度阶段枚举
PHASE_LOCAL = "local"
PHASE_INCREMENTAL = "incremental"
PHASE_GLOBAL = "global"


class RiskAdaptiveReplanInterval:
    """风险驱动的自适应重规划间隔调度器（论文3 PER 思想启发）。

    原理
    ----
    维护滑动窗口"碰撞风险历史"作为重要性代理量。
    高风险 → 缩短间隔（更频繁重规划），低风险 → 延长间隔（节省计算）。
    """

    def __init__(self, base_interval: float = 0.4, min_interval: float = 0.1, max_interval: float = 1.0):
        self.base = float(base_interval)
        self.min = float(min_interval)
        self.max = float(max_interval)
        self._risk_history: list[float] = []

    def update(self, min_sdf: float, sensor_min: float = float("inf")) -> None:
        """记录当前帧的风险指标。"""
        risk = 1.0 - min(min_sdf / 5.0, 1.0)
        if sensor_min < float("inf"):
            risk = max(risk, 1.0 - min(sensor_min / 5.0, 1.0))
        self._risk_history.append(risk)
        if len(self._risk_history) > 20:
            self._risk_history.pop(0)

    @property
    def interval(self) -> float:
        if not self._risk_history:
            return self.base
        avg_risk = sum(self._risk_history) / len(self._risk_history)
        return self.max - (self.max - self.min) * avg_risk


class WindowReplanner:
    """滑动窗口在线重规划调度器（三级分层 + 双模式）。

    参数
    ----
    planner: 局部层规划器实例（如 RRTStar）。
    grid: OccupancyGrid 占据栅格。
    interval: 重规划周期 s。
    horizon: 滑动窗口前瞻距离 m。
    epsilon: 路径偏差阈值 m。
    deviation_metric: 偏差度量方法 "hausdorff" | "dtw" | "max_pairwise"。
    global_planner: 全局参考层规划器（如 InformedRRTStar），可选。
    incremental_planner: 增量退化层规划器（如 DStarLite），可选。
    danger_planner: Danger 模式规划器（如 GNNPlanner），可选。
    dual_mode: DualModeScheduler 实例，可选。
    adaptive_interval: RiskAdaptiveReplanInterval 实例，可选。
    obstacle_field: ObstacleField 实例，用于 Danger 模式。
    local_fail_threshold: 局部修正连续失败触发增量层的阈值。
    """

    def __init__(
        self,
        planner: Planner,
        grid,
        interval: float = 0.4,
        horizon: float = 6.0,
        epsilon: float = 0.5,
        deviation_metric: str = "hausdorff",
        global_planner: Planner | None = None,
        incremental_planner=None,
        danger_planner: Planner | None = None,
        dual_mode=None,
        adaptive_interval: RiskAdaptiveReplanInterval | None = None,
        obstacle_field=None,
        local_fail_threshold: int = 3,
        sensor_obstacle_ttl_steps: int = 3,
        sensor_clear_confirm_steps: int = 1,
        gnn_path_ratio_limit: float = 1.2,
    ):
        self.planner = planner
        self.grid = grid
        self._static_occupied = (np.asarray(grid.data) >= 1).copy()
        self._sensor_occupied = np.zeros_like(grid.data, dtype=bool)
        self._sensor_ttl = np.zeros_like(grid.data, dtype=np.int16)
        self._sensor_clear_hits = np.zeros_like(grid.data, dtype=np.int16)
        self.base_interval = float(interval)
        self.horizon = float(horizon)
        self.epsilon = float(epsilon)
        self.deviation_metric = deviation_metric
        self.sensor_obstacle_ttl_steps = max(1, int(sensor_obstacle_ttl_steps))
        self.sensor_clear_confirm_steps = max(1, int(sensor_clear_confirm_steps))
        self.gnn_path_ratio_limit = max(1.0, float(gnn_path_ratio_limit))

        # 分层规划器
        self.global_planner = global_planner          # InformedRRTStar
        self.incremental_planner = incremental_planner # DStarLite
        self.danger_planner = danger_planner           # GNNPlanner (论文2)
        self.local_fail_threshold = int(local_fail_threshold)

        # 双模式调度（论文1+2）
        self.dual_mode = dual_mode
        self.obstacle_field = obstacle_field

        # 风险驱动自适应间隔（论文3）
        self.adaptive_interval = adaptive_interval

        self._current_path: np.ndarray | None = None
        self._global_ref_path: np.ndarray | None = None  # 全局参考路径
        self._last_replan_time: float = -float("inf")
        self._events: list[dict] = []
        self._events_consumed: int = 0  # 外部已消费的事件数

        # 调度状态
        self._phase: str = PHASE_LOCAL
        self._local_fail_count: int = 0
        self._global_fail_count: int = 0     # 全局规划连续失败计数
        self._changed_cells_since_last: list[tuple[tuple, bool]] = []
        self._step_count: int = 0
        self.path_refiner = None
        self._last_fallback_reason: str | None = None
        self._last_path_quality: dict | None = None

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def step(
        self,
        t: float,
        leader_pose: np.ndarray,
        sensor_reading: np.ndarray | None,
        goal: np.ndarray,
    ) -> np.ndarray | None:
        """执行一步重规划逻辑（分层调度 + 双模式）。

        参数
        ----
        t: 当前仿真时间。
        leader_pose: (3,) 领航机位置。
        sensor_reading: (6,) 六向距离，或无传感器时为 None。
        goal: (3,) 目标位置。

        返回
        ----
        新路径 (K,3) 或 None（保持原路径）。
        """
        # 风险驱动自适应间隔
        current_interval = self._get_current_interval()

        if t - self._last_replan_time < current_interval:
            return None

        self._last_replan_time = t
        self._last_fallback_reason = None
        self._last_path_quality = None

        # 目标距离过近（< 2×栅格分辨率），无需重规划
        if np.linalg.norm(goal - leader_pose) < self.grid.resolution * 2:
            return None

        self._step_count += 1
        self._decay_sensor_obstacles()

        # 用传感器数据局部更新 grid（仅 horizon 内）
        changed = []
        if sensor_reading is not None:
            changed = self._update_grid_from_sensor(leader_pose, sensor_reading)
        self._changed_cells_since_last.extend(changed)

        # 更新风险历史
        if self.adaptive_interval is not None:
            sensor_min = float(np.min(sensor_reading)) if sensor_reading is not None and len(sensor_reading) > 0 else float("inf")
            sdf_val = self._get_min_sdf(leader_pose)
            self.adaptive_interval.update(sdf_val, sensor_min)

        # 通知增量规划器网格变更
        if changed and self.incremental_planner is not None:
            self.incremental_planner.update_cells(changed)

        # ---- 双模式检查 ----
        if self.dual_mode is not None and self.danger_planner is not None:
            mode = self.dual_mode.classify(t, sensor_reading, self.obstacle_field, leader_pose)
            if mode == "danger":
                new_path = self._replan_danger(leader_pose, goal)
                if new_path is not None and len(new_path) > 0:
                    self._local_fail_count = 0
                    return self._publish_path(t, new_path)
                if self._last_fallback_reason is None:
                    self._last_fallback_reason = "danger_planner_failed"

        # ---- 分层调度 ----
        new_path = None

        try:
            phase = self._classify_phase(leader_pose, goal)
            self._phase = phase

            if phase == PHASE_GLOBAL and self.global_planner is not None:
                new_path = self._replan_global(leader_pose, goal)
                if new_path is not None and len(new_path) >= 2:
                    self._local_fail_count = 0
                    self._changed_cells_since_last.clear()
                else:
                    self._local_fail_count += 1
            elif phase == PHASE_INCREMENTAL and self.incremental_planner is not None:
                new_path = self._replan_incremental(leader_pose, goal)
                if new_path is not None:
                    self._local_fail_count = 0
                else:
                    self._local_fail_count += 1
            else:
                new_path = self._replan_local(leader_pose, goal)
                if new_path is not None:
                    self._local_fail_count = 0
                else:
                    self._local_fail_count += 1
        except Exception:
            self._local_fail_count += 1
            return None

        # ---- 路径发布 ----
        if new_path is not None and len(new_path) > 0:
            return self._publish_path(t, new_path)

        return None

    # ------------------------------------------------------------------
    # 三级规划
    # ------------------------------------------------------------------

    def _replan_local(self, pose: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """局部层：滑动窗口修正。"""
        subgoal = self._compute_subgoal(pose, goal)
        path = self.planner.plan(pose, subgoal, self.grid)
        return self.planner.smooth(path)

    def _replan_incremental(self, pose: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """增量层：D* Lite 增量更新。"""
        dstar = self.incremental_planner
        if dstar is None:
            return self._replan_local(pose, goal)

        # 更新起点并扩展
        dstar.update_start(pose)
        dstar.compute_shortest_path()

        if not dstar.is_consistent():
            return None

        path = dstar.extract_path()
        if path is None or len(path) < 2:
            return None

        return self.planner.smooth(path)

    def _replan_global(self, pose: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """全局层：Informed RRT* 重建参考路径。"""
        gp = self.global_planner
        if gp is None:
            return self._replan_local(pose, goal)

        try:
            path = gp.plan(pose, goal, self.grid)
            if path is not None and len(path) >= 2:
                self._global_ref_path = path
                self._global_fail_count = 0
                return path
        except Exception:
            pass

        self._global_fail_count += 1
        self._global_ref_path = None
        return self._replan_local(pose, goal)

    # ------------------------------------------------------------------
    # 调度分类
    # ------------------------------------------------------------------

    def _classify_phase(self, pose: np.ndarray, _goal: np.ndarray) -> str:
        """判断当前应采用哪个规划层。

        决策树:
        1. 全局参考尚未建立 → GLOBAL（但连续失败 >3 次则退化到 LOCAL）
        2. 局部连续失败达阈值 → GLOBAL
        3. 传感器变更累积显著 → INCREMENTAL (如果可用) 或 GLOBAL
        4. 全局参考在 horizon 内仍可行 → LOCAL
        5. 大范围地图变更 → GLOBAL
        """
        # 首次或无全局参考 → GLOBAL（连续失败过多则退化到 LOCAL）
        if self.global_planner is not None and self._global_ref_path is None:
            if self._global_fail_count >= 4:
                return PHASE_LOCAL
            return PHASE_GLOBAL

        # 连续失败过多 → 上升到全局
        if self._local_fail_count >= self.local_fail_threshold:
            if self.global_planner is not None and self._global_fail_count < 4:
                return PHASE_GLOBAL

        # 大量网格变更 → INCREMENTAL 或 GLOBAL
        if len(self._changed_cells_since_last) > 20:
            if self.incremental_planner is not None:
                return PHASE_INCREMENTAL
            elif self.global_planner is not None and self._global_fail_count < 4:
                return PHASE_GLOBAL

        # 检查全局参考是否在 horizon 内仍连通
        if self._global_ref_path is not None and len(self._global_ref_path) > 0:
            if not self._ref_path_feasible_in_horizon(pose):
                if self.incremental_planner is not None:
                    return PHASE_INCREMENTAL
                elif self.global_planner is not None and self._global_fail_count < 4:
                    return PHASE_GLOBAL

        # 默认 → LOCAL
        return PHASE_LOCAL

    def _ref_path_feasible_in_horizon(self, pose: np.ndarray) -> bool:
        """检查全局参考路径在 horizon 范围内是否仍可行。

        对参考路径在 horizon 范围内的点做抽样碰撞检查。
        """
        if self._global_ref_path is None:
            return False

        ref = self._global_ref_path
        for i in range(len(ref)):
            if np.linalg.norm(ref[i] - pose) > self.horizon:
                break
            idx = self.grid.world_to_index(ref[i])
            # 该点被新障碍占据 → 不可行
            if self.grid.data[idx] >= 1:
                return False

        return True

    def _compute_subgoal(self, pose: np.ndarray, goal: np.ndarray) -> np.ndarray:
        pose = np.asarray(pose, dtype=float)
        goal = np.asarray(goal, dtype=float)
        dist_to_goal = float(np.linalg.norm(goal - pose))
        if dist_to_goal <= self.horizon and self._segment_is_safe(pose, goal):
            return goal

        candidates = self._candidate_subgoals(pose, goal)
        if candidates:
            best = max(candidates, key=lambda item: item[0])
            return best[1]

        if dist_to_goal > self.horizon:
            direction = (goal - pose) / dist_to_goal
            return pose + direction * self.horizon
        return goal

    def _candidate_subgoals(self, pose: np.ndarray, goal: np.ndarray) -> list[tuple[float, np.ndarray]]:
        candidates: list[tuple[float, np.ndarray]] = []
        seen: set[tuple[float, float, float]] = set()

        def add_candidate(point: np.ndarray) -> None:
            pt = np.asarray(point, dtype=float)
            key = tuple(np.round(pt, 4))
            if key in seen:
                return
            seen.add(key)
            if np.linalg.norm(pt - pose) < self.grid.resolution:
                return
            if np.linalg.norm(pt - pose) > self.horizon + 1e-6:
                return
            if not self._point_is_free(pt):
                return
            if not self._segment_is_safe(pose, pt):
                return
            score = self._score_subgoal(pose, goal, pt)
            candidates.append((score, pt))

        # 1) 优先沿全局参考路径选择当前窗口内的可达点
        if self._global_ref_path is not None and len(self._global_ref_path) > 0:
            for ref_pt in self._global_ref_path:
                if np.linalg.norm(ref_pt - pose) <= self.horizon + 1e-6:
                    add_candidate(ref_pt)

        # 2) 沿直达目标方向采样
        dist_to_goal = float(np.linalg.norm(goal - pose))
        if dist_to_goal > 1e-9:
            direction = (goal - pose) / dist_to_goal
            n_samples = max(3, int(np.ceil(self.horizon / max(self.grid.resolution, 1e-6))))
            for ratio in np.linspace(0.35, 1.0, n_samples):
                add_candidate(pose + direction * min(self.horizon * ratio, dist_to_goal))

        # 3) 在占据栅格邻域中做一圈保守 BFS，补充局部绕行候选
        for point in self._free_frontier_points(pose, goal):
            add_candidate(point)

        return candidates

    def _free_frontier_points(self, pose: np.ndarray, goal: np.ndarray, max_points: int = 24) -> list[np.ndarray]:
        start_idx = self.grid.world_to_index(pose)
        max_steps = max(2, int(np.ceil(self.horizon / max(self.grid.resolution, 1e-6))))
        queue: deque[tuple[tuple[int, int, int], int]] = deque([(start_idx, 0)])
        visited = {start_idx}
        results: list[np.ndarray] = []
        goal_dir = goal - pose
        goal_norm = float(np.linalg.norm(goal_dir))
        if goal_norm > 1e-9:
            goal_dir = goal_dir / goal_norm

        neighbors = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1), (1, 1, 0), (1, -1, 0),
            (-1, 1, 0), (-1, -1, 0),
        ]

        while queue and len(results) < max_points:
            idx, depth = queue.popleft()
            if depth >= max_steps:
                continue
            for dx, dy, dz in neighbors:
                nxt = (idx[0] + dx, idx[1] + dy, idx[2] + dz)
                if nxt in visited:
                    continue
                visited.add(nxt)
                if any(i < 0 or i >= dim for i, dim in zip(nxt, self.grid.shape)):
                    continue
                if self.grid.is_occupied(nxt):
                    continue
                queue.append((nxt, depth + 1))
                point = self.grid.index_to_world(nxt)
                if np.linalg.norm(point - pose) > self.horizon + 1e-6:
                    continue
                if goal_norm > 1e-9:
                    offset = point - pose
                    off_norm = float(np.linalg.norm(offset))
                    if off_norm > 1e-9 and np.dot(offset / off_norm, goal_dir) < 0.15:
                        continue
                results.append(point)
                if len(results) >= max_points:
                    break
        return results

    def _score_subgoal(self, pose: np.ndarray, goal: np.ndarray, candidate: np.ndarray) -> float:
        progress = float(np.linalg.norm(goal - pose) - np.linalg.norm(goal - candidate))
        sdf_val = self._get_min_sdf(candidate)
        clearance = min(max(sdf_val, 0.0), self.horizon)
        path_bonus = 0.0
        if self._global_ref_path is not None and len(self._global_ref_path) > 0:
            min_ref_dist = min(float(np.linalg.norm(candidate - ref_pt)) for ref_pt in self._global_ref_path)
            path_bonus = -0.15 * min_ref_dist
        distance_penalty = 0.05 * float(np.linalg.norm(candidate - pose))
        return progress + 0.35 * clearance + path_bonus - distance_penalty

    def _point_is_free(self, point: np.ndarray) -> bool:
        idx = self.grid.world_to_index(point)
        if self.grid.is_occupied(idx):
            return False
        if self.obstacle_field is not None:
            try:
                return float(self.obstacle_field.signed_distance(point)) >= -1e-6
            except Exception:
                return True
        return True

    def _segment_is_safe(self, start: np.ndarray, goal: np.ndarray) -> bool:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        dist = float(np.linalg.norm(goal - start))
        if dist < 1e-9:
            return True
        direction = goal - start
        n_samples = max(2, int(np.ceil(dist / max(self.grid.resolution * 0.5, 1e-6))))
        for i in range(n_samples + 1):
            t = i / n_samples
            point = start + direction * t
            if not self._point_is_free(point):
                return False
        return True

    def _publish_path(self, t: float, new_path: np.ndarray) -> np.ndarray:
        """偏差检查后发布路径。始终更新 _current_path 返回新路径，
        仅在偏差超过阈值时记录重规划事件，避免因小幅差异而跟随过时航点。
        """
        if self.path_refiner is not None:
            try:
                new_path = self.path_refiner.refine(new_path, seeds=new_path)
            except Exception:
                pass
        if self._current_path is not None and len(new_path) > 0:
            deviation = self._path_deviation(self._current_path, new_path)
            if deviation > self.epsilon:
                event = {
                    "t": float(t),
                    "phase": self._phase,
                    "from": self._current_path[0].tolist() if len(self._current_path) > 0 else [],
                    "to": new_path[0].tolist(),
                    "reason": f"deviation={deviation:.3f}m",
                }
                if self._last_fallback_reason is not None:
                    event["fallback_reason"] = self._last_fallback_reason
                if self._last_path_quality is not None:
                    event["path_quality"] = self._last_path_quality.copy()
                self._events.append(event)
        self._current_path = new_path
        return new_path

    # ------------------------------------------------------------------
    # 传感器地图更新
    # ------------------------------------------------------------------

    def _update_grid_from_sensor(self, pose: np.ndarray, readings: np.ndarray) -> list[tuple[tuple, bool]]:
        """用六向测距结果标记 horizon 内的占据/自由体素。

        返回变更体素列表 [(idx, occupied), ...]。
        """
        directions = np.array([
            [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
        ], dtype=float)
        max_r = self.horizon
        changed = []
        for di, d_vec in enumerate(directions):
            r = min(readings[di], max_r)
            num_steps = max(1, int(r / self.grid.resolution))
            for s in range(num_steps + 1):
                t_step = s * self.grid.resolution
                if t_step > r:
                    break
                pt = pose + d_vec * t_step
                idx = self.grid.world_to_index(pt)
                if any(i < 0 or i >= dim for i, dim in zip(idx, self.grid.shape)):
                    continue
                hit_cell = t_step >= r - self.grid.resolution
                old_val = int(self.grid.data[idx])
                if self._static_occupied[idx]:
                    continue
                if hit_cell:
                    self._sensor_ttl[idx] = self.sensor_obstacle_ttl_steps
                    self._sensor_clear_hits[idx] = 0
                    if old_val != 1 or not self._sensor_occupied[idx]:
                        self.grid.data[idx] = 1
                        self._sensor_occupied[idx] = True
                        changed.append((idx, True))
                elif self._sensor_occupied[idx] and old_val == 1:
                    self._sensor_clear_hits[idx] = min(
                        self.sensor_clear_confirm_steps,
                        int(self._sensor_clear_hits[idx]) + 1,
                    )
                    if self._sensor_clear_hits[idx] >= self.sensor_clear_confirm_steps:
                        self._sensor_ttl[idx] = max(0, int(self._sensor_ttl[idx]) - 1)
                        if self._sensor_ttl[idx] <= 0:
                            self.grid.data[idx] = 0
                            self._sensor_occupied[idx] = False
                            self._sensor_clear_hits[idx] = 0
                            changed.append((idx, False))
        return changed

    def _decay_sensor_obstacles(self) -> None:
        sensor_indices = np.argwhere(self._sensor_occupied)
        for idx_arr in sensor_indices:
            idx = tuple(int(v) for v in idx_arr)
            if self._static_occupied[idx]:
                self._sensor_occupied[idx] = False
                self._sensor_ttl[idx] = 0
                self._sensor_clear_hits[idx] = 0
                continue
            self._sensor_ttl[idx] = max(0, int(self._sensor_ttl[idx]) - 1)
            if self._sensor_ttl[idx] <= 0:
                self.grid.data[idx] = 0
                self._sensor_occupied[idx] = False
                self._sensor_clear_hits[idx] = 0
                self._changed_cells_since_last.append((idx, False))

    # ------------------------------------------------------------------
    # 路径偏差度量
    # ------------------------------------------------------------------

    def _path_deviation(self, old_path: np.ndarray, new_path: np.ndarray) -> float:
        if self.deviation_metric == "hausdorff":
            return self._hausdorff(old_path, new_path)
        elif self.deviation_metric == "dtw":
            return self._dtw(old_path, new_path)
        return self._max_pairwise(old_path, new_path)

    @staticmethod
    def _max_pairwise(old_path: np.ndarray, new_path: np.ndarray) -> float:
        if len(old_path) == 0:
            return float("inf")
        K = min(len(old_path), len(new_path))
        max_dev = 0.0
        for i in range(K):
            dev = np.linalg.norm(old_path[i] - new_path[i])
            max_dev = max(max_dev, dev)
        return max_dev

    @staticmethod
    def _hausdorff(old_path: np.ndarray, new_path: np.ndarray) -> float:
        try:
            from scipy.spatial.distance import directed_hausdorff
        except ImportError:
            return WindowReplanner._max_pairwise(old_path, new_path)
        d_forward = directed_hausdorff(old_path, new_path)[0]
        d_backward = directed_hausdorff(new_path, old_path)[0]
        return float(max(d_forward, d_backward))

    @staticmethod
    def _dtw(old_path: np.ndarray, new_path: np.ndarray, n_resample: int = 100) -> float:
        A = WindowReplanner._resample_path(old_path, n_resample)
        B = WindowReplanner._resample_path(new_path, n_resample)
        n, m = len(A), len(B)
        dtw = np.full((n + 1, m + 1), float("inf"))
        dtw[0, 0] = 0.0
        for i in range(1, n + 1):
            for j in range(1, m + 1):
                cost = np.linalg.norm(A[i - 1] - B[j - 1])
                dtw[i, j] = cost + min(dtw[i - 1, j], dtw[i, j - 1], dtw[i - 1, j - 1])
        return float(dtw[n, m] / max(n, m))

    @staticmethod
    def _resample_path(path: np.ndarray, n_points: int) -> np.ndarray:
        path = np.asarray(path, dtype=float)
        if len(path) < 2:
            if len(path) == 1:
                return np.tile(path, (n_points, 1))
            return np.zeros((n_points, 3))
        diffs = np.diff(path, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
        total = cum_len[-1]
        if total < 1e-12:
            return np.tile(path[0], (n_points, 1))
        sample_lens = np.linspace(0, total, n_points)
        result = np.zeros((n_points, 3))
        for d in range(3):
            result[:, d] = np.interp(sample_lens, cum_len, path[:, d])
        return result

    # ------------------------------------------------------------------
    # 双模式 / 风险驱动
    # ------------------------------------------------------------------

    def _get_current_interval(self) -> float:
        """返回当前应使用的重规划间隔。"""
        if self.adaptive_interval is not None:
            return self.adaptive_interval.interval
        return self.base_interval

    def _get_min_sdf(self, position: np.ndarray) -> float:
        """获取当前位置的最小 SDF 值。"""
        if self.obstacle_field is None:
            return float("inf")
        try:
            return float(self.obstacle_field.signed_distance(position))
        except Exception:
            return float("inf")

    def _replan_danger(self, pose: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """Danger 模式：使用 GNN+可见图规划（论文2），惰性构建可见图，复用更新起终点。"""
        if self.danger_planner is None:
            return self._replan_local(pose, goal)

        try:
            # 惰性构建可见图（首次 Danger 触发时，避免 init 长时间阻塞）
            cached_vg = getattr(self.danger_planner, '_cached_vis_graph', None)
            if cached_vg is None and hasattr(self.danger_planner, '_lazy_obstacles'):
                from core.planning.visibility_graph import VisibilityGraph
                vg = VisibilityGraph(
                    angular_res=self.danger_planner._lazy_angular_res,
                    buffer_zone=self.danger_planner._lazy_buffer_zone,
                )
                vg._build_obstacle_graph(
                    self.danger_planner._lazy_obstacles,
                    visible_range=self.danger_planner._lazy_visible_range,
                )
                self.danger_planner._cached_vis_graph = vg
                cached_vg = vg

            if cached_vg is not None and len(cached_vg.vertices) > 0:
                visible_range = self.horizon * 4
                cached_vg.set_start_goal(pose, goal, self.obstacle_field, visible_range)
                path = self.danger_planner.plan(
                    pose, goal, self.grid,
                    visibility_graph=cached_vg,
                )
            else:
                path = self.danger_planner.plan(
                    pose, goal, self.grid,
                    obstacle_field=self.obstacle_field,
                )
            path = self.danger_planner.smooth(path)
            if not self._path_segments_visible(path):
                self._last_fallback_reason = "danger_path_not_visible"
                return None
            quality = self._evaluate_path_quality(pose, goal, path)
            self._last_path_quality = quality
            if quality["ratio_to_local"] > self.gnn_path_ratio_limit:
                self._last_fallback_reason = "danger_path_too_long"
                return None
            return path
        except Exception:
            self._last_fallback_reason = "danger_exception"
            return self._replan_local(pose, goal)

    def _resolve_obstacle_field(self, pose: np.ndarray, sensor_reading: np.ndarray | None) -> None:
        """利用传感器读数更新障碍物场的局部视图（Danger 模式辅助）。"""
        # 当前版本委托给 _update_grid_from_sensor；未来可扩展动态障碍物注入
        if sensor_reading is not None:
            self._update_grid_from_sensor(pose, sensor_reading)

    def _path_segments_visible(self, path: np.ndarray) -> bool:
        path = np.asarray(path, dtype=float)
        if len(path) < 2:
            return False
        for left, right in zip(path[:-1], path[1:]):
            if not self._segment_is_safe(left, right):
                return False
        return True

    @staticmethod
    def _path_length(path: np.ndarray) -> float:
        path = np.asarray(path, dtype=float)
        if len(path) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    def _evaluate_path_quality(self, pose: np.ndarray, goal: np.ndarray, path: np.ndarray) -> dict:
        path_len = self._path_length(path)
        local_path = self._replan_local(pose, goal)
        local_len = self._path_length(local_path) if local_path is not None and len(local_path) > 1 else path_len
        return {
            "planner": "danger",
            "path_length": path_len,
            "local_length": local_len,
            "ratio_to_local": path_len / max(local_len, 1e-6),
        }

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def events(self) -> list[dict]:
        return self._events

    def get_new_events(self) -> list[dict]:
        """返回自上次调用以来新增的事件，供外部消费。"""
        new_events = self._events[self._events_consumed:]
        self._events_consumed = len(self._events)
        return new_events

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def last_fallback_reason(self) -> str | None:
        return self._last_fallback_reason

    @property
    def last_path_quality(self) -> dict | None:
        return self._last_path_quality
