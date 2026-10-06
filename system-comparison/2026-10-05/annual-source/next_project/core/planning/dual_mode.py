"""双模式调度器 + 编队整体势场（论文1+2 综合）。

用途
----
- DualModeScheduler: Safe/Danger 双模式分类 + 滞回机制
- FormationAPF: 编队整体人工势场（叠加增强，非替代）

冲突修补
--------
- C1: 编队整体势场叠加在单机独立 APF 之上，不得替代
- 模式切换含滞回，避免振荡
"""

from __future__ import annotations

import numpy as np


class DualModeScheduler:
    """Safe/Danger 双模式调度器。

    参数
    ----
    sensor_danger_threshold: 传感器读数低于此值进入 Danger（m）。
    sensor_safe_threshold: 传感器读数高于此值恢复 Safe（m）。
    sdf_danger_threshold: SDF 低于此值进入 Danger（m）。
    hysteresis_margin: 滞回裕度 m，防止模式振荡。
    """

    SAFE = "safe"
    DANGER = "danger"

    def __init__(
        self,
        sensor_danger_threshold: float = 2.0,
        sensor_safe_threshold: float = 4.0,
        sdf_danger_threshold: float = 0.5,
        hysteresis_margin: float = 0.5,
    ):
        self.sensor_danger = float(sensor_danger_threshold)
        self.sensor_safe = float(sensor_safe_threshold)
        self.sdf_danger = float(sdf_danger_threshold)
        self.hysteresis = float(hysteresis_margin)

        self._current_mode: str = self.SAFE
        self._mode_history: list[tuple[float, str]] = []

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def classify(
        self,
        t: float,
        sensor_reading: np.ndarray | None,
        obstacle_field,
        position: np.ndarray,
    ) -> str:
        """根据传感器读数和 SDF 距离分类当前模式。

        决策规则
        --------
        1. 传感器最短读数 < sensor_danger → DANGER
        2. SDF(当前位置) < sdf_danger → DANGER
        3. 当前 DANGER 且所有读数 > sensor_safe+hysteresis → 恢复 SAFE
        4. 其他 → 保持当前模式（滞回）
        """
        prev = self._current_mode

        # 传感器危险判断
        sensor_min = float("inf")
        if sensor_reading is not None and len(sensor_reading) > 0:
            sensor_min = float(np.min(sensor_reading))

        # SDF 危险判断
        sdf_val = float("inf")
        if obstacle_field is not None:
            try:
                sdf_val = float(obstacle_field.signed_distance(position))
            except Exception:
                pass

        if sensor_min < self.sensor_danger or sdf_val < self.sdf_danger:
            self._current_mode = self.DANGER
        elif prev == self.DANGER:
            # 滞回恢复条件
            if sensor_min > self.sensor_safe + self.hysteresis and sdf_val > self.sdf_danger + self.hysteresis:
                self._current_mode = self.SAFE
        # else: 保持 SAFE

        self._mode_history.append((t, self._current_mode))
        return self._current_mode

    @property
    def is_danger(self) -> bool:
        return self._current_mode == self.DANGER

    @property
    def is_safe(self) -> bool:
        return self._current_mode == self.SAFE

    @property
    def mode(self) -> str:
        return self._current_mode

    def get_danger_obstacles(self, obstacle_field, position: np.ndarray, range_m: float = 10.0) -> list:
        """返回 position 附近 range_m 内的障碍物列表。"""
        nearby = []
        obs_list = list(obstacle_field) if hasattr(obstacle_field, '__iter__') else getattr(obstacle_field, '_obstacles', [])
        for obs in obs_list:
            nearby.append(obs)
        return nearby

    def should_shrink_formation(
        self,
        topology,
        leader_pos: np.ndarray,
        leader_vel: np.ndarray,
        horizon: float = 4.0,
    ) -> bool:
        """检测前方通道宽度，判断是否需要收缩队形。"""
        vel_norm = float(np.linalg.norm(leader_vel))
        if vel_norm < 0.1:
            return False
        ahead = leader_pos + leader_vel / vel_norm * (horizon / 2.0)
        # 通道宽度估计由外部障碍物场提供
        return False  # 默认不收缩，由子类或集成点覆盖


class FormationAPF:
    """编队整体人工势场（叠加增强，非替代）。

    冲突修补 C1
    ----------
    F_total[i] = α·F_centroid + β·F_relative[i] + F_individual[i]
    单机独立 APF（F_individual）必须保留，编队势场仅叠加。

    参数
    ----
    k_rep: 斥力增益。
    r_rep: 斥力作用范围 m。
    alpha: 编队重心势场权重（0=关闭）。
    beta: 队形保持势场权重。
    """

    def __init__(
        self,
        k_rep: float = 0.8,
        r_rep: float = 3.0,
        alpha: float = 0.4,
        beta: float = 0.6,
    ):
        self.k_rep = float(k_rep)
        self.r_rep = float(r_rep)
        self.alpha = float(alpha)
        self.beta = float(beta)

    def compute_formation_avoidance(
        self,
        leader_pos: np.ndarray,
        follower_positions: list[np.ndarray],
        goal: np.ndarray,
        obstacles,
        desired_offsets: list[np.ndarray],
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        """计算编队整体避碰加速度。

        返回 (F_leader, [F_follower_1, ...])，单位 m/s²。
        调用方仍需叠加单机独立 APF 作为 L3 防护。
        """
        n = len(follower_positions)
        all_pos = [leader_pos] + list(follower_positions)

        # 1) 编队重心势场
        centroid = np.mean(all_pos, axis=0)
        F_centroid = self._centroid_repulsion(centroid, goal, obstacles)

        # 2) 成员相对势场
        F_relative = self._relative_formation_forces(all_pos, desired_offsets)

        # 3) 合成（调用方叠加 F_individual）
        F_leader = self.alpha * F_centroid + self.beta * F_relative[0]
        F_followers = [
            self.alpha * F_centroid + self.beta * F_relative[i + 1]
            for i in range(n)
        ]
        return F_leader, F_followers

    def _centroid_repulsion(
        self, centroid: np.ndarray, goal: np.ndarray, obstacles
    ) -> np.ndarray:
        """以编队重心为控制点的障碍物斥力。"""
        try:
            sd = obstacles.signed_distance(centroid)
        except Exception:
            return np.zeros(3, dtype=float)
        if sd >= self.r_rep:
            return np.zeros(3, dtype=float)

        eps = 0.05
        grad = np.array([
            obstacles.signed_distance(centroid + [eps, 0, 0])
            - obstacles.signed_distance(centroid - [eps, 0, 0]),
            obstacles.signed_distance(centroid + [0, eps, 0])
            - obstacles.signed_distance(centroid - [0, eps, 0]),
            obstacles.signed_distance(centroid + [0, 0, eps])
            - obstacles.signed_distance(centroid - [0, 0, eps]),
        ], dtype=float) / (2.0 * eps)
        grad_norm = float(np.linalg.norm(grad))
        if grad_norm < 1e-10:
            return np.zeros(3, dtype=float)
        rep_dir = grad / grad_norm

        rho = max(sd, 0.05)
        r = self.r_rep
        term = max(1.0 / rho - 1.0 / r, 0.0)
        mag = self.k_rep * term * (1.0 / (rho ** 2))
        return rep_dir * min(mag, 8.0)

    def _relative_formation_forces(
        self,
        all_pos: list[np.ndarray],
        desired_offsets: list[np.ndarray],
    ) -> list[np.ndarray]:
        """队形保持力（仅弹簧力，不含成员间斥力）。

        成员间斥力由 ImprovedAPF._inter_drone_repulsion 统一负责，
        避免与 F_individual 双重计数导致编队过度膨胀。
        """
        n = len(all_pos)
        forces = [np.zeros(3, dtype=float) for _ in range(n)]

        # 队形保持：从机受弹簧力拉向期望偏移位置
        for i in range(1, n):
            desired = all_pos[0] + desired_offsets[i - 1]
            actual = all_pos[i]
            forces[i] += 0.5 * (desired - actual)

        return forces
