"""改进人工势场法（Improved APF）避碰模块。

用途
----
为四旋翼无人机编队提供避碰力：障碍物斥力（含目标距离衰减）、
机间斥力、局部极小值检测与逃逸调控力。

原理（参考论文 4.2.2 节）
----
1) 传统 APF 的缺陷：
   - 不可达问题：目标附近有障碍物时斥力 > 引力，无法到达目标
   - 局部极小值：引力与斥力平衡时陷入陷阱

2) 改进措施：
   a) 斥力势函数引入目标距离项 (X-X_d)^n，n 为衰减指数。
      越接近目标，障碍物斥力越小，保证目标可达。
   b) 局部极小值检测：|F_r|==|F_a| 且夹角 ≈ π 时触发，
      施加垂直于原斥力方向的调控力打破平衡。
   c) 机间斥力：编队成员之间设置排斥势场防止内部碰撞。

3) 核心公式（注意：F_r1 与 F_r2 方向不同）：
   斥力（障碍物）：F_r = F_r1 + F_r2
     F_r1 = k_r·(1/ρ-1/r)·(1/ρ²)·|X-X_d|^n · n_obs    (沿障碍物外法向)
     F_r2 = n/2·k_r·(1/ρ-1/r)²·|X-X_d|^(n-1) · n_goal  (沿目标方向)
   其中 n_obs = -∇ρ/|∇ρ|（远离障碍物），n_goal = (X_d-X)/|X_d-X|（指向目标）
   机间斥力：F_r3 = Σ k_inter·(1/p_i-1/s)·(1/p_i²)  (p_i ≤ s)
   逃逸调控力：F_c = μ·(F_r + F_r')  其中 F_r' ⊥ F_r
   总力：F_total = F_r + F_r3 (+ F_c 若处于局部极小值)

4) 力以加速度 (m/s²) 形式输出，叠加到控制器 target_acc 前馈。

"""

from __future__ import annotations

import numpy as np


class ImprovedArtificialPotentialField:
    """改进人工势场法避碰控制器。

    参数
    ----
    k_rep:        障碍物斥力增益
    r_rep:        斥力场作用范围 m
    n_decay:      斥力衰减指数（目标距离调制，越大在目标附近衰减越快）
    k_inter:      机间斥力增益
    s_inter:      机间斥力作用范围 m
    mu_escape:    逃逸调控系数 (0, 1)
    escape_angle_threshold: 局部极小值检测的角度阈值 rad
    escape_history: 逃逸力保持步数，防止在极小值附近振荡
    """

    def __init__(
        self,
        k_rep: float = 0.8,
        r_rep: float = 3.0,
        n_decay: int = 2,
        k_inter: float = 0.5,
        s_inter: float = 2.0,
        mu_escape: float = 0.5,
        max_acc: float = 8.0,
        escape_angle_threshold: float = 0.94 * np.pi,
        escape_history: int = 20,
        adaptive_n_decay: bool = False,
        k_comm: float = 0.3,
        comm_range: float = 10.0,
    ):
        self.k_rep = float(k_rep)
        self.r_rep = float(r_rep)
        self.n_decay = int(n_decay)
        self.k_inter = float(k_inter)
        self.s_inter = float(s_inter)
        self.mu_escape = float(mu_escape)
        self.max_acc = float(max_acc)
        self.escape_angle_threshold = float(escape_angle_threshold)
        self.escape_history = int(escape_history)

        # 论文1 新增参数
        self.adaptive_n_decay = bool(adaptive_n_decay)
        self.k_comm = float(k_comm)          # m/s²，通信约束力增益
        self.comm_range = float(comm_range)  # m

        # 逃逸状态跟踪
        self._in_escape: bool = False
        self._escape_counter: int = 0
        self._escape_direction: np.ndarray = np.zeros(3, dtype=float)
        self._prev_total_repulsion: np.ndarray = np.zeros(3, dtype=float)

        # n_decay 自适应缓存（冲突修补 C4）
        self._density_cache: dict = {}

    def reset(self) -> None:
        """重置逃逸状态和密度缓存。"""
        self._in_escape = False
        self._escape_counter = 0
        self._escape_direction = np.zeros(3, dtype=float)
        self._prev_total_repulsion = np.zeros(3, dtype=float)
        self._density_cache.clear()

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def compute_avoidance_acceleration(
        self,
        position: np.ndarray,
        goal: np.ndarray,
        obstacles,
        other_positions: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        """计算单架无人机的总避碰加速度 (m/s²)。

        参数
        ----
        position: (3,) 当前位置
        goal: (3,) 目标位置（用于斥力衰减调制）
        obstacles: ObstacleField 实例
        other_positions: 编队其他成员位置列表（用于机间斥力）

        返回
        ----
        (3,) 加速度向量 m/s²
        """
        # 1) 障碍物斥力（含目标距离衰减）
        f_rep = self._obstacle_repulsion(position, goal, obstacles)

        # 2) 机间斥力
        f_inter = np.zeros(3, dtype=float)
        if other_positions:
            f_inter = self._inter_drone_repulsion(position, other_positions)

        # 3) 局部极小值检测与逃逸
        f_escape = np.zeros(3, dtype=float)
        total_rep = f_rep + f_inter
        rep_norm = float(np.linalg.norm(total_rep))

        if rep_norm > 1e-6:
            # 计算引力方向（指向目标）
            to_goal = goal - position
            goal_dist = float(np.linalg.norm(to_goal))
            if goal_dist > 1e-6:
                f_att_dir = to_goal / goal_dist
                rep_dir = total_rep / rep_norm

                # 检测局部极小值条件：斥力与引力反向
                cos_angle = float(np.dot(rep_dir, f_att_dir))
                if cos_angle < np.cos(self.escape_angle_threshold) and rep_norm > 1.0:
                    self._enter_escape(total_rep)
                else:
                    self._in_escape = False
                    self._escape_counter = 0

        # 若处于逃逸状态，产生调控力
        if self._in_escape and self._escape_counter < self.escape_history:
            f_escape = self._compute_escape_force(total_rep)
            self._escape_counter += 1
        else:
            self._in_escape = False
            self._escape_counter = 0

        # 4) 通信约束力（论文1 §2.3，量纲 m/s²，前馈注入）
        f_comm = np.zeros(3, dtype=float)
        if other_positions and self.k_comm > 0:
            f_comm = self._communication_constraint_force(position, other_positions)

        self._prev_total_repulsion = total_rep
        result = total_rep + f_escape + f_comm
        # 总量钳制
        result_norm = float(np.linalg.norm(result))
        if result_norm > self.max_acc:
            result = result / result_norm * self.max_acc
        return result

    # ------------------------------------------------------------------
    # 障碍物斥力（改进版：含目标距离衰减）
    # ------------------------------------------------------------------

    def _obstacle_repulsion(
        self, position: np.ndarray, goal: np.ndarray, obstacles
    ) -> np.ndarray:
        """计算障碍物对无人机的斥力加速度 (m/s²)。

        基于式 (4-15)(4-16)(4-17)，但将目标距离归一化到斥力范围：
        decay = (goal_dist / r_rep)^n，确保：
        - 远离目标 (goal_dist ≈ r_rep): decay ≈ 1，斥力全开
        - 接近目标 (goal_dist → 0): decay → 0，斥力衰减让位引力

        F_r1 沿障碍物外法向（远离障碍物），F_r2 沿目标方向（指向目标）。
        靠近目标时 F_r2 抵消部分 F_r1，保证目标可达性。
        """
        sd = obstacles.signed_distance(position)
        if sd >= self.r_rep:
            return np.zeros(3, dtype=float)

        # SDF 梯度方向（远离障碍物）
        eps = 0.05
        grad = np.array([
            obstacles.signed_distance(position + [eps, 0, 0])
            - obstacles.signed_distance(position - [eps, 0, 0]),
            obstacles.signed_distance(position + [0, eps, 0])
            - obstacles.signed_distance(position - [0, eps, 0]),
            obstacles.signed_distance(position + [0, 0, eps])
            - obstacles.signed_distance(position - [0, 0, eps]),
        ], dtype=float) / (2.0 * eps)
        grad_norm = float(np.linalg.norm(grad))
        if grad_norm < 1e-10:
            return np.zeros(3, dtype=float)
        rep_dir = grad / grad_norm

        rho = max(sd, 0.05)  # 到障碍物距离
        r = self.r_rep
        goal_dist = float(np.linalg.norm(goal - position))

        # 自适应衰减指数（冲突修补 C4：带缓存）
        n = self._adaptive_n_decay(position, obstacles) if self.adaptive_n_decay else self.n_decay

        # 归一化目标距离因子：goal_dist/r_rep ∈ [0, ~map_size/r_rep]
        # 钳制在 [0, 1]：当 goal_dist ≥ r_rep 时因子饱和为 1
        goal_factor = min(goal_dist / r, 1.0)
        decay = goal_factor ** n

        # F_r1: 基本斥力（含目标距离调制），沿障碍物外法向 — 式(4-16)
        term = max(1.0 / rho - 1.0 / r, 0.0)
        f_r1_mag = self.k_rep * term * (1.0 / (rho ** 2)) * decay
        f_r1_vec = rep_dir * f_r1_mag

        # F_r2: 目标距离衰减梯度项，沿目标方向（抵消部分斥力确保目标可达）— 式(4-17)
        if n >= 1 and goal_dist < r:
            f_r2_mag = 0.5 * n * self.k_rep * (term ** 2) * (goal_factor ** max(n - 1, 0))
            goal_dir = (goal - position) / max(goal_dist, 1e-6)
            f_r2_vec = goal_dir * f_r2_mag
        else:
            f_r2_vec = np.zeros(3, dtype=float)

        f_rep = f_r1_vec + f_r2_vec
        # 钳制到最大加速度
        f_rep_norm = float(np.linalg.norm(f_rep))
        if f_rep_norm > self.max_acc:
            f_rep = f_rep / f_rep_norm * self.max_acc
        return f_rep

    # ------------------------------------------------------------------
    # 机间斥力
    # ------------------------------------------------------------------

    def _inter_drone_repulsion(
        self, position: np.ndarray, other_positions: list[np.ndarray]
    ) -> np.ndarray:
        """计算编队内部无人机之间的排斥力。式(4-20)。

        力以加速度 (m/s²) 输出，钳制到 max_acc 避免干扰编队保持。
        """
        total_force = np.zeros(3, dtype=float)
        s = self.s_inter

        for other_pos in other_positions:
            diff = position - other_pos
            p = float(np.linalg.norm(diff))
            if p < 1e-6 or p >= s:
                continue

            # F = k * (1/p - 1/s) * (1/p²) * direction
            direction = diff / p
            magnitude = self.k_inter * (1.0 / p - 1.0 / s) * (1.0 / (p ** 2))
            total_force += direction * magnitude

        # 钳制机间斥力总幅值
        total_norm = float(np.linalg.norm(total_force))
        if total_norm > self.max_acc * 0.5:
            total_force = total_force / total_norm * (self.max_acc * 0.5)

        return total_force

    # ------------------------------------------------------------------
    # 论文1: n_decay 自适应衰减指数（冲突修补 C4：带空间缓存）
    # ------------------------------------------------------------------

    def _adaptive_n_decay(self, position: np.ndarray, obstacles) -> float:
        """根据局部障碍密度自适应调节衰减指数。

        n = 1 + 3 * (local_density / max_density)
        稀疏环境 n≈1：弱衰减，更安全
        密集环境 n≈4：强衰减，确保目标可达

        缓存策略：位置距上次采样点 < r_rep/4 时复用；每 K=10 步强制重算。
        """
        agent_id = 0  # 单机 APF 时固定为 0
        cache = self._density_cache.get(agent_id)
        if cache is not None:
            last_pos, last_density, step = cache
            if (np.linalg.norm(position - last_pos) < self.r_rep / 4.0
                    and step < 10):
                self._density_cache[agent_id] = (last_pos, last_density, step + 1)
                return 1.0 + 3.0 * min(last_density, 1.0)
        local_density = self._estimate_local_obstacle_density(position, obstacles)
        self._density_cache[agent_id] = (position.copy(), local_density, 0)
        return 1.0 + 3.0 * min(local_density, 1.0)

    def _estimate_local_obstacle_density(self, position: np.ndarray, obstacles) -> float:
        """在 r_rep 半径内采样，统计 SDF < r_rep 的采样点比例。"""
        r = self.r_rep
        # 3×3×3 立方体网格采样
        samples_per_axis = 3
        total = 0
        blocked = 0
        step = 2.0 * r / (samples_per_axis - 1) if samples_per_axis > 1 else 1.0
        for i in range(samples_per_axis):
            for j in range(samples_per_axis):
                for k in range(samples_per_axis):
                    offset = np.array([
                        -r + i * step,
                        -r + j * step,
                        -r + k * step,
                    ], dtype=float)
                    pt = position + offset
                    total += 1
                    try:
                        if obstacles.signed_distance(pt) < r:
                            blocked += 1
                    except Exception:
                        pass
        return blocked / total if total > 0 else 0.0

    # ------------------------------------------------------------------
    # 论文1: 通信约束惩罚项（冲突修补 C2：量纲固化 + tanh 平滑）
    # ------------------------------------------------------------------

    def _communication_constraint_force(
        self, position: np.ndarray, other_positions: list[np.ndarray]
    ) -> np.ndarray:
        """通信距离约束力。

        量纲：m/s²，作为 target_acc 前馈注入，禁止叠加 target_pos。
        上限：||F_comm|| ≤ k_comm * n_others（运行时 assert 保护）。

        平滑门控（避免阶跃）：
            gate = 0.5 * (1 + tanh((dist/comm_range - 0.8) / 0.05))
        """
        force = np.zeros(3, dtype=float)
        for other in other_positions:
            diff = other - position
            dist = float(np.linalg.norm(diff))
            if dist < 1e-6:
                continue
            gate = 0.5 * (1.0 + np.tanh((dist / self.comm_range - 0.8) / 0.05))
            # dist/comm_range 钳制到 [0, 1]，确保 ||F_comm|| ≤ k_comm
            force += self.k_comm * gate * min(1.0, dist / self.comm_range) * (diff / dist)
        n_others = len(other_positions)
        assert float(np.linalg.norm(force)) <= self.k_comm * n_others + 1e-6, \
            f"F_comm={float(np.linalg.norm(force)):.4f} 超过 k_comm*n={self.k_comm * n_others:.4f}，违反量纲断言"
        return force  # m/s²

    # ------------------------------------------------------------------
    # 论文1: 旋转力场替代 Gram-Schmidt（冲突修补 C6：Rodrigues 公式）
    # ------------------------------------------------------------------

    def _compute_rotational_escape(
        self, total_repulsion: np.ndarray, position: np.ndarray, goal: np.ndarray
    ) -> np.ndarray:
        """旋转势场逃逸力（Rodrigues 公式实现）。

        F_rot = mu_escape * (rep_dir*cos(θ) + cross(rot_axis, rep_dir)*sin(θ))
        θ ≈ π/2，rot_axis = rep_dir × to_goal / ||·||

        验证：dot(F_rot, total_repulsion) ≈ 0（真正切向力）。
        """
        rep_norm = float(np.linalg.norm(total_repulsion))
        if rep_norm < 1e-9:
            return np.zeros(3, dtype=float)
        rep_dir = total_repulsion / rep_norm
        to_goal = goal - position
        goal_norm = float(np.linalg.norm(to_goal))
        if goal_norm < 1e-9:
            return np.zeros(3, dtype=float)
        rot_axis = np.cross(rep_dir, to_goal / goal_norm)
        if float(np.linalg.norm(rot_axis)) < 1e-6:
            rot_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        rot_axis /= float(np.linalg.norm(rot_axis))
        theta = 0.5 * np.pi
        tangent = rep_dir * np.cos(theta) + np.cross(rot_axis, rep_dir) * np.sin(theta)
        return self.mu_escape * tangent  # m/s²

    # ------------------------------------------------------------------
    # 局部极小值逃逸（使用 Rodrigues 旋转力场）
    # ------------------------------------------------------------------

    def _enter_escape(self, total_repulsion: np.ndarray) -> None:
        """进入逃逸模式。使用 Rodrigues 公式替代 Gram-Schmidt（C6 修补）。"""
        if self._in_escape:
            return
        self._in_escape = True
        self._escape_counter = 0
        # 逃逸方向由 _compute_escape_force 通过 Rodrigues 公式实时计算

    def _compute_escape_force(self, total_repulsion: np.ndarray) -> np.ndarray:
        """计算逃逸调控力（使用 Rodrigues 旋转力场）。"""
        rep_norm = float(np.linalg.norm(total_repulsion))
        if rep_norm < 1e-6:
            return np.zeros(3, dtype=float)

        rep_dir = total_repulsion / rep_norm
        esc = self._escape_direction

        # 若逃逸方向未初始化或与斥力平行，找一个正交方向
        if float(np.linalg.norm(esc)) < 1e-9 or abs(float(np.dot(esc, rep_dir))) > 0.99:
            if abs(rep_dir[0]) < 0.9:
                basis = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                basis = np.array([0.0, 1.0, 0.0], dtype=float)
            esc = basis - np.dot(basis, rep_dir) * rep_dir
            esc_norm = float(np.linalg.norm(esc))
            if esc_norm < 1e-9:
                esc = np.cross(rep_dir, np.array([0.0, 0.0, 1.0], dtype=float))
                esc_norm = float(np.linalg.norm(esc))
            if esc_norm > 1e-9:
                esc /= esc_norm
            self._escape_direction = esc

        # Rodrigues 旋转：绕斥力方向旋转逃逸方向（随时间变化）
        angle = 0.3 * self._escape_counter
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        rotated_esc = cos_a * esc + sin_a * np.cross(rep_dir, esc)

        f_perp = rep_norm * rotated_esc
        return self.mu_escape * (total_repulsion + f_perp)
