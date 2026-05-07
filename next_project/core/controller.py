"""串级控制器。

用途
----
为无人机提供位置环、姿态环和混合姿态环控制，支持 PID 与 SMC 的统一调用接口。

原理
----
采用经典串级结构：位置环 -> 速度环 -> 姿态环 -> 角速度环。

1) 位置环（PI）计算期望速度：

    e_p = p_ref - p
    v_ref = v_ff + Kp_pos * e_p + Ki_pos * integral(e_p)

2) 速度环（PID）计算期望加速度：

    e_v = v_ref - v
    a_ref = a_ff + Kp_vel * e_v + Ki_vel * integral(e_v) + Kd_vel * d(e_v)/dt

3) 推力计算：

    T = m * ||a_ref + g * e3||

4) 由推力方向求期望姿态（给定目标偏航 yaw_ref）：

    b3_ref = (a_ref + g * e3) / ||a_ref + g * e3||

    再将 b3_ref 与 yaw_ref 映射为 [roll_ref, pitch_ref, yaw_ref]。

5) 姿态环与角速度环：

    e_eta = eta_ref - eta
    omega_ref = Kp_att * e_eta
    e_omega = omega_ref - omega
    tau = I * (Kp_rate * e_omega + Ki_rate * integral(e_omega) + Kd_rate * d(e_omega)/dt)


"""

from __future__ import annotations

import numpy as np

from .smc import SecondOrderSMC


class Controller:
    """串级 PID 控制器。"""

    def __init__(self, m: float = 1.0, g: float = 9.81, inertia=None, dt: float = 0.01):
        self.I = np.diag([0.01, 0.01, 0.02]) if inertia is None else np.array(inertia, dtype=float)
        self.m = float(m)
        self.g = float(g)
        self.dt = float(dt)

        self.kp_pos = np.array([1.9263, 1.9320, 2.3606], dtype=float)
        self.ki_pos = np.array([0.5982, 0.5982, 0.6183], dtype=float)
        self.kp_vel = np.array([3.6808, 3.2772, 3.2623], dtype=float)
        self.ki_vel = np.array([0.8264, 0.8244, 0.8212], dtype=float)
        self.kd_vel = np.array([1.5675, 1.5856, 1.6716], dtype=float)
        self.kp_att = np.array([8.8343, 9.2681, 4.1071], dtype=float)
        self.kp_rate = np.array([2.5789, 2.5789, 1.7014], dtype=float)
        self.ki_rate = np.array([0.7444, 0.7444, 0.7185], dtype=float)
        self.kd_rate = np.array([0.8544, 0.8544, 0.7710], dtype=float)

        self.integral_pos = np.zeros(3, dtype=float)
        self.integral_vel = np.zeros(3, dtype=float)
        self.integral_rate = np.zeros(3, dtype=float)
        self.prev_vel_err = np.zeros(3, dtype=float)
        self.prev_rate_err = np.zeros(3, dtype=float)
        self.last_des_att = np.zeros(3, dtype=float)

        self.max_acc = 10.0
        self.max_tilt = np.deg2rad(30.0)
        self.max_vel = 10.0
        self.max_des_rate = np.deg2rad(220.0)
        self.integral_pos_limit = 2.0
        self.integral_vel_limit = 2.0
        self.integral_rate_limit = 1.0

    def compute_position_loop(
        self,
        state,
        target_pos,
        target_vel=None,
        target_acc=None,
        target_yaw: float = 0.0,
        target_yaw_rate: float = 0.0,
    ):
        del target_yaw_rate
        if target_vel is None:
            target_vel = np.zeros(3, dtype=float)
        if target_acc is None:
            target_acc = np.zeros(3, dtype=float)

        pos = state[0:3]
        vel = state[3:6]
        dt = self.dt

        pos_err = target_pos - pos
        self.integral_pos += pos_err * dt
        self.integral_pos = np.clip(self.integral_pos, -self.integral_pos_limit, self.integral_pos_limit)
        des_vel = target_vel + self.kp_pos * pos_err + self.ki_pos * self.integral_pos
        des_vel = np.clip(des_vel, -self.max_vel, self.max_vel)

        vel_err = des_vel - vel
        self.integral_vel += vel_err * dt
        self.integral_vel = np.clip(self.integral_vel, -self.integral_vel_limit, self.integral_vel_limit)
        vel_deriv = (vel_err - self.prev_vel_err) / dt if dt > 0 else np.zeros(3, dtype=float)
        self.prev_vel_err = vel_err.copy()

        des_acc = target_acc + self.kp_vel * vel_err + self.ki_vel * self.integral_vel + self.kd_vel * vel_deriv
        acc_norm = np.linalg.norm(des_acc)
        if acc_norm > self.max_acc:
            des_acc = des_acc / acc_norm * self.max_acc

        thrust_vec = self.m * (des_acc + np.array([0.0, 0.0, self.g], dtype=float))
        return self._limited_thrust_and_attitude(
            thrust_vec,
            target_yaw=target_yaw,
            max_thrust_factor=2.0,
        )

    def _limited_thrust_and_attitude(
        self,
        thrust_vec: np.ndarray,
        target_yaw: float = 0.0,
        max_thrust_factor: float = 2.0,
    ) -> tuple[float, np.ndarray]:
        """Convert desired world-frame force to a tilt-limited attitude and consistent thrust."""
        thrust_vec = np.asarray(thrust_vec, dtype=float)
        thrust_mag = float(np.linalg.norm(thrust_vec))
        if thrust_mag < 1e-6:
            thrust_norm = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            thrust_norm = thrust_vec / thrust_mag

        sy = np.sin(target_yaw)
        cy = np.cos(target_yaw)
        roll_arg = np.clip(thrust_norm[0] * sy - thrust_norm[1] * cy, -1.0, 1.0)
        des_roll = np.arcsin(roll_arg)
        cos_roll = np.cos(des_roll)
        if abs(cos_roll) > 1e-6:
            pitch_arg = (thrust_norm[0] * cy + thrust_norm[1] * sy) / cos_roll
            pitch_arg = np.clip(pitch_arg, -1.0, 1.0)
            des_pitch = np.arcsin(pitch_arg)
        else:
            des_pitch = 0.0

        des_roll = float(np.clip(des_roll, -self.max_tilt, self.max_tilt))
        des_pitch = float(np.clip(des_pitch, -self.max_tilt, self.max_tilt))
        b3_limited = self._body_z_axis(des_roll, des_pitch, target_yaw)

        vertical_force = max(float(thrust_vec[2]), 0.1 * self.m * self.g)
        if b3_limited[2] > 1e-6:
            T_thrust = vertical_force / b3_limited[2]
        else:
            T_thrust = thrust_mag
        T_thrust = float(np.clip(
            T_thrust,
            0.1 * self.m * self.g,
            max_thrust_factor * self.m * self.g,
        ))

        des_att = np.array([des_roll, des_pitch, target_yaw], dtype=float)
        self.last_des_att = des_att.copy()
        return T_thrust, des_att

    @staticmethod
    def _body_z_axis(roll: float, pitch: float, yaw: float) -> np.ndarray:
        c_r, s_r = np.cos(roll), np.sin(roll)
        c_p, s_p = np.cos(pitch), np.sin(pitch)
        c_y, s_y = np.cos(yaw), np.sin(yaw)
        return np.array([
            c_y * s_p * c_r + s_y * s_r,
            s_y * s_p * c_r - c_y * s_r,
            c_p * c_r,
        ], dtype=float)

    def compute_attitude_loop(self, state, des_att):
        att = state[6:9]
        rate = state[9:12]
        dt = self.dt
        att_err = des_att - att
        att_err[2] = (att_err[2] + np.pi) % (2.0 * np.pi) - np.pi
        des_rate = self.kp_att * att_err
        des_rate = np.clip(des_rate, -self.max_des_rate, self.max_des_rate)
        rate_err = des_rate - rate
        self.integral_rate += rate_err * dt
        self.integral_rate = np.clip(self.integral_rate, -self.integral_rate_limit, self.integral_rate_limit)
        rate_deriv = (rate_err - self.prev_rate_err) / dt if dt > 0 else np.zeros(3, dtype=float)
        self.prev_rate_err = rate_err.copy()
        torques = self.I @ (self.kp_rate * rate_err + self.ki_rate * self.integral_rate + self.kd_rate * rate_deriv)
        return torques

    def compute_control(
        self,
        state,
        target_pos,
        target_vel=None,
        target_acc=None,
        target_yaw: float = 0.0,
        target_yaw_rate: float = 0.0,
    ):
        T_thrust, des_att = self.compute_position_loop(
            state,
            target_pos,
            target_vel,
            target_acc,
            target_yaw,
            target_yaw_rate,
        )
        torques = self.compute_attitude_loop(state, des_att)
        return np.array([T_thrust, torques[0], torques[1], torques[2]], dtype=float)

    def reset(self) -> None:
        self.integral_pos[:] = 0.0
        self.integral_vel[:] = 0.0
        self.integral_rate[:] = 0.0
        self.prev_vel_err[:] = 0.0
        self.prev_rate_err[:] = 0.0
        self.last_des_att[:] = 0.0


class HybridAttitudeController(Controller):
    """位置环 PID + 姿态环 PID/SMC 可切换控制器。"""

    def __init__(self, m: float = 1.0, g: float = 9.81, inertia=None, dt: float = 0.01):
        super().__init__(m=m, g=g, inertia=inertia, dt=dt)
        self.smc = SecondOrderSMC(dt=dt)
        self.use_smc = True
        self._prev_des_rate: np.ndarray | None = None

    def compute_control(
        self,
        state,
        target_pos,
        target_vel=None,
        target_acc=None,
        target_yaw: float = 0.0,
        target_yaw_rate: float = 0.0,
    ):
        T, des_att = self.compute_position_loop(
            state,
            target_pos,
            target_vel,
            target_acc,
            target_yaw,
            target_yaw_rate,
        )
        if not self.use_smc:
            torques = self.compute_attitude_loop(state, des_att)
            return np.array([T, torques[0], torques[1], torques[2]], dtype=float)

        att = state[6:9]
        rate = state[9:12]
        att_err = des_att - att
        att_err[2] = (att_err[2] + np.pi) % (2.0 * np.pi) - np.pi
        des_rate = self.kp_att * att_err
        des_rate = np.clip(des_rate, -self.max_des_rate, self.max_des_rate)

        # 切换控制角加速度: λ·e_ω + ε·sat(s/δ) + k·s + ki·∫s
        ang_acc_sw = self.smc.update(att_err, rate, des_rate=des_rate)

        # 期望角加速度 ω̇_des（有限差分），补全等效控制项
        if self._prev_des_rate is not None:
            des_rate_dot = (des_rate - self._prev_des_rate) / self.dt
        else:
            des_rate_dot = np.zeros(3, dtype=float)
        self._prev_des_rate = des_rate.copy()

        # 总角加速度: ω̇_des + λ·e_ω + ε·sat + k·s + ki·∫s
        ang_acc_cmd = des_rate_dot + ang_acc_sw

        # 等效控制: 抵消 Coriolis 项 ω×Iω
        # 注: τ_gyro 因控制器不持有 Jr/Ω_net 暂不抵消
        omega_cross_Iomega = np.cross(rate, self.I @ rate)
        torques = self.I @ ang_acc_cmd + omega_cross_Iomega
        return np.array([T, torques[0], torques[1], torques[2]], dtype=float)

    def reset(self) -> None:
        super().reset()
        self.smc.reset()
        self._prev_des_rate = None


class BacksteppingController(HybridAttitudeController):
    """反步法位置环 + SMC 姿态环控制器。

    用途
    ----
    用反步法替代 PID 串级位置环，直接推导期望推力和期望姿态，
    然后由 SMC 姿态环跟踪期望姿态。消除 PID 串级的逐层误差放大。

    原理
    ----
    对严格反馈系统 p→v→R(η)·F/m→η→ω 构造 Lyapunov 函数：

    Step 0: z₀ = ∫(p - p_des) dt       (积分增强)
            V₀ = ½z₀ᵀK₀z₀

    Step 1: z₁ = p - p_des
            α₁ = -K₁·z₁ - K₀·z₀ + v_des   (虚拟速度控制)
            V₁ = V₀ + ½z₁ᵀz₁

    Step 2: z₂ = v - α₁
            α̇₁ = -K₁·(v - v_des) - K₀·z₁ + a_des
            F_des = m·(-K₂·z₂ - z₁ + g·e₃ + α̇₁)  (期望推力向量)
            T = ||F_des||                           (推力幅值)
            b₃_des = F_des / T                      (推力方向→期望姿态)

            V₂ = ½z₀ᵀK₀z₀ + ½z₁ᵀz₁ + ½z₂ᵀz₂
            V̇₂ = -z₁ᵀK₁z₁ - z₂ᵀK₂z₂ ≤ 0   (LaSalle 不变原理保证收敛)

    Step 3: 姿态环 SMC 跟踪 b₃_des 提取的 (roll_des, pitch_des, yaw_des)

    参数
    ----
    K1: 位置误差增益 (3×3 正定对角阵)
    K2: 速度误差增益 (3×3 正定对角阵)
    """

    def __init__(self, m: float = 1.0, g: float = 9.81, inertia=None, dt: float = 0.01,
                 K1: np.ndarray | None = None, K2: np.ndarray | None = None):
        super().__init__(m=m, g=g, inertia=inertia, dt=dt)
        self.use_smc = True

        # 反步法增益（正定对角阵 → 向量形式，逐轴独立）
        self.K0 = np.array([0.5, 0.5, 0.6], dtype=float)  # 积分增益
        self.K1 = np.array([1.8, 1.8, 2.2], dtype=float) if K1 is None else np.asarray(K1, dtype=float)
        self.K2 = np.array([3.0, 3.0, 3.0], dtype=float) if K2 is None else np.asarray(K2, dtype=float)

        # 误差饱和上界（防止初始大误差 → 发散）
        self.z0_limit = 3.0    # 位置积分饱和 m·s
        self.z1_limit = 5.0    # 位置误差饱和 m
        self.z2_limit = 10.0   # 速度误差饱和 m/s
        self.alpha1_limit = 12.0  # 虚拟控制量饱和 m/s

        # 积分误差
        self.integral_z0 = np.zeros(3, dtype=float)

    def compute_position_loop(
        self,
        state,
        target_pos,
        target_vel=None,
        target_acc=None,
        target_yaw: float = 0.0,
        target_yaw_rate: float = 0.0,
    ):
        del target_yaw_rate
        """反步法位置环：计算 T 与期望姿态角。

        返回: (T_thrust, des_att=[roll_des, pitch_des, yaw_des])
        """
        if target_vel is None:
            target_vel = np.zeros(3, dtype=float)
        if target_acc is None:
            target_acc = np.zeros(3, dtype=float)

        pos = state[0:3]
        vel = state[3:6]

        # ---- Step 0: 积分误差积累 ----
        z1_raw = pos - target_pos
        self.integral_z0 += z1_raw * self.dt
        self.integral_z0 = np.clip(self.integral_z0, -self.z0_limit, self.z0_limit)
        z0 = self.integral_z0

        # ---- Step 1: 位置误差 → 虚拟速度控制 ----
        z1 = np.clip(z1_raw, -self.z1_limit, self.z1_limit)       # 饱和抑制
        alpha1 = -self.K1 * z1 - self.K0 * z0 + target_vel
        alpha1 = np.clip(alpha1, -self.alpha1_limit, self.alpha1_limit)

        # ---- Step 2: 速度误差 → 期望推力向量 ----
        z2 = vel - alpha1
        z2 = np.clip(z2, -self.z2_limit, self.z2_limit)       # 饱和抑制
        # ż₀=z₁, ż₁=v-v_des → α̇₁ = -K₁(v-v_des) - K₀·z₁ + a_des
        alpha1_dot = -self.K1 * (vel - target_vel) - self.K0 * z1 + target_acc

        # 期望推力向量（世界坐标系）: F_des = m·(-K₂z₂ - z₁ + g·e₃ + α̇₁)
        F_des = self.m * (-self.K2 * z2 - z1
                          + np.array([0.0, 0.0, self.g], dtype=float)
                          + alpha1_dot)

        # ---- Step 3: 从 F_des + yaw_des 提取期望 roll/pitch，并在倾角限幅后重算推力 ----
        return self._limited_thrust_and_attitude(
            F_des,
            target_yaw=target_yaw,
            max_thrust_factor=2.5,
        )

    # compute_control 继承 HybridAttitudeController:
    # 调用 self.compute_position_loop → (T, des_att)
    # 调用 self.compute_attitude_loop (SMC) → torques
    # 或使用 self.smc.update 直接计算 angular_acc

    # compute_control 继承 HybridAttitudeController:
    #   → self.compute_position_loop (backstepping) → (T, des_att)
    #   → self.smc.update (SMC 姿态跟踪) → ang_acc → torques

    def reset(self) -> None:
        super().reset()
        self.integral_z0[:] = 0.0


class GeometricSE3Controller(Controller):
    """Experimental SE(3)-style controller that preserves the existing 4D actuator contract."""

    def __init__(self, m: float = 1.0, g: float = 9.81, inertia=None, dt: float = 0.01):
        super().__init__(m=m, g=g, inertia=inertia, dt=dt)
        self.kR = np.array([6.0, 6.0, 3.0], dtype=float)
        self.kOmega = np.array([1.8, 1.8, 1.2], dtype=float)

    def compute_control(
        self,
        state,
        target_pos,
        target_vel=None,
        target_acc=None,
        target_yaw: float = 0.0,
        target_yaw_rate: float = 0.0,
    ) -> np.ndarray:
        T, des_att = self.compute_position_loop(
            state,
            target_pos,
            target_vel,
            target_acc,
            target_yaw,
            target_yaw_rate,
        )
        att = np.asarray(state[6:9], dtype=float)
        omega = np.asarray(state[9:12], dtype=float)
        e_R = att - des_att
        e_R[2] = (e_R[2] + np.pi) % (2.0 * np.pi) - np.pi
        omega_d = np.array([0.0, 0.0, float(target_yaw_rate)], dtype=float)
        e_omega = omega - omega_d
        ang_acc_cmd = -(self.kR * e_R) - (self.kOmega * e_omega)
        torques = self.I @ ang_acc_cmd + np.cross(omega, self.I @ omega)
        assert torques.shape == (3,)
        assert np.all(np.isfinite(torques))
        return np.array([T, torques[0], torques[1], torques[2]], dtype=float)
