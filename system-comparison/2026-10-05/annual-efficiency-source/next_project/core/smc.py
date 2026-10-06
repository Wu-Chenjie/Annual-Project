"""滑模控制器。

用途
----
为姿态环提供鲁棒控制能力，可替代传统 PID 角速度环，提升模型不确定性和扰动下的稳定性。

原理
----
1) 定义滑模面：

    s = e_omega + lambda * e_eta

    其中 e_eta 是姿态误差，e_omega 是角速度误差。

2) 指数趋近律滑模（SecondOrderSMC）采用：

    s_dot = -epsilon * sat(s/delta) - k * s - ki * integral(s)

    返回切换控制角加速度（不含等效控制）：
    ang_acc_sw = lambda * rate_error + epsilon * sat(s/delta) + k * s + ki * integral(s)

    调用方负责叠加期望角加速度 ω̇_des 与 Coriolis 抵消，转换为力矩：
    ang_acc_cmd = ω̇_des + ang_acc_sw
    torques = I @ ang_acc_cmd + ω×Iω

    完整力矩公式（代入后）：
    τ = I·(ω̇_des + λ·e_ω + ε·sat + k·s + ki·∫s) + ω×Iω

    其中 sat(·) 用于替代 sign(·) 抑制抖振。

3) Super-Twisting 滑模（SuperTwistingSMC）采用：

    u = u1 + v
    u1 = -alpha * |s|^(1/2) * sign(s)
    v_dot = -beta * sign(s)

    该结构在满足增益条件时具备有限时间收敛，并生成连续控制输入。

"""

from __future__ import annotations

import numpy as np


class SecondOrderSMC:
    """指数趋近律滑模控制器。"""

    def __init__(self, dt: float, lam=None, epsilon=None, k=None, ki=None, delta: float = 0.05, ki_limit: float = 2.0):
        self.dt = float(dt)
        self.lam = np.array([7.8579, 8.2384, 3.9090], dtype=float) if lam is None else np.array(lam, dtype=float)
        self.epsilon = np.array([5.1161, 5.0003, 3.0693], dtype=float) if epsilon is None else np.array(epsilon, dtype=float)
        self.k = np.array([7.9432, 8.2315, 3.9736], dtype=float) if k is None else np.array(k, dtype=float)
        self.ki = np.array([0.8638, 1.1520, 0.9825], dtype=float) if ki is None else np.array(ki, dtype=float)
        self.delta = float(delta)
        self.ki_limit = float(ki_limit)
        self.integral_s = np.zeros(3, dtype=float)

    def _sat(self, s: np.ndarray) -> np.ndarray:
        return np.where(np.abs(s) <= self.delta, s / self.delta, np.sign(s))

    def compute_sliding_surface(self, angle_error: np.ndarray, rate_error: np.ndarray) -> np.ndarray:
        return rate_error + self.lam * angle_error

    def update(self, angle_error: np.ndarray, angle_rate: np.ndarray, des_rate=None) -> np.ndarray:
        if des_rate is None:
            des_rate = np.zeros(3, dtype=float)
        rate_error = des_rate - angle_rate
        s = self.compute_sliding_surface(angle_error, rate_error)
        sat_s = self._sat(s)
        self.integral_s += s * self.dt
        self.integral_s = np.clip(self.integral_s, -self.ki_limit, self.ki_limit)
        return self.lam * rate_error + self.epsilon * sat_s + self.k * s + self.ki * self.integral_s

    def reset(self) -> None:
        self.integral_s[:] = 0.0


class SuperTwistingSMC:
    """Super-Twisting 二阶滑模控制器。"""

    def __init__(self, dt: float, lam=None, alpha=None, beta=None):
        self.dt = float(dt)
        self.lam = np.array([8.0, 8.0, 4.0], dtype=float) if lam is None else np.array(lam, dtype=float)
        self.alpha = np.array([6.0, 6.0, 3.0], dtype=float) if alpha is None else np.array(alpha, dtype=float)
        self.beta = np.array([10.0, 10.0, 5.0], dtype=float) if beta is None else np.array(beta, dtype=float)
        self.v = np.zeros(3, dtype=float)

    def compute_sliding_surface(self, angle_error: np.ndarray, rate_error: np.ndarray) -> np.ndarray:
        return rate_error + self.lam * angle_error

    def update(self, angle_error: np.ndarray, angle_rate: np.ndarray, des_rate=None) -> np.ndarray:
        if des_rate is None:
            des_rate = np.zeros(3, dtype=float)
        rate_error = des_rate - angle_rate
        s = self.compute_sliding_surface(angle_error, rate_error)
        abs_s = np.abs(s)
        sign_s = np.sign(s)
        u1 = -self.alpha * np.sqrt(abs_s + 1e-12) * sign_s
        self.v += -self.beta * sign_s * self.dt
        return u1 + self.v

    def reset(self) -> None:
        self.v[:] = 0.0
