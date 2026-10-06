"""旋翼模型。

用途
----
描述单个旋翼的推力与反扭矩关系，并提供一阶电机动态响应。

原理
----
1) 简化旋翼模型：

    T = kf * omega^2
    Q = km * omega^2

2) 电机一阶动态：

    tau_m * d(omega)/dt + omega = omega_cmd

    离散实现采用指数形式：

    omega_{k+1} = omega_k + (1 - exp(-dt/tau_m)) * (omega_cmd - omega_k)

3) BEM 旋翼采用叶素理论与动量理论联立，按展向离散积分推力和阻力矩。

"""

from __future__ import annotations

import numpy as np


class Rotor:
    """简化旋翼模型。"""

    def __init__(self, kf: float = 1e-5, km: float = 2e-7, direction: int = 1,
                 tau_motor: float = 0.02, omega_max: float = 1000.0):
        self.kf = float(kf)
        self.km = float(km)
        self.direction = 1 if direction >= 0 else -1
        self.tau_motor = float(tau_motor)
        self.omega_max = float(omega_max)
        self.omega = 0.0

    def thrust(self) -> float:
        return self.kf * self.omega ** 2

    def torque(self) -> float:
        return self.direction * self.km * self.omega ** 2

    def update(self, omega_cmd: float, dt: float) -> None:
        omega_cmd = float(np.clip(omega_cmd, 0.0, self.omega_max))
        if self.tau_motor > 1e-9:
            alpha = 1.0 - np.exp(-dt / self.tau_motor)
            self.omega = self.omega + (omega_cmd - self.omega) * alpha
        else:
            self.omega = omega_cmd
        self.omega = float(np.clip(self.omega, 0.0, self.omega_max))


class BEMRotor(Rotor):
    """基于叶素动量理论的旋翼近似实现。"""
    '''
    叶素动量理论(Blade Element Momentum Theory, BEM)是一种结合了叶素理论和动量理论的旋翼分析方法。
    它通过将旋翼划分为多个小段（叶素），并对每个叶素应用动量理论来计算推力和扭矩,然后逐段积分得到总推力与总阻力矩。
    这种方法能够更准确地描述旋翼在不同工况下的性能，特别是在高迎角和非线性区域。
    '''

    def __init__(self, kf: float = 1e-5, km: float = 2e-7, direction: int = 1,
                 tau_motor: float = 0.02, omega_max: float = 1000.0,
                 num_blades: int = 2, radius: float = 0.12, chord: float = 0.02,
                 theta_tip: float = 0.05, theta_root: float = 0.40,
                 rho: float = 1.225, n_sections: int = 10, cd0: float = 0.01):
        super().__init__(kf=kf, km=km, direction=direction,
                         tau_motor=tau_motor, omega_max=omega_max)
        self.num_blades = num_blades
        self.radius = radius
        self.chord = chord
        self.theta_tip = theta_tip
        self.theta_root = theta_root
        self.rho = rho
        self.n_sections = n_sections
        self.cd0 = cd0
        self._bem_thrust = 0.0
        self._bem_torque = 0.0

    def _theta(self, r: float) -> float:
        return self.theta_root + (self.theta_tip - self.theta_root) * (r / self.radius)

    def _solve_inflow_factor(self, r: float, v_inf: float, omega: float, dr: float) -> float:
        omega_r = omega * r
        if omega_r < 1e-6:
            return 0.0
        a = 0.01
        for _ in range(25):
            v_i = a * omega_r
            v_axial = v_inf + v_i
            phi = np.arctan2(v_axial, omega_r)
            alpha = self._theta(r) - phi
            W2 = v_axial ** 2 + omega_r ** 2
            cl = 2.0 * np.pi * alpha
            dT_be = 0.5 * self.rho * self.num_blades * self.chord * W2 * cl * np.cos(phi) * dr
            dT_mom = 4.0 * np.pi * self.rho * r * v_axial * v_i * dr
            f_val = dT_be - dT_mom

            da = 1e-6
            a_p = a + da
            v_i_p = a_p * omega_r
            v_axial_p = v_inf + v_i_p
            phi_p = np.arctan2(v_axial_p, omega_r)
            alpha_p = self._theta(r) - phi_p
            W2_p = v_axial_p ** 2 + omega_r ** 2
            cl_p = 2.0 * np.pi * alpha_p
            dT_be_p = 0.5 * self.rho * self.num_blades * self.chord * W2_p * cl_p * np.cos(phi_p) * dr
            dT_mom_p = 4.0 * np.pi * self.rho * r * v_axial_p * v_i_p * dr
            df = (dT_be_p - dT_mom_p - f_val) / da
            if abs(df) < 1e-15:
                break
            a_new = np.clip(a - f_val / df, -0.5, 0.95)
            if abs(a_new - a) < 1e-8:
                return float(a_new)
            a = a_new
        return float(a)

    def thrust_bem(self, v_inf: float = 0.0) -> float:
        if self.omega < 1e-6:
            return 0.0
        r_inner = 0.1 * self.radius
        dr = (self.radius - r_inner) / self.n_sections
        total = 0.0
        for i in range(self.n_sections):
            r = r_inner + (i + 0.5) * dr
            a = self._solve_inflow_factor(r, v_inf, self.omega, dr)
            v_i = a * self.omega * r
            v_axial = v_inf + v_i
            omega_r = self.omega * r
            phi = np.arctan2(v_axial, omega_r)
            alpha = self._theta(r) - phi
            W2 = v_axial ** 2 + omega_r ** 2
            cl = 2.0 * np.pi * alpha
            dT = 0.5 * self.rho * self.num_blades * self.chord * W2 * (cl * np.cos(phi) - self.cd0 * np.sin(phi)) * dr
            total += dT
        return float(max(total, 0.0))

    def torque_bem(self, v_inf: float = 0.0) -> float:
        if self.omega < 1e-6:
            return 0.0
        r_inner = 0.1 * self.radius
        dr = (self.radius - r_inner) / self.n_sections
        total = 0.0
        for i in range(self.n_sections):
            r = r_inner + (i + 0.5) * dr
            a = self._solve_inflow_factor(r, v_inf, self.omega, dr)
            v_i = a * self.omega * r
            v_axial = v_inf + v_i
            omega_r = self.omega * r
            phi = np.arctan2(v_axial, omega_r)
            alpha = self._theta(r) - phi
            W2 = v_axial ** 2 + omega_r ** 2
            cl = 2.0 * np.pi * alpha
            dQ = 0.5 * self.rho * self.num_blades * self.chord * W2 * (cl * np.sin(phi) + self.cd0 * np.cos(phi)) * r * dr
            total += dQ
        return float(self.direction * total)

    def update(self, omega_cmd: float, dt: float, v_inf: float = 0.0) -> None:
        super().update(omega_cmd, dt)
        self._bem_thrust = self.thrust_bem(v_inf)
        self._bem_torque = self.torque_bem(v_inf)
