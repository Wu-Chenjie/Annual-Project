"""控制分配器。

用途
----
把高层控制器输出的总推力与三个轴力矩，转换为四个旋翼的推力和转速。

原理
----
采用线性控制分配：

    u = B * f

其中 u = [T, Mx, My, Mz]^T，f = [f1, f2, f3, f4]^T。

已知期望控制量 u 后，求旋翼推力：

    f = B^{-1} * u

再由推力-转速关系求电机转速：

    omega_i = sqrt(max(f_i, 0) / kf)


"""

from __future__ import annotations

import numpy as np


class ControlAllocator:
    """四旋翼推力分配器。"""

    def __init__(self, arm_length: float = 0.2, kf: float = 1e-5, km: float = 2e-7):
        self.arm_length = float(arm_length)
        self.kf = float(kf)
        self.km = float(km)
        gamma = self.km / self.kf
        self.B = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [self.arm_length, -self.arm_length, -self.arm_length, self.arm_length],
            [self.arm_length, self.arm_length, -self.arm_length, -self.arm_length],
            [-gamma, gamma, -gamma, gamma],
        ], dtype=float)
        self.B_inv = np.linalg.inv(self.B)

    def allocate_thrusts(self, u) -> np.ndarray:
        thrusts = self.B_inv @ np.array(u, dtype=float)
        return np.clip(thrusts, 0.0, None)

    def thrusts_to_omegas(self, thrusts) -> np.ndarray:
        thrusts = np.array(thrusts, dtype=float)
        return np.sqrt(np.clip(thrusts / self.kf, 0.0, None))

    def omegas_to_u(self, omegas) -> np.ndarray:
        omegas = np.array(omegas, dtype=float)
        thrusts = self.kf * omegas ** 2
        return self.B @ thrusts
