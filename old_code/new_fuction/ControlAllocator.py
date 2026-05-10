import numpy as np
#四旋翼的推力分布

class ControlAllocator:
    """
    四旋翼推力分配器（X型布局）

    旋翼编号与位置（俯视图）：
        0(前左,CW)    1(前右,CCW)
              \\      /
               中心
              /      \\
        3(后左,CCW)   2(后右,CW)

    映射关系 u = B * f：
        u = [T, Mx, My, Mz]^T
        f = [f0, f1, f2, f3]^T（各旋翼推力）
    
    控制逻辑：
        1. 位置环计算得到期望总推力 T 和期望姿态（转换为 Mx, My）
        2. 偏航环计算得到期望偏航力矩 Mz
        3. 通过 B 的逆矩阵将 [T, Mx, My, Mz] 转换为各旋翼推力 [f0, f1, f2, f3]
        4. 推力转换为转速指令，发送给电机控制器

    其中：
        T  = f0 + f1 + f2 + f3
        Mx = L*(f0 - f1 - f2 + f3)  （滚转：左升右降为正）
        My = L*(f0 + f1 - f2 - f3)  （俯仰：前升后降为正）
        Mz = km/kf*(-f0 + f1 - f2 + f3) （偏航：由旋转方向决定，绕z轴旋转）
    """

    def __init__(self, arm_length=0.2, kf=1e-5, km=2e-7):
        self.L = arm_length
        self.kf = kf
        self.km = km
        gamma = km / kf  # 力矩-推力比

        # 统一分配矩阵 u = B * f
        self.B = np.array([#B是一个4x4矩阵，将四个旋翼的推力映射到总推力和三个轴的力矩，由旋翼臂长L和力矩-推力比gamma决定）
            [1.0,       1.0,       1.0,       1.0],
            [self.L,   -self.L,   -self.L,    self.L],
            [self.L,    self.L,   -self.L,   -self.L],
            [-gamma,    gamma,    -gamma,     gamma]
        ])
        self.B_inv = np.linalg.inv(self.B)#B_inv是B的逆矩阵，用于将期望的总推力和力矩转换为各旋翼的推力指令

    def allocate_thrusts(self, u):#将期望控制量分配到四个旋翼推力
        """
        将期望控制量分配到四个旋翼推力：u = B*f -> f = B_inv * u
        :param u: [T, Mx, My, Mz]
        :return:  [f0, f1, f2, f3] 各旋翼推力
        """
        thrusts = self.B_inv @ np.array(u, dtype=float)
        thrusts = np.clip(thrusts, 0.0, None)
        return thrusts

    def thrusts_to_omegas(self, thrusts):
        """推力 -> 转速"""
        return np.sqrt(np.clip(thrusts / self.kf, 0.0, None))

    def omegas_to_u(self, omegas):#由f=kf·ω²得到u=B·f，再通过B @ f算实际总控制量
        """
        转速 -> 实际控制量
        由f=kf·ω²得到u=B·f，再通过B @ f算实际总控制量
        :param omegas: [ω0, ω1, ω2, ω3]
        :return: [T, Mx, My, Mz]
        """
        thrusts = self.kf * omegas ** 2
        return self.B @ thrusts
