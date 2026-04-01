import numpy as np


class SecondOrderSMC:
    """
    指数趋近律滑模控制器（姿态环）

    参考PDF第47页公式(4-4)、(4-6)设计。

    滑模面：
        s = rate_error + lam * angle_error

    指数趋近律：
        ds/dt = -epsilon * sat(s) - k * s

    控制律：
        u = lam * rate_error + epsilon * sat(s) + k * s + ki * integral(s)

    其中：
        - lam:     滑模面斜率系数，决定滑模面上的收敛速度
        - epsilon: 等速趋近项，保证远离滑模面时快速趋近
        - k:       指数趋近项，保证接近滑模面时平滑收敛
        - ki:      积分补偿项，消除边界层内稳态误差
        - Delta:   饱和函数边界层宽度，用sat替代sgn消除抖振

    输出为角加速度级别，需在外层乘以惯量矩阵 I 转换为力矩
    """

    def __init__(self, dt,
                 lam=np.array([8.0, 8.0, 4.0]),
                 epsilon=np.array([5.0, 5.0, 3.0]),
                 k=np.array([8.0, 8.0, 4.0]),
                 ki=np.array([0.5, 0.5, 0.3]),
                 delta=0.05,
                 ki_limit=2.0):
        """
        :param dt:       控制周期 [s]
        :param lam:      滑模面系数 [3],误差收敛速度
        :param epsilon:  等速趋近增益 [3]，远离滑模面时快速趋近
        :param k:        指数趋近增益 [3]，接近滑模面时平滑收敛
        :param ki:       积分补偿增益 [3]，消除稳态误差
        :param delta:    饱和函数边界层宽度
        :param ki_limit: 积分项限幅
        """
        self.dt = dt
        self.lam = np.array(lam, dtype=float)
        self.epsilon = np.array(epsilon, dtype=float)
        self.k = np.array(k, dtype=float)
        self.ki = np.array(ki, dtype=float)
        self.delta = delta
        self.ki_limit = ki_limit

        # 积分状态
        self.integral_s = np.zeros(3)

    def _sat(self, s):
        """
        饱和函数：替代 sgn(s) 消除抖振
        |s| <= delta 时线性过渡，|s| > delta 时饱和为 ±1
        """
        return np.where(
            np.abs(s) <= self.delta,
            s / self.delta,
            np.sign(s)
        )

    def compute_sliding_surface(self, angle_error, rate_error):
        """
        滑模面: s = rate_error + lam * angle_error
        """
        return rate_error + self.lam * angle_error

    def update(self, angle_error, angle_rate, des_rate=None):
        """
        计算控制输出角加速度

        :param angle_error: 姿态角误差 [roll_e, pitch_e, yaw_e]
        :param angle_rate:  当前角速度 [wx, wy, wz]
        :param des_rate:    期望角速度（默认为零）
        :return:            角加速度指令 [3]
        """
        if des_rate is None:
            des_rate = np.zeros(3)

        rate_error = des_rate - angle_rate

        # 滑模面
        s = self.compute_sliding_surface(angle_error, rate_error)

        # 饱和函数
        sat_s = self._sat(s)

        # 积分补偿：消除边界层内稳态误差
        self.integral_s += s * self.dt
        self.integral_s = np.clip(
            self.integral_s,
            -self.ki_limit,
            self.ki_limit
        )

        # 控制律：指数趋近律 + 积分补偿
        u = (
            self.lam * rate_error
            + self.epsilon * sat_s
            + self.k * s
            + self.ki * self.integral_s
        )

        return u

    def reset(self):
        """重置积分状态"""
        self.integral_s = np.zeros(3)
