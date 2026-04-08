import numpy as np


class SuperTwistingSMC:
    """
    基于 Super-Twisting 算法的二阶滑模控制器（姿态环）

    滑模面定义：
        s = angle_rate_error + lambda * angle_error

    标准 Super-Twisting 控制律：
        u = u1 + u2
        u1 = -alpha * |s|^(1/2) * sign(s)
        du2/dt = -beta * sign(s)

    特性：
        - 无需已知扰动上界的精确值
        - 控制信号连续（消除传统滑模的抖振问题）
        - alpha, beta 满足一定条件即可保证有限时间收敛

    输出为角加速度级别，需在外层乘以惯量矩阵 I 转换为力矩
    """

    def __init__(self, dt,
                 lam=np.array([8.0, 8.0, 4.0]),
                 alpha=np.array([15.0, 15.0, 8.0]),
                 beta=np.array([10.0, 10.0, 5.0])):
        """
        :param dt:    控制周期 [s]
        :param lam:   滑模面系数 lambda [3]
        :param alpha: Super-Twisting 比例增益 [3]
        :param beta:  Super-Twisting 积分增益 [3]
        """
        self.dt = dt
        self.lam = lam
        self.alpha = alpha
        self.beta = beta

        # 积分状态 v（Super-Twisting 的积分项）
        self.v = np.zeros(3)

    def compute_sliding_surface(self, angle_error, rate_error):
        """
        计算滑模面
        s = rate_error + lambda * angle_error
        """
        return rate_error + self.lam * angle_error

    def update(self, angle_error, angle_rate, des_rate=None):
        """
        计算控制输出（角加速度级别）

        :param angle_error: 姿态角误差 [roll_e, pitch_e, yaw_e]
        :param angle_rate:  当前角速度 [wx, wy, wz]
        :param des_rate:    期望角速度（默认为0）
        :return: 角加速度指令 [3]
        """
        if des_rate is None:
            des_rate = np.zeros(3)

        rate_error = des_rate - angle_rate

        # 滑模面
        s = self.compute_sliding_surface(angle_error, rate_error)

        # Super-Twisting 控制律
        # u1 = -alpha * |s|^(1/2) * sign(s)
        abs_s = np.abs(s)
        sqrt_abs_s = np.sqrt(abs_s + 1e-12)  # 避免除零
        sign_s = np.sign(s)

        u1 = -self.alpha * sqrt_abs_s * sign_s

        # u2 由积分得到: dv/dt = -beta * sign(s)
        self.v += -self.beta * sign_s * self.dt

        # 总控制量 = 连续项 + 积分项
        u = u1 + self.v

        return u

    def reset(self):
        """重置积分状态"""
        self.v = np.zeros(3)
