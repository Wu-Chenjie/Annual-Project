import numpy as np
#单个旋翼的模型，包含推力、反扭矩和电机动力学

class Rotor:
    """
    简化旋翼模型
    推力   T = kf * omega^2
    反扭矩 Q = direction * km * omega^2
    含一阶电机动力学: tau * d(omega)/dt + omega = omega_cmd
    """

    def __init__(self, kf=1e-5, km=2e-7, direction=1, tau_motor=0.02, omega_max=1000.0):
        """
        :param kf:        推力系数 [N/(rad/s)^2]
        :param km:        力矩系数 [Nm/(rad/s)^2]
        :param direction: 旋转方向 +1 或 -1
        :param tau_motor: 电机时间常数 [s]
        :param omega_max: 最大转速 [rad/s]
        """
        self.kf = kf
        self.km = km
        self.direction = direction
        self.tau_motor = tau_motor
        self.omega_max = omega_max
        self.omega = 0.0

    def thrust(self):#计算推力
        return self.kf * self.omega ** 2

    def torque(self):#计算反扭矩，根据旋转方向决定符号
        return self.direction * self.km * self.omega ** 2

    def update(self, omega_cmd, dt):#更新实际转速
        """
        一阶电机动力学:
            tau * d(omega)/dt = omega_cmd - omega
        离散化（一阶指数衰减）:
            omega_new = omega + (omega_cmd - omega) * (1 - exp(-dt/tau))
        当 tau 极小时直接赋值。
        """
        omega_cmd = np.clip(omega_cmd, 0.0, self.omega_max)
        if self.tau_motor > 1e-9:
            alpha = 1.0 - np.exp(-dt / self.tau_motor)
            self.omega = self.omega + (omega_cmd - self.omega) * alpha
        else:
            self.omega = omega_cmd
        self.omega = np.clip(self.omega, 0.0, self.omega_max)
