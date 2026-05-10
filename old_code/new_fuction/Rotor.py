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


class BEMRotor(Rotor):
    """
    基于叶素动量理论（Blade Element Momentum, BEM）的旋翼模型。

    继承自 Rotor，保留一阶电机动力学和简化推力/力矩接口。
    新增 thrust_bem / torque_bem 方法，通过沿展向积分叶素气动力
    并与动量理论联立，迭代求解各截面入流因子 a，从而获得更精确的
    推力和气动力矩。

    桨距角沿展向线性分布：
        theta(r) = theta_root + (theta_tip - theta_root) * (r / R)

    升力系数采用薄翼理论线性近似：
        Cl(alpha) = 2 * pi * alpha    (alpha 为有效攻角)
    阻力系数取常数：
        Cd0 = 0.01

    动量理论：
        dT = 4 * pi * rho * r * (v_inf + v_i) * v_i * dr
    叶素理论：
        dT = 0.5 * rho * num_blades * chord * W^2 * Cl * dr

    两者联立，用牛顿迭代求每个截面的入流因子 a。
    """

    def __init__(self,
                 kf=1e-5, km=2e-7, direction=1, tau_motor=0.02, omega_max=1000.0,
                 num_blades=2, R_blade=0.12, chord=0.02,
                 theta_tip=0.05, theta_root=0.40, rho=1.225,
                 N_sections=10, Cd0=0.01):
        """
        :param kf:          推力系数（父类，备用） [N/(rad/s)^2]
        :param km:          力矩系数（父类，备用） [Nm/(rad/s)^2]
        :param direction:   旋转方向 +1 或 -1
        :param tau_motor:   电机时间常数 [s]
        :param omega_max:   最大转速 [rad/s]
        :param num_blades:  桨叶数
        :param R_blade:     桨叶半径 [m]
        :param chord:       桨叶弦长（假设等弦长） [m]
        :param theta_tip:   桨尖桨距角 [rad]
        :param theta_root:  桨根桨距角 [rad]
        :param rho:         空气密度 [kg/m^3]
        :param N_sections:  展向积分段数
        :param Cd0:         零升阻力系数
        """
        super().__init__(kf=kf, km=km, direction=direction,
                         tau_motor=tau_motor, omega_max=omega_max)
        self.num_blades = num_blades
        self.R_blade = R_blade
        self.chord = chord
        self.theta_tip = theta_tip
        self.theta_root = theta_root
        self.rho = rho
        self.N_sections = N_sections
        self.Cd0 = Cd0

        # BEM 推力/力矩缓存，避免重复计算
        self._bem_thrust = 0.0
        self._bem_torque = 0.0

    def _theta(self, r):
        """
        计算径向位置 r 处的桨距角（线性插值）。

        :param r: 距旋转轴距离 [m]
        :return:  桨距角 [rad]
        """
        return self.theta_root + (self.theta_tip - self.theta_root) * (r / self.R_blade)

    def _solve_inflow_factor(self, r, v_inf, omega, dr):
        """
        在径向位置 r 处，用牛顿迭代求解入流因子 a。

        动量理论给出的微元推力：
            dT_mom = 4 * pi * rho * r * (v_inf + v_i) * v_i * dr
        其中诱导速度 v_i = a * omega * r。

        叶素理论给出的微元推力：
            dT_be  = 0.5 * rho * B * c * W^2 * Cl(alpha) * dr
        其中：
            phi   = arctan((v_inf + v_i) / (omega * r))
            alpha = theta - phi
            W     = sqrt((v_inf + v_i)^2 + (omega * r)^2)
            Cl    = 2 * pi * alpha

        令 f(a) = dT_be(a) - dT_mom(a) = 0，用牛顿法求 a。

        :param r:     径向位置 [m]
        :param v_inf: 轴向来流速度（悬停时为0） [m/s]
        :param omega: 当前转速 [rad/s]
        :param dr:    微元展向宽度 [m]
        :return:      入流因子 a
        """
        B = self.num_blades
        c = self.chord
        rho = self.rho
        theta = self._theta(r)

        omega_r = omega * r
        if omega_r < 1e-6:
            return 0.0

        # 初始猜测
        a = 0.01
        max_iter = 30
        tol = 1e-8

        for _ in range(max_iter):
            v_i = a * omega_r
            v_axial = v_inf + v_i

            phi = np.arctan2(v_axial, omega_r)
            alpha = theta - phi
            W2 = v_axial ** 2 + omega_r ** 2

            Cl = 2.0 * np.pi * alpha

            # 叶素微元推力
            dT_be = 0.5 * rho * B * c * W2 * Cl * np.cos(phi) * dr
            # 动量微元推力
            dT_mom = 4.0 * np.pi * rho * r * v_axial * v_i * dr

            f_val = dT_be - dT_mom

            # 数值导数 df/da（中心差分）
            da = 1e-6
            a_p = a + da
            v_i_p = a_p * omega_r
            v_axial_p = v_inf + v_i_p
            phi_p = np.arctan2(v_axial_p, omega_r)
            alpha_p = theta - phi_p
            W2_p = v_axial_p ** 2 + omega_r ** 2
            Cl_p = 2.0 * np.pi * alpha_p
            dT_be_p = 0.5 * rho * B * c * W2_p * Cl_p * np.cos(phi_p) * dr
            dT_mom_p = 4.0 * np.pi * rho * r * v_axial_p * v_i_p * dr
            f_val_p = dT_be_p - dT_mom_p

            df = (f_val_p - f_val) / da
            if abs(df) < 1e-15:
                break

            a_new = a - f_val / df
            # 限制 a 在合理范围内，防止发散
            a_new = np.clip(a_new, -0.5, 0.95)

            if abs(a_new - a) < tol:
                a = a_new
                break
            a = a_new

        return a

    def thrust_bem(self, v_inf=0.0):
        """
        使用叶素动量理论计算旋翼推力。

        沿展向将桨叶分为 N_sections 段，每段用牛顿迭代求入流因子，
        再用叶素理论公式积分得到总推力。

        :param v_inf: 轴向来流速度 [m/s]，悬停时为 0
        :return:      旋翼推力 [N]（始终 >= 0）
        """
        omega = self.omega
        if omega < 1e-6:
            return 0.0

        R = self.R_blade
        B = self.num_blades
        c = self.chord
        rho = self.rho

        # 避开 r=0 处的奇异性，从 10% 半径开始积分
        r_inner = 0.1 * R
        dr = (R - r_inner) / self.N_sections
        T_total = 0.0

        for i in range(self.N_sections):
            r = r_inner + (i + 0.5) * dr  # 取段中点
            a = self._solve_inflow_factor(r, v_inf, omega, dr)

            v_i = a * omega * r
            v_axial = v_inf + v_i
            omega_r = omega * r

            phi = np.arctan2(v_axial, omega_r)
            alpha = self._theta(r) - phi
            W2 = v_axial ** 2 + omega_r ** 2

            Cl = 2.0 * np.pi * alpha
            Cd = self.Cd0

            # 推力方向分量: Cl*cos(phi) - Cd*sin(phi)
            dT = 0.5 * rho * B * c * W2 * (Cl * np.cos(phi) - Cd * np.sin(phi)) * dr
            T_total += dT

        return max(T_total, 0.0)

    def torque_bem(self, v_inf=0.0):
        """
        使用叶素动量理论计算旋翼气动力矩。

        与 thrust_bem 类似，但积分的是切向力分量乘以力臂 r：
            dQ = 0.5 * rho * B * c * W^2 * (Cl*sin(phi) + Cd*cos(phi)) * r * dr

        返回值已考虑旋转方向（self.direction），用于反扭矩补偿。

        :param v_inf: 轴向来流速度 [m/s]
        :return:      旋翼气动力矩 [Nm]（含旋转方向符号）
        """
        omega = self.omega
        if omega < 1e-6:
            return 0.0

        R = self.R_blade
        B = self.num_blades
        c = self.chord
        rho = self.rho

        r_inner = 0.1 * R
        dr = (R - r_inner) / self.N_sections
        Q_total = 0.0

        for i in range(self.N_sections):
            r = r_inner + (i + 0.5) * dr
            a = self._solve_inflow_factor(r, v_inf, omega, dr)

            v_i = a * omega * r
            v_axial = v_inf + v_i
            omega_r = omega * r

            phi = np.arctan2(v_axial, omega_r)
            alpha = self._theta(r) - phi
            W2 = v_axial ** 2 + omega_r ** 2

            Cl = 2.0 * np.pi * alpha
            Cd = self.Cd0

            # 力矩方向分量: Cl*sin(phi) + Cd*cos(phi)，乘以力臂 r
            dQ = 0.5 * rho * B * c * W2 * (Cl * np.sin(phi) + Cd * np.cos(phi)) * r * dr
            Q_total += dQ

        return self.direction * Q_total

    def update(self, omega_cmd, dt, v_inf=0.0):
        """
        更新旋翼状态：先调用父类更新电机转速 omega，再重新计算 BEM 推力和力矩缓存。

        :param omega_cmd: 目标转速指令 [rad/s]
        :param dt:        仿真步长 [s]
        :param v_inf:     轴向来流速度 [m/s]，默认 0（悬停）
        """
        # 父类更新电机动力学（一阶指数衰减）
        super().update(omega_cmd, dt)
        # 更新 BEM 缓存
        self._bem_thrust = self.thrust_bem(v_inf)
        self._bem_torque = self.torque_bem(v_inf)
