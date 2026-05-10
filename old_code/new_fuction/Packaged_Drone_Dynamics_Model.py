import numpy as np
from Rotor import Rotor
from ControlAllocator import ControlAllocator

def rotation_matrix(roll, pitch, yaw):#用于将机体坐标系和世界坐标系相互转换
    # ZYX欧拉角顺序旋转矩阵
    c_r, s_r = np.cos(roll), np.sin(roll)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    c_y, s_y = np.cos(yaw), np.sin(yaw)

    R_x = np.array([[1, 0, 0],[0, c_r, -s_r],[0, s_r, c_r]])
    R_y = np.array([[c_p, 0, s_p],[0, 1, 0],[-s_p, 0, c_p]])
    R_z = np.array([[c_y, -s_y, 0],[s_y, c_y, 0],[0,0,1]])

    return R_z @ R_y @ R_x

class Drone:
    def __init__(self, m=1.0, inertia=None, L=0.2, dt=0.01):
        if inertia is None:
            inertia = np.diag([0.01, 0.01, 0.02])#默认惯量矩阵，假设无人机在x和y轴的惯量较小，在z轴的惯量较大
        self.m = m
        self.I = inertia
        self.I_inv = np.linalg.inv(self.I)#惯量矩阵求逆
        self.g = 9.81
        self.L = L
        self.dt = dt
        self.k_drag = 0.05
        self.wind = np.zeros(3)
        self.J_rotor = 3.0e-5  # 单旋翼转动惯量 [kg·m²]

        self.state = np.zeros(12)  # [pos(3), vel(3), euler(3), ang_vel(3)]

        # 初始化4个旋翼
        self.rotors = [
            Rotor(direction=1),
            Rotor(direction=-1),
            Rotor(direction=1),
            Rotor(direction=-1),
        ]
        self.allocator = ControlAllocator(arm_length=self.L, kf=self.rotors[0].kf, km=self.rotors[0].km)

    def set_initial_state(self, position, velocity, attitude=None, angular_velocity=None, dt=None):
        if attitude is None:
            attitude = [0, 0, 0]
        if angular_velocity is None:
            angular_velocity = [0, 0, 0]
        if dt is not None:
            self.dt = dt
        self.state[0:3] = np.array(position, dtype=float)
        self.state[3:6] = np.array(velocity, dtype=float)
        self.state[6:9] = np.array(attitude, dtype=float)
        self.state[9:12] = np.array(angular_velocity, dtype=float)

    def set_wind(self, wind):
        self.wind = np.array(wind, dtype=float)

    def _resolve_wind(self, wind, position=None):
        """解析风场输入：None使用默认风、callable则传入(position, dt)、array直接使用"""
        if wind is None:
            return self.wind
        if callable(wind):
            pos = position if position is not None else self.state[0:3]
            return np.array(wind(pos, self.dt), dtype=float)
        return np.array(wind, dtype=float)

    def _rotor_step(self, desired_u):#将控制量转化为旋翼转速，并更新旋翼状态
        thrusts_cmd = self.allocator.allocate_thrusts(desired_u)
        omegas_cmd = self.allocator.thrusts_to_omegas(thrusts_cmd)

        for i, rotor in enumerate(self.rotors):
            rotor.update(omegas_cmd[i], self.dt)

        omegas = np.array([r.omega for r in self.rotors], dtype=float)
        return self.allocator.omegas_to_u(omegas)

    def dynamics(self, t, state, u, wind=None):
        # 状态拆分
        x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = state
        R = rotation_matrix(roll, pitch, yaw)  # 旋转矩阵用于将机体坐标系的力转换到世界坐标系
        # 把风速分解为相对于地面和相对于无人机的风速，计算空气阻力
        wind_vec = self._resolve_wind(wind, position=state[0:3])
        v_vector = np.array([vx, vy, vz])
        v_rel = v_vector - wind_vec#相对风速
        drag = -self.k_drag * np.linalg.norm(v_rel) * v_rel
        #空气阻力计算方法：f= k*|v_rel|*v_rel，方向与相对风速相反，大小与相对风速的平方成正比

        acc = 1/self.m * (R @ np.array([0,0,u[0]]) + drag - np.array([0,0,self.m * self.g]))#R @ np.array([0,0,u[0]])绝对推力
        omega = np.array([wx, wy, wz])
        tau = np.array([u[1], u[2], u[3]])  # 控制力矩由分配矩阵计算得到，包含了旋翼产生的滚转、俯仰和偏航力矩

        # 旋翼陀螺效应: 由 Newton-Euler 方程推导
        # H_total = I*ω + J_r*Ω_net*e_z
        # I*ω_dot = τ - ω×(I*ω) - J_r*Ω_net*(ω×e_z)
        Omega_net = sum(r.direction * r.omega for r in self.rotors)
        e_z = np.array([0.0, 0.0, 1.0])
        tau_gyro = -self.J_rotor * Omega_net * np.cross(omega, e_z)

        omega_dot = self.I_inv @ (tau + tau_gyro - np.cross(omega, self.I @ omega))  # 欧拉方程：I*omega_dot = tau + tau_gyro - omega x (I*omega)
        #把机体系的角速度[wx, wy, wz]转换为角加速度[roll_dot, pitch_dot, yaw_dot]
        c_pitch = np.cos(pitch)#限幅，避免cos(pitch)过小导致数值不稳定
        if abs(c_pitch) < 1e-6:
            c_pitch = np.sign(c_pitch) * 1e-6 if c_pitch != 0 else 1e-6
        t_pitch = np.sin(pitch) / c_pitch
        c_roll = np.cos(roll)
        s_roll = np.sin(roll)

        roll_dot = wx + s_roll * t_pitch * wy + c_roll * t_pitch * wz
        pitch_dot = c_roll * wy - s_roll * wz
        yaw_dot = (s_roll / c_pitch) * wy + (c_roll / c_pitch) * wz

        state_dot = np.array([vx, vy, vz,
                              acc[0], acc[1], acc[2],
                              roll_dot, pitch_dot, yaw_dot,
                              omega_dot[0], omega_dot[1], omega_dot[2]])
        return state_dot

    def update_state(self, control, wind=None):
        dt = self.dt
        state = self.state

        control = np.array(control, dtype=float)
        if control.shape[0] == 4:
            control_eff = self._rotor_step(control)
        elif control.shape[0] == 3:
            total_thrust = self.m * (self.g + control[2])
            control_eff = np.array([total_thrust, 0.0, 0.0, 0.0])
        else:
            raise ValueError("control must be 3D or 4D vector")

        k1 = self.dynamics(0, state, control_eff, wind)
        k2 = self.dynamics(0, state + 0.5 * dt * k1, control_eff, wind)
        k3 = self.dynamics(0, state + 0.5 * dt * k2, control_eff, wind)
        k4 = self.dynamics(0, state + dt * k3, control_eff, wind)

        self.state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)#RK4数值积分更新状态

    def get_state(self):
        return self.state[0:3].copy(), self.state[3:6].copy(), self.state[6:9].copy(), self.state[9:12].copy()
