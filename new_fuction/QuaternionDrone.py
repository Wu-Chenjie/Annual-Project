"""
QuaternionDrone.py -- 基于四元数的四旋翼无人机动力学模型

继承自 Packaged_Drone_Dynamics_Model.Drone，将内部姿态表示从欧拉角替换为
单位四元数 [q0, q1, q2, q3]（Hamilton 约定，q0 为标量部分），避免万向锁
（Gimbal Lock）并提高大姿态角下的数值稳定性。

对外接口（get_state / set_initial_state / update_state）与原 Drone 完全一致，
控制器无需任何修改即可直接使用。

状态向量（13维）:
    [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
     位置(3)   速度(3)     四元数(4)        角速度(3)

四元数约定:
    q = q0 + q1*i + q2*j + q3*k
    单位四元数满足 |q| = 1
    旋转矩阵由 quat_to_rotation_matrix(q) 给出

作者: AI辅助生成
日期: 2026-04-08
"""

import numpy as np
from Packaged_Drone_Dynamics_Model import Drone


# ============================================================
#  四元数工具函数
# ============================================================

def euler_to_quat(roll, pitch, yaw):
    """
    ZYX 欧拉角 -> 单位四元数 [q0, q1, q2, q3]

    参数:
        roll  (float): 滚转角 (rad)，绕 X 轴
        pitch (float): 俯仰角 (rad)，绕 Y 轴
        yaw   (float): 偏航角 (rad)，绕 Z 轴

    返回:
        numpy.ndarray: 四元数 [q0, q1, q2, q3]，q0 为标量部分

    说明:
        等价旋转顺序为 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，
        对应四元数 q = qz * qy * qx（Hamilton 乘法）。
    """
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)

    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy

    return np.array([q0, q1, q2, q3], dtype=float)


def quat_to_euler(q):
    """
    单位四元数 [q0, q1, q2, q3] -> ZYX 欧拉角 [roll, pitch, yaw]

    参数:
        q (array-like): 四元数 [q0, q1, q2, q3]

    返回:
        numpy.ndarray: [roll, pitch, yaw]（单位: rad）

    注意:
        当 pitch 接近 ±90° 时存在奇异性（万向锁），此处做了 clamp 处理
        以避免数值异常。使用四元数内部积分即可规避该问题，此函数仅用于
        输出转换。
    """
    q0, q1, q2, q3 = q

    # 滚转角 roll (绕 X 轴)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # 俯仰角 pitch (绕 Y 轴)，clamp 防止 asin 溢出
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # 偏航角 yaw (绕 Z 轴)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=float)


def quat_to_rotation_matrix(q):
    """
    单位四元数 [q0, q1, q2, q3] -> 3x3 旋转矩阵 R

    参数:
        q (array-like): 四元数 [q0, q1, q2, q3]

    返回:
        numpy.ndarray: 3x3 旋转矩阵，将机体坐标系向量转换到世界坐标系

    公式:
        R = I + 2*q0*[q_vec]_x + 2*[q_vec]_x^2
        其中 [q_vec]_x 为 q_vec = [q1, q2, q3] 的反对称矩阵。
        等价于直接展开的九元素表达式，此处采用后者以提高计算效率。
    """
    q0, q1, q2, q3 = q

    # 预计算常用乘积
    q0q0 = q0 * q0
    q1q1 = q1 * q1
    q2q2 = q2 * q2
    q3q3 = q3 * q3
    q0q1 = q0 * q1
    q0q2 = q0 * q2
    q0q3 = q0 * q3
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q2q3 = q2 * q3

    R = np.array([
        [q0q0 + q1q1 - q2q2 - q3q3, 2.0 * (q1q2 - q0q3),       2.0 * (q1q3 + q0q2)],
        [2.0 * (q1q2 + q0q3),       q0q0 - q1q1 + q2q2 - q3q3, 2.0 * (q2q3 - q0q1)],
        [2.0 * (q1q3 - q0q2),       2.0 * (q2q3 + q0q1),       q0q0 - q1q1 - q2q2 + q3q3],
    ], dtype=float)

    return R


def _quat_multiply(p, q):
    """
    四元数 Hamilton 乘法: p ⊗ q

    参数:
        p, q (array-like): 四元数 [w, x, y, z]

    返回:
        numpy.ndarray: 乘积四元数 [w, x, y, z]
    """
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return np.array([
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
    ], dtype=float)


# ============================================================
#  QuaternionDrone 类
# ============================================================

class QuaternionDrone(Drone):
    """
    基于四元数的四旋翼无人机动力学模型。

    继承自 Drone，内部使用 13 维状态向量：
        [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]

    优势:
        - 避免欧拉角万向锁（Gimbal Lock）
        - 大姿态角下数值稳定性更好
        - 四元数微分方程为双线性形式，无三角函数和除法

    对外接口与 Drone 完全一致:
        - set_initial_state(position, velocity, attitude_euler, angular_velocity)
        - get_state() -> (pos, vel, euler, ang_vel)
        - update_state(control, wind)
    控制器无需修改即可直接使用。
    """

    def __init__(self, m=1.0, inertia=None, L=0.2, dt=0.01):
        """
        初始化四元数无人机模型。

        参数:
            m       (float): 质量 (kg)，默认 1.0
            inertia (ndarray): 3x3 惯量矩阵，默认 diag([0.01, 0.01, 0.02])
            L       (float): 机臂长度 (m)，默认 0.2
            dt      (float): 仿真步长 (s)，默认 0.01
        """
        super().__init__(m=m, inertia=inertia, L=L, dt=dt)
        # 将父类的 12 维状态扩展为 13 维（四元数替代欧拉角）
        self.state = np.zeros(13)
        self.state[6] = 1.0  # q0=1 表示零旋转（单位四元数）

    # ----------------------------------------------------------
    #  接口重写
    # ----------------------------------------------------------

    def set_initial_state(self, position, velocity, attitude=None,
                          angular_velocity=None, dt=None):
        """
        设置初始状态，接受欧拉角输入，内部自动转换为四元数存储。

        参数:
            position        (array-like): [x, y, z] 初始位置 (m)
            velocity        (array-like): [vx, vy, vz] 初始速度 (m/s)
            attitude        (array-like): [roll, pitch, yaw] 初始欧拉角 (rad)，默认 [0,0,0]
            angular_velocity(array-like): [wx, wy, wz] 初始角速度 (rad/s)，默认 [0,0,0]
            dt              (float):      仿真步长 (s)，None 表示不修改
        """
        if attitude is None:
            attitude = [0.0, 0.0, 0.0]
        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        if dt is not None:
            self.dt = dt

        self.state[0:3] = np.array(position, dtype=float)
        self.state[3:6] = np.array(velocity, dtype=float)

        # 欧拉角 -> 四元数
        att = np.array(attitude, dtype=float)
        self.state[6:10] = euler_to_quat(att[0], att[1], att[2])

        self.state[10:13] = np.array(angular_velocity, dtype=float)

    def get_state(self):
        """
        返回当前状态（欧拉角形式），与 Drone.get_state() 接口完全一致。

        返回:
            tuple: (pos[3], vel[3], euler[3], ang_vel[3])
                   euler = [roll, pitch, yaw]
        """
        pos = self.state[0:3].copy()
        vel = self.state[3:6].copy()
        quat = self.state[6:10]
        euler = quat_to_euler(quat)
        ang_vel = self.state[10:13].copy()
        return pos, vel, euler, ang_vel

    # ----------------------------------------------------------
    #  动力学方程（四元数版本）
    # ----------------------------------------------------------

    def dynamics(self, t, state, u, wind=None):
        """
        13 维状态的动力学方程，使用四元数微分方程替代欧拉角运动学。

        参数:
            t     (float):   时间（RK4 接口需要，实际未使用）
            state (ndarray): 13 维状态向量
            u     (ndarray): 控制量 [T, Mx, My, Mz]
            wind  (varies):  风场输入（None / callable / array）

        返回:
            numpy.ndarray: 13 维状态导数

        四元数微分方程:
            q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
        其中 ⊗ 为 Hamilton 四元数乘法。
        """
        # --- 状态拆分 ---
        pos = state[0:3]           # [x, y, z]
        vel = state[3:6]           # [vx, vy, vz]
        quat = state[6:10]         # [q0, q1, q2, q3]
        omega = state[10:13]       # [wx, wy, wz]

        # --- 旋转矩阵（由四元数计算）---
        R = quat_to_rotation_matrix(quat)

        # --- 平移动力学（与父类一致）---
        wind_vec = self._resolve_wind(wind, position=pos)
        v_rel = vel - wind_vec                              # 相对风速
        drag = -self.k_drag * np.linalg.norm(v_rel) * v_rel  # 空气阻力

        # 加速度 = (推力(机体z轴) + 阻力 - 重力) / 质量
        acc = (1.0 / self.m) * (
            R @ np.array([0.0, 0.0, u[0]]) + drag
            - np.array([0.0, 0.0, self.m * self.g])
        )

        # --- 旋转动力学 ---
        tau = np.array([u[1], u[2], u[3]])  # 控制力矩

        # 旋翼陀螺效应: I*ω_dot = τ - ω×(I*ω) - J_r*Ω_net*(ω×e_z)
        Omega_net = sum(r.direction * r.omega for r in self.rotors)
        e_z = np.array([0.0, 0.0, 1.0])
        tau_gyro = -self.J_rotor * Omega_net * np.cross(omega, e_z)

        # 欧拉方程: I * omega_dot = tau + tau_gyro - omega x (I * omega)
        omega_dot = self.I_inv @ (tau + tau_gyro - np.cross(omega, self.I @ omega))

        # --- 四元数微分方程 ---
        # q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
        q_dot = 0.5 * _quat_multiply(quat, omega_quat)

        # --- 组装 13 维状态导数 ---
        state_dot = np.zeros(13)
        state_dot[0:3] = vel          # 位置导数 = 速度
        state_dot[3:6] = acc          # 速度导数 = 加速度
        state_dot[6:10] = q_dot       # 四元数导数
        state_dot[10:13] = omega_dot  # 角速度导数

        return state_dot

    # ----------------------------------------------------------
    #  状态更新（RK4 积分 + 四元数归一化）
    # ----------------------------------------------------------

    def update_state(self, control, wind=None):
        """
        使用 RK4 积分更新 13 维状态，并在每步后归一化四元数。

        参数:
            control (array-like): 控制量，支持4维 [T, Mx, My, Mz] 或
                                  3维 [ax, ay, az]（向后兼容）
            wind    (varies):     风场输入（None / callable / array）

        说明:
            - 4维输入经过旋翼子系统（电机动力学 + 推力分配）
            - 3维输入直接转换为推力（向后兼容模式，力矩为零）
            - 每个 RK4 步后对四元数进行归一化，防止数值漂移
        """
        dt = self.dt
        state = self.state

        # 控制量预处理（与父类一致）
        control = np.array(control, dtype=float)
        if control.shape[0] == 4:
            control_eff = self._rotor_step(control)
        elif control.shape[0] == 3:
            total_thrust = self.m * (self.g + control[2])
            control_eff = np.array([total_thrust, 0.0, 0.0, 0.0])
        else:
            raise ValueError("control must be 3D or 4D vector")

        # RK4 四阶龙格-库塔积分
        k1 = self.dynamics(0, state, control_eff, wind)
        k2 = self.dynamics(0, state + 0.5 * dt * k1, control_eff, wind)
        k3 = self.dynamics(0, state + 0.5 * dt * k2, control_eff, wind)
        k4 = self.dynamics(0, state + dt * k3, control_eff, wind)

        self.state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # 四元数归一化，防止数值漂移导致 |q| != 1
        q_norm = np.linalg.norm(self.state[6:10])
        if q_norm > 1e-10:
            self.state[6:10] /= q_norm
        else:
            # 极端异常情况，重置为单位四元数
            self.state[6:10] = np.array([1.0, 0.0, 0.0, 0.0])
