"""
VectorizedSwarm -- 多架无人机批量向量化状态更新模块

将 N 架无人机的 12 维状态打包为 (N, 12) 矩阵，利用 NumPy 广播和批量矩阵运算
一次性完成所有无人机的动力学计算和 RK4 积分，消除 Python for 循环瓶颈。

简化说明：
    - 不包含旋翼子系统（Rotor / ControlAllocator），直接使用控制量 [T, Mx, My, Mz]
    - 不包含旋翼陀螺效应
    - 适用于大规模集群的快速批量仿真

状态向量定义（与 Drone 类一致）：
    state[i] = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]

控制量定义：
    control[i] = [T, Mx, My, Mz]  （总推力, 滚转力矩, 俯仰力矩, 偏航力矩）
"""

import numpy as np
import time as _time


class VectorizedSwarm:
    """多无人机批量向量化仿真器。

    Parameters
    ----------
    num_drones : int
        无人机数量 N。
    m : float
        单架无人机质量 (kg)，默认 1.0。
    inertia : np.ndarray or None
        3x3 惯量矩阵。None 时使用默认对角矩阵 diag(0.01, 0.01, 0.02)。
    L : float
        机臂长度 (m)，默认 0.2（本类未直接使用，保留接口兼容性）。
    dt : float
        仿真步长 (s)，默认 0.01。
    k_drag : float
        空气阻力系数，默认 0.05。
    """

    def __init__(self, num_drones, m=1.0, inertia=None, L=0.2, dt=0.01, k_drag=0.05):
        self.N = num_drones
        self.m = m
        self.g = 9.81
        self.L = L
        self.dt = dt
        self.k_drag = k_drag

        # 惯量矩阵及其逆（对角阵，存为 (3,) 向量以便广播）
        if inertia is None:
            inertia = np.diag([0.01, 0.01, 0.02])
        self.I = inertia                          # (3, 3)
        self.I_inv = np.linalg.inv(inertia)        # (3, 3)

        # 提取对角惯量，用于向量化欧拉方程（假设惯量矩阵为对角阵）
        self.I_diag = np.diag(self.I).copy()       # (3,)
        self.I_inv_diag = 1.0 / self.I_diag        # (3,)

        # 状态矩阵: (N, 12) -- [pos(3), vel(3), euler(3), ang_vel(3)]
        self.states = np.zeros((num_drones, 12))

    # ========================================================================
    # 初始化方法
    # ========================================================================

    def set_initial_states(self, positions, velocities=None, attitudes=None, angular_velocities=None):
        """批量设置所有无人机初始状态。

        Parameters
        ----------
        positions : array_like, shape (N, 3)
            初始位置 [x, y, z]。
        velocities : array_like, shape (N, 3) or None
            初始速度，默认全零。
        attitudes : array_like, shape (N, 3) or None
            初始欧拉角 [roll, pitch, yaw]，默认全零。
        angular_velocities : array_like, shape (N, 3) or None
            初始角速度 [wx, wy, wz]，默认全零。
        """
        self.states[:, 0:3] = np.asarray(positions, dtype=float)
        if velocities is not None:
            self.states[:, 3:6] = np.asarray(velocities, dtype=float)
        else:
            self.states[:, 3:6] = 0.0
        if attitudes is not None:
            self.states[:, 6:9] = np.asarray(attitudes, dtype=float)
        else:
            self.states[:, 6:9] = 0.0
        if angular_velocities is not None:
            self.states[:, 9:12] = np.asarray(angular_velocities, dtype=float)
        else:
            self.states[:, 9:12] = 0.0

    # ========================================================================
    # 批量旋转矩阵
    # ========================================================================

    @staticmethod
    def _batch_rotation_matrices(euler):
        """批量计算 ZYX 欧拉角旋转矩阵。

        Parameters
        ----------
        euler : np.ndarray, shape (N, 3)
            欧拉角 [roll, pitch, yaw]。

        Returns
        -------
        R : np.ndarray, shape (N, 3, 3)
            每架无人机的旋转矩阵（从机体系到世界系）。
        """
        roll  = euler[:, 0]   # (N,)
        pitch = euler[:, 1]
        yaw   = euler[:, 2]

        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)

        N = euler.shape[0]
        R = np.zeros((N, 3, 3))

        # ZYX 旋转矩阵 R = Rz @ Ry @ Rx，逐元素向量化赋值
        R[:, 0, 0] = cy * cp
        R[:, 0, 1] = cy * sp * sr - sy * cr
        R[:, 0, 2] = cy * sp * cr + sy * sr
        R[:, 1, 0] = sy * cp
        R[:, 1, 1] = sy * sp * sr + cy * cr
        R[:, 1, 2] = sy * sp * cr - cy * sr
        R[:, 2, 0] = -sp
        R[:, 2, 1] = cp * sr
        R[:, 2, 2] = cp * cr

        return R

    # ========================================================================
    # 批量动力学
    # ========================================================================

    def vectorized_dynamics(self, states, controls, winds=None):
        """批量计算所有无人机的状态导数（无 Python 循环）。

        Parameters
        ----------
        states : np.ndarray, shape (N, 12)
            当前状态。
        controls : np.ndarray, shape (N, 4)
            控制量 [T, Mx, My, Mz]。
        winds : np.ndarray, shape (N, 3) or None
            风速向量，默认无风。

        Returns
        -------
        state_dots : np.ndarray, shape (N, 12)
            状态导数。
        """
        N = states.shape[0]

        # ---- 状态拆分 ----
        pos   = states[:, 0:3]    # (N, 3) 位置
        vel   = states[:, 3:6]    # (N, 3) 速度
        euler = states[:, 6:9]    # (N, 3) 欧拉角
        omega = states[:, 9:12]   # (N, 3) 角速度

        T  = controls[:, 0]       # (N,)   总推力
        tau = controls[:, 1:4]    # (N, 3) 力矩 [Mx, My, Mz]

        # ---- 风场 ----
        if winds is None:
            winds = np.zeros((N, 3))

        # ---- 批量旋转矩阵 ----
        R = self._batch_rotation_matrices(euler)  # (N, 3, 3)

        # ---- 推力在世界系的分量 ----
        # 机体系推力向量 [0, 0, T]，经旋转矩阵变换到世界系
        thrust_body = np.zeros((N, 3))
        thrust_body[:, 2] = T                        # (N, 3)
        # 批量矩阵-向量乘法: R @ thrust_body（逐无人机）
        thrust_world = np.einsum('nij,nj->ni', R, thrust_body)  # (N, 3)

        # ---- 空气阻力（批量计算） ----
        v_rel = vel - winds                           # (N, 3) 相对风速
        v_rel_norm = np.linalg.norm(v_rel, axis=1, keepdims=True)  # (N, 1)
        drag = -self.k_drag * v_rel_norm * v_rel      # (N, 3)  f = -k * |v_rel| * v_rel

        # ---- 平动加速度 ----
        gravity = np.zeros((N, 3))
        gravity[:, 2] = -self.m * self.g              # 重力沿 z 轴向下
        acc = (thrust_world + drag + gravity) / self.m  # (N, 3)

        # ---- 欧拉方程：I * omega_dot = tau - omega x (I * omega) ----
        # 对角惯量矩阵的简化：I * omega 逐元素乘法
        I_omega = omega * self.I_diag[np.newaxis, :]           # (N, 3)
        cross_term = np.cross(omega, I_omega)                   # (N, 3)
        omega_dot = (tau - cross_term) * self.I_inv_diag[np.newaxis, :]  # (N, 3)

        # ---- 欧拉角运动学（机体角速度 -> 欧拉角导数） ----
        roll  = euler[:, 0]
        pitch = euler[:, 1]

        c_pitch = np.cos(pitch)
        # 避免 cos(pitch) 过小导致数值不稳定
        c_pitch = np.where(np.abs(c_pitch) < 1e-6,
                           np.sign(c_pitch) * 1e-6, c_pitch)
        # 修复 cos(pitch)==0 的边界情况
        c_pitch = np.where(c_pitch == 0, 1e-6, c_pitch)

        s_pitch = np.sin(pitch)
        t_pitch = s_pitch / c_pitch  # tan(pitch)
        c_roll  = np.cos(roll)
        s_roll  = np.sin(roll)

        wx = omega[:, 0]
        wy = omega[:, 1]
        wz = omega[:, 2]

        roll_dot  = wx + s_roll * t_pitch * wy + c_roll * t_pitch * wz
        pitch_dot = c_roll * wy - s_roll * wz
        yaw_dot   = (s_roll / c_pitch) * wy + (c_roll / c_pitch) * wz

        # ---- 组装状态导数 ----
        state_dots = np.zeros_like(states)
        state_dots[:, 0:3]  = vel                                      # 位置导数 = 速度
        state_dots[:, 3:6]  = acc                                      # 速度导数 = 加速度
        state_dots[:, 6]    = roll_dot                                 # 欧拉角导数
        state_dots[:, 7]    = pitch_dot
        state_dots[:, 8]    = yaw_dot
        state_dots[:, 9:12] = omega_dot                                # 角速度导数

        return state_dots

    # ========================================================================
    # 批量 RK4 积分
    # ========================================================================

    def update_states(self, controls, winds=None):
        """使用 RK4 四阶龙格-库塔方法批量更新所有无人机状态。

        Parameters
        ----------
        controls : np.ndarray, shape (N, 4)
            控制量 [T, Mx, My, Mz]。
        winds : np.ndarray, shape (N, 3) or None
            风速向量，默认无风。
        """
        dt = self.dt
        s = self.states
        controls = np.asarray(controls, dtype=float)

        if winds is not None:
            winds = np.asarray(winds, dtype=float)

        k1 = self.vectorized_dynamics(s,                    controls, winds)
        k2 = self.vectorized_dynamics(s + 0.5 * dt * k1,   controls, winds)
        k3 = self.vectorized_dynamics(s + 0.5 * dt * k2,   controls, winds)
        k4 = self.vectorized_dynamics(s + dt * k3,          controls, winds)

        self.states = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # ========================================================================
    # 状态读取
    # ========================================================================

    def get_states(self):
        """返回所有无人机的状态分量。

        Returns
        -------
        positions : np.ndarray, shape (N, 3)
        velocities : np.ndarray, shape (N, 3)
        attitudes : np.ndarray, shape (N, 3)
        angular_velocities : np.ndarray, shape (N, 3)
        """
        return (
            self.states[:, 0:3].copy(),
            self.states[:, 3:6].copy(),
            self.states[:, 6:9].copy(),
            self.states[:, 9:12].copy(),
        )

    def get_state(self, idx):
        """获取单架无人机的状态（兼容 Drone.get_state 接口）。

        Parameters
        ----------
        idx : int
            无人机索引。

        Returns
        -------
        tuple of np.ndarray
            (position(3,), velocity(3,), attitude(3,), angular_velocity(3,))
        """
        s = self.states[idx]
        return s[0:3].copy(), s[3:6].copy(), s[6:9].copy(), s[9:12].copy()


# ============================================================================
# 性能对比 Benchmark
# ============================================================================

if __name__ == "__main__":
    import sys
    import os

    # 将当前目录加入路径，以便导入同级模块
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    from Packaged_Drone_Dynamics_Model import Drone

    # ---------- 参数设置 ----------
    num_drones = 50          # 无人机数量
    num_steps  = 1000        # 仿真步数
    dt = 0.01

    print(f"性能对比 Benchmark")
    print(f"  无人机数量: {num_drones}")
    print(f"  仿真步数:   {num_steps}")
    print(f"  仿真步长:   {dt} s")
    print(f"  总仿真时长: {num_steps * dt:.1f} s")
    print()

    # ---------- 准备初始位置和控制量 ----------
    np.random.seed(42)
    init_positions = np.random.uniform(-5, 5, (num_drones, 3))
    init_positions[:, 2] = np.abs(init_positions[:, 2])  # z >= 0

    # 恒定控制量：悬停推力 + 微小力矩
    hover_thrust = 1.0 * 9.81  # m * g
    controls = np.zeros((num_drones, 4))
    controls[:, 0] = hover_thrust
    controls[:, 1] = 0.01  # 微小滚转力矩
    controls[:, 2] = 0.01  # 微小俯仰力矩

    # ---------- 方法 1: 逐个 Drone 对象 (Python 循环) ----------
    drones = []
    for i in range(num_drones):
        d = Drone(dt=dt)
        d.set_initial_state(init_positions[i], [0, 0, 0])
        drones.append(d)

    t0 = _time.perf_counter()
    for step in range(num_steps):
        for i, d in enumerate(drones):
            d.update_state(controls[i])
    t_loop = _time.perf_counter() - t0

    # 收集最终位置
    final_pos_loop = np.array([d.get_state()[0] for d in drones])

    # ---------- 方法 2: VectorizedSwarm (向量化) ----------
    swarm = VectorizedSwarm(num_drones, dt=dt)
    swarm.set_initial_states(init_positions)

    t0 = _time.perf_counter()
    for step in range(num_steps):
        swarm.update_states(controls)
    t_vec = _time.perf_counter() - t0

    # 收集最终位置
    final_pos_vec = swarm.get_states()[0]

    # ---------- 结果对比 ----------
    print(f"逐个循环 (Drone 对象):   {t_loop:.3f} s")
    print(f"向量化 (VectorizedSwarm): {t_vec:.3f} s")
    speedup = t_loop / t_vec if t_vec > 0 else float('inf')
    print(f"加速比: {speedup:.1f}x")
    print()

    # ---------- 数值一致性检验 ----------
    # 注意：由于 VectorizedSwarm 不包含旋翼子系统（电机动力学），
    # 与逐个 Drone 对象的结果会有差异。这里仅展示量级一致性。
    pos_diff = np.linalg.norm(final_pos_loop - final_pos_vec, axis=1)
    print(f"最终位置差异统计（因旋翼子系统差异导致）:")
    print(f"  均值: {np.mean(pos_diff):.4f} m")
    print(f"  最大: {np.max(pos_diff):.4f} m")
    print(f"  最小: {np.min(pos_diff):.4f} m")
    print()

    # ---------- 纯向量化一致性自检 ----------
    # 用两个独立 VectorizedSwarm 实例验证确定性
    swarm_a = VectorizedSwarm(num_drones, dt=dt)
    swarm_a.set_initial_states(init_positions)
    swarm_b = VectorizedSwarm(num_drones, dt=dt)
    swarm_b.set_initial_states(init_positions)

    for step in range(100):
        swarm_a.update_states(controls)
        swarm_b.update_states(controls)

    diff = np.max(np.abs(swarm_a.states - swarm_b.states))
    print(f"向量化确定性自检（两次独立运行最大差异）: {diff:.2e}")
    if diff < 1e-12:
        print("  -> 通过：结果完全一致（确定性计算）")
    else:
        print("  -> 警告：存在数值差异")
