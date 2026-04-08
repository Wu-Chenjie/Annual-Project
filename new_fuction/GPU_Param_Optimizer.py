"""
GPU 加速 PID + 滑模控制参数联合优化器

使用 PyTorch 可微分仿真 (BPTT) 同时优化：
  - 串级 PID 控制器参数（位置环 + 速度环 + 姿态环）
  - 指数趋近律滑模控制器参数（姿态环替代方案）

支持 CUDA / MPS / CPU 自动检测，优先使用 GPU 加速。
运行后在终端输出可直接复制使用的最佳参数。

用法：
    cd new_fuction && python GPU_Param_Optimizer.py
"""

import time
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np


# ============================================================
# 设备检测
# ============================================================

def get_device():
    """自适应选择最优计算设备：CUDA > MPS > CPU"""
    if torch.cuda.is_available():
        dev = torch.device("cuda")
        name = torch.cuda.get_device_name(0)
        print(f"  [GPU] CUDA 可用 -- {name}")
    elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        dev = torch.device("mps")
        print("  [GPU] Apple MPS 可用")
    else:
        dev = torch.device("cpu")
        print("  [CPU] 未检测到 GPU，使用 CPU")
    return dev


# ============================================================
# 可微分无人机动力学
# ============================================================

class DifferentiableDrone(nn.Module):
    """
    全 PyTorch 张量实现的四旋翼动力学，保持计算图可微分。
    物理参数与 Packaged_Drone_Dynamics_Model.Drone 完全一致。
    支持批量仿真 (batch_size, 12)。
    """

    def __init__(self, dt=0.01, m=1.0, g=9.81,
                 inertia=None, k_drag=0.05, device="cpu"):
        super().__init__()
        self.dt = dt
        self.m = m
        self.g = g
        self.k_drag = k_drag
        if inertia is None:
            inertia = [0.01, 0.01, 0.02]
        # 注册为 buffer（不参与梯度，但会跟随 .to(device)）
        self.register_buffer("I", torch.tensor(inertia, dtype=torch.float32))
        self.register_buffer(
            "g_vec_template",
            torch.tensor([0.0, 0.0, -m * g], dtype=torch.float32),
        )

    def dynamics(self, state, u):
        """
        计算 state_dot (batch, 12)
        state: (B, 12)  u: (B, 4) = [T, Mx, My, Mz]
        """
        vel = state[:, 3:6]
        att = state[:, 6:9]
        omega = state[:, 9:12]

        roll, pitch, yaw = att[:, 0], att[:, 1], att[:, 2]

        # ZYX 旋转矩阵（向量化）
        c_r, s_r = torch.cos(roll), torch.sin(roll)
        c_p, s_p = torch.cos(pitch), torch.sin(pitch)
        c_y, s_y = torch.cos(yaw), torch.sin(yaw)

        B = state.shape[0]
        R = torch.zeros((B, 3, 3), dtype=state.dtype, device=state.device)
        R[:, 0, 0] = c_y * c_p
        R[:, 0, 1] = c_y * s_p * s_r - s_y * c_r
        R[:, 0, 2] = c_y * s_p * c_r + s_y * s_r
        R[:, 1, 0] = s_y * c_p
        R[:, 1, 1] = s_y * s_p * s_r + c_y * c_r
        R[:, 1, 2] = s_y * s_p * c_r - c_y * s_r
        R[:, 2, 0] = -s_p
        R[:, 2, 1] = c_p * s_r
        R[:, 2, 2] = c_p * c_r

        # 空气阻力
        v_norm = torch.norm(vel, dim=1, keepdim=True).clamp(min=1e-8)
        drag = -self.k_drag * v_norm * vel

        # 推力（机体 z 轴 -> 世界坐标系）
        thrust_body = torch.zeros_like(vel)
        thrust_body[:, 2] = u[:, 0]
        thrust_world = torch.bmm(R, thrust_body.unsqueeze(2)).squeeze(2)

        # 线加速度
        g_vec = self.g_vec_template.unsqueeze(0).expand(B, -1)
        acc = (thrust_world + drag + g_vec) / self.m

        # 角加速度（欧拉方程，忽略陀螺效应简化计算图）
        tau = u[:, 1:4]
        I = self.I.unsqueeze(0)  # (1, 3)
        omega_cross = torch.cross(omega, omega * I, dim=1)
        omega_dot = (tau - omega_cross) / I

        # 欧拉角速率
        c_pitch = torch.where(
            torch.abs(c_p) < 1e-6,
            torch.ones_like(c_p) * 1e-6,
            c_p,
        )
        t_pitch = s_p / c_pitch

        roll_dot = omega[:, 0] + s_r * t_pitch * omega[:, 1] + c_r * t_pitch * omega[:, 2]
        pitch_dot = c_r * omega[:, 1] - s_r * omega[:, 2]
        yaw_dot = (s_r / c_pitch) * omega[:, 1] + (c_r / c_pitch) * omega[:, 2]
        att_dot = torch.stack([roll_dot, pitch_dot, yaw_dot], dim=1)

        return torch.cat([vel, acc, att_dot, omega_dot], dim=1)

    def step(self, state, u):
        """RK4 单步积分"""
        k1 = self.dynamics(state, u)
        k2 = self.dynamics(state + 0.5 * self.dt * k1, u)
        k3 = self.dynamics(state + 0.5 * self.dt * k2, u)
        k4 = self.dynamics(state + self.dt * k3, u)
        return state + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# ============================================================
# 可微分 PID 控制器
# ============================================================

class DifferentiablePID(nn.Module):
    """
    串级 PID 控制器（位置环 -> 速度环 -> 姿态环 -> 角速度环）。
    所有增益为 nn.Parameter，通过 softplus 映射保证非负。
    """

    def __init__(self, m=1.0, g=9.81, inertia=None, dt=0.01, device="cpu"):
        super().__init__()
        self.m = m
        self.g = g
        self.dt = dt
        if inertia is None:
            inertia = [0.01, 0.01, 0.02]
        self.register_buffer("I", torch.tensor(inertia, dtype=torch.float32))

        # 位置环 PID
        self.kp_pos_raw = nn.Parameter(torch.tensor([1.50, 1.50, 2.00]))
        self.ki_pos_raw = nn.Parameter(torch.tensor([0.05, 0.05, 0.10]))
        # 速度环 PID
        self.kp_vel_raw = nn.Parameter(torch.tensor([3.50, 3.50, 3.50]))
        self.ki_vel_raw = nn.Parameter(torch.tensor([0.50, 0.50, 0.50]))
        self.kd_vel_raw = nn.Parameter(torch.tensor([1.10, 1.10, 1.20]))
        # 姿态环 P + 角速度环 PID
        self.kp_att_raw = nn.Parameter(torch.tensor([9.00, 9.00, 4.00]))
        self.kp_rate_raw = nn.Parameter(torch.tensor([2.50, 2.50, 1.50]))
        self.ki_rate_raw = nn.Parameter(torch.tensor([0.10, 0.10, 0.05]))
        self.kd_rate_raw = nn.Parameter(torch.tensor([0.30, 0.30, 0.15]))

        # 限幅常量
        self.int_pos_lim = 2.0
        self.int_vel_lim = 2.0
        self.int_rate_lim = 1.0
        self.max_vel = 10.0
        self.max_acc = 10.0
        self.max_tilt = 0.5236  # 30 deg

    # --- softplus 保证参数非负 ---
    def _sp(self, raw):
        return nn.functional.softplus(raw)

    @property
    def kp_pos(self): return self._sp(self.kp_pos_raw)
    @property
    def ki_pos(self): return self._sp(self.ki_pos_raw)
    @property
    def kp_vel(self): return self._sp(self.kp_vel_raw)
    @property
    def ki_vel(self): return self._sp(self.ki_vel_raw)
    @property
    def kd_vel(self): return self._sp(self.kd_vel_raw)
    @property
    def kp_att(self): return self._sp(self.kp_att_raw)
    @property
    def kp_rate(self): return self._sp(self.kp_rate_raw)
    @property
    def ki_rate(self): return self._sp(self.ki_rate_raw)
    @property
    def kd_rate(self): return self._sp(self.kd_rate_raw)

    def forward(self, state, target_pos, target_yaw,
                prev_vel_err, prev_rate_err,
                int_pos, int_vel, int_rate):
        """
        一步控制律计算。
        state: (B, 12)  target_pos: (B, 3)  target_yaw: (B,)
        返回: u(B,4), vel_err, rate_err, int_pos, int_vel, int_rate
        """
        pos, vel = state[:, 0:3], state[:, 3:6]
        att, rate = state[:, 6:9], state[:, 9:12]

        # --- 位置环 ---
        pos_err = target_pos - pos
        int_pos = (int_pos + pos_err * self.dt).clamp(-self.int_pos_lim, self.int_pos_lim)
        des_vel = (self.kp_pos * pos_err + self.ki_pos * int_pos).clamp(-self.max_vel, self.max_vel)

        # --- 速度环 ---
        vel_err = des_vel - vel
        int_vel = (int_vel + vel_err * self.dt).clamp(-self.int_vel_lim, self.int_vel_lim)
        vel_deriv = (vel_err - prev_vel_err) / self.dt
        des_acc = self.kp_vel * vel_err + self.ki_vel * int_vel + self.kd_vel * vel_deriv
        acc_norm = torch.norm(des_acc, dim=1, keepdim=True)
        des_acc = torch.where(acc_norm > self.max_acc, des_acc / acc_norm * self.max_acc, des_acc)

        # --- 推力 + 期望姿态 ---
        thrust_vec = des_acc.clone()
        thrust_vec[:, 2] = thrust_vec[:, 2] + self.g
        T_mag = self.m * torch.norm(thrust_vec, dim=1, keepdim=True)
        T_mag = T_mag.clamp(0.1 * self.m * self.g, 2.0 * self.m * self.g)

        t_norm = thrust_vec / torch.norm(thrust_vec, dim=1, keepdim=True).clamp(min=1e-8)
        sy, cy = torch.sin(target_yaw), torch.cos(target_yaw)
        roll_arg = (t_norm[:, 0] * sy - t_norm[:, 1] * cy).clamp(-0.999, 0.999)
        des_roll = torch.asin(roll_arg).clamp(-self.max_tilt, self.max_tilt)
        cos_roll = torch.cos(des_roll).clamp(min=1e-6)
        pitch_arg = ((t_norm[:, 0] * cy + t_norm[:, 1] * sy) / cos_roll).clamp(-0.999, 0.999)
        des_pitch = torch.asin(pitch_arg).clamp(-self.max_tilt, self.max_tilt)
        des_att = torch.stack([des_roll, des_pitch, target_yaw], dim=1)

        # --- 姿态环 P ---
        att_err = des_att - att
        att_err_yaw = (att_err[:, 2:3] + torch.pi) % (2 * torch.pi) - torch.pi
        att_err = torch.cat([att_err[:, :2], att_err_yaw], dim=1)
        des_rate = self.kp_att * att_err

        # --- 角速度环 PID ---
        rate_err = des_rate - rate
        int_rate = (int_rate + rate_err * self.dt).clamp(-self.int_rate_lim, self.int_rate_lim)
        rate_deriv = (rate_err - prev_rate_err) / self.dt
        torques = (self.kp_rate * rate_err + self.ki_rate * int_rate + self.kd_rate * rate_deriv) * self.I

        u = torch.cat([T_mag, torques], dim=1)
        return u, vel_err, rate_err, int_pos, int_vel, int_rate


# ============================================================
# 可微分滑模控制器（姿态环）
# ============================================================

class DifferentiableSMC(nn.Module):
    """
    指数趋近律滑模控制器（姿态环），对应 SecondOrderSMC.py。
    所有参数为 nn.Parameter，通过 softplus 保证非负。
    输出为力矩（已乘惯量矩阵）。
    """

    def __init__(self, inertia=None, dt=0.01, delta=0.05, device="cpu"):
        super().__init__()
        self.dt = dt
        self.delta = delta
        self.ki_limit = 2.0
        if inertia is None:
            inertia = [0.01, 0.01, 0.02]
        self.register_buffer("I", torch.tensor(inertia, dtype=torch.float32))

        # 滑模参数
        self.lam_raw = nn.Parameter(torch.tensor([8.0, 8.0, 4.0]))
        self.epsilon_raw = nn.Parameter(torch.tensor([5.0, 5.0, 3.0]))
        self.k_raw = nn.Parameter(torch.tensor([8.0, 8.0, 4.0]))
        self.ki_raw = nn.Parameter(torch.tensor([0.5, 0.5, 0.3]))

    def _sp(self, raw):
        return nn.functional.softplus(raw)

    @property
    def lam(self): return self._sp(self.lam_raw)
    @property
    def epsilon(self): return self._sp(self.epsilon_raw)
    @property
    def k(self): return self._sp(self.k_raw)
    @property
    def ki(self): return self._sp(self.ki_raw)

    def _sat(self, s):
        """可微分饱和函数"""
        return torch.where(
            torch.abs(s) <= self.delta,
            s / self.delta,
            torch.sign(s),
        )

    def forward(self, att_err, rate, des_rate, int_s):
        """
        att_err: (B,3)  rate: (B,3)  des_rate: (B,3)  int_s: (B,3)
        返回: torques(B,3), int_s(B,3)
        """
        rate_err = des_rate - rate
        s = rate_err + self.lam * att_err
        sat_s = self._sat(s)
        int_s = (int_s + s * self.dt).clamp(-self.ki_limit, self.ki_limit)
        u_acc = self.lam * rate_err + self.epsilon * sat_s + self.k * s + self.ki * int_s
        torques = u_acc * self.I
        return torques, int_s


# ============================================================
# 混合控制器（位置环 PID + 姿态环 SMC）
# ============================================================

class HybridController(nn.Module):
    """
    位置环使用 PID（与 DifferentiablePID 共享），
    姿态环使用 SMC 替代 PID 角速度环。
    """

    def __init__(self, pid: DifferentiablePID, smc: DifferentiableSMC):
        super().__init__()
        self.pid = pid
        self.smc = smc

    def forward(self, state, target_pos, target_yaw,
                prev_vel_err, prev_rate_err,
                int_pos, int_vel, int_rate, int_s):
        """
        返回: u, vel_err, rate_err, int_pos, int_vel, int_rate, int_s
        """
        pos, vel = state[:, 0:3], state[:, 3:6]
        att, rate = state[:, 6:9], state[:, 9:12]

        # --- 位置环（复用 PID 代码） ---
        pos_err = target_pos - pos
        int_pos = (int_pos + pos_err * self.pid.dt).clamp(-self.pid.int_pos_lim, self.pid.int_pos_lim)
        des_vel = (self.pid.kp_pos * pos_err + self.pid.ki_pos * int_pos).clamp(-self.pid.max_vel, self.pid.max_vel)

        vel_err = des_vel - vel
        int_vel = (int_vel + vel_err * self.pid.dt).clamp(-self.pid.int_vel_lim, self.pid.int_vel_lim)
        vel_deriv = (vel_err - prev_vel_err) / self.pid.dt
        des_acc = self.pid.kp_vel * vel_err + self.pid.ki_vel * int_vel + self.pid.kd_vel * vel_deriv
        acc_norm = torch.norm(des_acc, dim=1, keepdim=True)
        des_acc = torch.where(acc_norm > self.pid.max_acc, des_acc / acc_norm * self.pid.max_acc, des_acc)

        thrust_vec = des_acc.clone()
        thrust_vec[:, 2] = thrust_vec[:, 2] + self.pid.g
        T_mag = self.pid.m * torch.norm(thrust_vec, dim=1, keepdim=True)
        T_mag = T_mag.clamp(0.1 * self.pid.m * self.pid.g, 2.0 * self.pid.m * self.pid.g)

        t_norm = thrust_vec / torch.norm(thrust_vec, dim=1, keepdim=True).clamp(min=1e-8)
        sy, cy = torch.sin(target_yaw), torch.cos(target_yaw)
        roll_arg = (t_norm[:, 0] * sy - t_norm[:, 1] * cy).clamp(-0.999, 0.999)
        des_roll = torch.asin(roll_arg).clamp(-self.pid.max_tilt, self.pid.max_tilt)
        cos_roll = torch.cos(des_roll).clamp(min=1e-6)
        pitch_arg = ((t_norm[:, 0] * cy + t_norm[:, 1] * sy) / cos_roll).clamp(-0.999, 0.999)
        des_pitch = torch.asin(pitch_arg).clamp(-self.pid.max_tilt, self.pid.max_tilt)
        des_att = torch.stack([des_roll, des_pitch, target_yaw], dim=1)

        # --- 姿态环（使用 SMC） ---
        att_err = des_att - att
        att_err_yaw = (att_err[:, 2:3] + torch.pi) % (2 * torch.pi) - torch.pi
        att_err = torch.cat([att_err[:, :2], att_err_yaw], dim=1)
        des_rate = self.pid.kp_att * att_err  # P 环计算期望角速度

        # SMC 替代 PID 角速度环
        torques, int_s = self.smc(att_err, rate, des_rate, int_s)
        rate_err = des_rate - rate  # 记录用于下一步

        u = torch.cat([T_mag, torques], dim=1)
        return u, vel_err, rate_err, int_pos, int_vel, int_rate, int_s


# ============================================================
# 优化器
# ============================================================

def make_scenarios(device, dt=0.01, sim_duration=3.0):
    """
    构建多个测试场景，评估参数鲁棒性。
    sim_duration: 每个场景的仿真时长（秒）。缩短可加速优化。
    """
    steps = int(sim_duration / dt)
    scenarios = [
        # 场景1: 垂直起飞阶跃响应
        {
            "init": torch.zeros(1, 12, device=device),
            "target": torch.tensor([[0.0, 0.0, 2.0]], device=device),
            "yaw": torch.tensor([0.0], device=device),
            "steps": steps,
            "label": "垂直起飞 [0,0,0]->[0,0,2]",
        },
        # 场景2: 水平对角线移动
        {
            "init": torch.zeros(1, 12, device=device),
            "target": torch.tensor([[3.0, 3.0, 2.0]], device=device),
            "yaw": torch.tensor([0.0], device=device),
            "steps": steps,
            "label": "对角线移动 [0,0,0]->[3,3,2]",
        },
        # 场景3: 带初始高度的水平移动
        {
            "init": _make_init(torch.tensor([[0.0, 0.0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]), device),
            "target": torch.tensor([[5.0, 0.0, 2.0]], device=device),
            "yaw": torch.tensor([0.0], device=device),
            "steps": steps,
            "label": "水平移动 [0,0,2]->[5,0,2]",
        },
    ]
    return scenarios


def _make_init(state, device):
    return state.to(device)


def compute_loss(states_list, targets_list, controls_list, dt):
    """
    复合损失函数：
      L = L_itae + 0.001*L_energy + 0.05*L_stability + 0.1*L_overshoot
    """
    total_loss = torch.tensor(0.0, device=states_list[0][0].device)

    for states, target, controls in zip(states_list, targets_list, controls_list):
        n_steps = len(states)
        for i in range(n_steps):
            st = states[i]
            u = controls[i]
            t_weight = i * dt

            # ITAE: 时间加权绝对位置误差
            pos_err = torch.norm(target - st[:, 0:3], dim=1)
            total_loss = total_loss + torch.sum(t_weight * pos_err * dt)

            # 控制能量惩罚
            total_loss = total_loss + 0.001 * torch.sum(u ** 2) * dt

            # 姿态稳定性惩罚（限制倾角和角速度）
            total_loss = total_loss + 0.05 * torch.sum(st[:, 6:9] ** 2) * dt
            total_loss = total_loss + 0.01 * torch.sum(st[:, 9:12] ** 2) * dt

    # 归一化
    n_scenarios = len(states_list)
    return total_loss / max(n_scenarios, 1)


def run_sim(drone, controller, scenario, mode="pid"):
    """
    运行单场景仿真，返回 states 和 controls 列表。
    mode: "pid" 或 "hybrid"
    """
    state = scenario["init"].clone()
    target = scenario["target"]
    yaw = scenario["yaw"]
    steps = scenario["steps"]
    B = state.shape[0]
    dev = state.device

    z3 = torch.zeros(B, 3, device=dev)
    prev_vel_err = z3.clone()
    prev_rate_err = z3.clone()
    int_pos = z3.clone()
    int_vel = z3.clone()
    int_rate = z3.clone()
    int_s = z3.clone()

    states_rec = []
    controls_rec = []

    for _ in range(steps):
        if mode == "pid":
            u, prev_vel_err, prev_rate_err, int_pos, int_vel, int_rate = controller(
                state, target, yaw, prev_vel_err, prev_rate_err,
                int_pos, int_vel, int_rate,
            )
        else:  # hybrid
            u, prev_vel_err, prev_rate_err, int_pos, int_vel, int_rate, int_s = controller(
                state, target, yaw, prev_vel_err, prev_rate_err,
                int_pos, int_vel, int_rate, int_s,
            )

        state = drone.step(state, u)
        states_rec.append(state)
        controls_rec.append(u)

    return states_rec, controls_rec


# ============================================================
# 参数提取与格式化输出
# ============================================================

def extract_pid_params(pid: DifferentiablePID):
    """提取 PID 控制器的实际增益值（经过 softplus）"""
    with torch.no_grad():
        return {
            "kp_pos": pid.kp_pos.cpu().numpy(),
            "ki_pos": pid.ki_pos.cpu().numpy(),
            "kp_vel": pid.kp_vel.cpu().numpy(),
            "ki_vel": pid.ki_vel.cpu().numpy(),
            "kd_vel": pid.kd_vel.cpu().numpy(),
            "kp_att": pid.kp_att.cpu().numpy(),
            "kp_rate": pid.kp_rate.cpu().numpy(),
            "ki_rate": pid.ki_rate.cpu().numpy(),
            "kd_rate": pid.kd_rate.cpu().numpy(),
        }


def extract_smc_params(smc: DifferentiableSMC):
    """提取 SMC 控制器的实际参数值（经过 softplus）"""
    with torch.no_grad():
        return {
            "lam": smc.lam.cpu().numpy(),
            "epsilon": smc.epsilon.cpu().numpy(),
            "k": smc.k.cpu().numpy(),
            "ki": smc.ki.cpu().numpy(),
        }


def fmt_arr(arr):
    """格式化 numpy 数组为可复制的字符串"""
    return "[{:.4f}, {:.4f}, {:.4f}]".format(arr[0], arr[1], arr[2])


def evaluate_performance(drone, controller, device, mode="pid"):
    """用最佳参数跑一次仿真，计算性能指标"""
    dt = drone.dt
    state = torch.zeros(1, 12, device=device)
    target = torch.tensor([[3.0, 3.0, 2.0]], device=device)
    yaw = torch.tensor([0.0], device=device)
    steps = int(8.0 / dt)

    z3 = torch.zeros(1, 3, device=device)
    prev_vel_err = z3.clone()
    prev_rate_err = z3.clone()
    int_pos = z3.clone()
    int_vel = z3.clone()
    int_rate = z3.clone()
    int_s = z3.clone()

    positions = []
    with torch.no_grad():
        for _ in range(steps):
            if mode == "pid":
                u, prev_vel_err, prev_rate_err, int_pos, int_vel, int_rate = controller(
                    state, target, yaw, prev_vel_err, prev_rate_err,
                    int_pos, int_vel, int_rate,
                )
            else:
                u, prev_vel_err, prev_rate_err, int_pos, int_vel, int_rate, int_s = controller(
                    state, target, yaw, prev_vel_err, prev_rate_err,
                    int_pos, int_vel, int_rate, int_s,
                )
            state = drone.step(state, u)
            positions.append(state[:, 0:3].cpu().numpy()[0])

    positions = np.array(positions)
    target_np = target.cpu().numpy()[0]
    errors = np.linalg.norm(positions - target_np, axis=1)

    rmse = np.sqrt(np.mean(errors ** 2))
    steady_err = np.mean(errors[-100:])  # 最后1秒
    max_overshoot = 0.0
    # 检测各轴超调
    for ax in range(3):
        final = target_np[ax]
        if abs(final) > 0.01:
            peak = np.max(np.abs(positions[:, ax]))
            overshoot = max(0, (peak - abs(final)) / abs(final) * 100)
            max_overshoot = max(max_overshoot, overshoot)

    # 调节时间（误差持续 < 5% 最终值的时间点）
    threshold = 0.05 * np.linalg.norm(target_np)
    settling_time = steps * dt
    for i in range(len(errors) - 1, -1, -1):
        if errors[i] > threshold:
            settling_time = (i + 1) * dt
            break

    return rmse, steady_err, max_overshoot, settling_time


# ============================================================
# 主优化流程
# ============================================================

def optimize(epochs=200, lr=0.01, dt=0.01, mode="hybrid", sim_duration=3.0):
    """
    主优化函数。
    mode: "pid" -- 仅优化 PID 参数
          "hybrid" -- 联合优化 PID + SMC 参数
    """
    print("=" * 56)
    print("  GPU 加速 PID + 滑模参数联合优化器")
    print("=" * 56)
    print()

    device = get_device()
    print()

    # 初始化模块
    drone = DifferentiableDrone(dt=dt, device=device).to(device)
    pid = DifferentiablePID(dt=dt, device=device).to(device)

    if mode == "hybrid":
        smc = DifferentiableSMC(dt=dt, device=device).to(device)
        controller = HybridController(pid, smc).to(device)
        all_params = list(pid.parameters()) + list(smc.parameters())
        print(f"  模式: PID + SMC 联合优化")
        print(f"  可训练参数: PID {sum(p.numel() for p in pid.parameters())} + "
              f"SMC {sum(p.numel() for p in smc.parameters())} = "
              f"{sum(p.numel() for p in all_params)} 个")
    else:
        controller = pid
        smc = None
        all_params = list(pid.parameters())
        print(f"  模式: 仅 PID 优化")
        print(f"  可训练参数: {sum(p.numel() for p in all_params)} 个")

    scenarios = make_scenarios(device, dt=dt, sim_duration=sim_duration)
    print(f"  测试场景: {len(scenarios)} 个")
    print(f"  优化轮次: {epochs}")
    print(f"  学习率: {lr}")
    print()

    optimizer = optim.Adam(all_params, lr=lr)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs, eta_min=lr * 0.01)

    best_loss = float("inf")
    best_pid_params = None
    best_smc_params = None
    start_time = time.time()

    for epoch in range(1, epochs + 1):
        optimizer.zero_grad()

        all_states = []
        all_targets = []
        all_controls = []

        for sc in scenarios:
            states, controls = run_sim(drone, controller, sc, mode=mode)
            all_states.append(states)
            all_targets.append(sc["target"])
            all_controls.append(controls)

        loss = compute_loss(all_states, all_targets, all_controls, dt)

        # 反向传播
        loss.backward()
        torch.nn.utils.clip_grad_norm_(all_params, max_norm=1.0)
        optimizer.step()
        scheduler.step()

        loss_val = loss.item()

        if loss_val < best_loss and not (torch.isnan(loss) or torch.isinf(loss)):
            best_loss = loss_val
            best_pid_params = extract_pid_params(pid)
            if smc is not None:
                best_smc_params = extract_smc_params(smc)

        # 进度输出
        if epoch == 1 or epoch % 20 == 0 or epoch == epochs:
            elapsed = time.time() - start_time
            current_lr = scheduler.get_last_lr()[0]
            print(f"  Epoch {epoch:4d}/{epochs} | "
                  f"Loss: {loss_val:10.4f} | "
                  f"Best: {best_loss:10.4f} | "
                  f"LR: {current_lr:.6f} | "
                  f"Time: {elapsed:.1f}s")

    elapsed = time.time() - start_time
    print()

    # --- 恢复最佳参数并评估 ---
    with torch.no_grad():
        for name, val in best_pid_params.items():
            raw_name = name + "_raw"
            # softplus 的逆函数: raw = log(exp(val) - 1)
            raw_val = torch.log(torch.exp(torch.tensor(val)) - 1.0)
            getattr(pid, raw_name).copy_(raw_val)
        if smc is not None and best_smc_params is not None:
            for name, val in best_smc_params.items():
                raw_name = name + "_raw"
                raw_val = torch.log(torch.exp(torch.tensor(val)) - 1.0)
                getattr(smc, raw_name).copy_(raw_val)

    rmse, steady_err, overshoot, settling = evaluate_performance(
        drone, controller, device, mode=mode
    )

    # --- 格式化输出 ---
    dev_name = str(device).upper()
    if device.type == "cuda":
        dev_name = f"CUDA ({torch.cuda.get_device_name(0)})"
    elif device.type == "mps":
        dev_name = "Apple MPS (Metal Performance Shaders)"

    print("=" * 56)
    print(f"  GPU 加速参数优化完成")
    print(f"  设备: {dev_name}")
    print(f"  耗时: {elapsed:.1f} 秒 | 轮次: {epochs}")
    print(f"  最终最佳损失: {best_loss:.4f}")
    print("=" * 56)
    print()

    print("  [PID 控制器参数]")
    print()
    print("  # 位置环")
    print(f"  kp_pos = np.array({fmt_arr(best_pid_params['kp_pos'])})")
    print(f"  ki_pos = np.array({fmt_arr(best_pid_params['ki_pos'])})")
    print()
    print("  # 速度环")
    print(f"  kp_vel = np.array({fmt_arr(best_pid_params['kp_vel'])})")
    print(f"  ki_vel = np.array({fmt_arr(best_pid_params['ki_vel'])})")
    print(f"  kd_vel = np.array({fmt_arr(best_pid_params['kd_vel'])})")
    print()
    print("  # 姿态环")
    print(f"  kp_att = np.array({fmt_arr(best_pid_params['kp_att'])})")
    print(f"  kp_rate = np.array({fmt_arr(best_pid_params['kp_rate'])})")
    print(f"  ki_rate = np.array({fmt_arr(best_pid_params['ki_rate'])})")
    print(f"  kd_rate = np.array({fmt_arr(best_pid_params['kd_rate'])})")

    if best_smc_params is not None:
        print()
        print("  [滑模控制器参数 (SecondOrderSMC)]")
        print()
        print(f"  lam     = np.array({fmt_arr(best_smc_params['lam'])})")
        print(f"  epsilon = np.array({fmt_arr(best_smc_params['epsilon'])})")
        print(f"  k       = np.array({fmt_arr(best_smc_params['k'])})")
        print(f"  ki      = np.array({fmt_arr(best_smc_params['ki'])})")

    print()
    print("  [性能指标] (场景: [0,0,0]->[3,3,2], 8秒仿真)")
    print()
    print(f"  位置 RMSE:   {rmse:.4f} m")
    print(f"  稳态误差:    {steady_err:.4f} m")
    print(f"  最大超调:    {overshoot:.1f} %")
    print(f"  调节时间:    {settling:.2f} s")
    print()
    print("=" * 56)
    print("  以上参数可直接复制到 Controller.py / SecondOrderSMC.py")
    print("=" * 56)

    return best_pid_params, best_smc_params


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="GPU 加速 PID + SMC 参数优化器")
    parser.add_argument("--mode", choices=["pid", "hybrid"], default="hybrid",
                        help="优化模式: pid=仅PID, hybrid=PID+SMC联合 (默认: hybrid)")
    parser.add_argument("--epochs", type=int, default=200,
                        help="优化轮次 (默认: 200)")
    parser.add_argument("--lr", type=float, default=0.01,
                        help="初始学习率 (默认: 0.01)")
    parser.add_argument("--dt", type=float, default=0.01,
                        help="仿真步长 (默认: 0.01)")
    parser.add_argument("--sim-time", type=float, default=3.0,
                        help="每个场景仿真时长/秒 (默认: 3.0, 增大可提升精度但更慢)")
    args = parser.parse_args()

    optimize(epochs=args.epochs, lr=args.lr, dt=args.dt, mode=args.mode,
             sim_duration=args.sim_time)
