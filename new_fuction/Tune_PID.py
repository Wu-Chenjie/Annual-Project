import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from Controller import Controller
from Packaged_Drone_Dynamics_Model import Drone

class DifferentiableDroneController(nn.Module):
    def __init__(self, dt=0.01):
        super().__init__()
        self.dt = dt
        self.m = 1.0
        self.g = 9.81
        self.I = torch.tensor([0.01, 0.01, 0.02])
        
        # Learnable PID Parameters (Initialized with safe bounds via abs)
        self.kp_pos = nn.Parameter(torch.tensor([2.5, 2.5, 3.0]))
        self.ki_pos = nn.Parameter(torch.tensor([0.05, 0.05, 0.10]))
        self.kp_vel = nn.Parameter(torch.tensor([3.0, 3.0, 4.0]))
        self.ki_vel = nn.Parameter(torch.tensor([0.50, 0.50, 0.50]))
        self.kd_vel = nn.Parameter(torch.tensor([0.1, 0.1, 0.2]))

        self.kp_att = nn.Parameter(torch.tensor([8.0, 8.0, 5.0]))
        self.kp_rate = nn.Parameter(torch.tensor([1.5, 1.5, 1.0]))
        self.ki_rate = nn.Parameter(torch.tensor([0.10, 0.10, 0.05]))
        self.kd_rate = nn.Parameter(torch.tensor([0.1, 0.1, 0.1]))

        # 积分限幅
        self.integral_pos_limit = 2.0
        self.integral_vel_limit = 2.0
        self.integral_rate_limit = 1.0

    def drone_forward(self, state, u, wind):
        def dynamics(st):
            vel = st[:, 3:6]
            att = st[:, 6:9]
            omega = st[:, 9:12]
            
            roll, pitch, yaw = att[:, 0], att[:, 1], att[:, 2]
            wx, wy, wz = omega[:, 0], omega[:, 1], omega[:, 2]
            
            # Vectorized Rotation Matrix
            B = roll.shape[0]
            c_r, s_r = torch.cos(roll), torch.sin(roll)
            c_p, s_p = torch.cos(pitch), torch.sin(pitch)
            c_y, s_y = torch.cos(yaw), torch.sin(yaw)
            
            R = torch.zeros((B, 3, 3), dtype=torch.float32, device=st.device)
            R[:, 0, 0] = c_y * c_p
            R[:, 0, 1] = c_y * s_p * s_r - s_y * c_r
            R[:, 0, 2] = c_y * s_p * c_r + s_y * s_r
            R[:, 1, 0] = s_y * c_p
            R[:, 1, 1] = s_y * s_p * s_r + c_y * c_r
            R[:, 1, 2] = s_y * s_p * c_r - c_y * s_r
            R[:, 2, 0] = -s_p
            R[:, 2, 1] = c_p * s_r
            R[:, 2, 2] = c_p * c_r
            
            v_rel = vel - wind
            v_norm = torch.norm(v_rel, dim=1, keepdim=True)
            drag = -0.05 * v_norm * v_rel
            
            thrust_local = torch.zeros_like(vel)
            thrust_local[:, 2] = u[:, 0]
            thrust_global = torch.bmm(R, thrust_local.unsqueeze(2)).squeeze(2)
            
            g_vec = torch.zeros_like(vel)
            g_vec[:, 2] = -self.m * self.g
            acc = (thrust_global + drag + g_vec) / self.m
            
            tau = u[:, 1:4]
            I_device = self.I.to(st.device)
            omega_cross = torch.cross(omega, omega * I_device, dim=1)
            omega_dot = (tau - omega_cross) / I_device
            
            # Avoid singularity
            c_pitch = torch.cos(pitch)
            c_pitch = torch.where(torch.abs(c_pitch) < 1e-6, torch.sign(c_pitch)*1e-6 + 1e-6*(c_pitch==0), c_pitch)
            t_pitch = torch.sin(pitch) / c_pitch
            c_roll = torch.cos(roll)
            s_roll = torch.sin(roll)
            
            roll_dot = wx + s_roll * t_pitch * wy + c_roll * t_pitch * wz
            pitch_dot = c_roll * wy - s_roll * wz
            yaw_dot = (s_roll / c_pitch) * wy + (c_roll / c_pitch) * wz
            att_dot = torch.stack([roll_dot, pitch_dot, yaw_dot], dim=1)
            
            return torch.cat([vel, acc, att_dot, omega_dot], dim=1)

        # Simplified RK4 Step
        k1 = dynamics(state)
        k2 = dynamics(state + 0.5 * self.dt * k1)
        k3 = dynamics(state + 0.5 * self.dt * k2)
        k4 = dynamics(state + self.dt * k3)
        return state + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def controller_forward(self, state, target_pos, target_yaw, prev_vel_err, prev_rate_err,
                           integral_pos, integral_vel, integral_rate):
        pos, vel, att, rate = state[:, 0:3], state[:, 3:6], state[:, 6:9], state[:, 9:12]

        pos_err = target_pos - pos
        integral_pos = integral_pos + pos_err * self.dt
        integral_pos = torch.clamp(integral_pos, -self.integral_pos_limit, self.integral_pos_limit)
        des_vel = pos_err * torch.abs(self.kp_pos) + integral_pos * torch.abs(self.ki_pos)
        des_vel = torch.clamp(des_vel, -10.0, 10.0)

        vel_err = des_vel - vel
        integral_vel = integral_vel + vel_err * self.dt
        integral_vel = torch.clamp(integral_vel, -self.integral_vel_limit, self.integral_vel_limit)
        vel_deriv = (vel_err - prev_vel_err) / self.dt

        des_acc = (vel_err * torch.abs(self.kp_vel)
                   + integral_vel * torch.abs(self.ki_vel)
                   + vel_deriv * torch.abs(self.kd_vel))
        acc_norm = torch.norm(des_acc, dim=1, keepdim=True)
        des_acc = torch.where(acc_norm > 10.0, des_acc / acc_norm * 10.0, des_acc)
        
        thrust_vec = des_acc.clone()
        thrust_vec[:, 2] += self.g
        T_thrust = self.m * torch.norm(thrust_vec, dim=1, keepdim=True)
        T_thrust = torch.clamp(T_thrust, 0.1 * self.m * self.g, 2.0 * self.m * self.g)
        
        thrust_norm = thrust_vec / torch.norm(thrust_vec, dim=1, keepdim=True)
        # Avoid asin nan
        roll_arg = torch.clamp(thrust_norm[:, 0]*torch.sin(target_yaw) - thrust_norm[:, 1]*torch.cos(target_yaw), -0.999, 0.999)
        des_roll = torch.asin(roll_arg)
        cos_roll = torch.cos(des_roll)
        pitch_arg = torch.clamp((thrust_norm[:, 0]*torch.cos(target_yaw) + thrust_norm[:, 1]*torch.sin(target_yaw))/cos_roll, -0.999, 0.999)
        des_pitch = torch.asin(pitch_arg)
        
        des_roll = torch.clamp(des_roll, -0.523, 0.523)
        des_pitch = torch.clamp(des_pitch, -0.523, 0.523)
        des_att = torch.stack([des_roll, des_pitch, target_yaw.expand(state.shape[0])], dim=1)
        
        att_err = des_att - att
        att_err[:, 2] = (att_err[:, 2] + torch.pi) % (2 * torch.pi) - torch.pi
        des_rate = att_err * torch.abs(self.kp_att)
        
        rate_err = des_rate - rate
        integral_rate = integral_rate + rate_err * self.dt
        integral_rate = torch.clamp(integral_rate, -self.integral_rate_limit, self.integral_rate_limit)
        rate_deriv = (rate_err - prev_rate_err) / self.dt
        des_torque = (rate_err * torch.abs(self.kp_rate)
                      + integral_rate * torch.abs(self.ki_rate)
                      + rate_deriv * torch.abs(self.kd_rate))
        torques = des_torque * self.I.to(des_torque.device)

        u = torch.cat([T_thrust, torques], dim=1)
        return u, vel_err, rate_err, integral_pos, integral_vel, integral_rate

def run_pytorch_optimization():
    print("🤖 启动基于 PyTorch 计算图的 PID 调参 (Adam 梯度下降)...")
    print("⚡ 正在利用反向传播(BPTT)全盘加速参数搜索...\n")
    
    dt = 0.01
    sim_time = 5.0
    steps = int(sim_time / dt)
    
    model = DifferentiableDroneController(dt=dt)
    optimizer = optim.Adam(model.parameters(), lr=1.0)
    
    target_pos = torch.tensor([[5.0, 5.0, 5.0]], dtype=torch.float32)
    target_yaw = torch.tensor([0.0], dtype=torch.float32)
    wind = torch.zeros((1, 3))
    
    best_loss = float('inf')
    best_params = None
    
    for epoch in range(1, 41):
        optimizer.zero_grad()
        
        state = torch.zeros((1, 12), dtype=torch.float32)
        prev_vel_err = torch.zeros((1, 3), dtype=torch.float32)
        prev_rate_err = torch.zeros((1, 3), dtype=torch.float32)
        integral_pos = torch.zeros((1, 3), dtype=torch.float32)
        integral_vel = torch.zeros((1, 3), dtype=torch.float32)
        integral_rate = torch.zeros((1, 3), dtype=torch.float32)

        loss = 0.0

        for i in range(steps):
            u, prev_vel_err, prev_rate_err, integral_pos, integral_vel, integral_rate = model.controller_forward(
                state, target_pos, target_yaw, prev_vel_err, prev_rate_err,
                integral_pos, integral_vel, integral_rate
            )
            state = model.drone_forward(state, u, wind)
            
            # ITAE Loss
            error = torch.norm(target_pos - state[:, 0:3], dim=1)
            time_weight = i * dt
            loss += torch.sum(time_weight * error * dt)
            
            # Penalize wild angular rates and crazy attitudes to stay stable
            loss += torch.sum(torch.abs(state[:, 6:9])) * 0.01
        
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        
        if epoch % 5 == 0 or epoch == 1:
            print(f"Epoch {epoch:2d}/{40} | ITAE Loss: {loss.item():.2f}")
            
        if loss.item() < best_loss and not torch.isnan(loss):
            best_loss = loss.item()
            best_params = {name: param.clone().detach().abs().numpy() for name, param in model.named_parameters()}
            
    print("\n🎉 PyTorch BPTT 优化完成!")
    print(f"🏆 最佳 ITAE 得分: {best_loss:.2f}")
    
    print("\n🔍 >>> 提取的最佳参数映射结果 <<<")
    for name, val in best_params.items():
        print(f"ctrl.{name} = np.array([{val[0]:.2f}, {val[1]:.2f}, {val[2]:.2f}])")
    print("-----------------------------------")
    return best_params

def plot_final_result(best_params):
    print("\n📈 正在基于真实物理模型进行绘图...")
    dt = 0.01
    sim_time = 10.0
    steps = int(sim_time / dt)
    
    drone = Drone(dt=dt)
    ctrl = Controller(dt=dt)
    
    ctrl.kp_pos = best_params['kp_pos']
    ctrl.ki_pos = best_params['ki_pos']
    ctrl.kp_vel = best_params['kp_vel']
    ctrl.ki_vel = best_params['ki_vel']
    ctrl.kd_vel = best_params['kd_vel']
    ctrl.kp_att = best_params['kp_att']
    ctrl.kp_rate = best_params['kp_rate']
    ctrl.ki_rate = best_params['ki_rate']
    ctrl.kd_rate = best_params['kd_rate']
    
    target_pos = np.array([5.0, 5.0, 5.0])
    target_yaw = 0.0
    
    history_pos = np.zeros((steps, 3))
    history_att = np.zeros((steps, 3))
    history_time = np.zeros(steps)
    
    error_integral = 0.0
    for i in range(steps):
        u = ctrl.compute_control(drone.state, target_pos, target_yaw=target_yaw)
        drone.update_state(u, wind=np.zeros(3))
        
        history_pos[i] = drone.state[0:3]
        history_att[i] = np.degrees(drone.state[6:9])
        history_time[i] = i * dt
        
        error_integral += (i * dt) * np.linalg.norm(target_pos - drone.state[0:3]) * dt

    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(history_time, history_pos[:, 0], label='X')
    plt.plot(history_time, history_pos[:, 1], label='Y')
    plt.plot(history_time, history_pos[:, 2], label='Z')
    plt.axhline(5.0, color='r', linestyle='--', label='Target (5.0)')
    plt.title(f'PyTorch Adam Optimized Step Response (Real ITAE Score: {error_integral:.2f})')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(history_time, history_att[:, 0], label='Roll (deg)')
    plt.plot(history_time, history_att[:, 1], label='Pitch (deg)')
    plt.plot(history_time, history_att[:, 2], label='Yaw (deg)')
    plt.xlabel('Time (s)')
    plt.ylabel('Attitude (deg)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    best = run_pytorch_optimization()
    plot_final_result(best)
