import numpy as np
import matplotlib.pyplot as plt
from Packaged_Drone_Dynamics_Model import Drone
from Controller import Controller
from HybridAttitudeController import HybridAttitudeController
from WindField import WindField


def run_formation_simulation(use_smc=True):
    dt = 0.01
    max_sim_time = 150.0

    # ========== 主机初始化 ==========
    leader = Drone(dt=dt)
    leader.set_initial_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    # 风场：每架无人机独立风场实例（相同稳态风，不同湍流种子）
    base_steady = (0.5, 0.0, 0.0)
    leader_wind = WindField(steady=base_steady, turbulence_std=0.02, tau=0.8, seed=42)

    # ========== 从机初始化 ==========
    delta_offsets = [
        np.array([-2.0, 2.0, 0.0]),
        np.array([-2.0, -2.0, 0.0]),
        np.array([-4.0, 0.0, 0.0]),
    ]
    followers = []
    follower_winds = []
    for idx, offset in enumerate(delta_offsets):
        follower = Drone(dt=dt)
        init_pos = leader.state[0:3] + offset
        follower.set_initial_state(init_pos, [0.0, 0.0, 0.0])
        followers.append(follower)
        # 每架从机独立风场（相同稳态风，不同湍流种子）
        follower_winds.append(WindField(steady=base_steady, turbulence_std=0.02, tau=0.8, seed=100 + idx))

    # ========== 控制器初始化（支持 PID/SMC 切换） ==========
    if use_smc:
        leader_ctrl = HybridAttitudeController(dt=dt)
        leader_ctrl.use_smc = True
        follower_ctrls = []
        for _ in followers:
            ctrl = HybridAttitudeController(dt=dt)
            ctrl.use_smc = True
            follower_ctrls.append(ctrl)
        print("控制器模式: PID位置环 + SMC姿态环 (HybridAttitudeController)")
    else:
        leader_ctrl = Controller(dt=dt)
        follower_ctrls = [Controller(dt=dt) for _ in followers]
        print("控制器模式: 纯PID串级控制 (Controller)")

    # ========== 航点定义 ==========
    waypoints = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 15.0],
        [30.0, 0.0, 15.0],
        [30.0, 30.0, 15.0],
        [-10.0, 30.0, 15.0],
        [-10.0, 0.0, 15.0],
        [0.0, 0.0, 15.0],
        [0.0, 0.0, 0.0],
    ]

    # 修复问题4：缩小切换半径，末段航点使用更严格判定
    wp_radius = 0.8
    wp_radius_final = 0.3
    current_wp_idx = 0
    finished_waypoints = False
    time = 0.0

    # ========== 数据记录 ==========
    history_time = []
    history_leader_pos = []
    history_followers_pos = [[] for _ in followers]
    formation_errors = [[] for _ in followers]
    # ========== 主仿真循环 ==========
    while time < max_sim_time:
        # 确定当前目标航点
        if not finished_waypoints:
            wp = np.array(waypoints[current_wp_idx])
        else:
            wp = np.array(waypoints[-1])

        # 采样主机风场
        wind_leader = leader_wind.sample(dt)

        # -------- 主机控制与更新 --------
        leader_pos, leader_vel, _, _ = leader.get_state()
        u_leader = leader_ctrl.compute_control(leader.state, wp)
        leader.update_state(u_leader, wind=wind_leader)
        leader_pos_new, leader_vel_new, _, _ = leader.get_state()

        # -------- 航点切换判定 --------
        if not finished_waypoints:
            dist_to_wp = np.linalg.norm(wp - leader_pos_new)

            # 修复问题4：最后一个航点用更严格半径
            is_final_wp = (current_wp_idx == len(waypoints) - 1)
            radius = wp_radius_final if is_final_wp else wp_radius

            if dist_to_wp < radius:
                current_wp_idx += 1
                if current_wp_idx >= len(waypoints):
                    finished_waypoints = True
                # 不调用 reset()，保持积分器连续性

        # -------- 主机加速度估计（用于从机前馈） --------
        leader_acc = (leader_vel_new - leader_vel) / dt

        # -------- 从机控制与更新 --------
        for i, f in enumerate(followers):
            offset = delta_offsets[i]
            tgt_pos = leader_pos_new + offset

            # 每架从机独立风场采样
            wind_follower = follower_winds[i].sample(dt)

            u_f = follower_ctrls[i].compute_control(
                f.state,
                tgt_pos,
                target_vel=leader_vel_new,
                target_acc=leader_acc
            )
            f.update_state(u_f, wind=wind_follower)

            # 记录编队误差
            f_pos_new = f.get_state()[0]
            error = np.linalg.norm(f_pos_new - tgt_pos)
            formation_errors[i].append(error)

        # -------- 记录历史数据 --------
        history_time.append(time)
        history_leader_pos.append(leader_pos_new.copy())
        for i, f in enumerate(followers):
            history_followers_pos[i].append(f.get_state()[0].copy())

        time += dt
    # 转换为numpy数组以便绘图
    history_time = np.array(history_time)
    history_leader_pos = np.array(history_leader_pos)
    history_followers_pos = [np.array(hf) for hf in history_followers_pos]

    # ========== 图1：三维轨迹图 ==========
    fig1 = plt.figure(figsize=(10, 8))
    ax1 = fig1.add_subplot(111, projection='3d')
    
    # 绘制航点
    wp_arr = np.array(waypoints)
    ax1.plot(wp_arr[:, 0], wp_arr[:, 1], wp_arr[:, 2], 'ro--', alpha=0.6, markersize=5, label='Waypoints')
    
    # 绘制长机轨迹
    ax1.plot(history_leader_pos[:, 0], history_leader_pos[:, 1], history_leader_pos[:, 2], 'b-', linewidth=2, label='Leader')
    
    # 绘制僚机轨迹
    plot_colors = ['g', 'c', 'm', 'y', 'k']
    for i, hf in enumerate(history_followers_pos):
        c = plot_colors[i % len(plot_colors)]
        ax1.plot(hf[:, 0], hf[:, 1], hf[:, 2], color=c, linestyle='--', linewidth=1.5, label=f'Follower {i+1}')
        
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Formation Flight Trajectory')
    ax1.legend()

    # ========== 图3：XYZ分量跟踪曲线 ==========
    fig3, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axis_labels = ['X (m)', 'Y (m)', 'Z (m)']
    colors = ['r', 'g', 'c', 'm', 'y', 'k']

    for axis_idx in range(3):
        ax = axes[axis_idx]
        ax.plot(
            history_time,
            history_leader_pos[:, axis_idx],
            'b-', linewidth=1.5, label='Leader'
        )
        for i, hf in enumerate(history_followers_pos):
            c = colors[i % len(colors)]
            ax.plot(
                history_time[:len(hf)],
                hf[:, axis_idx],
                color=c, linestyle='--', linewidth=1.0,
                label=f'Follower {i+1}'
            )
        ax.set_ylabel(axis_labels[axis_idx])
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True)

    axes[2].set_xlabel('Time (s)')
    axes[0].set_title('Position Components vs Time')
    plt.tight_layout()

    # ========== 图4：编队误差统计 ==========
    fig4, ax4 = plt.subplots(figsize=(8, 4))
    stats_labels = []
    stats_mean = []
    stats_max = []
    for i, errors in enumerate(formation_errors):
        err_arr = np.array(errors)
        stats_labels.append(f'F{i+1}')
        stats_mean.append(np.mean(err_arr))
        stats_max.append(np.max(err_arr))

    x_bar = np.arange(len(stats_labels))
    width = 0.35
    ax4.bar(x_bar - width/2, stats_mean, width, label='Mean Error', color='steelblue')
    ax4.bar(x_bar + width/2, stats_max, width, label='Max Error', color='salmon')
    ax4.set_xticks(x_bar)
    ax4.set_xticklabels(stats_labels)
    ax4.set_ylabel('Error (m)')
    ax4.set_title('Formation Error Statistics')
    ax4.legend()
    ax4.grid(True, axis='y')
    plt.tight_layout()

    # ========== 打印误差统计 ==========
    print("\n===== 编队误差统计 =====")
    for i, errors in enumerate(formation_errors):
        err_arr = np.array(errors)
        print(f"Follower {i+1}: "
              f"Mean={np.mean(err_arr):.4f} m, "
              f"Max={np.max(err_arr):.4f} m, "
              f"Final={err_arr[-1]:.4f} m")

    # ========== 显示所有图表 ==========
    plt.show()
    print("仿真完成。")


if __name__ == "__main__":
    run_formation_simulation()
