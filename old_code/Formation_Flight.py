'''实例化多个飞行器动力学模型，模仿编队飞行，主机跟踪航线，从机依照编队队形和主机当前状态，构建一个期望的位置，跟踪这个位置
在主函数里面实例化多个无人机来模拟动力学，在调用pid控制器那里，主机的期望位置是航点，从机的期望位置是主机+期望队形
'''
import numpy as np
from Packaged_Drone_Dynamics_Model import Drone
from Controller import Controller
import plotly.graph_objects as go
def run_formation_simulation():
    dt = 0.01
    max_sim_time = 150.0
    leader = Drone(dt=dt)
    leader.set_initial_state([0.0,0.0,0.0],[0.0,0.0,0.0],dt=dt)
    delta_offsets = [
        np.array([-2.0,2.0,0.0]),
        np.array([-2.0,-2.0,0.0]),
        np.array([-4.0,0.0,0.0]),
    ]
    followers = []
    for offset in delta_offsets:
        follower = Drone(dt=dt)
        init_pos = leader.position + offset
        follower.set_initial_state(init_pos,[0.0,0.0,0.0],dt=dt)
        followers.append(follower)
    leader_controller = Controller(kp_pos=3.0,ki_pos=0.0,kd_vel=4.5,max_acc=10.0,max_integral=10.0)
    follower_controllers = []
    for _ in range(len(followers)):
        follower_controllers.append(Controller(kp_pos=3.0,ki_pos=0.0,kd_vel=4.5,max_acc=10.0,max_integral=10.0))
    waypoints = [
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 15.0],
        [19.7, 3.47, 15.0],
        [18.79, 6.84, 15.0],
        [17.32, 10.0, 15.0],
        [15.32, 12.86, 15.0],
        [12.86, 15.32, 15.0],
        [10.0, 17.32, 15.0],
        [6.84, 18.79, 15.0],
        [3.47, 19.7, 15.0],
        [0.0, 20.0, 15.0],
        [-3.47, 19.7, 15.0],
        [-6.84, 18.79, 15.0],
        [-10.0, 17.32, 15.0],
        [-12.86, 15.32, 15.0],
        [-15.32, 12.86, 15.0],
        [-17.32, 10.0, 15.0],
        [-18.79, 6.84, 15.0],
        [-19.7, 3.47, 15.0],
        [-20.0, 0.0, 15.0],
        [-19.7, -3.47, 15.0],
        [-18.79, -6.84, 15.0],
        [-17.32, -10.0, 15.0],
        [-15.32, -12.86, 15.0],
        [-12.86, -15.32, 15.0],
        [-10.0, -17.32, 15.0],
        [-6.84, -18.79, 15.0],
        [-3.47, -19.7, 15.0],
        [-0.0, -20.0, 15.0],
        [3.47, -19.7, 15.0],
        [6.84, -18.79, 15.0],
        [10.0, -17.32, 15.0],
        [12.86, -15.32, 15.0],
        [15.32, -12.86, 15.0],
        [17.32, -10.0, 15.0],
        [18.79, -6.84, 15.0],
        [19.7, -3.47, 15.0],
        [20.0, -0.0, 15.0],
        [0.0, 0.0, 0.0]
    ]
    current_wp_idx = 0
    wp_radius = 0.2#判定范围
    history_leader = [leader.position.copy()]
    history_followers = []
    formation_errors = [] # 存储误差
    for _ in followers:
        history_followers.append([_.position.copy()])
        formation_errors.append([])
    time_stamps = []
    current_time = 0.0
    finished_waypoints = False#加一个判定点，检测是否飞完
    while current_time < max_sim_time:
        # 主机控制
        if not finished_waypoints:
            target_wp = waypoints[current_wp_idx]
        else:
            target_wp = waypoints[-1]
        leader_pos, leader_vel = leader.get_state()
        leader_acc = leader_controller.compute_acceleration(leader_pos, leader_vel, target_wp, dt=dt)
        leader.update_state(leader_acc)
        if not finished_waypoints:
            dist_to_wp = np.linalg.norm(target_wp - leader_pos)
            speed = np.linalg.norm(leader_vel)
            if dist_to_wp < wp_radius:
                current_wp_idx += 1
                if current_wp_idx >= len(waypoints):
                    finished_waypoints = True
        # 从机控制
        leader_pos_new, leader_vel_new = leader.get_state()
        for i, follower in enumerate(followers):
            f_controller = follower_controllers[i]
            offset = delta_offsets[i]
            target_follower_pos = leader_pos_new + offset
            target_follower_vel = leader_vel_new
            target_follower_acc = leader_acc
            follower_pos, follower_vel = follower.get_state()
            follower_acc = f_controller.compute_acceleration(
                follower_pos, 
                follower_vel, 
                target_follower_pos, 
                target_follower_vel, 
                target_acceleration=target_follower_acc,
                dt=dt
            )
            follower.update_state(follower_acc)
            error = np.linalg.norm(follower.position - target_follower_pos)
            formation_errors[i].append(error)
        history_leader.append(leader.position.copy())
        for i, follower in enumerate(followers):
            history_followers[i].append(follower.position.copy())
        current_time += dt
        time_stamps.append(current_time)
    print("仿真结束，开始绘图...")
    history_leader = np.array(history_leader)
    history_followers_np = []
    for hf in history_followers:
        history_followers_np.append(np.array(hf))
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=history_leader[:,0],y=history_leader[:,1],z=history_leader[:,2],mode='lines+markers',marker=dict(size=2,color='red'),line=dict(color='red',width=2),name='Leader'))
    num_followers = len(history_followers_np)
    for i, hf in enumerate(history_followers_np):    
        # 使用 HSL 颜色空间自动分配颜色，色相均匀分布
        hue = int(360 * i / num_followers)
        color = f'hsl({hue}, 100%, 50%)'
        fig.add_trace(go.Scatter3d(x=hf[:,0],y=hf[:,1],z=hf[:,2],mode='lines+markers',marker=dict(size=2,color=color),line=dict(color=color,width=2),name=f'Follower {i+1}'))
    # 绘制航点
    waypoints_np = np.array(waypoints)
    fig.add_trace(go.Scatter3d(
        x=waypoints_np[:, 0],
        y=waypoints_np[:, 1],
        z=waypoints_np[:, 2],
        mode='lines+markers+text',
        marker=dict(size=5, color='purple', symbol='diamond'),
        text=[f'WP{i}' for i in range(len(waypoints))],
        textposition="top center",
        name='Waypoints'
    ))

    fig.update_layout(title="无人机编队飞行仿真",scene=dict(xaxis_title='X (m)',yaxis_title='Y (m)',zaxis_title='Z (m)'))
    fig.show()

    # 绘制编队误差图
    fig_error = go.Figure()
    for i, errors in enumerate(formation_errors):
        hue = int(360 * i / num_followers)
        color = f'hsl({hue}, 100%, 50%)'
        fig_error.add_trace(go.Scatter(
            x=time_stamps, 
            y=errors, 
            mode='lines', 
            name=f'Follower {i+1} Error',
            line=dict(color=color)
        ))
    
    fig_error.update_layout(
        title="编队位置误差随时间变化",
        xaxis_title="Time (s)",
        yaxis_title="Position Error (m)"
    )
    fig_error.show()
if __name__ == "__main__":
    run_formation_simulation()
                



