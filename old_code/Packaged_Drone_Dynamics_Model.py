'''动力学类的封装：
类成员变量：位置、速度、步长
类的方法：
1.	设置初始状态、步长：外部传入状态和步长，更改成员变量，无返回值
2.	传入控制量，进行状态更新：外部传入控制量，计算导数，更新下一时刻状态，无返回值
3.	获取方法：返回希望获取的状态


实际最后在主函数中使用的时候：
实例化这个类；
设置初始状态；


在控制器状态更新循环里面，调用传入控制量更新状态和获取方法。
'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import sys
import os
import plotly.graph_objects as go
class Drone:
    def __init__(self, dt=0.01):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.dt = dt
        self.max_acc = 10.0
    def set_initial_state(self, position, velocity, dt=None,max_acc =10.0):
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        if dt is not None:
            self.dt = dt
        self.max_acc = max_acc
    def update_state(self, control):
        if len(control) != 3:
            raise ValueError("加速度必须是3维向量")
        if np.linalg.norm(control) > self.max_acc:
            control = (np.array(control) / np.linalg.norm(control)) * self.max_acc
        acceleration = np.array(control, dtype=float)
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt
    def get_state(self):
        return self.position.copy(), self.velocity.copy()
if __name__ == "__main__":
    drone = Drone(dt=0.1)
    print("设置无人机初始位置(x,y,z)")
    try:
        initial_pos = [float(input("X(m)[默认0.0]:") or 0.0),
                       float(input("Y(m)[默认0.0]:") or 0.0),
                       float(input("Z(m)[默认0.0]:") or 0.0)]
    except ValueError:
        initial_pos = [0.0, 0.0, 0.0]
    print("设置无人机初始速度(vx,vy,vz)")
    try:
        initial_vel = [float(input("Vx(m/s)[默认0.0]:") or 0.0),
                       float(input("Vy(m/s)[默认0.0]:") or 0.0),
                       float(input("Vz(m/s)[默认0.0]:") or 0.0)]
    except ValueError:
        initial_vel = [0.0, 0.0, 0.0]
    drone.set_initial_state(initial_pos, initial_vel)
    history = [drone.get_state()[0]]
    current_time = 0.0
    stage = 1
    while True:
        print("第%d次输入" % stage)
        print("当前状态：位置 (%.2f, %.2f, %.2f)，速度 (%.2f, %.2f, %.2f)" % (drone.position[0], drone.position[1], drone.position[2],drone.velocity[0], drone.velocity[1], drone.velocity[2]))
        duration = float(input("输入阶段持续时间(s)，或输入0退出:[默认0]").strip() or "0")
        try:
            duration = float(duration)
        except ValueError:
            duration = 0.0
        if duration <= 1e-3:
            print("退出仿真。")
            break
        steps = int(duration / drone.dt)
        if steps == 0 and duration > 0: steps = 1
        try:
            acc_input = [float(input("加速度Ax(m/s²)[默认0.0]:") or 0.0),
                         float(input("加速度Ay(m/s²)[默认0.0]:") or 0.0),
                         float(input("加速度Az(m/s²)[默认0.0]:") or 0.0)]
        except ValueError:
            acc_input = [0.0, 0.0, 0.0]
        for _ in range(steps):
            drone.update_state(acc_input)
            history.append(drone.get_state()[0])
            current_time += drone.dt
        stage += 1
    history = np.array(history)
    
    # 尝试使用 Plotly 绘制交互式图表
    try:
        print("正在生成交互式三维轨迹图...")
        fig = go.Figure(data=[go.Scatter3d(x=history[:,0], y=history[:,1], z=history[:,2],
                                           mode='lines+markers',
                                           line=dict(color='blue', width=2),
                                           marker=dict(size=3))])
        fig.update_layout(title='无人机三维轨迹', scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)'))
        
        # 保存为 HTML 文件（最稳妥的方式）
        output_file = "drone_trajectory_3d.html"
        fig.write_html(output_file)
        print(f"图表已保存至: {os.path.abspath(output_file)}")
        print("如果浏览器没有自动弹出，请手动打开上述 HTML 文件。")
        
        # 尝试自动显示
        fig.show()
        
    except Exception as e:
        print(f"Plotly 绘图遇到问题: {e}")
        print("正在切换使用 Matplotlib 绘制静态图...")
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(history[:,0], history[:,1], history[:,2], label='Trajectory', marker='.')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.legend()
        plt.title("Drone 3D Path Simulation (Matplotlib)")
        plt.show()


