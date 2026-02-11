'''控制飞行器飞到指定航点，完成航线跟踪，使用pid控制（考虑位置环和速度环）使无人机稳定在航点位置
一个类写一个py文件，放到一个项目里面，最后在一个主函数py文件里面导入所有的类，然后计算
所有类的功能，完成一个完整的无人机飞行控制系统'''
import time
import numpy as np
from Packaged_Drone_Dynamics_Model import Drone
import os
import plotly.graph_objects as go


class Controller:
    def __init__(self,kp_pos=3.985,ki_pos=0.0,kd_vel=2.5694,max_acc=10.0,max_integral=10.0):
        '''Ziegler-Nichols法
        1）将系统的控制器参数设置为Kp=0, Ki=0, Kd=0，即不带控制器。
        2）逐渐增大比例参数Kp，直到系统开始产生振荡。
        3）记录此时的增益Kc和周期Tc。
        4）根据Ziegler-Nichols法则计算比例、积分和微分参数：
        比例参数Kp = 0.6 * Kc
        积分参数Ki = 1.2 * Kc / Tc
        微分参数Kd = 0.075 * Kc * Tc
        其中，Kc是系统的临界增益，Tc是系统的临界周期。
        '''
        self.kp_pos = float(kp_pos)
        self.ki_pos = float(ki_pos)
        self.kd_vel = float(kd_vel)
        self.max_acc = float(max_acc)
        self.max_integral = float(max_integral)
        self.integral_pos_err = np.zeros(3)
    def compute_acceleration(self,position,velocity,target_position,target_velocity=None, target_acceleration=None, dt=0.01):
        position = np.array(position,dtype=float)
        velocity = np.array(velocity,dtype=float)
        target_position = np.array(target_position,dtype=float)
        if target_velocity is None:
            target_velocity = np.zeros(3,dtype=float)
        else:
            target_velocity = np.array(target_velocity,dtype=float)
        if target_acceleration is None:
            target_acceleration = np.zeros(3, dtype=float)
        else:
            target_acceleration = np.array(target_acceleration, dtype=float)
        pos_err = target_position - position
        vel_err = target_velocity - velocity
        self.integral_pos_err += pos_err * dt
        integral_norm = np.linalg.norm(self.integral_pos_err)
        if integral_norm > self.max_integral:
                self.integral_pos_err = self.integral_pos_err / integral_norm * self.max_integral#不改变方向，改变大小
        # 前馈控制
        acc_cmd = (self.kp_pos * pos_err) + (self.ki_pos * self.integral_pos_err) + (self.kd_vel * vel_err) + target_acceleration
        norm = np.linalg.norm(acc_cmd)
        if norm > self.max_acc and norm > 0:
            acc_cmd = acc_cmd / norm * self.max_acc
        return acc_cmd
    def reset(self):
        self.integral_pos_err = np.zeros(3)
if __name__ == "__main__":
    drone = Drone(dt=0.01)
    drone.set_initial_state([0.0,0.0,0.0],[0.0,0.0,0.0],dt=0.01,max_acc=10.0)
    controller = Controller(kp_pos=3.985,ki_pos=0.0,kd_vel=2.5694,max_acc=10.0,max_integral=14.1153)
    waypoints = [
        [15.0, 13.0, 20.0],
        [0.0, 10.0, 5.0],
        [5.0, 15.0, 10.0],
        [0.0, 0.0, 0.0]
    ]
    history = [drone.get_state()[0]]
    current_time = 0.0
    dt = 0.01  
    for i, wp in enumerate(waypoints):
        target_position = np.array(wp)
        # 每个航点的最大飞行时间，防止飞不到一直卡住
        wp_duration = 20.0 
        start_time = current_time
        while current_time - start_time < wp_duration:
            position, velocity = drone.get_state()
            acc_cmd = controller.compute_acceleration(position, velocity, target_position, dt=dt)
            drone.update_state(acc_cmd)
            history.append(drone.get_state()[0])
            current_time += dt
            if np.linalg.norm(position - target_position) < 0.2 and np.linalg.norm(velocity) < 0.5:
                print(f"  已到达航点 {i+1}，耗时: {current_time - start_time:.2f}s")
                break
    history = np.array(history)
    fig = go.Figure(data=[go.Scatter3d(x=history[:,0],y=history[:,1],z=history[:,2],mode='lines+markers',marker=dict(size=2,color='blue'),line=dict(color='blue',width=2))])
    fig .update_layout(title='无人机三维轨迹',scene=dict(xaxis_title='X (m)',yaxis_title='Y (m)',zaxis_title='Z (m)'))
    fig.show()           