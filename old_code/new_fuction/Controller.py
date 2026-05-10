import numpy as np


class Controller:
    def __init__(self, m=1.0, g=9.81, inertia=None, dt=0.01):
        if inertia is None:
            self.I = np.diag([0.01, 0.01, 0.02])
        else:
            self.I = np.array(inertia, dtype=float)
        self.m = m
        self.g = g
        self.dt = dt
# PID参数（位置，速度，姿态，角速度）
        self.kp_pos = np.array([1.6027, 1.7593, 2.2393])
        self.ki_pos = np.array([0.6124, 0.7109, 0.6840])

        self.kp_vel = np.array([3.3808, 3.5732, 3.6211])
        self.ki_vel = np.array([0.8229, 0.9532, 0.8860])
        self.kd_vel = np.array([1.4913, 1.3792, 1.4040])

        self.kp_att = np.array([8.9545, 9.0160, 4.2124])
        self.kp_rate = np.array([2.5789, 2.5789, 1.7014])
        self.ki_rate = np.array([0.7444, 0.7444, 0.7185])
        self.kd_rate = np.array([0.8544, 0.8544, 0.7710])
#误差初始化
        self.integral_pos = np.zeros(3)
        self.integral_vel = np.zeros(3)
        self.integral_rate = np.zeros(3)
        self.prev_vel_err = np.zeros(3)
        self.prev_rate_err = np.zeros(3)
        self.last_des_att = np.zeros(3)
#限制参数
        self.max_acc = 10.0
        self.max_tilt = np.deg2rad(30.0)
        self.max_vel = 10.0
        self.integral_pos_limit = 2.0
        self.integral_vel_limit = 2.0
        self.integral_rate_limit = 1.0

    def compute_position_loop(self, state, target_pos, target_vel=None, target_acc=None, target_yaw=0.0):
        """
        位置环计算：位置PID -> 速度PID -> 推力+期望姿态
        返回: (T_thrust, des_att)
        """
        if target_vel is None:
            target_vel = np.zeros(3)
        if target_acc is None:
            target_acc = np.zeros(3)

        pos = state[0:3]
        vel = state[3:6]
        dt = self.dt

        #位置环
        pos_err = target_pos - pos
        self.integral_pos += pos_err * dt
        self.integral_pos = np.clip(
            self.integral_pos,
            -self.integral_pos_limit,
            self.integral_pos_limit
        )
        des_vel = (
            target_vel
            + self.kp_pos * pos_err
            + self.ki_pos * self.integral_pos
        )
        des_vel = np.clip(des_vel, -self.max_vel, self.max_vel)

        #速度环
        vel_err = des_vel - vel
        self.integral_vel += vel_err * dt
        self.integral_vel = np.clip(
            self.integral_vel,
            -self.integral_vel_limit,
            self.integral_vel_limit
        )
        vel_deriv = (vel_err - self.prev_vel_err) / dt if dt > 0 else np.zeros(3)
        self.prev_vel_err = vel_err.copy()

        des_acc = (
            target_acc
            + self.kp_vel * vel_err
            + self.ki_vel * self.integral_vel
            + self.kd_vel * vel_deriv
        )
        acc_norm = np.linalg.norm(des_acc)
        if acc_norm > self.max_acc:
            des_acc = des_acc / acc_norm * self.max_acc

        #加速度输出转换为推力和期望姿态
        thrust_vec = des_acc + np.array([0.0, 0.0, self.g])
        thrust_mag = np.linalg.norm(thrust_vec)
        T_thrust = self.m * thrust_mag
        T_thrust = np.clip(
            T_thrust,
            0.1 * self.m * self.g,
            2.0 * self.m * self.g
        )

        if thrust_mag < 1e-6:
            thrust_norm = np.array([0.0, 0.0, 1.0])
        else:
            thrust_norm = thrust_vec / thrust_mag

        sy = np.sin(target_yaw)
        cy = np.cos(target_yaw)

        roll_arg = thrust_norm[0] * sy - thrust_norm[1] * cy
        roll_arg = np.clip(roll_arg, -1.0, 1.0)
        des_roll = np.arcsin(roll_arg)

        cos_roll = np.cos(des_roll)
        if abs(cos_roll) > 1e-6:
            pitch_arg = (
                thrust_norm[0] * cy + thrust_norm[1] * sy
            ) / cos_roll
            pitch_arg = np.clip(pitch_arg, -1.0, 1.0)
            des_pitch = np.arcsin(pitch_arg)
        else:
            des_pitch = 0.0

        des_roll = np.clip(des_roll, -self.max_tilt, self.max_tilt)
        des_pitch = np.clip(des_pitch, -self.max_tilt, self.max_tilt)
        des_att = np.array([des_roll, des_pitch, target_yaw])
        self.last_des_att = des_att.copy()

        return T_thrust, des_att

    def compute_attitude_loop(self, state, des_att):
        """
        姿态环计算：姿态P -> 角速度PID -> 力矩
        返回: torques [3]
        """
        att = state[6:9]
        rate = state[9:12]
        dt = self.dt

        att_err = des_att - att
        att_err[2] = (att_err[2] + np.pi) % (2.0 * np.pi) - np.pi
        des_rate = self.kp_att * att_err

        rate_err = des_rate - rate
        self.integral_rate += rate_err * dt
        self.integral_rate = np.clip(
            self.integral_rate,
            -self.integral_rate_limit,
            self.integral_rate_limit
        )
        rate_deriv = (rate_err - self.prev_rate_err) / dt if dt > 0 else np.zeros(3)
        self.prev_rate_err = rate_err.copy()

        torques = self.I @ (
            self.kp_rate * rate_err
            + self.ki_rate * self.integral_rate
            + self.kd_rate * rate_deriv
        )
        return torques

    def compute_control(self, state, target_pos, target_vel=None, target_acc=None, target_yaw=0.0):
        T_thrust, des_att = self.compute_position_loop(state, target_pos, target_vel, target_acc, target_yaw)
        torques = self.compute_attitude_loop(state, des_att)
        return np.array([T_thrust, torques[0], torques[1], torques[2]])  # 返回推力和三个轴的力矩

    def reset(self):
        self.integral_pos = np.zeros(3)
        self.integral_vel = np.zeros(3)
        self.integral_rate = np.zeros(3)
        self.prev_vel_err = np.zeros(3)
        self.prev_rate_err = np.zeros(3)
        self.last_des_att = np.zeros(3)

        