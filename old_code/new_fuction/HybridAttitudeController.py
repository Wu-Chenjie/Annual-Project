import numpy as np
from Controller import Controller
from SecondOrderSMC import SecondOrderSMC
# 实现PID和滑模的混合控制，位置环使用PID，姿态环可切换PID/SMC


class HybridAttitudeController(Controller):
    """
    混合姿态控制器：位置环使用父类PID，姿态环可切换PID/SMC

    工作流程：
        1. 调用 compute_position_loop 完成位置环，得到推力T和期望姿态
        2. 姿态环根据 use_smc 开关选择：
           - SMC模式：指数趋近律滑模计算角加速度，乘惯量矩阵得力矩
           - PID模式：调用 compute_attitude_loop 使用PID计算力矩
    """

    def __init__(self, m=1.0, g=9.81, inertia=None, dt=0.01):
        super().__init__(m=m, g=g, inertia=inertia, dt=dt)
        self.smc = SecondOrderSMC(dt=dt)
        self.use_smc = True

    def compute_control(self, state, target_pos,
                        target_vel=None, target_acc=None,
                        target_yaw=0.0):
        # 步骤1：位置环（位置PID -> 速度PID -> 推力+期望姿态）
        T, des_att = self.compute_position_loop(
            state, target_pos, target_vel,
            target_acc, target_yaw
        )

        if not self.use_smc:
            # PID模式：使用父类姿态环
            torques = self.compute_attitude_loop(state, des_att)
            return np.array([T, torques[0], torques[1], torques[2]])

        # 步骤2：SMC姿态环
        att = state[6:9]
        rate = state[9:12]

        att_err = des_att - att
        att_err[2] = (att_err[2] + np.pi) % (2.0 * np.pi) - np.pi

        des_rate = self.kp_att * att_err

        ang_acc = self.smc.update(att_err, rate, des_rate=des_rate)

        # 角加速度 -> 力矩
        torques = self.I @ ang_acc

        return np.array([T, torques[0], torques[1], torques[2]])

    def reset(self):
        super().reset()
        self.smc.reset()
