"""基于状态异常的在线故障检测（论文4: 容错拓扑重构 L2）。

用途
----
检测无人机执行器故障：速度突变、位置跳变、控制饱和。
所有阈值从 SimulationConfig 注入，禁止硬编码。

冲突修补 C3
-----------
- 阈值配置化，禁止 __init__ 硬编码兜底
- 简单场景默认 fault_detection_enabled=False
"""

from __future__ import annotations

import numpy as np


class FaultDetector:
    """基于状态异常的在线故障检测器。

    检测规则（任一触发即报故障）
    ---------------------------
    1. 速度突变: ||dv/dt|| > max_acc 持续 0.5s
    2. 位置跳变: ||actual - desired|| > pos_dev_threshold 持续 1s
    3. 控制饱和: 连续 saturate_steps 步控制量在 [0.95*max, max]

    参数
    ----
    max_acc: 速度变化率阈值 m/s³。
    pos_dev_threshold: 位置偏差阈值 m。
    saturate_steps: 控制饱和连续步数阈值。
    dt: 仿真步长 s。
    """

    def __init__(
        self,
        max_acc: float = 10.0,
        pos_dev_threshold: float = 5.0,
        saturate_steps: int = 50,
        dt: float = 0.012,
    ):
        self.max_acc = float(max_acc)
        self.pos_dev_threshold = float(pos_dev_threshold)
        self.saturate_steps = int(saturate_steps)
        self.dt = float(dt)

        # 状态历史
        self._prev_vel: dict[int, np.ndarray] = {}
        self._acc_duration: dict[int, float] = {}
        self._pos_dev_duration: dict[int, float] = {}
        self._saturate_count: dict[int, int] = {}

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def check(
        self,
        agent_id: int,
        state: np.ndarray,
        desired_state: np.ndarray,
        control: np.ndarray,
        control_max: float = 10.0,
    ) -> bool:
        """检查指定 agent 是否存在故障。

        返回 True 表示检测到故障。
        """
        vel = state[3:6]
        pos = state[0:3]
        des_pos = desired_state[0:3]

        # 规则 1: 速度突变（实际计算加速度 ||Δv||/Δt）
        acc_detected = False
        if agent_id in self._prev_vel:
            dv = vel - self._prev_vel[agent_id]
            acc_mag = float(np.linalg.norm(dv)) / self.dt
            if acc_mag > self.max_acc:
                self._acc_duration[agent_id] = self._acc_duration.get(agent_id, 0.0) + self.dt
                if self._acc_duration[agent_id] >= 0.5:
                    acc_detected = True
            else:
                self._acc_duration[agent_id] = 0.0
        self._prev_vel[agent_id] = vel.copy()

        # 规则 2: 位置跳变
        pos_dev = float(np.linalg.norm(pos - des_pos))
        if pos_dev > self.pos_dev_threshold:
            self._pos_dev_duration[agent_id] = self._pos_dev_duration.get(agent_id, 0.0) + self.dt
        else:
            self._pos_dev_duration[agent_id] = 0.0
        pos_detected = self._pos_dev_duration.get(agent_id, 0.0) >= 1.0

        # 规则 3: 控制饱和
        ctrl_norm = float(np.linalg.norm(control))
        if ctrl_norm > 0.95 * control_max:
            self._saturate_count[agent_id] = self._saturate_count.get(agent_id, 0) + 1
        else:
            self._saturate_count[agent_id] = 0
        sat_detected = self._saturate_count.get(agent_id, 0) >= self.saturate_steps

        return acc_detected or pos_detected or sat_detected

    def reset(self, agent_id: int | None = None) -> None:
        """重置指定 agent（或全部）的检测状态。"""
        if agent_id is not None:
            self._prev_vel.pop(agent_id, None)
            self._acc_duration.pop(agent_id, None)
            self._pos_dev_duration.pop(agent_id, None)
            self._saturate_count.pop(agent_id, None)
        else:
            self._prev_vel.clear()
            self._acc_duration.clear()
            self._pos_dev_duration.clear()
            self._saturate_count.clear()
