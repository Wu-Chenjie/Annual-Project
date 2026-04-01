import numpy as np


class SecondOrderSMC:
    """
    基于 Super-Twisting 算法的二阶滑模控制器（姿态环）

    滑模面定义：
        s = angle_rate_error + lambda * angle_error

    标准 Super-Twisting 控制律：
        u = u1 + u2
        u1 = -alpha * |s|^(1/2) * sign(s)
        du2/dt = -beta * sign(s)

    特性：
        - 无需已知扰动上界的精确值
        - 控制信号连续（消除传统滑模的抖振问题）
        - alpha, beta 满足一定条件即可保证有限时间收敛

    输出为角加速度级别，需在外层乘以惯量矩阵 I 转换为力矩
    """

    def __init__(self, dt,
                 lam=np.array([8.0, 8.0, 4.0]),
                 alpha=np.array([15.0, 15.0, 8.0]),
                 beta=np.array([10.0, 10.0, 5.0])):
        """
        :param dt:    控制周期 [s]
        :param lam:   滑模面系数 lambda [3]
        :param alpha: Super-Twisting 比例增益 [3]
        :param beta:  Super-Twisting 积分增益 [3]
        """
        self.dt = dt
        self.lam = lam
        self.alpha = alpha
        self.beta = beta

        # 积分状态 v（Super-Twisting 的积分项）
        self.v = np.zeros(3)

    def compute_sliding_surface(self, angle_error, rate_error):
        """
        计算滑模面
        s = rate_error + lambda * angle_error
        """
        return rate_error + self.lam * angle_error

    def update(self, angle_error, angle_rate, des_rate=None):
        """
        计算控制输出（角加速度级别）

        :param angle_error: 姿态角误差 [roll_e, pitch_e, yaw_e]
        :param angle_rate:  当前角速度 [wx, wy, wz]
        :param des_rate:    期望角速度（默认为0）
        :return: 角加速度指令 [3]
        """
        if des_rate is None:
            des_rate = np.zeros(3)

        rate_error = des_rate - angle_rate

        # 滑模面
        s = self.compute_sliding_surface(angle_error, rate_error)

        # Super-Twisting 控制律
        # u1 = -alpha * |s|^(1/2) * sign(s)
        abs_s = np.abs(s)
        sqrt_abs_s = np.sqrt(abs_s + 1e-12)  # 避免除零
        sign_s = np.sign(s)

        u1 = -self.alpha * sqrt_abs_s * sign_s

        # u2 由积分得到: dv/dt = -beta * sign(s)
        self.v += -self.beta * sign_s * self.dt

        # 总控制量 = 连续项 + 积分项
        u = u1 + self.v

        return u

    def reset(self):
        """重置积分状态"""
        self.v = np.zeros(3)
'''
## 两种滑模控制方案对比分析

### 方案A：原 Super-Twisting 算法

```python
# 核心控制律
u1 = -alpha * sqrt(|s|) * sign(s)   # 连续项
v += -beta * sign(s) * dt            # 积分项
u = u1 + v
```

### 方案B：新 指数趋近律滑模

```python
# 核心控制律
sign_s = sat(s, Delta)               # 饱和函数替代sgn
u = lam * rate_error + epsilon * sign_s + k * s
```

---

### 逐项对比

| 对比维度 | 方案A（Super-Twisting） | 方案B（指数趋近律） |
|---------|------------------------|-------------------|
| **理论基础** | 二阶滑模理论，Levant提出 | 经典变结构控制，指数+等速趋近律 |
| **抖振抑制** | 理论上无抖振（控制信号连续） | 依赖饱和函数 sat 近似消抖，边界层内退化为线性 |
| **有无积分状态** | 有（积分项 v 需要维护） | 无（纯代数运算，无内部状态） |
| **数值稳定性** | 积分项 v 可能漂移，长时间仿真需注意 | 无状态量，天然稳定，不存在漂移问题 |
| **实现复杂度** | 中等（需维护积分状态和 reset） | 简单（纯代数，reset 为空） |
| **参数整定** | alpha, beta, lam 三组，整定较难 | lam, epsilon, k 三组，物理含义直观 |
| **收敛速度** | 有限时间收敛（理论保证） | 指数收敛 + 等速趋近，收敛速度可调 |
| **对扰动鲁棒性** | 强（积分项自动补偿未知扰动） | 中等（无积分补偿，依赖 epsilon 覆盖扰动） |
| **计算开销** | 稍高（sqrt运算 + 积分更新） | 极低（仅乘加运算） |
| **与RK4兼容性** | 积分项在RK4外独立更新，存在步长耦合 | 无状态，与任何积分器完全兼容 |

---

### 关键优劣分析

**方案A 的优势：**
- 对未知扰动有积分补偿能力，风场变化时自动适应
- 理论上控制信号完全连续，对执行器友好

**方案A 的劣势：**
- 积分项 `v` 在长时间仿真中可能漂移
- `sqrt(|s|)` 在 s 接近零时梯度趋于无穷，数值敏感
- 积分更新在 RK4 之外用欧拉法，步长一致性差

**方案B 的优势：**
- 无内部状态，实现简单，调试容易
- 饱和函数 sat 消除了边界层内的高频抖振
- 参数物理含义清晰：epsilon 控制趋近速度，k 控制指数衰减
- 与您的 PDF 教材公式完全对应，便于论文撰写和答辩

**方案B 的劣势：**
- 边界层内退化为线性控制，存在稳稳态误差（由 Delta 大小决定，Delta 越小精度越高但抖振越明显）
无积分补偿，面对持续未知扰动时无法自动消除稳态偏差
本质上仍是一阶滑模的改进，理论层面不如二阶滑模"高级"
综合推荐

推荐采用方案B（指数趋近律滑模），理由如下：

考量因素	结论
您的项目性质	仿真项目，不涉及真实执行器磨损，抖振问题影响有限
与教材对应	方案B直接对应PDF第47页公式，答辩和论文撰写有据可依
数值可靠性	无内部状态，不存在积分漂移，长时间仿真更稳定
调试效率	纯代数运算，参数物理含义直观，调参快
扰动补偿	虽无积分项，但位置外环的PID已有积分器补偿稳态误差，整体系统不缺积分能力
实现风险	方案A的sqrt(0)数值敏感问题在快速姿态变化时可能引发异常

'''