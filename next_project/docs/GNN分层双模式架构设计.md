# 多无人机编队在线规划综合改进方案

> 编制日期：2026-04-30（v2，按反效果归因分析 §6 修补冲突）
> 适用范围：`next_project/`（Python 主线）
> 参考论文：
>
> - [论文1] "An Improved Artificial Potential Field Method for Path Planning and Formation Control of Multi-UAV Systems"（改进 APF）
> - [论文2] Wang, Yadav, Balakrishnan. "Cooperative UAV Formation Flying With Obstacle/Collision Avoidance", IEEE TCST, Vol.15, No.4, 2007（GNN 分层架构）
> - [论文3] "Multi-UAV Rendezvous Trajectory Planning Based on Improved MADDPG Algorithm in Complex Dynamic Obstacle Environments"（MADDPG）
> - [论文4] "Optimal Cooperative Fault-Tolerant Control for UAV Formation Based on Topology Reconfiguration Strategy"（容错拓扑重构）

## 0. 冲突修补总览（v2 新增）

本设计在 v1 评审中识别出 7 项与项目历史反效果根因（[优化反效果归因分析.md §2 R1–R8](优化反效果归因分析.md)）冲突的设计意图，已全部修补并在正文相应章节标注 **冲突修补 C{n}** 锚点：

| 编号 | 冲突点 | 关联根因 | 修补章节 |
|------|--------|---------|---------|
| C1 | 编队整体势场原计划"替代"逐机独立 APF | R6（航点级 ≠ 轨迹级） | §2.1 改为叠加层 |
| C2 | 通信约束力未声明量纲 + 阈值阶跃 | R3（修正量物理层错位） | §2.3 量纲断言 + tanh 平滑 |
| C3 | FaultDetector 阈值硬编码 | R4（跨场景副作用） | §5.3 配置化 + §7.1 预设 |
| C4 | n_decay 自适应每步采样性能伏笔 | 性能新瓶颈 | §2.2 空间缓存 |
| C5 | 4 个 APF 独立开关组合空间 16 种 | R5（多参数耦合） | §7.1 profile 三档化 |
| C6 | GNN 载体改写 + 旋转力场公式错误 | R9 论文复现错位（新增） | §2.4 Rodrigues + §3.3 变体声明 |
| C7 | 一次性合入 9 文件 | R8 实现噪声 | §八 拆 PR-A/PR-B |

详见 [优化反效果归因分析.md §6](优化反效果归因分析.md)。

---

## 一、总体目标与论文分工

### 1.1 四篇论文在项目中的定位

```
┌────────────────────────────────────────────────────────────────┐
│                    综合改进架构                                  │
├────────────────────────────────────────────────────────────────┤
│  论文2（核心框架）          论文1（安全增强）                      │
│  ┌─────────────────┐      ┌──────────────────┐                  │
│  │ Safe/Danger 双模式│◄────│ 编队整体势场       │                  │
│  │ GNN + 可见图规划  │◄────│ n_decay 自适应     │                  │
│  │ 分级调度器        │◄────│ 通信约束惩罚项     │                  │
│  └────────┬────────┘      │ 旋转力场替代GS逃逸 │                  │
│           │               └──────────────────┘                  │
│           ▼                                                     │
│  ┌─────────────────┐      ┌──────────────────┐                  │
│  │  在线重规划器     │◄────│ PER自适应调度     │ ← 论文3          │
│  │  WindowReplanner │      │ (MADDPG 经验回放) │   (远期v2.0)     │
│  └────────┬────────┘      └──────────────────┘                  │
│           │                                                     │
│           ▼                                                     │
│  ┌─────────────────┐      ┌──────────────────┐                  │
│  │  编队拓扑管理     │◄────│ Laplacian λ₂ 建模 │ ← 论文4          │
│  │  FormationTopology│◄───│ 故障检测+拓扑重构  │                  │
│  │  容错角色重分配   │◄────│ 执行器故障注入     │                  │
│  └─────────────────┘      └──────────────────┘                  │
└────────────────────────────────────────────────────────────────┘
```

### 1.2 核心目标

| 论文 | 角色 | 核心贡献 | 落地优先级 |
|------|------|---------|-----------|
| 论文2 | 架构框架 | Safe/Danger 双模式 + GNN 可见图规划 | P0（本次主线） |
| 论文1 | 安全增强 | 编队整体势场、n_decay 自适应、通信约束、旋转力场 | P0（与主线并行） |
| 论文4 | 鲁棒性 | 容错拓扑重构、Laplacian 建模、故障注入 | P1（主线后跟进） |
| 论文3 | 远期方向 | PER 自适应调度、MADDPG 框架 | P2（v2.0 规划） |

### 1.3 覆盖文件

| 文件 | 操作 | 关联论文 | 说明 |
|------|------|---------|------|
| `core/planning/visibility_graph.py` | 新增 | 论文2 | 障碍物顶点可见图 |
| `core/planning/gnn_planner.py` | 新增 | 论文2 | GNN 路径规划器 |
| `core/planning/dual_mode.py` | 新增 | 论文1+2 | Safe/Danger 调度 + 编队整体势场 |
| `core/planning/replanner.py` | 修改 | 论文2+3 | 集成双模式 + PER 自适应调度 |
| `core/artificial_potential_field.py` | 修改 | 论文1 | n_decay 自适应 + 旋转力场 + 通信约束 |
| `core/topology.py` | 修改 | 论文4 | Laplacian λ₂ + 容错拓扑重构 |
| `core/drone.py` | 修改 | 论文4 | 执行器故障注入 |
| `core/planning/__init__.py` | 修改 | 全部 | 导出新模块 |
| `config.py` | 修改 | 全部 | 综合配置 |

---

## 二、论文1：改进 APF 增强方案

> **冲突防护说明**：本节所有 4 个增强项已对照 [优化反效果归因分析.md §6](优化反效果归因分析.md) 的 C1–C6 修补准则重写。落地前必须读懂关联条款。

### 2.1 编队整体势场（叠加层，非替代层）

**问题**：当前 `artificial_potential_field.py:92` 的 `compute_avoidance_acceleration` 对每架无人机独立计算 APF，缺乏编队整体一致性。

**论文方案**：将编队视为整体，计算编队重心势场 + 成员相对势场。

> **冲突修补 C1（关联反效果根因 R6）**：编队整体势场**只能叠加在逐机独立 APF 之上，不得替代**。L3 实时排斥层必须由每架机基于自身位姿独立感知；以重心为代理点会让贴障碍物一侧的从机感受不到斥力，等同于关闭 L3 防护。

**落地设计**：

```python
class FormationAPF(ImprovedArtificialPotentialField):
    """编队整体人工势场（叠加增强，非替代）。

    叠加规则
    --------
    F_total[i] = α · F_centroid                # 编队整体势场（重心代理）
               + β · F_relative[i]              # 队形保持 + 成员间斥力
               + F_individual[i]                # 单机独立 APF（L3 防护，必须保留）

    其中 α, β ∈ [0, 1] 由场景配置；α=β=0 时退化为现有逐机独立 APF。

    1) 编队重心势场：以编队几何中心为控制点，计算障碍物斥力
       centroid = mean(leader_pos, follower_1, ..., follower_n)
       F_centroid = compute_avoidance_acceleration(centroid, goal, obstacles)

    2) 成员相对势场：保持队形 + 防内部碰撞
       F_relative[i] = k_form * (desired_offset[i] - actual_offset[i])
                      + k_inter * Σ(1/p_ij - 1/s) * (1/p_ij²)

    3) 单机独立势场（不可省略）：
       F_individual[i] = compute_avoidance_acceleration(pos[i], goal, obstacles)
    """

    def compute_formation_avoidance(
        self,
        leader_pos: np.ndarray,
        follower_positions: list[np.ndarray],
        goal: np.ndarray,
        obstacles,
        desired_offsets: list[np.ndarray],
        alpha: float = 0.4,
        beta: float = 0.6,
    ) -> tuple[np.ndarray, list[np.ndarray]]:
        """返回 (F_leader, [F_follower_1, ...])，单位 m/s²。

        调用方仍需叠加单机独立 APF 作为 L3 防护。
        """
```

**改动位置**：`formation_simulation.py` 的 `run()` 循环中，在控制计算前**追加**编队整体 APF 调用，与现有逐机独立 APF **共存叠加**。

**风险**：中等。编队重心与领航机位置可能偏差较大，需在队形切换过渡期平滑。

---

### 2.2 n_decay 自适应衰减指数（带缓存）

**问题**：当前 `artificial_potential_field.py:57` 硬编码 `n_decay=2`，不随环境变化。

**论文方案**：n 值与障碍密度/编队规模自适应调节。

> **冲突修补 C4**：直接每步采样会让 dt=0.012s × N 机 × M 障碍物的高频调用拉长仿真步长。必须加缓存，性能不退化。

**落地设计**：

```python
def _adaptive_n_decay(self, position, obstacles, formation_size):
    """根据局部障碍密度自适应调节衰减指数（带空间缓存）。

    n = 1 + 3 * (local_density / max_density)
    稀疏环境 n≈1：弱衰减，更安全
    密集环境 n≈4：强衰减，确保目标可达

    缓存策略
    --------
    - 当前位置距上次采样点 < r_rep/4 时复用上次密度估计
    - 每 K=10 步强制重算一次，避免长时间缓存失效
    """
    cache = self._density_cache  # dict: {agent_id: (last_pos, last_density, step)}
    if (cache.last_pos is not None
        and np.linalg.norm(position - cache.last_pos) < self.r_rep / 4
        and cache.step_since_last < 10):
        return 1.0 + 3.0 * cache.last_density
    local_density = self._estimate_local_obstacle_density(position, obstacles)
    cache.update(position, local_density)
    return 1.0 + 3.0 * min(local_density / self.max_density, 1.0)

def _estimate_local_obstacle_density(self, position, obstacles):
    """在 r_rep 半径内采样，统计 SDF < r_rep 的采样点比例。"""
    # 球形规则采样（默认 27 点 3×3×3 网格）
    # density = count(SDF < r_rep) / total_samples
```

**改动位置**：`artificial_potential_field.py:192` 的 `_obstacle_repulsion` 方法中，将固定 `n_decay` 替换为 `self._adaptive_n_decay(...)`。新增 `self._density_cache` 字段在 `__init__` 中。

**性能门槛（验收硬指标）**：`tests/test_apf_perf.py` —— 启用自适应 n_decay 后单步耗时不超过原值 1.2 倍。

**风险**：低。仅影响衰减指数的数值，不改变算法结构。

---

### 2.3 通信约束惩罚项（量纲固化 + 平滑过渡）

**问题**：当前无通信距离概念，机间距离超过通信半径时无恢复机制。

**论文方案**：势场函数包含通信距离惩罚项，机间距离 > 通信半径时施加额外引力。

> **冲突修补 C2（关联反效果根因 R3）**：
>
> 1. R3 是项目史上最严重的反效果根源（量纲混淆使航点完成率 100%→1%）。本项必须显式标注量纲、注入点，并加运行时断言。
> 2. 原始公式在 `dist > 0.8*comm_range` 时阶跃出现引力，存在不连续。改为 tanh 平滑门控。

**落地设计**：

```python
def _communication_constraint_force(
    self, position, other_positions, comm_range=10.0
) -> np.ndarray:
    """通信距离约束力。

    量纲：m/s²，作为 target_acc 前馈注入，禁止叠加 target_pos。
    上限：||F_comm|| ≤ self.k_comm（运行时 assert 保护）。

    平滑门控（避免阶跃）：
        gate = 0.5 * (1 + tanh((dist/comm_range - 0.8) / 0.05))
        F_comm = k_comm * gate * (dist/comm_range) * (diff/dist)
    """
    force = np.zeros(3)
    for other in other_positions:
        diff = other - position
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            continue
        gate = 0.5 * (1.0 + np.tanh((dist / comm_range - 0.8) / 0.05))
        force += self.k_comm * gate * (dist / comm_range) * (diff / dist)
    assert np.linalg.norm(force) <= self.k_comm * len(other_positions) + 1e-6, \
        "F_comm 超过 k_comm 上限，违反量纲断言"
    return force  # m/s²
```

**参数**：`k_comm = 0.3`（**单位 m/s²**，弱于障碍物斥力，强于一般扰动）。

**改动位置**：在 `compute_avoidance_acceleration` 中追加 `f_comm` 到加速度前馈，**禁止用于叠加 target_pos**。

**单测兜底**：`tests/test_comm_force_dim.py` 验证返回值范围与量纲注释一致。

---

### 2.4 旋转力场替代 Gram-Schmidt 逃逸（Rodrigues 公式）

**问题**：当前 `artificial_potential_field.py:267-270` 的 Gram-Schmidt + np.cross 逃逸方向保持恒定，路径振荡明显。

**论文方案**：绕斥力方向施加连续旋转的切向力，方向随位置平滑变化。

> **冲突修补 C6（关联反效果根因 R9 — 论文复现错位）**：
>
> 早期草案写作 `cross(total_repulsion, cross(rep_dir, to_goal_dir))`，根据 BAC-CAB 恒等式 `a×(a×b) = (a·b)a − (a·a)b`，结果几乎平行于 `to_goal`，等价于一个朝目标的引力，与论文1"切向旋转力打破势场对称"语义不符。
> 修正方案：使用 **Rodrigues 旋转公式**绕 `rot_axis` 把 `rep_dir` 旋转 θ ≈ π/2，得到真正的切向方向。

**落地设计**：

```python
def _compute_rotational_escape(self, total_repulsion, position, goal):
    """旋转势场逃逸力（Rodrigues 公式实现）。

    F_rot = mu_escape * (rep_dir * cos(θ) + cross(rot_axis, rep_dir) * sin(θ))

    其中:
      rep_dir   = total_repulsion / ||total_repulsion||
      rot_axis  = (rep_dir × to_goal) / ||·||  （旋转轴始终偏转朝向目标）
      θ ≈ π/2                                  （切向方向）
    """
    rep_norm = np.linalg.norm(total_repulsion)
    if rep_norm < 1e-9:
        return np.zeros(3)
    rep_dir = total_repulsion / rep_norm
    to_goal = goal - position
    goal_norm = np.linalg.norm(to_goal)
    if goal_norm < 1e-9:
        return np.zeros(3)
    rot_axis = np.cross(rep_dir, to_goal / goal_norm)
    if np.linalg.norm(rot_axis) < 1e-6:
        rot_axis = np.array([0.0, 1.0, 0.0])
    rot_axis /= np.linalg.norm(rot_axis)
    theta = 0.5 * np.pi
    tangent = rep_dir * np.cos(theta) + np.cross(rot_axis, rep_dir) * np.sin(theta)
    return self.mu_escape * tangent  # m/s²
```

**改动位置**：替换 `_enter_escape` 和 `_compute_escape_force` 中的 Gram-Schmidt 逻辑。

**单测兜底**：`tests/test_rotation_force_tangent.py` —— 验证 `dot(F_rot, total_repulsion) ≈ 0`（即真正的切向力）。

**风险**：低。旋转力场的逃逸路径通常比固定方向更平滑。

---

## 三、论文2：GNN 分层双模式（核心框架）

### 3.1 论文原始架构

```
┌─────────────────────────────────────────────────┐
│                 Mode Selector                     │
│      (传感器检测障碍物威胁 / 碰撞可能性)           │
├─────────────────────┬───────────────────────────┤
│    Safe Mode        │      Danger Mode           │
├─────────────────────┼───────────────────────────┤
│ Upper: 相对动力学    │ Upper: 改进GNN + 可见图     │
│        最优控制      │       在障碍物顶点上规划    │
├─────────────────────┼───────────────────────────┤
│ Lower: MPC 跟踪     │ Lower: MPC 跟踪            │
└─────────────────────┴───────────────────────────┘
```

### 3.2 项目映射

| 论文组件 | 项目实现 |
|---------|---------|
| Mode Selector | `DualModeScheduler`：传感器读数 + SDF 距离 + 滞回机制 + 论文1的通信约束作为额外危险信号 |
| Safe Mode Upper | `FormationTopology` + `BacksteppingController` + `AStar`/`HybridAStar` 路径规划 |
| Danger Mode Upper | `VisibilityGraph` + `GNNPlanner`：障碍物顶点上的最短路径 |
| Danger Mode 补充 | `ImprovedAPF`（论文1增强版）作为 Danger 模式的在线安全层 |
| Safe/Danger Lower | `Controller` / `HybridAttitudeController`（PID+SMC）/ 反步法轨迹跟踪 |

### 3.3 GNN 核心公式（论文2 式 19-20）

> **冲突修补 C6（论文复现严谨性，关联根因 R9）**：
>
> 论文2 原始 GNN 公式 19-20 作用于**栅格神经元**上的稠密活动场（活动场是空间稠密场）。本项目把它套到稀疏的**可见图节点**上，是**工程改写**而非论文严格复现，称为"GNN 可见图变体（vertex-only diffusion）"。验收阶段必须同时跑"原始栅格 GNN" + "可见图 GNN"两个基线对比路径长度。

```
dx_i/dt = -A·x_i + (B - x_i)·[I_i + Σ_j(w_ij·x_j)] - (D + x_i)·J_i
```

| 参数 | 含义 | 默认值 |
|------|------|--------|
| x_i | 神经元 i 的活动水平 | 初始 0 |
| A | 被动衰减率 | 10 |
| B | 活动上界 | 1 |
| D | 活动下界 | 1 |
| I_i | 外部激励 | 见下文 |
| w_ij | 连接权重 = 1/(1+d_ij)^γ | γ=2 |
| J_i | 抑制输入 | 0 |

**激励输入**：
```
I_i = V · [1/(1+d_goal_i)^α] · [1/(1+d_path_i)^β]
I_goal = E（目标神经元固定高激励）
```

**路径提取**：从起点沿最大活动值邻居贪心移动至目标。GNN 稳态活动场从目标向外单调递减，贪心等价于势场最陡下降。

### 3.4 3D 扩展策略

- **AABB**：8 个角点
- **Cylinder**：上/下圆各 angular_res 个切线点（共 2×angular_res）
- **Sphere**：Fibonacci 球面分布（n_dir=12）
- **顶点膨胀**：沿障碍物外法向偏移 buffer_zone
- **可见性**：SDF 采样判定线段碰撞

### 3.5 队形自适应收缩（论文2 窄通道策略）

**问题**：当前 `config.py:124` 的 `formation_schedule` 是时间硬编码的手动配置。

**论文方案**：基于通道宽度自动触发队形缩放。

**落地设计**：

```python
def _maybe_auto_shrink_formation(self, leader_pos, leader_vel, topology):
    """检测前方通道宽度，自动触发队形收缩。"""
    # 在前方 horizon/2 处采样横向宽度
    ahead = leader_pos + leader_vel / norm * (self.horizon / 2)
    channel_width = self._estimate_channel_width(ahead, leader_vel)
    envelope = topology.envelope_radius("diamond")

    if channel_width < 2.0 * envelope:
        # 自动收缩为 line 队形
        topology.switch_formation("line", transition_time=1.5)
        return True
    elif channel_width > 3.0 * envelope:
        # 通道恢复，展开回 diamond
        topology.switch_formation("diamond", transition_time=2.0)
        return True
    return False
```

**集成点**：`DualModeScheduler` 或 `WindowReplanner.step()` 中，在 Danger 模式激活时额外检查通道宽度。

**风险**：中。通道宽度估计依赖于传感器读数的准确性。

### 3.6 复杂度对比

| 方法 | 搜索节点 | 100×100×20 栅格 |
|------|---------|---------------|
| A* (26邻域) | 200,000 | O(N³logN) ≈ 数百万次扩展 |
| GNN+可见图 (n=30) | ~722 | O(n²) ≈ 数万次运算 |

---

## 四、论文3：风险驱动重规划调度（远期方向）

### 4.1 论文核心思路

MADDPG（Multi-Agent DDPG）采用集中训练+分布式执行框架。完整 MADDPG 框架是 v2.0 工作；本节仅提取论文中**优先级经验回放（PER）**的"按重要性差异化采样"启发，**改造为基于风险的自适应重规划间隔调度**——并非完整 PER 实现。

> 命名说明：早期草案称为"PER 自适应间隔"易误导（PER 严格定义为按 TD-error 加权采样经验池）。本节正式命名为 **`RiskAdaptiveReplanInterval`**，行为是按风险信号缩放间隔。

### 4.2 风险驱动重规划间隔

**问题**：当前 `WindowReplanner.interval` 固定（如 0.4s），不考虑环境风险变化。

**落地设计**：

```python
class RiskAdaptiveReplanInterval:
    """风险驱动的自适应重规划间隔调度器（PER 思想启发，非完整 PER）。

    原理
    ----
    维护滑动窗口内的"碰撞风险历史"作为重要性代理量。
    高风险 → 缩短间隔（更频繁重规划）
    低风险 → 延长间隔（节省计算）
    """

    def __init__(self, base_interval=0.4, min_interval=0.1, max_interval=1.0):
        self.base = base_interval
        self.min = min_interval
        self.max = max_interval
        self._risk_history = []  # 环形缓冲区

    def update(self, min_sdf: float, sensor_min: float):
        """记录当前帧的风险指标。"""
        risk = 1.0 - min(min_sdf / 5.0, 1.0)
        risk = max(risk, 1.0 - min(sensor_min / 5.0, 1.0))
        self._risk_history.append(risk)
        if len(self._risk_history) > 20:
            self._risk_history.pop(0)

    @property
    def interval(self) -> float:
        if not self._risk_history:
            return self.base
        avg_risk = sum(self._risk_history) / len(self._risk_history)
        return self.max - (self.max - self.min) * avg_risk
```

**改动位置**：`WindowReplanner.step()` 中用自适应间隔替代固定 `self.interval`。

**风险**：低。仅影响重规划频率，不改变路径生成逻辑。

### 4.3 MADDPG 完整框架（v2.0 保留项）

完整的集中训练+分布式执行框架是远期规划，需引入 PyTorch/TensorFlow 依赖。当前设计文档仅预留接口，不落地。

---

## 五、论文4：容错拓扑重构

### 5.1 问题与方案概述

当前项目**完全缺失容错能力**——任何一架无人机故障会导致编队崩溃或碰撞。

论文4 提供三个层面的解决方案：

| 层次 | 内容 | 落地复杂度 |
|------|------|-----------|
| L1 故障注入 | 模拟执行器完全/部分失效 | 低 |
| L2 故障检测 | 基于状态异常的在线检测 | 中 |
| L3 拓扑重构 | Laplacian λ₂ 引导的最优拓扑切换 | 高 |

### 5.2 L1：执行器故障注入

**落地设计**（`core/drone.py` 修改）：

```python
# Drone 类新增故障状态
class Drone:
    def __init__(self, ...):
        ...
        self.fault_mask = np.ones(4)      # 1.0 = 正常, 0.0 = 完全失效
        self.fault_active = False

    def inject_fault(self, rotor_index: int, severity: float):
        """注入执行器故障。
        severity: 0.0=完全失效, 0.5=推力减半, 1.0=正常
        """
        self.fault_mask[rotor_index] = severity
        self.fault_active = True

    def _rotor_step(self, desired_u):
        thrusts_cmd = self.allocator.allocate_thrusts(desired_u)
        thrusts_cmd = thrusts_cmd * self.fault_mask  # 故障注入点
        ...
```

### 5.3 L2：故障检测

> **冲突修补 C3（关联反效果根因 R4）**：阈值不得硬编码在 `__init__`，必须并入 `SimulationConfig`，且简单场景预设默认禁用故障检测，避免风扰被误触发引发不必要的拓扑重构。

```python
class FaultDetector:
    """基于状态异常的在线故障检测。

    检测规则（任一触发）：
    1. 速度突变: ||dv/dt|| > max_jerk (默认 10 m/s³) 持续 0.5s
    2. 位置跳变: 实际位置与期望位置偏差 > pos_dev (默认 5m) 持续 1s
    3. 控制饱和: 连续 saturate_steps (默认 50) 步控制量在 [0.95*max, max]

    所有阈值从 SimulationConfig 注入，禁止在 __init__ 中硬编码兜底。
    简单场景（basic / obstacle）默认 fault_detection_enabled=False。
    """

    def __init__(self, cfg: SimulationConfig):
        self.max_jerk = cfg.fault_detector_max_jerk
        self.pos_dev_threshold = cfg.fault_detector_pos_dev
        self.saturate_steps = cfg.fault_detector_saturate_steps

    def check(self, state, desired_state, control, dt) -> int | None:
        """返回故障无人机索引或 None。"""
```

**回归断言**：`tests/test_scene_regression.py` 增加"basic 场景无故障运行 30s 不应触发误检"。

### 5.4 L3：图论拓扑重构

**核心概念 — Laplacian 矩阵与代数连通度**：

编队拓扑用图 `G = (V, E)` 表示，其中：
- 节点 V = 无人机集合
- 边 E = 通信/编队约束关系

Laplacian 矩阵：`L = D - A`，其中 D 为度矩阵，A 为邻接矩阵。
代数连通度 `λ₂` = L 的第二小特征值。λ₂ 越大，图连通性越强，编队越稳定。

**落地设计**（`core/topology.py` 新增）：

```python
class TopologyGraph:
    """编队图论拓扑模型。

    用途
    ----
    用图 Laplacian 矩阵描述编队拓扑关系，提取代数连通度 λ₂，
    作为拓扑鲁棒性度量和故障重构选择的依据。
    """

    def __init__(self, offsets: list[np.ndarray]):
        self.n = len(offsets) + 1  # followers + leader
        self._build_from_offsets(offsets)

    def _build_from_offsets(self, offsets):
        """从几何偏移构建邻接矩阵。"""
        # 节点 0 = 领航机，节点 1..n = 从机
        # 边权重 a_ij = exp(-||offset_i - offset_j||² / σ²)
        ...

    @property
    def algebraic_connectivity(self) -> float:
        """λ₂ = Laplacian 的第二小特征值。

        用途：λ₂ 越大表示拓扑连通性越强，容错重构时选择 λ₂ 最大的可行拓扑。
        """
        eigenvals = np.linalg.eigvalsh(self.laplacian)
        return float(eigenvals[1])

    def best_reconfig_topology(
        self,
        failed_indices: list[int],
        candidates: list[str] = ("v_shape", "diamond", "line", "triangle"),
    ) -> str:
        """故障后选择 λ₂ 最大的可行拓扑。

        1. 排除含故障机的候选配置
        2. 对每个候选拓扑构建 Laplacian
        3. 返回 λ₂ 最大的拓扑类型
        """
        best = candidates[0]
        best_lambda2 = -1
        for topo in candidates:
            offsets = FormationTopology._compute_offsets_static(
                topo, self.n - 1 - len(failed_indices)
            )
            tg = TopologyGraph(offsets)
            if tg.algebraic_connectivity > best_lambda2:
                best_lambda2 = tg.algebraic_connectivity
                best = topo
        return best
```

**集成**：`FormationTopology` 增加 `fault_reconfigure(failed_indices)` 方法，自动选择最佳拓扑并生成平滑过渡轨迹。

---

## 六、模块设计汇总

### 6.1 新增文件

#### `core/planning/visibility_graph.py` — 可见图

```python
class VisibilityGraph:
    vertices: list[np.ndarray]          # (M,3) 世界坐标
    adjacency: list[set[int]]           # 邻接表
    vertex_to_obs: list[int]            # 顶点→障碍物映射

    def build(start, goal, obstacle_field, visible_range) -> None
    def add_obstacle(obs) -> list[int]  # 增量弹窗障碍物
    def is_visible(p1, p2) -> bool      # SDF 采样判定
```

#### `core/planning/gnn_planner.py` — GNN 规划器

```python
class GNNPlanner(Planner):  # 继承 Planner 基类
    A=10, B=1, D=1, gamma=2, alpha=2, beta=2, V=100, E=50

    def plan(start, goal, grid, **kw) -> np.ndarray
    def _integrate_gnn(w, I) -> np.ndarray            # Euler 积分
    def _compute_excitation(vertices, start, goal)     # 激励
    def _extract_path_from_activities(x, adj)          # 贪心提取
```

#### `core/planning/dual_mode.py` — 双模式 + 编队整体势场调度

```python
class DualModeScheduler:
    SAFE = "safe"; DANGER = "danger"

    def classify(t, sensor_reading, obstacle_field, position) -> str
    def should_shrink_formation(topology, leader_pos, leader_vel) -> bool
    def get_danger_obstacles(obstacle_field, position, range) -> list

class FormationAPF(ImprovedArtificialPotentialField):  # 论文1
    def compute_formation_avoidance(leader_pos, followers, goal, obs, offsets)
```

#### `core/fault_detector.py` — 故障检测（论文4）

```python
class FaultDetector:
    def check(state, desired_state, control, dt) -> int | None
```

### 6.2 修改文件

#### `core/artificial_potential_field.py`

| 改动 | 关联论文 | 修改行 |
|------|---------|--------|
| `n_decay` → 自适应 `_adaptive_n_decay()` | 论文1 | 行57, 192 |
| `_enter_escape` 用旋转力场替代 Gram-Schmidt | 论文1 | 行245-275 |
| 新增 `_communication_constraint_force()` | 论文1 | 新方法 |
| `reset()` 清零通信状态 | 论文1 | 行81 |

#### `core/topology.py`

| 改动 | 关联论文 | 修改行 |
|------|---------|--------|
| 新增 `TopologyGraph` 类 | 论文4 | 新类 |
| `FormationTopology` 新增 `fault_reconfigure()` | 论文4 | 新方法 |
| `FormationTopology` 新增 `auto_shrink()` | 论文2 | 新方法 |

#### `core/drone.py`

| 改动 | 关联论文 | 修改行 |
|------|---------|--------|
| 新增 `fault_mask` 属性 | 论文4 | `__init__` |
| 新增 `inject_fault()` | 论文4 | 新方法 |
| `_rotor_step` 应用故障掩码 | 论文4 | 行140 |

#### `core/planning/replanner.py`

| 改动 | 关联论文 | 修改行 |
|------|---------|--------|
| `__init__` 新增 `danger_planner`, `dual_mode`, `adaptive_interval` | 论文2+3 | 行51-86 |
| `step()` 插入双模式检查 + PER 自适应间隔 | 论文2+3 | 行91-159 |
| 新增 `_replan_danger()` | 论文2 | 新方法 |
| 新增 `_resolve_obstacle_field()` | 论文2 | 新方法 |

---

## 七、配置扩展

### 7.1 SimulationConfig 新增字段

> **冲突修补 C5（关联反效果根因 R5）**：4 个独立 APF 开关组合空间 16 种，违反 R5 多参数耦合。改为 `apf_paper1_profile` 三档预设；独立开关仅作为开发模式高级选项保留。
>
> **冲突修补 C3（关联反效果根因 R4）**：FaultDetector 阈值字段全部并入 `SimulationConfig`，并在简单场景预设默认禁用。

```python
from typing import Literal

@dataclass
class SimulationConfig:
    # ... 现有字段 ...

    # ---- 论文2: 双模式 GNN ----
    danger_mode_enabled: bool = False
    gnn_V: float = 100.0
    gnn_E: float = 50.0
    gnn_alpha: float = 2.0
    gnn_beta: float = 2.0
    gnn_gamma: float = 2.0
    gnn_angular_res: int = 8
    gnn_buffer_zone: float = 0.3

    # ---- 论文1: 改进 APF（profile 化，C5 修补） ----
    apf_paper1_profile: Literal["off", "conservative", "aggressive"] = "off"
    # off:          基线行为，所有 4 项关闭
    # conservative: n_decay 自适应 + Rodrigues 旋转力场（仅风险最低的两项）
    # aggressive:   全开（含编队整体势场叠加 + 通信约束）
    apf_comm_range: float = 10.0
    apf_centroid_alpha: float = 0.4   # 整体势场叠加权重 α（C1 修补）
    apf_centroid_beta: float = 0.6    # 队形保持势场权重 β
    # 高级开发开关（profile 之外的细粒度覆盖，谨慎使用）
    apf_dev_override: bool = False
    apf_adaptive_n_decay: bool = False
    apf_formation_centroid: bool = False
    apf_comm_constraint: bool = False
    apf_rotational_escape: bool = False

    # ---- 论文3: 风险驱动重规划间隔 ----
    replan_adaptive_interval: bool = False
    replan_interval_min: float = 0.1
    replan_interval_max: float = 1.0

    # ---- 论文4: 容错 ----
    fault_injection_enabled: bool = False
    fault_detection_enabled: bool = False
    fault_reconfig_enabled: bool = False
    # FaultDetector 阈值（C3 修补：禁止硬编码）
    fault_detector_max_jerk: float = 10.0      # m/s³
    fault_detector_pos_dev: float = 5.0        # m
    fault_detector_saturate_steps: int = 50

    # ---- 双模式阈值 ----
    sensor_danger_threshold: float = 2.0
    sensor_safe_threshold: float = 4.0
    sdf_danger_threshold: float = 0.5
```

### 7.2 新增预设场景

```python
# config.py 新增预设
def _config_warehouse_danger() -> SimulationConfig:
    """仓库在线版 + GNN 双模式 + 改进 APF（保守档）"""
    cfg = _config_warehouse_online()
    cfg.danger_mode_enabled = True
    cfg.apf_paper1_profile = "conservative"
    return cfg

def _config_fault_tolerance() -> SimulationConfig:
    """容错测试场景：注入单机故障，验证拓扑重构"""
    cfg = _config_warehouse()
    cfg.fault_injection_enabled = True
    cfg.fault_detection_enabled = True
    cfg.fault_reconfig_enabled = True
    return cfg

# 简单场景必须显式禁用故障检测，避免误检（C3 修补）
def _config_basic() -> SimulationConfig:
    cfg = ...
    cfg.fault_detection_enabled = False
    cfg.apf_paper1_profile = "off"
    return cfg
```

---

## 八、落地里程碑

> **冲突修补 C7（关联反效果根因 R8 经验）**：原"7.5 人日一次性合入 9 个文件"违反"小步快跑、单测先行"准则。改为 **PR-A / PR-B 两批合入**，中间必须跑完三场景回归。

### 8.1 PR-A：论文 2 + 论文 3 风险调度（主线）

| 阶段 | 内容 | 关联论文 | 文件 | 工时 | 验收 |
|------|------|---------|------|------|------|
| M1 | 可见图模块 | 论文2 | `visibility_graph.py` | 1d | 3障碍物场景：M ≤ 2+24n；M²·is_visible 平均耗时 < 50ms |
| M2 | GNN 规划器（可见图变体） | 论文2 | `gnn_planner.py` | 1d | 路径长度 ≤ 1.05×最优；活动场单调性单测通过（C6） |
| M5 | WindowReplanner 集成 + 风险驱动间隔 | 论文2+3 | `replanner.py` | 0.5d | `danger_mode_enabled=False` 行为不变；三场景回归无劣化 |
| M7a | PR-A 配置预设 | 全部 | `config.py` | 0.3d | `_config_warehouse_danger` 可跑通 |
| M8a | PR-A 端到端验证 | 全部 | 集成测试 | 0.5d | 0 碰撞，GNN < A* 50% 耗时 |

**PR-A 合入门槛**：三场景回归（basic/obstacle/warehouse）全部 PASS，且 `tests/test_scene_regression.py` 守门通过。

### 8.2 PR-B：论文 1 APF 增强 + 论文 4 容错（增强）

| 阶段 | 内容 | 关联论文 | 文件 | 工时 | 验收 |
|------|------|---------|------|------|------|
| M3 | 双模式 + 编队势场（叠加非替代） | 论文1+2 | `dual_mode.py` | 1d | 模式切换含滞回；编队 APF 与单机 APF 共存（C1） |
| M4 | APF 增强（n_decay 缓存 + Rodrigues 旋转 + 通信约束 tanh） | 论文1 | `artificial_potential_field.py` | 1d | C2/C4/C6 全部单测通过；perf 不退化 |
| M6 | 容错注入+检测+重构（阈值配置化） | 论文4 | `drone.py`, `topology.py`, `fault_detector.py` | 2d | basic 场景 30s 不误检（C3）；故障后 3s 内完成拓扑重构 |
| M7b | PR-B 配置预设 + APF profile | 全部 | `config.py` | 0.3d | profile=off/conservative/aggressive 三档回归通过（C5） |
| M8b | PR-B 端到端验证 | 全部 | 集成测试 | 0.5d | 容错预设故障注入后 0 碰撞 |

**PR-B 前置条件**：PR-A 已合入并稳定运行至少一个迭代周期。

### 8.3 总计与关键路径

合计约 **8.6 人日**（含修补冲突的额外 1.1 天）。关键路径：M1→M2→M5→M7a→M8a（PR-A）→ 回归 → M3/M4→M6→M7b→M8b（PR-B）。

---

## 九、风险矩阵

| 风险 | 概率 | 影响 | 关联论文 | 对策 |
|------|------|------|---------|------|
| GNN 不收敛 | 低 | 高 | 论文2 | 自动回退到现有三级规划管线 |
| 可见图过密（M²边爆炸） | 中 | 中 | 论文2 | 限制 visible_range，跳过远距离顶点对；M1 性能门槛守门 |
| 编队整体势场导致从机过度约束 | 中 | 中 | 论文1 | 整体势场仅作为叠加层（C1），权重 α/β 可调；α=0 退化为独立 APF |
| 故障误检测触发不必要重构 | 中 | 中 | 论文4 | 阈值配置化（C3）+ 滞回 + 简单场景默认禁用 |
| 旋转力场公式实现错位 | 中 | 中 | 论文1 | Rodrigues 公式（C6），单测验证切向性 |
| APF 多开关组合非单调 | 中 | 中 | 论文1 | profile 三档预设（C5），独立开关仅开发模式 |
| 自适应 n_decay 拖慢仿真 | 中 | 低 | 论文1 | 空间缓存 + K 步重算（C4）；perf 单测守门 |
| 通信约束力量纲混淆 | 低 | 高 | 论文1 | 显式 m/s² 注释 + 运行时 assert（C2） |
| 一次性合入 9 文件回归爆炸 | 中 | 高 | 全部 | 拆 PR-A/PR-B（C7），中间三场景回归守门 |
| 风险驱动间隔在静态环境浪费计算 | 低 | 低 | 论文3 | 风险低时 interval→max，反而节省计算 |

---

## 十、关键接口签名速查

```python
# core/planning/visibility_graph.py (论文2)
class VisibilityGraph:
    def build(start, goal, obstacle_field, visible_range) -> None
    def add_obstacle(obs) -> list[int]
    def is_visible(p1, p2) -> bool

# core/planning/gnn_planner.py (论文2)
class GNNPlanner(Planner):
    def plan(start, goal, grid, **kw) -> np.ndarray

# core/planning/dual_mode.py (论文1+2)
class DualModeScheduler:
    def classify(t, sensor_reading, obstacle_field, position) -> str
    @property
    def is_danger(self) -> bool

class FormationAPF(ImprovedArtificialPotentialField):
    def compute_formation_avoidance(leader, followers, goal, obs, offsets)
        -> tuple[np.ndarray, list[np.ndarray]]

# core/artificial_potential_field.py (论文1)
class ImprovedArtificialPotentialField:
    # 新增: _adaptive_n_decay, _communication_constraint_force,
    #       _compute_rotational_escape (替代 Gram-Schmidt)

# core/topology.py (论文4)
class TopologyGraph:
    @property
    def algebraic_connectivity(self) -> float
    def best_reconfig_topology(failed_indices, candidates) -> str

class FormationTopology:
    def fault_reconfigure(failed_indices) -> None  # 新增
    def auto_shrink(channel_width, envelope) -> bool  # 新增

# core/drone.py (论文4)
class Drone:
    fault_mask: np.ndarray  # 新增
    def inject_fault(rotor_index, severity) -> None  # 新增

# core/fault_detector.py (论文4)
class FaultDetector:
    def check(state, desired_state, control, dt) -> int | None
```
