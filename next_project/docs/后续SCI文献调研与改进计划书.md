# 四旋翼集群编队仿真系统：SCI文献对标与低空标准转化计划书

> 调研日期：2026-05-05  
> 适用范围：`next_project` 当前 Python/C++ 双线四旋翼编队仿真系统  
> 关联文档：
> - `docs/项目总改进规划.md` —— 改进路线、实施顺序、里程碑与工时（2026-05-03）
> - `docs/技术文档.md` —— 已实现系统的技术结构、数学模型、模块接口
> - `docs/优化反效果归因分析.md` —— R1-R8 反效果根因、决策清单、合入流程
> - `docs/需求符合性检查.md` —— 当前需求满足度与优化项

---

## 1. 文档定位与边界

本文档回答的问题是：**"项目后续还能从哪些 SCI/SCIE 顶刊论文和低空安全标准中借鉴思想，以及这些思想如何转化为可运行的代码增量"**。

它与现有文档的分工如下：

| 本文档负责 | 本文档不重复 | 去向 |
|---|---|---|
| SCI/SCIE 文献的关键思想提炼 | 已落地文献的实现细节 | `docs/技术文档.md` 对应章节 |
| 低空安全标准（T/CICC）的参数转化 | 实施顺序、工时、里程碑 | `docs/项目总改进规划.md` |
| 新增数据结构与接口变更建议 | 反效果根因与避坑准则全文 | `docs/优化反效果归因分析.md`（本文仅引用） |
| C++ 同步评估建议 | 现有回归测试命令与基线 | `docs/技术文档.md` 第12节 |

**阅读顺序建议**：先读 `项目总改进规划.md` 了解当前已完成的 P0 可靠性补强和 P1 轨迹质量方向，再读本文获取文献对标与标准转化视角。

---

## 2. 核心目标

在已验证的零碰撞与多场景可运行基础上（`项目总改进规划.md` 第1节基线），把系统从"能走通"推进到：

1. **轨迹更平滑**：将离散路径点升级为满足速度、加速度、jerk/snap 约束的连续时间轨迹。
2. **参数更可解释**：将硬编码无人机模型参数升级为可切换、可标定、可对照论文的 `DroneParams` profile。
3. **多机更安全**：将单机 obstacle clearance 扩展到队形包络、机间间隔、下洗区和动态冲突消解。
4. **标准可复现**：将低空安全标准中的安全间隔、风险评估转化为可选的 `safety_profile`，用于标准化演示。

这些目标与 `项目总改进规划.md` 的对应关系：

| 本文档目标 | 对应 项目总改进规划.md |
|---|---|
| 目标1 连续轨迹 | P1-1 FIRI 多段走廊轨迹优化 |
| 目标2 参数profile | P1-2 APF profile 化之后的下一阶段 |
| 目标3 多机安全 | P2-2 动态多障碍分区策略的延伸 |
| 目标4 标准转化 | 新增维度，现有规划未覆盖 |

---

## 3. SCI文献对标矩阵

### 3.1 轨迹生成与路径规划

| 文献 | 来源 | 关键思想 | 对本项目的可用价值 | 落地状态 | 对应文件/规划 |
|---|---|---|---|---|---|
| Mellinger & Kumar, Minimum snap trajectory generation and control for quadrotors, ICRA 2011 | IEEE ICRA, DOI: 10.1109/ICRA.2011.5980409 | 利用四旋翼微分平坦性，把航点序列转化为最小 snap 多项式轨迹 | 当前 A*/Hybrid A*/FIRI 输出可作为航点和走廊输入，后端求解 minimum snap/jerk 轨迹 | **待落地** | 新增 `core/planning/trajectory_optimizer.py` |
| Zhou et al., Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight, RA-L 2019 | IEEE RA-L, DOI: 10.1109/LRA.2019.2927938 | kinodynamic search + B-spline 优化 + EDF/动态约束 + 时间调整 | 项目已有 ESDF/FIRI，可参考 B-spline 优化与动态可行性时间重分配 | **待评估** | 作为 P1 之后的高阶方向 |
| Zhang et al., Search-based Path Planning and Receding Horizon Based Trajectory Generation, IJCAS 2024 | IJCAS, DOI: 10.1007/s12555-022-0742-z | 搜索式前端、非线性平滑、B-spline、滚动时域 | 与 `WindowReplanner` 高度相关，可改进在线窗口轨迹质量 | **待落地** | `core/planning/replanner.py` 扩展 |
| Hoenig et al., Trajectory Planning for Quadrotor Swarms, T-RO 2018 | IEEE T-RO, DOI: 10.1109/TRO.2018.2853613 | 稀疏路网、离散调度、连续精修、考虑下洗影响 | 适合改进多机编队执行层，加入机间安全模型和下洗区 | **部分落地** | `core/artificial_potential_field.py`（编队势场已存在，下洗区待加） |
| Park et al., Online Distributed Trajectory Planning with Linear Safe Corridor, RA-L 2022 | IEEE RA-L, DOI: 10.1109/LRA.2022.3152702 | Linear Safe Corridor 将多机轨迹优化约束化 | 可作为 FIRI 走廊的多机扩展方向 | **待落地** | `core/planning/firi.py` 多机扩展 |
| Quan et al., Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments, T-RO 2023 | IEEE T-RO | 面向密集环境编队飞行，鲁棒、高效、队形保持 | 对实验室/仓库密集场景有直接参考意义 | **待评估** | 作为密集场景 benchmark 对照 |
| Romero et al., Model Predictive Contouring Control for Time-Optimal Quadrotor Flight, T-RO 2022 | IEEE T-RO, DOI: 10.1109/TRO.2022.3173711 | MPCC 将"沿轨迹进度"作为优化目标 | 不建议短期替换控制器，可做离线对比 | **暂缓** | `项目总改进规划.md` 第8节不建议短期实施 |
| Foehn et al., Time-optimal planning for quadrotor waypoint flight, Science Robotics 2021 | Science Robotics, DOI: 10.1126/scirobotics.abh1221 | 同时优化时间分配和轨迹本身 | 可作为 P1 之后的高阶研究方向 | **暂缓** | 不作为短期主线 |
| EGO-Planner / EGO-Swarm | RA-L/ICRA 系列 | ESDF-free 梯度优化、局部拓扑候选、多机去中心化规划 | 可借鉴"局部拓扑候选 + 快速轨迹优化" | **待评估** | 项目已有 ESDF，需评估替换收益 |

### 3.2 动力学建模与控制

| 文献 | 来源 | 关键思想 | 对本项目的可用价值 | 落地状态 | 对应文件/规划 |
|---|---|---|---|---|---|
| Lee, Leok & McClamroch, Geometric tracking control on SE(3), CDC 2010 | CDC/arXiv 2010 | 直接在 SE(3) 上设计控制律，避免 Euler 角奇异 | 可新增 `GeometricSE3Controller` 做对照 | **待落地** | 新增 `core/controller.py` 可选 profile |
| Faessler, Franchi & Scaramuzza, Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag, RA-L 2018 | IEEE RA-L, DOI: 10.1109/LRA.2017.2776353 | 带线性 rotor drag 的四旋翼仍可微分平坦 | 当前 `Drone` 有简化阻力项 `k_drag`，可升级为机体/转子阻力 profile | **部分落地** | `core/drone.py` 扩展 rotor drag profile |
| Tal & Karaman, Accurate Tracking of Aggressive Quadrotor Trajectories using INDI and Differential Flatness, TCST 2021 | IEEE TCST | 增量非线性动态逆 + 微分平坦 | 可作为高阶跟踪器对照，不建议短期替代 | **暂缓** | 保留为研究接口 |
| Data-Driven System Identification of Quadrotors Subject to Motor Delays, 2024 | arXiv:2404.07837 | 从飞行数据估计惯量、推力曲线、力矩系数和电机延迟 | 可作为 `DroneParams` 标定流程的研究接口 | **待落地** | 新增标定脚本入口 |
| A two-step method for system identification of low-cost quadrotor, Aerospace Science and Technology | Elsevier Aerospace Sci. Tech. | EKF/RTS 飞行路径重建 + 时域最大似然辨识 | 对低成本传感器与仿真参数校验有参考价值 | **待评估** | 参数校验参考 |
| Evaluation of drag coefficient for a quadrotor model, Int. J. Micro Air Vehicles | SAGE, DOI: 10.1177/17568293221148378 | 从风洞/飞行数据估计阻力系数 | 可支撑 `k_drag` 从固定常数变成可标定参数 | **待落地** | `core/drone_params.py` 标定接口 |

### 3.3 低空安全、感知和标准

| 文献/标准 | 来源 | 关键思想 | 对本项目的可用价值 | 落地状态 | 对应文件/规划 |
|---|---|---|---|---|---|
| T/CICC 27002-2025 低空运行安全风险评估与分级指南 | 用户提供 PDF | CONOPS、ORA、危险识别、剩余风险 | 可转成仿真实验风险模板和安全评估框架 | **待落地** | 新增 `core/risk_report.py` |
| T/CICC 27006-2025 低空飞行通信导航监视安全通用要求 | 用户提供 PDF | CNS、身份认证、导航源冗余、监视融合 | 可用于增加通信延迟、定位误差、监视周期等扰动 | **待落地** | `SafetyProfile` 扰动参数 |
| T/CICC 27007-2025 低空飞行安全间隔管控规范 | 用户提供 PDF | 安全间隔保护区、定位精度、监视周期、水平/垂直间隔 | 可做外场/低空尺度 `regulatory_profile` | **待落地** | 新增 `core/safety_profiles.py` |
| Butt et al., Review of perception sensors for low-altitude UAV obstacle avoidance, RAS 2024 | Elsevier RAS, DOI: 10.1016/j.robot.2024.104629 | 低空非合作障碍物避障分类 | 可指导传感器模型从六向距离升级为多源感知 | **待评估** | `core/sensors.py` 扩展参考 |

---

## 4. 当前项目差距分析

### 4.1 轨迹层差距

当前 `core/planning` 已有 A*、Hybrid A*、RRT*、D* Lite、ESDF、FIRI（`项目总改进规划.md` 第1节），但最终执行路径仍主要是离散点列和局部平滑。与 RA-L/T-RO 文献相比，缺少：

- 连续时间轨迹参数化（polynomial、B-spline 或 minimum snap）。
- 显式速度、加速度、jerk、snap、yaw/yaw-rate 约束。
- 时间分配优化和动态可行性检查。
- 多机之间的时空冲突约束和下洗保护区。

**已有基础**：`项目总改进规划.md` P1-1 已规划"FIRI 多段走廊轨迹优化"，当前 FIRI 可做分段精修，尚未形成完整多段时间化轨迹。

### 4.2 动力学层差距

当前 `core/drone.py` 默认参数偏演示用途；`core/rotor.py` 已实现 `BEMRotor`，但默认 `Drone` 仍接入简化 `Rotor`。与系统辨识和 rotor drag 文献相比，缺少：

- `DroneParams` 参数 profile（名称、来源、单位、适用场景）。
- 电机一阶延迟、推力曲线、力矩系数和阻力系数的标定流程。
- 模型 profile 与控制器 profile 的联动测试。

**已有基础**：`需求符合性检查.md` 已确认"BEM 与简化模型统一切换"部分满足，`BEMRotor` 已存在。

### 4.3 标准化安全层差距

PDF 标准中的安全间隔适用于低空管服系统和外场低空运行，尺度远大于室内仿真地图。例如 T/CICC 27007 中轻型无人机水平安全保持间隔为 30 m、垂直 10 m，而项目当前室内地图 `safety_margin=0.2-0.3 m`。缺少：

- `indoor_demo`、`indoor_research`、`low_altitude_regulatory` 三类 safety profile。
- 将定位误差、监视更新周期、处置时间、规避时间转化为仿真扰动。
- 安全事件与 ORA 风险评估表的对应关系。

---

## 5. 从文献到代码的映射与增量设计

### 5.1 建议新增数据结构

建议先增加轻量数据结构，不急于大重构。

```python
@dataclass
class TrajectorySample:
    t: float
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    yaw: float = 0.0
    yaw_rate: float = 0.0

@dataclass
class TrajectoryResult:
    samples: list[TrajectorySample]
    success: bool
    reason: str = ""
    cost_smooth: float = 0.0
    min_clearance: float = float("inf")
    max_vel: float = 0.0
    max_acc: float = 0.0
    solve_time_ms: float = 0.0

@dataclass
class DroneParams:
    name: str
    mass: float
    inertia: np.ndarray
    arm_length: float
    rotor_kf: float
    rotor_km: float
    motor_tau: float
    omega_max: float
    drag_linear: np.ndarray
    drag_quadratic: np.ndarray
    rotor_drag: np.ndarray  # 见 Faessler et al. RA-L 2018

    def __post_init__(self):
        # 一致性检查：参照 R5（多参数耦合），在初始化时固化约束
        assert self.max_total_thrust > self.mass * 9.81 * 1.3, "推力不足"
        assert self.hover_omega < self.omega_max, "悬停转速超限"
        assert np.all(np.linalg.eigvals(self.inertia) > 0), "惯量非正定"

    @property
    def max_total_thrust(self) -> float:
        return 4.0 * self.rotor_kf * self.omega_max ** 2

    @property
    def hover_omega(self) -> float:
        return np.sqrt(self.mass * 9.81 / (4.0 * self.rotor_kf))

@dataclass
class SafetyProfile:
    name: str
    horizontal_position_error: float
    vertical_position_error: float
    surveillance_update_period: float
    horizontal_hold_spacing: float
    vertical_hold_spacing: float
    decision_time: float
    horizontal_avoid_time: float
    vertical_avoid_time: float
```

**设计约束**：
- `TrajectoryResult` 可直接进入回放、benchmark 和风险报告，减少"代码跑了但不知道哪里变好"的问题。
- `DroneParams.__post_init__` 参照 `优化反效果归因分析.md` R5，将参数耦合的物理约束固化到数据结构层。

### 5.2 连续轨迹优化（对应 Mellinger & Kumar, Zhang et al.）

当前 FIRI 主要做分段精修（`项目总改进规划.md` P1-1）。后续增量建议：

**输入**：FIRI 走廊 `list[FIRICorridor]`、原始路径点 `np.ndarray`、最大速度/加速度、最小 clearance。

**输出**：`TrajectoryResult`，内部含时间化轨迹采样。

**第一版技术路线**：

```text
minimize  integral ||d4p/dt4||^2 dt   (minimum snap)
或
minimize  integral ||d3p/dt3||^2 dt   (minimum jerk)

约束：
p(0) = waypoint_start
p(T) = waypoint_goal
|v(t_k)| <= v_max
|a(t_k)| <= a_max
A_corridor p(t_k) <= b_corridor
signed_distance(p(t_k)) >= safety_margin
```

**实现要点**：
- 第一版使用 minimum jerk/snap polynomial 或 uniform B-spline，在采样点上检查走廊和 SDF 约束。
- 若 `cvxpy/osqp` 不可用，fallback 到当前 FIRI 点投影路径。
- 在线模式先限制为局部窗口内启用，避免全量求解超时。

**回退链路**（参照 `项目总改进规划.md` 回退策略风格）：

```text
FIRI corridor -> trajectory optimization success -> use optimized trajectory
FIRI corridor -> trajectory optimization fail -> use current FIRI refined path
FIRI refined path unsafe -> fallback segment planner
fallback unsafe -> reject path and log clearance_blocked
```

**与现有代码的对接**：
- `core/planning/firi.py`：需暴露 corridor 构建结果（当前可能只返回 refined path）。
- `core/planning/replanner.py`：`WindowReplanner` 输出路径后可选调用轨迹优化。
- `core/controller.py`：控制器输入需扩展为支持 `target_vel`、`target_acc` 前馈（见5.3节）。

**反效果预防**（嵌入 R1-R8）：
- **R6**：轨迹优化后必须在采样点上重新经过 SDF/胶囊体门禁，"航点安全≠轨迹安全"。
- **R3**：`target_acc` 前馈的量纲必须是 m/s²，禁止叠加到 `target_pos`。
- **R1**： corridor 约束不足时禁止用全局参数修补，应退回当前 FIRI 路径。

### 5.3 控制器接口扩展（对应 Lee et al. SE(3)）

先扩展当前控制器接口，新增 `target_vel`、`target_acc`、`target_yaw`、`target_yaw_rate` 前馈：

```python
compute_control(
    state,
    target_pos,
    target_vel=None,
    target_acc=None,
    target_yaw=0.0,
    target_yaw_rate=0.0,
)
```

**新增可选 `GeometricSE3Controller`**：

SE(3) 控制器直接在旋转矩阵群上计算姿态误差：

```text
e_R = 1/2 * vee(R_d^T R - R^T R_d)
e_omega = omega - R^T R_d omega_d
```

**接入方式**：作为实验 profile，不替换默认 PID/SMC/Backstepping：

```python
controller_kind: Literal["pid_smc", "backstepping_smc", "se3_geometric"] = "pid_smc"
```

**反效果预防**：
- **R3**：SE(3) 输出必须经量纲断言后注入力矩通道，禁止直接改写 `target_pos`。
- **R4**：SE(3) 默认关闭，仅在配置中显式启用，避免简单场景被高阶控制器干扰。
- **R6**：高曲率轨迹的跟踪安全需配套轨迹级防护，不能仅靠控制器闭环。

### 5.4 DroneParams 参数 profile（对应 Faessler et al., 系统辨识文献）

**三类 profile**：

| profile | 用途 | 特点 |
|---|---|---|
| `default_1kg` | 保持当前回归基线 | 不改变现有仿真表现 |
| `indoor_micro` | 室内微型机研究对照 | 质量/尺寸/推力接近 Crazyflie 类平台（~27 g） |
| `light_uav_regulatory` | 低空标准演示 | 速度上限参考 T/CICC 27007 轻型无人机 |

**加载路径**：

```text
config.py / web UI
    -> SimulationConfig.drone_profile
    -> DroneParams registry
    -> Drone / QuaternionDrone / Rotor / ControlAllocator
    -> Controller mass and inertia
```

**关键一致性检查**（`__post_init__` 自动执行）：

```text
max_total_thrust > mass * g * thrust_margin
hover_omega < omega_max
inertia positive definite
rotor_kf and allocator_kf consistent
rotor_km and allocator_km consistent
```

**反效果预防**：
- **R5**：参数耦合关系在 `DroneParams.__post_init__` 中固化，避免 `mass` 改了但 `hover_omega` 未联动。
- **R4**：默认 profile 保持 `default_1kg`，切换需显式配置，避免回归测试被破坏。

### 5.5 多机安全走廊与冲突消解（对应 Hoenig et al., Park et al.）

**三层策略**（第一版不建议直接做复杂分布式优化）：

1. **Leader 路径**：使用队形包络进行保守规划。
2. **Followers 目标点**：若不安全，则沿 leader 方向收缩。
3. **机间距离不足**：触发局部错层或队形压缩。

**编队包络计算**：

```text
formation_radius_lateral   = max(|offset_i.x|) + drone_radius + safety_margin
formation_radius_longitudinal = max(|offset_i.y|) + drone_radius + safety_margin
formation_radius_vertical  = max(|offset_i.z|) + drone_radius + safety_margin
```

> **注意**：参照 `优化反效果归因分析.md` **R7**，**禁用各向同性范数** `max(||offset||)`。线性队形纵向延伸大但横向包络小，使用分轴包络才能保留窄通道可通行性。

**窄通道队形收缩**：

```text
if corridor_width < 2 * formation_radius_lateral:
    switch formation profile to line / compact
```

**当前代码状态与修复建议**：

`core/topology.py` 中已有 `auto_shrink(channel_width, envelope)` 方法（第178行），但该方法**未被 `simulations/obstacle_scenario.py` 主流程调用**，当前队形切换仍依赖 `config.py` 中的定时 `formation_schedule`。

更关键的是，`FormationTopology.envelope_radius()` 当前实现为：

```python
max_dist = max(float(np.linalg.norm(off)) for off in offsets)
return max_dist + self.arm_length
```

这使用了各向同性范数 `max(||offset||)`，正是 `优化反效果归因分析.md` **R7** 批评的写法。该缺陷已导致实际故障：线性队形 `spacing=6 m` 时 `envelope_radius ≈ 18.2 m`，`simulations/obstacle_scenario.py::_inflate_r()` 将其叠加 `safety_margin` 后栅格膨胀至 **18.7 m**（约等于整个仓库横向尺寸），A* 直接失败。

**修复步骤**：
1. `core/topology.py` 新增 `envelope_per_axis(formation)` 返回 `(lateral, longitudinal, vertical)` 三轴包络。
2. 修改 `envelope_radius()` 标记为 **deprecated**，内部转发为 `max(envelope_per_axis())` 仅用于向后兼容。
3. `simulations/obstacle_scenario.py::_inflate_r()` 改为使用 `envelope_per_axis()` 的横向+垂直分量。
4. `core/topology.py` 的 `auto_shrink()` 改为接收 `channel_width: tuple[float, float, float]`（三轴通道宽度），分别与三轴包络比较。
5. 在 `simulations/obstacle_scenario.py` 的在线重规划循环中接入 `auto_shrink()`，在窄通道前主动触发队形收缩。

**下洗保护区**：

```text
vertical_downwash_zone = cylinder(radius=r_downwash, height=h_downwash)
```

**反效果预防**：
- **R1**：队形包络是局部队形属性，禁止用全局栅格膨胀实现（否则仓库窄通道会被全部填满）。
- **R7**：必须分轴计算包络，不能用 `max(||offset||)` 替代。
- **R6**：followers 安全不能只靠 leader 路径安全，需独立检查 follower 轨迹级 clearance。

### 5.6 标准化 Safety Profile 与风险报告（对应 T/CICC 27002/27006/27007）

#### 5.6.1 标准参数转化

T/CICC 27007 中可转化为仿真参数的内容：

| 标准项 | 可转化配置 | 室内场景取值 | 低空标准取值 |
|---|---|---|---|
| 目标更新周期不大于 1 s | `surveillance_update_period` | 0.05 s（仿真步长级） | 1.0 s |
| 水平定位精度不大于 10 m | `horizontal_position_error` | 0.02 m（理想定位） | 10.0 m |
| 垂直定位精度不大于 15 m | `vertical_position_error` | 0.02 m | 15.0 m |
| 轻型无人机水平安全保持间隔 30 m | `horizontal_hold_spacing` | 0.5 m（室内紧凑） | 30.0 m |
| 轻型无人机垂直安全保持间隔 10 m | `vertical_hold_spacing` | 0.3 m | 10.0 m |
| 处置时间 5 s | `decision_time` | 0.2 s | 5.0 s |
| 水平/垂直规避时间 | `horizontal_avoid_time` / `vertical_avoid_time` | 0.1 s | 3.0 s |

#### 5.6.2 安全间隔计算（外场/低空尺度）

纵向最小安全间隔：

```text
longitudinal_interval =
    horizontal_position_error
  + horizontal_hold_spacing
  + max_flight_speed * (
        surveillance_update_period
      + decision_time
      + horizontal_avoid_time
    )
```

横向和垂直间隔：

```text
lateral_interval  = 2 * horizontal_position_error + horizontal_hold_spacing
vertical_interval = 2 * vertical_position_error + vertical_hold_spacing
```

> **重要**：这些数值适合低空外场或标准化演示，**不适合直接套到当前几米到几十米尺度的室内地图**。室内场景仍使用 `indoor_demo` 或 `indoor_research` profile。

#### 5.6.3 风险报告结构

```json
{
  "safety_profile": "indoor_research",
  "risk_level": "low",
  "events": [
    {
      "type": "clearance_blocked",
      "t": 24.4,
      "frame": 62,
      "position": [6.2, 3.1, 2.5],
      "min_clearance": 0.18,
      "threshold": 0.30,
      "reason": "no_continuous_clearance_path"
    }
  ],
  "summary": {
    "collision_count": 0,
    "clearance_violation_count": 0,
    "inter_drone_violation_count": 0,
    "restricted_airspace_violation_count": 0
  }
}
```

**预期效果**：
- 室内 profile 不改变当前地图尺度。
- 低空标准 profile 能输出符合 T/CICC 27007 口径的安全间隔计算说明。
- 风险报告能用于答辩说明"项目如何对应 CONOPS、危险识别和剩余风险"。

---

## 6. 与现有改进规划的衔接

本文档建议的增量与 `项目总改进规划.md` 的 P0/P1/P2 对应关系：

| 本文档章节 | 增量内容 | 对接 项目总改进规划.md | 建议批次 |
|---|---|---|---|
| 5.1 数据结构 | `TrajectorySample`, `TrajectoryResult`, `DroneParams`, `SafetyProfile` | P1-1 前置工作 | 第一批 |
| 5.2 连续轨迹优化 | `trajectory_optimizer.py`, B-spline / minimum snap | **P1-1 FIRI 多段走廊轨迹优化** | 第一批 |
| 5.3 控制器扩展 | `compute_control` 接口扩展, SE(3) 实验 profile | P1-3 MPC 跟踪可行性评估之后的对照项 | 第二批 |
| 5.4 参数 profile | `core/drone_params.py`, 三类 profile | P1-2 APF profile 化之后的扩展 | 第一批 |
| 5.5 多机安全 | 队形包络分轴, 下洗区, 错层/收缩 | P2-2 动态多障碍分区策略的延伸 | 第二批 |
| 5.6 标准转化 | `core/safety_profiles.py`, `core/risk_report.py` | **新增维度** | 第一批 |

**注意**：`项目总改进规划.md` P0（GNN 闭环、辅助目标、语义层、终端保持）已基本完成，本文档不重复这些已完成项。

---

## 7. 反效果预防检查表

在评估本文档任何增量是否值得合入时，按以下顺序自查（引用 `docs/优化反效果归因分析.md` 第3节决策清单）：

| 检查项 | 对应根因 | 在本文档的应用 |
|---|---|---|
| **量纲检查** — 新加项的物理单位是否与注入点一致？ | R3 | 5.3节 `target_acc` 必须是 m/s²；5.5节下洗区参数必须有 m 注释 |
| **作用域检查** — 局部问题禁止用全局参数修 | R1 | 5.5节队形包络禁止用全局栅格膨胀实现 |
| **饱和与可跟踪性** — 修正量上限是否超过控制器可吸收能力？ | R3 | 5.2节轨迹优化速度/加速度约束必须 ≤ profile 上限 |
| **离散 vs 连续** — 安全性是航点级还是轨迹级？ | R6 | 5.2节优化后轨迹采样点必须全部经过 SDF clearance 门禁 |
| **跨场景副作用** — 默认开启的逻辑在简单场景下是否仍中性？ | R4 | 5.3节 SE(3) 默认关闭；5.4节默认 profile 为 `default_1kg` |
| **耦合参数** — 新参数是否与既有参数耦合？ | R5 | 5.4节 `DroneParams.__post_init__` 自动固化耦合 |
| **算法实现单测** — 关键不变量是否有冒烟测试？ | R8 | 新增 `tests/test_trajectory_optimizer.py`、`tests/test_drone_params.py` |
| **回退开关** — 是否提供 `enable_xxx` 配置能一键退回基线？ | 横切 | 5.2节 `trajectory_optimizer_enabled`；5.3节 `controller_kind` |

---

## 8. 效果矩阵

| 改进项 | 直接效果 | 间接效果 | 主要风险 | 反效果预防 |
|---|---|---|---|---|
| S1 连续轨迹优化 | 减少速度/加速度/jerk 突变 | 降低跟踪误差和在线路径切换抖动 | 优化器耗时、走廊过窄 | R1 局部问题局部修；R6 轨迹级门禁 |
| S2 参数 profile | 动力学参数可解释 | 便于实机/论文/标准对照 | profile 切换后需重新调参 | R5 耦合参数固化；R4 默认基线不变 |
| S3 SE(3)/前馈控制 | 高曲率轨迹跟踪更稳 | 提升控制理论完整性 | 替换默认控制器有风险 | R4 默认关闭；R3 量纲断言 |
| S4 多机安全走廊 | followers 和机间间隔更安全 | 更像真实集群飞行 | 在线计算复杂 | R7 分轴包络；R6 follower 独立检查 |
| S5 safety profile | 标准要求变成可运行配置 | 可生成风险报告 | 低空标准与室内尺度不一致 | 显式区分 indoor/low_altitude profile |

---

## 9. C++ 双线同步评估

| 增量项 | Python 优先级 | C++ 同步建议 | 理由 |
|---|---|---|---|
| `TrajectorySample` / `TrajectoryResult` | P0 | **即时同步** | C++ 动态回放已使用路径级 `path_is_clearance_safe()`，数据结构语义需保持一致 |
| `DroneParams` | P0 | **第二批同步** | C++ 当前使用 `struct DroneConfig`，可在 Python 验证后批量同步字段 |
| 轨迹优化器（minimum snap） | P0 | **暂缓** | C++ 当前无对应后端，收益待 Python 验证后评估 |
| SE(3) 控制器 | P1 | **暂缓** | 纯实验 profile，C++ 已有 PID/SMC 稳定基线 |
| SafetyProfile / risk_report | P0 | **第二批同步** | C++ 动态回放已有 `clearance_blocked` 字段，风险报告 JSON 可自然扩展 |
| 队形包络分轴 | P1 | **即时同步** | C++ `formation_simulation.cpp` 当前使用各向同性包络，存在 R7 风险 |

---

## 10. 验证命令

后续每次改动至少运行以下验证（与 `项目总改进规划.md` 第7节对齐）：

```powershell
# 已有回归
python -m pytest "tests\test_obstacle_scenario.py" -q -k "firi or obstacle_simulation_zero_collision or hybrid_astar_obstacle"
python -m pytest "tests\test_replanner_semantics.py" "tests\test_replanner_subgoal.py" -q
python -m pytest "tests\test_esdf_correct.py" -q
python -m pytest "tests\test_gnn_planner.py" -q
python -m pytest "tests\test_fault_tolerance_online.py" -q

# 本文档建议新增
python -m pytest "tests\test_trajectory_optimizer.py" -q
python -m pytest "tests\test_drone_params.py" -q
python -m pytest "tests\test_safety_profiles.py" -q
python -m pytest "tests\test_risk_report.py" -q
```

---

## 11. 风险与应对

| 风险 | 表现 | 应对 |
|---|---|---|
| 优化器依赖过重 | cvxpy/osqp 安装或运行耗时影响演示 | 第一版支持无优化器回退；默认仍能走当前 FIRI 路径 |
| 连续轨迹过度平滑 | 路径切角导致 clearance 降低 | **R6**：所有轨迹采样点必须重新经过 SDF/胶囊体门禁 |
| 控制器替换引入不稳定 | 高姿态角、推力饱和、跟踪发散 | SE(3)/MPC 先作为实验 profile，不替换默认控制器 |
| 标准参数尺度不匹配 | 低空间隔远大于室内地图 | **显式分离**：indoor profile 与 low_altitude_regulatory profile 互斥 |
| 多机走廊计算复杂 | 在线耗时上升 | 先做队形包络和事件触发，再做逐机 LSC |
| DroneParams 切换后控制发散 | 质量/惯量变化但 PID 增益未联动 | **R5**：profile 切换时自动提示"需重新整定控制器增益" |

---

## 12. 参考链接

- Mellinger & Kumar, Minimum snap trajectory generation and control for quadrotors: https://cir.nii.ac.jp/crid/1360292619897878272?lang=en
- Zhou et al., Robust and Efficient Quadrotor Trajectory Generation: https://researchportal.hkust.edu.hk/en/publications/robust-and-efficient-quadrotor-trajectory-generation-for-fast-aut/
- Zhang et al., Search-based Path Planning and Receding Horizon Based Trajectory Generation: https://link.springer.com/article/10.1007/s12555-022-0742-z
- Hoenig et al., Trajectory Planning for Quadrotor Swarms: https://cir.nii.ac.jp/crid/1360302870464140544
- Park et al., Online Distributed Trajectory Planning with Linear Safe Corridor: https://pure.seoultech.ac.kr/en/publications/online-distributed-trajectory-planning-for-quadrotor-swarm-with-f/
- Romero et al., Model Predictive Contouring Control: https://cir.nii.ac.jp/crid/1360306914243135232
- Foehn et al., Time-optimal planning for quadrotor waypoint flight: https://github.com/uzh-rpg/rpg_time_optimal
- Faessler et al., Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag: https://arxiv.org/abs/1712.02402
- Data-Driven System Identification of Quadrotors: https://arxiv.org/abs/2404.07837
- Low-altitude UAV perception review: https://www.sciencedirect.com/science/article/abs/pii/S0921889024000125
- CICC 低空安全体系标准公告: https://www.c2.org.cn/h-nd-2205.html
