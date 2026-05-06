# 四旋翼集群编队仿真系统后续 SCI 文献调研与改进计划书

> 调研日期：2026-05-05  
> 适用范围：`next_project` 当前 Python/C++ 双线四旋翼编队仿真系统  
> 调研口径：优先参考 SCI/SCIE 期刊、IEEE 机器人顶刊/期刊论文与近年高水平会议论文；标准依据结合 `PDF/TCICC27000-TCICC27008.pdf` 中的低空安全体系团体标准。

## 1. 计划书目标

本计划书用于回答“项目后续还能从动力学设计、路径规划、无人机模型和标准化安全约束方面如何继续提升”的问题。当前系统已经具备 A*、Hybrid A*、D* Lite、RRT*、ESDF、FIRI 风格凸走廊、Safe/Danger 调度、GNN 可见图变体、APF、容错拓扑和动态回放能力。下一阶段不建议推翻已有主线，而应在已验证的零碰撞与多场景可运行基础上，把系统从“能走通”推进到“轨迹更平滑、动态更可行、参数更可信、指标更可复现”。

核心目标分为四个层次：

1. 将离散路径点升级为满足速度、加速度、jerk/snap 和 clearance 约束的连续时间轨迹。
2. 将硬编码无人机模型参数升级为可切换、可标定、可对照论文和实机参数的 `DroneParams` profile。
3. 将多机编队安全从单机 obstacle clearance 扩展到队形包络、机间间隔、下洗区和动态冲突消解。
4. 将低空安全标准中的安全间隔、风险评估、通信导航监视要求转化为可选的 `safety_profile`，用于外场/标准化演示，不直接套用到室内小地图。

## 2. 已调研文献与可借鉴点

### 2.1 轨迹生成与路径规划

| 文献 | 来源 | 关键思想 | 对本项目的可用价值 |
|---|---|---|---|
| Mellinger & Kumar, Minimum snap trajectory generation and control for quadrotors, ICRA 2011 | IEEE ICRA, DOI: 10.1109/ICRA.2011.5980409 | 利用四旋翼微分平坦性，把航点序列转化为最小 snap 多项式轨迹 | 当前 A*/Hybrid A*/FIRI 输出可作为航点和走廊输入，后端求解 minimum snap/jerk 轨迹 |
| Zhou et al., Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight, RA-L 2019 | IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2019.2927938 | kinodynamic search + B-spline optimization + EDF/动态约束 + 时间调整 | 项目已有 ESDF/FIRI，可参考 B-spline 优化与动态可行性时间重分配 |
| Zhang et al., Search-based Path Planning and Receding Horizon Based Trajectory Generation for Quadrotor Motion Planning, IJCAS 2024 | International Journal of Control, Automation and Systems, DOI: 10.1007/s12555-022-0742-z | 搜索式前端、非线性平滑、B-spline、滚动时域和时间调整 | 与项目 `WindowReplanner` 高度相关，可用于改进在线窗口轨迹质量 |
| Hoenig et al., Trajectory Planning for Quadrotor Swarms, T-RO 2018 | IEEE Transactions on Robotics, DOI: 10.1109/TRO.2018.2853613 | 稀疏路网、离散调度、连续精修、考虑下洗影响 | 适合改进多机编队执行层，加入机间安全模型和下洗区 |
| Park et al., Online Distributed Trajectory Planning for Quadrotor Swarm with Feasibility Guarantee Using Linear Safe Corridor, RA-L 2022 | IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2022.3152702 | Linear Safe Corridor 将多机轨迹优化约束化，降低软约束失效风险 | 可作为 FIRI 走廊的多机扩展方向，减少狭窄通道中的死锁和互相让行失败 |
| Quan et al., Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments, T-RO 2023 | IEEE Transactions on Robotics | 面向密集环境编队飞行，强调鲁棒、高效、队形保持 | 对实验室、公司格子间、仓库这类障碍密集场景有直接参考意义 |
| Romero et al., Model Predictive Contouring Control for Time-Optimal Quadrotor Flight, T-RO 2022 | IEEE Transactions on Robotics, DOI: 10.1109/TRO.2022.3173711 | MPCC 将“沿轨迹进度”作为优化目标，适合时间最优飞行 | 不建议短期替换控制器，但可做离线对比或轻量 MPC 评估 |
| Foehn et al., Time-optimal planning for quadrotor waypoint flight, Science Robotics 2021 | Science Robotics, DOI: 10.1126/scirobotics.abh1221 | 同时优化时间分配和轨迹本身 | 可作为 P2 之后的高阶研究方向，不作为短期主线 |
| EGO-Planner / EGO-Swarm | RA-L/ICRA 系列 | ESDF-free 梯度优化、局部拓扑候选、多机去中心化规划 | 项目已有 ESDF 和 GNN 可见图变体，可借鉴“局部拓扑候选 + 快速轨迹优化” |

### 2.2 动力学建模与控制

| 文献 | 来源 | 关键思想 | 对本项目的可用价值 |
|---|---|---|---|
| Lee, Leok & McClamroch, Geometric tracking control on SE(3), CDC/arXiv 2010 | SE(3) 几何控制经典文献 | 直接在 SE(3) 上设计控制律，避免 Euler 角奇异和大姿态问题 | 项目已有 `QuaternionDrone`，可新增 `GeometricSE3Controller` 做对照 |
| Faessler, Franchi & Scaramuzza, Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag, RA-L 2018 | IEEE RA-L, DOI: 10.1109/LRA.2017.2776353 | 证明带线性 rotor drag 的四旋翼仍可微分平坦，并给出前馈控制和阻力系数识别 | 当前 `Drone` 有简化阻力项 `k_drag`，可升级为机体/转子阻力 profile |
| Tal & Karaman, Accurate Tracking of Aggressive Quadrotor Trajectories using INDI and Differential Flatness, TCST 2021 | IEEE Transactions on Control Systems Technology | 增量非线性动态逆 + 微分平坦，提高激烈轨迹跟踪精度 | 可作为高阶跟踪器对照，不建议短期替代稳定 PID/SMC 主线 |
| Data-Driven System Identification of Quadrotors Subject to Motor Delays, 2024 | arXiv, 面向近年系统辨识 | 从飞行数据估计惯量、推力曲线、力矩系数和一阶电机延迟 | 可作为 `DroneParams` 标定流程的研究接口 |
| A two-step method for system identification of low-cost quadrotor, Aerospace Science and Technology | Elsevier Aerospace Science and Technology | EKF/RTS 飞行路径重建 + 时域最大似然辨识 | 对低成本传感器与仿真参数校验有参考价值 |
| Evaluation of drag coefficient for a quadrotor model, International Journal of Micro Air Vehicles | SAGE, DOI: 10.1177/17568293221148378 | 从风洞/飞行数据估计四旋翼阻力系数 | 可支撑 `k_drag` 从固定常数变成可标定参数 |

### 2.3 低空安全、感知和标准

| 文献/标准 | 来源 | 关键思想 | 对本项目的可用价值 |
|---|---|---|---|
| T/CICC 27002-2025 低空运行安全风险评估与分级指南 | 用户提供 PDF | CONOPS、ORA、危险识别、剩余风险和持续风险监视 | 可转成仿真实验风险模板和答辩中的安全评估框架 |
| T/CICC 27006-2025 低空飞行通信导航监视安全通用要求 | 用户提供 PDF | CNS、身份认证、导航源冗余、监视融合 | 可用于后续增加通信延迟、定位误差、监视更新周期等扰动 |
| T/CICC 27007-2025 低空飞行安全间隔管控规范 | 用户提供 PDF | 安全间隔保护区、定位精度、监视更新周期、水平/垂直安全保持间隔 | 可做外场/低空尺度 `regulatory_profile`，不直接套用室内小比例地图 |
| Butt et al., Review of perception sensors for low-altitude UAV obstacle avoidance, Robotics and Autonomous Systems 2024 | Elsevier, DOI: 10.1016/j.robot.2024.104629 | 将低空非合作障碍物避障分为 gap-based、geometric、repulsive-force、AI 等方法 | 可指导后续传感器模型从六向距离升级为多源感知 |

## 3. 当前项目差距分析

### 3.1 轨迹层差距

当前 `core/planning` 已有 A*、Hybrid A*、RRT*、D* Lite、ESDF、FIRI，但最终执行路径仍主要是离散点列和局部平滑。与 RA-L/T-RO 文献相比，缺少：

- 连续时间轨迹参数化，例如 polynomial、B-spline 或 minimum snap 表示。
- 显式速度、加速度、jerk、snap、yaw/yaw-rate 约束。
- 时间分配优化和动态可行性检查。
- 多机之间的时空冲突约束和下洗保护区。

### 3.2 动力学层差距

当前 `core/drone.py` 默认质量、惯量、臂长和阻力参数偏演示用途；`core/rotor.py` 已实现 `BEMRotor`，但默认 `Drone` 仍接入简化 `Rotor`。与系统辨识和 rotor drag 文献相比，缺少：

- `DroneParams` 参数 profile。
- 参数来源标注和单位检查。
- 电机一阶延迟、推力曲线、力矩系数和阻力系数的标定流程。
- 模型 profile 与控制器 profile 的联动测试。

### 3.3 标准化安全层差距

PDF 标准中的安全间隔适用于低空管服系统和外场低空运行，尺度远大于室内仿真地图。例如 T/CICC 27007 中轻型无人机安全保持间隔为水平 30 m、垂直 10 m，定位精度水平不大于 10 m、垂直不大于 15 m，监视目标更新周期不大于 1 s。项目当前室内地图的 `safety_margin=0.2-0.3 m` 是实验室尺度，两者不能直接混用。缺少：

- `indoor_demo`、`indoor_research`、`low_altitude_regulatory` 三类 safety profile。
- 将定位误差、监视更新周期、处置时间、规避时间转化为仿真扰动。
- 安全事件与 ORA 风险评估表的对应关系。

## 4. 后续实施计划

### P0：文献与指标基线整理

目标：先把“是否变好”定义清楚，避免后续优化只看视觉效果。

涉及文件：

- `docs/技术文档.md`
- `docs/项目总改进规划.md`
- 新增 `docs/后续SCI文献调研与改进计划书.md`
- 新增或扩展 benchmark 脚本

任务：

1. 固定评估场景：`warehouse_online`、`school_corridor_online`、`company_cubicles_online`、`meeting_room_online`、`laboratory_online`。
2. 固定评估指标：成功率、碰撞数、最小 clearance、阻塞事件数、重规划次数、路径长度、平均/最大速度、平均/最大加速度、jerk 代价、leader tracking RMS、replan latency p50/p95。
3. 输出统一 CSV/JSON 结果，便于后续改动前后对比。

验收：

- 能一键跑出五个在线场景指标。
- 后续每项改进都能与 P0 基线对比。

### P1：FIRI 多段连续轨迹优化原型

目标：把当前 FIRI 分段精修升级为带时间参数的连续轨迹后端。

建议方案：

1. 新增 `core/planning/trajectory_optimizer.py`。
2. 输入：FIRI 走廊、原始路径点、最大速度、最大加速度、最小 clearance。
3. 输出：时间化轨迹 `TrajectorySample(t, pos, vel, acc, yaw)`。
4. 第一版使用 minimum jerk/snap polynomial 或 uniform B-spline，先做离线轨迹生成，不直接替换所有在线逻辑。
5. 失败回退：继续使用当前 FIRI 投影路径和路径级 clearance 门禁。

涉及文件：

- `core/planning/firi.py`
- `core/planning/trajectory_optimizer.py`
- `simulations/obstacle_scenario.py`
- `tests/test_trajectory_optimizer.py`

验收：

- 在 `laboratory`、`company_cubicles`、`school_corridor` 中轨迹仍零碰撞。
- leader tracking RMS 不高于当前基线。
- jerk/snap 代价相对当前重采样路径下降。
- 单次优化耗时可控，在线模式先限制为局部窗口内启用。

### P2：无人机动力学参数 profile 化

目标：把模型参数从“写死在构造函数里”变成可解释、可切换、可标定。

建议方案：

1. 新增 `core/drone_params.py`，定义 `DroneParams`：
   - `mass`
   - `inertia`
   - `arm_length`
   - `rotor_kf`
   - `rotor_km`
   - `motor_tau`
   - `omega_max`
   - `drag_linear`
   - `drag_quadratic`
   - `rotor_drag`
2. 提供 profile：
   - `indoor_micro`：参考 Crazyflie 类室内微型机，质量约 27 g 量级，仅用于小尺度对照。
   - `light_uav`：参考 T/CICC 27007 轻型无人机速度上限，用于低空标准演示。
   - `default_1kg`：保持当前项目行为，作为回归基线。
3. 将 `Drone` 和 `QuaternionDrone` 接受 `DroneParams`，默认仍用 `default_1kg`，避免破坏现有测试。

验收：

- 当前测试不因默认 profile 改变而失败。
- 切换 profile 后质量、最大推力、悬停转速有基本物理一致性检查。
- `Rotor` 与 `ControlAllocator` 参数来自同一 profile，避免力矩分配系数不一致。

### P3：轨迹跟踪控制器扩展评估

目标：在不破坏当前 PID/SMC/Backstepping 稳定性的前提下，加入更学术化的控制器对照。

建议方案：

1. 新增 `GeometricSE3Controller` 原型，先只支持单机轨迹跟踪。
2. 轨迹输入从 `target_pos` 扩展为 `target_pos/vel/acc/yaw/yaw_rate`。
3. 对比现有控制器在相同轨迹下的 tracking RMS、姿态峰值、控制输入饱和次数。
4. MPC/MPCC 只做离线可行性评估，不进入短期默认主线。

验收：

- 在无障碍和简单障碍场景中，SE(3) 控制器不发散。
- 对高曲率轨迹，SE(3) 或前馈项能降低 tracking RMS。
- 若收益不明显，保留为研究接口，不替换默认控制器。

### P4：多机安全走廊与冲突消解

目标：解决编队在窄通道、实验室和仓库密集障碍中可能出现的队形互相挤压、路径交叉和下洗风险。

建议方案：

1. 在 `FormationTopology` 基础上计算队形包络半径和成员相对位置 envelope。
2. 参考 Linear Safe Corridor，给每架无人机生成局部走廊或共享队形走廊。
3. 加入机间最小距离约束和垂直下洗保护区。
4. 对动态冲突使用事件触发重规划，不在每帧全量求解。

涉及文件：

- `core/topology.py`
- `core/planning/firi.py`
- `core/planning/replanner.py`
- `simulations/obstacle_scenario.py`

验收：

- 多机最小间距不低于配置阈值。
- 窄通道场景中不出现互相阻塞导致的无意义重规划刷屏。
- 下洗保护区启用时，可观察到垂直错层或队形收缩策略。

### P5：标准化安全 profile 与风险报告

目标：把 PDF 标准中的低空安全要求转化为仿真可选项，而不是只停留在文字引用。

建议方案：

1. 新增 `core/safety_profiles.py`：
   - `indoor_demo`
   - `indoor_research`
   - `low_altitude_light_uav`
   - `low_altitude_small_uav`
2. 将 T/CICC 27007 的定位精度、监视更新周期、安全保持间隔、处置时间、规避时间转为参数。
3. 在仿真输出中增加 `risk_report`：
   - 是否碰撞
   - 是否违反 clearance
   - 是否违反机间间隔
   - 是否违反限制空域/高度边界
   - 剩余风险等级
4. Web 回放中保留“当前帧事件”和“全局风险事件”两类显示。

验收：

- 室内 profile 不改变当前地图尺度。
- 低空标准 profile 能输出符合 T/CICC 27007 口径的安全间隔计算说明。
- 风险报告能用于答辩说明“项目如何对应 CONOPS、危险识别和剩余风险”。

### P6：动态回放和实验报告增强

目标：让改进结果可视化、可复盘、可答辩。

任务：

1. 动态回放加入轨迹平滑度、速度/加速度曲线和风险事件跳转。
2. 事件卡片增加“类型、发生帧、位置、最近障碍、最小 clearance”。
3. 输出 `outputs/reports/*.md`，自动汇总场景指标和改进前后对比表。

验收：

- 点击任一碰撞/阻塞/重规划卡片能跳到对应帧。
- 每次实验能导出一份包含指标、图像路径、关键事件的报告。

## 5. 技术化实施细则

本节把上面的 P0-P6 拆成更接近技术文档的工程设计。整体原则是：每个新能力都先作为可选 profile 接入，并保留当前稳定路径作为 fallback。这样既能开展论文方法复现与对比，又不会破坏当前已经能运行的仓库、走廊、会议室、格子间和实验室场景。

### 5.1 总体数据流

后续系统建议形成如下数据流：

```text
Map / Obstacles / Dynamic Events
        |
        v
Front-end Planner
A* / Hybrid A* / D* Lite / RRT* / GNN danger
        |
        v
Path Acceptance Gate
SDF clearance / capsule clearance / waypoint semantics
        |
        v
Corridor Builder
FIRI / Linear Safe Corridor / formation envelope
        |
        v
Trajectory Optimizer
minimum snap / minimum jerk / B-spline / time allocation
        |
        v
Trajectory Tracker
PID / Backstepping / SMC / optional SE(3)
        |
        v
Simulation, Replay, Risk Report
metrics / event cards / frame jump / benchmark CSV
```

这条链路的效果是把“路径能不能穿过障碍”与“轨迹能不能被四旋翼跟踪”分开处理。前端规划器负责找拓扑可行路线，后端轨迹优化负责让路线变成动力学可行曲线，最后由统一安全门禁和回放报告负责证明没有碰撞、没有堵塞、没有机间冲突。

### 5.2 建议新增数据结构

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

这些结构的效果是让后续模块之间传递明确对象，而不是到处传递散落的 `dict`、`np.ndarray` 和配置字段。尤其是 `TrajectoryResult` 可以直接进入回放、benchmark 和风险报告，减少“代码跑了但不知道哪里变好”的问题。

### 5.3 P0：指标基线与实验框架

#### 5.3.1 技术目标

建立统一 benchmark，记录每个场景的规划质量、执行质量、风险事件和运行耗时。P0 不改变算法行为，只建立评估基线。

#### 5.3.2 建议指标定义

| 指标 | 定义 | 作用 |
|---|---|---|
| `success_rate` | 多次运行中完成全部任务航点的比例 | 衡量任务完成能力 |
| `collision_count` | 碰撞日志数量 | 必须保持为 0 |
| `clearance_blocked_count` | 连续 clearance 门禁失败次数 | 衡量规划不可接受事件 |
| `min_clearance` | 执行轨迹到障碍物的最小 signed distance | 衡量安全裕度 |
| `path_length` | leader 执行轨迹弧长 | 衡量绕路程度 |
| `tracking_rms` | leader 实际位置与目标轨迹的均方根误差 | 衡量控制跟踪质量 |
| `max_vel` | 轨迹最大速度 | 检查是否超过 profile 上限 |
| `max_acc` | 轨迹最大加速度 | 检查动力学可行性 |
| `jerk_cost` | `sum(||a[k+1]-a[k]||^2 / dt)` | 衡量轨迹突变 |
| `replan_count` | 在线重规划次数 | 衡量规划稳定性 |
| `replan_latency_p95` | 重规划耗时 95 分位 | 衡量实时性 |
| `risk_event_count` | 碰撞、堵塞、间隔违反、限制区违反等事件总数 | 衡量安全事件规模 |

#### 5.3.3 输出格式

建议 benchmark 每次输出：

```text
outputs/benchmarks/YYYYMMDD_HHMM/
  summary.csv
  summary.json
  scenario_warehouse_online.json
  scenario_laboratory_online.json
  replay_events.json
  figures/
    clearance_curve.png
    tracking_error.png
    speed_acc_profile.png
```

#### 5.3.4 预期效果

P0 的直接效果不是提升飞行表现，而是建立“改进是否真实有效”的量化依据。之后 P1-P6 每一项都可以回答三件事：是否减少碰撞或堵塞，是否降低跟踪误差，是否让仿真明显变慢。

### 5.4 P1：FIRI 多段连续轨迹优化

#### 5.4.1 当前问题

当前规划结果更像一串空间点，虽然 FIRI 可以把点推到安全走廊内，但点与点之间的速度、加速度、jerk 和 snap 没有统一优化。在线重规划发生时，leader 可能突然切换局部路径，导致跟踪误差、速度突变或短时间停顿。

#### 5.4.2 技术方案

第一阶段建议使用 minimum jerk 或 minimum snap polynomial。对每段轨迹设多项式：

```text
p_i(t) = c0 + c1 t + c2 t^2 + ... + cn t^n
```

优化目标：

```text
minimize  integral ||d4p/dt4||^2 dt      minimum snap
or
minimize  integral ||d3p/dt3||^2 dt      minimum jerk
```

约束：

```text
p(0) = waypoint_start
p(T) = waypoint_goal
|v(t_k)| <= v_max
|a(t_k)| <= a_max
A_corridor p(t_k) <= b_corridor
signed_distance(p(t_k)) >= safety_margin
```

第一版可以只在采样点上检查走廊和 SDF 约束，不强行引入复杂非线性求解器。若 `cvxpy/osqp` 不可用，则 fallback 到当前 FIRI 点投影路径。

#### 5.4.3 模块落点

| 模块 | 改动 |
|---|---|
| `core/planning/trajectory_optimizer.py` | 新增轨迹优化器、时间分配、采样输出 |
| `core/planning/firi.py` | 暴露 corridor 构建结果，不只返回 refined path |
| `simulations/obstacle_scenario.py` | 在线/离线路径接收后可选调用轨迹优化 |
| `core/controller.py` | 控制器输入支持 target velocity 和 target acceleration |
| `tests/test_trajectory_optimizer.py` | 覆盖轨迹连续性、速度/加速度上限、clearance |

#### 5.4.4 回退策略

```text
FIRI corridor -> trajectory optimization success -> use optimized trajectory
FIRI corridor -> trajectory optimization fail -> use current FIRI refined path
FIRI refined path unsafe -> fallback segment planner
fallback unsafe -> reject path and log clearance_blocked
```

#### 5.4.5 预期效果

- 轨迹更平滑，速度和加速度突变减少。
- 在线重规划后的 leader 瞬态误差降低。
- 实验室和格子间场景中由于路径切换造成的“看起来停住/抖动”会减少。
- 控制器输入更接近四旋翼可跟踪曲线，不再只追逐离散航点。
- 动态回放中速度、加速度曲线会更连续，便于展示算法质量。

目标量化效果：

| 指标 | 目标 |
|---|---|
| `collision_count` | 保持 0 |
| `jerk_cost` | 相比基线下降 20%-40% |
| `tracking_rms` | 相比基线下降 10%-25% |
| `clearance_blocked_count` | 不高于基线 |
| `replan_latency_p95` | 在线模式不超过基线的 1.5 倍 |

### 5.5 P2：无人机动力学参数 profile 化

#### 5.5.1 当前问题

当前 `Drone` 默认质量、惯量、臂长、阻力和转子参数偏演示用途。若后续引用论文、标准或实机参数，需要能说明“这组参数从哪里来、代表什么机型、适用于什么场景”。否则项目容易被质疑为“控制器只是适配了一组随意参数”。

#### 5.5.2 技术方案

新增 `core/drone_params.py`，定义三类 profile：

| profile | 用途 | 特点 |
|---|---|---|
| `default_1kg` | 保持当前回归基线 | 不改变现有仿真表现 |
| `indoor_micro` | 室内微型机研究对照 | 质量、尺寸、推力更接近 Crazyflie 类平台 |
| `light_uav_regulatory` | 低空标准演示 | 速度上限参考 T/CICC 27007 轻型无人机 |

参数加载路径：

```text
config.py / web UI
    -> SimulationConfig.drone_profile
    -> DroneParams registry
    -> Drone / QuaternionDrone / Rotor / ControlAllocator
    -> Controller mass and inertia
```

关键一致性检查：

```text
max_total_thrust > mass * g * thrust_margin
hover_omega < omega_max
inertia positive definite
rotor_kf and allocator_kf consistent
rotor_km and allocator_km consistent
```

#### 5.5.3 预期效果

- 模型参数可解释，答辩时能说明使用的是默认仿真机、室内微型机还是低空轻型机。
- 控制器和动力学模型使用同一质量/惯量，避免参数不一致。
- 后续做系统辨识、风阻估计、转子故障和电机延迟时有统一入口。
- 可以做“同一算法在不同机型 profile 下的鲁棒性对比”。

目标量化效果：

| 指标 | 目标 |
|---|---|
| 默认 profile 回归 | 与当前 baseline 接近 |
| 悬停测试 | 位置误差不发散 |
| profile 切换 | 不需要修改算法源码 |
| 参数一致性 | 自动检查通过 |

### 5.6 P3：轨迹跟踪控制器扩展

#### 5.6.1 当前问题

当前控制器已经能支撑仿真，但更偏工程 PID/Backstepping/SMC。若后续引入连续轨迹，控制器最好能利用 `pos/vel/acc/yaw/yaw_rate` 前馈，而不是只追目标位置。

#### 5.6.2 技术方案

先扩展当前控制器接口：

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

再新增可选 `GeometricSE3Controller`。SE(3) 控制器不使用 Euler 角误差作为核心状态，而是直接在旋转矩阵群上计算姿态误差：

```text
e_R = 1/2 * vee(R_d^T R - R^T R_d)
e_omega = omega - R^T R_d omega_d
```

第一版只作为实验 profile：

```text
controller_kind = "pid_smc" | "backstepping_smc" | "se3_geometric"
```

#### 5.6.3 预期效果

- 对高曲率、连续轨迹，控制器能使用速度/加速度前馈，减少滞后。
- `QuaternionDrone` 的价值更明显，姿态大角度时比 Euler 表示更稳健。
- SE(3) 控制器能作为论文答辩中的“高阶控制对照”，但不会冒险替换默认稳定控制器。

目标量化效果：

| 指标 | 目标 |
|---|---|
| `tracking_rms` | 在高曲率轨迹中下降 |
| 姿态峰值 | 不超过安全 tilt 限制 |
| 推力饱和次数 | 不高于当前控制器 |
| 默认场景稳定性 | 不发散、不明显变慢 |

### 5.7 P4：多机安全走廊与冲突消解

#### 5.7.1 当前问题

当前规划更关注 leader 与障碍物的关系，followers 通过队形目标和安全修正跟随。密集障碍场景下，队形整体包络、成员间距、路径交叉和下洗影响还可以进一步建模。

#### 5.7.2 技术方案

先计算编队包络：

```text
formation_radius = max(||offset_i||) + drone_radius + safety_margin
```

leader 规划时可以使用队形包络膨胀障碍物，或者在窄通道中触发队形收缩：

```text
if corridor_width < 2 * formation_radius:
    switch formation profile to line / compact
```

多机冲突约束：

```text
||p_i(t_k) - p_j(t_k)|| >= inter_drone_min_distance
vertical_downwash_zone = cylinder(radius=r_downwash, height=h_downwash)
```

第一版不建议直接做复杂分布式优化，而是采用三层策略：

1. leader 路径使用队形包络进行保守规划。
2. followers 目标点若不安全，则沿 leader 方向收缩。
3. 若机间距离不足，则触发局部错层或队形压缩。

#### 5.7.3 预期效果

- followers 更少贴障碍，不再只保证 leader 路径安全。
- 窄通道中队形更容易自动收缩，减少“leader 能过、编队过不去”的问题。
- 多机之间最小距离更稳定，便于展示集群安全性。
- 下洗保护区让模型更接近真实多旋翼编队，而不是点质量编队。

目标量化效果：

| 指标 | 目标 |
|---|---|
| `min_inter_drone_distance` | 不低于配置阈值 |
| follower collision | 保持 0 |
| 队形误差峰值 | 不因收缩策略显著增大 |
| 窄通道完成率 | 高于当前基线 |

### 5.8 P5：标准化 safety profile 与风险报告

#### 5.8.1 当前问题

PDF 中 T/CICC 27002、27006、27007 提供了风险评估、通信导航监视、安全间隔等要求，但当前项目更多是室内仿真口径。需要把标准变成可选 profile，避免只在文档里引用。

#### 5.8.2 标准参数转化

T/CICC 27007 中可转化为仿真参数的内容包括：

| 标准项 | 可转化配置 |
|---|---|
| 目标更新周期不大于 1 s | `surveillance_update_period` |
| 水平定位精度不大于 10 m | `horizontal_position_error` |
| 垂直定位精度不大于 15 m | `vertical_position_error` |
| 轻型无人机水平安全保持间隔 30 m | `horizontal_hold_spacing` |
| 轻型无人机垂直安全保持间隔 10 m | `vertical_hold_spacing` |
| 处置时间 5 s，未来可缩至 3 s | `decision_time` |
| 水平/垂直规避时间 | `horizontal_avoid_time` / `vertical_avoid_time` |

无人机与障碍物纵向最小安全间隔可按标准思路抽象为：

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

横向和垂直间隔可按定位误差与安全保持间隔计算：

```text
lateral_interval = 2 * horizontal_position_error + horizontal_hold_spacing
vertical_interval = 2 * vertical_position_error + vertical_hold_spacing
```

注意：这些数值适合低空外场或标准化演示，不适合直接套到当前几米到几十米尺度的室内地图。室内场景仍使用 `indoor_demo` 或 `indoor_research` profile。

#### 5.8.3 风险报告结构

建议仿真输出：

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

#### 5.8.4 预期效果

- 项目可以明确对应 CONOPS、危险识别、剩余风险和持续监视。
- 室内仿真和低空标准演示的尺度被区分，避免参数误用。
- Web 回放中的碰撞、堵塞、间隔违反事件可以进入统一风险报告。
- 答辩时能说明“标准不是简单引用，而是被转成可运行配置和风险指标”。

### 5.9 P6：动态回放与实验报告增强

#### 5.9.1 当前问题

动态回放已经能显示事件，但后续计划中的轨迹优化、风险 profile 和多机间隔都需要更完整的证据链。

#### 5.9.2 技术方案

Web 回放建议增加三类视图：

| 视图 | 内容 |
|---|---|
| 事件视图 | 当前帧事件、全局风险事件、点击跳帧 |
| 轨迹质量视图 | 速度、加速度、jerk、tracking error 曲线 |
| 安全报告视图 | min clearance、机间最小距离、风险等级 |

报告生成建议：

```text
simulation result
    -> metrics extractor
    -> markdown report
    -> figures and json attachments
```

输出：

```text
outputs/reports/laboratory_online_YYYYMMDD_HHMM.md
outputs/reports/laboratory_online_YYYYMMDD_HHMM.json
```

#### 5.9.3 预期效果

- 出现“卡住、堵塞、碰撞、重规划刷屏”时能快速定位对应帧。
- 后续论文方法改进能用曲线和指标展示，而不是只凭肉眼看动画。
- 报告可以直接进入答辩材料和项目验收材料。

## 6. 效果矩阵

| 改进项 | 直接效果 | 间接效果 | 主要风险 |
|---|---|---|---|
| P0 指标基线 | 所有改动可量化对比 | 避免优化反效果难定位 | 初期整理耗时 |
| P1 连续轨迹优化 | 减少速度/加速度/jerk 突变 | 降低跟踪误差和在线路径切换抖动 | 优化器耗时、走廊过窄 |
| P2 参数 profile | 动力学参数可解释 | 便于实机/论文/标准对照 | profile 切换后需要重新调参 |
| P3 SE(3)/前馈控制 | 高曲率轨迹跟踪更稳 | 提升控制理论完整性 | 替换默认控制器有风险 |
| P4 多机安全走廊 | followers 和机间间隔更安全 | 更像真实集群飞行 | 在线计算复杂 |
| P5 safety profile | 标准要求变成可运行配置 | 可生成风险报告 | 低空标准与室内尺度不一致 |
| P6 回放报告 | 事件可追溯、结果可复盘 | 答辩材料更完整 | 前端展示复杂度上升 |

## 7. 建议验证命令

后续每次改动至少运行以下验证。若某些测试暂不存在，应先补测试再合入功能。

```powershell
python -m pytest "tests\test_obstacle_scenario.py" -q -k "firi or obstacle_simulation_zero_collision or hybrid_astar_obstacle"
python -m pytest "tests\test_replanner_semantics.py" "tests\test_replanner_subgoal.py" -q
python -m pytest "tests\test_esdf_correct.py" -q
python -m pytest "tests\test_gnn_planner.py" -q
python -m pytest "tests\test_fault_tolerance_online.py" -q
```

新增功能后建议补充：

```powershell
python -m pytest "tests\test_trajectory_optimizer.py" -q
python -m pytest "tests\test_drone_params.py" -q
python -m pytest "tests\test_safety_profiles.py" -q
python -m pytest "tests\test_risk_report.py" -q
```

## 8. 分阶段里程碑

| 阶段 | 预计工作 | 产物 | 验收重点 |
|---|---|---|---|
| M1 | P0 benchmark | `summary.csv/json`、基线报告 | 五个在线场景可重复评估 |
| M2 | P1 轨迹优化原型 | `trajectory_optimizer.py`、轨迹测试 | jerk 降低且零碰撞 |
| M3 | P2 参数 profile | `drone_params.py`、参数一致性测试 | 默认行为不破坏 |
| M4 | P5 safety profile | `safety_profiles.py`、风险报告 | 标准参数可切换 |
| M5 | P3 控制器对照 | `GeometricSE3Controller` | 高曲率轨迹跟踪不发散 |
| M6 | P4 多机安全 | 队形包络/机间约束 | followers 更安全 |
| M7 | P6 报告增强 | 自动 Markdown 报告 | 事件、指标、图表闭环 |

## 9. 实施顺序建议

短期优先顺序：

1. P0 指标基线。
2. P1 FIRI 多段连续轨迹优化。
3. P2 动力学参数 profile。
4. P5 标准化 safety profile。

中期推进：

1. P3 SE(3) 控制器对照。
2. P4 多机安全走廊与冲突消解。
3. P6 自动实验报告。

暂缓事项：

- 不建议立即把默认控制器换成 MPC/MPCC。
- 不建议立即引入强化学习作为主线规划器。
- 不建议把 T/CICC 低空安全间隔原样套入室内地图。
- 不建议一次性合并所有论文方法，避免无法定位回归来源。

## 10. 风险与应对

| 风险 | 表现 | 应对 |
|---|---|---|
| 优化器依赖过重 | cvxpy/osqp 安装或运行耗时影响演示 | 第一版支持无优化器回退；默认仍能走当前 FIRI 路径 |
| 连续轨迹过度平滑 | 路径切角导致 clearance 降低 | 所有轨迹采样点必须重新经过 SDF/胶囊体门禁 |
| 控制器替换引入不稳定 | 高姿态角、推力饱和、跟踪发散 | SE(3)/MPC 先作为实验 profile，不替换默认控制器 |
| 标准参数尺度不匹配 | 低空间隔远大于室内地图 | 将标准 profile 与 indoor profile 分离 |
| 多机走廊计算复杂 | 在线耗时上升 | 先做队形包络和事件触发，再做逐机 LSC |

## 11. 预期成果

完成以上计划后，项目可以从答辩层面形成更强闭环：

- 理论层：能对应 minimum snap、B-spline、safe corridor、SE(3)、rotor drag、system identification、低空安全标准。
- 工程层：能说明哪些论文思想已经落地，哪些作为可选 profile，哪些暂缓。
- 实验层：能用统一指标证明改进是否提升，而不是只靠动态图观察。
- 安全层：能把碰撞、堵塞、机间间隔、低空标准风险放进同一报告体系。

## 12. 参考链接

- Mellinger & Kumar, Minimum snap trajectory generation and control for quadrotors: https://cir.nii.ac.jp/crid/1360292619897878272?lang=en
- Zhou et al., Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight: https://researchportal.hkust.edu.hk/en/publications/robust-and-efficient-quadrotor-trajectory-generation-for-fast-aut/
- Zhang et al., Search-based Path Planning and Receding Horizon Based Trajectory Generation: https://link.springer.com/article/10.1007/s12555-022-0742-z
- Hoenig et al., Trajectory Planning for Quadrotor Swarms: https://cir.nii.ac.jp/crid/1360302870464140544
- Park et al., Online Distributed Trajectory Planning with Linear Safe Corridor: https://pure.seoultech.ac.kr/en/publications/online-distributed-trajectory-planning-for-quadrotor-swarm-with-f/
- Romero et al., Model Predictive Contouring Control for Time-Optimal Quadrotor Flight: https://cir.nii.ac.jp/crid/1360306914243135232
- Foehn et al., Time-optimal planning for quadrotor waypoint flight: https://github.com/uzh-rpg/rpg_time_optimal
- Faessler et al., Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag: https://arxiv.org/abs/1712.02402
- Data-Driven System Identification of Quadrotors Subject to Motor Delays: https://arxiv.org/abs/2404.07837
- Low-altitude UAV perception review: https://www.sciencedirect.com/science/article/abs/pii/S0921889024000125
- CICC 低空安全体系标准公告: https://www.c2.org.cn/h-nd-2205.html


代码计划书

一、总体目标

基于 后续SCI文献调研与改进计划书.md，后续代码不再优先新增零散规划算法，而是把当前系统从“能仿真、能避障”升级为“可复现实验、可论文对比、可安全报告”的研究平台。

核心代码目标：

建立统一 benchmark 基线。
引入连续时间轨迹优化。
建立无人机动力学参数 profile。
扩展轨迹跟踪控制器接口。
增强多机安全约束。
建立 safety profile 与风险报告。
增强动态回放与实验报告。
二、当前代码基础

远程仓库已有基础：

next_project/core/planning/：已有 A*、Hybrid A*、D* Lite、RRT*、Informed RRT*、ESDF、FIRI 相关能力。
next_project/simulations/obstacle_scenario.py：已有障碍物、在线重规划、APF、GNN/Danger 调度、容错检测等集成逻辑。
next_project/simulations/formation_simulation.py：已有 SimulationConfig，但配置项已经较多，后续应做分组和 profile 化。
next_project/simulations/benchmark.py：已有基础评测，但还不够支撑 SCI 级别的多场景、多指标、消融实验。
三、阶段 P0：SCI 指标基线与实验框架

新增：

next_project/experiments/
├── scenario_registry.py
├── metrics_extractor.py
├── run_sci_baseline.py
├── run_ablation.py
└── report_writer.py
主要实现：

scenario_registry.py：统一注册 warehouse_online、school_corridor_online、company_cubicles_online、meeting_room_online、laboratory_online。
metrics_extractor.py：从仿真结果中提取成功率、碰撞数、最小 clearance、重规划次数、路径长度、速度、加速度、jerk、tracking RMS、replan latency。
run_sci_baseline.py：一键跑所有标准场景，输出 CSV/JSON。
run_ablation.py：支持关闭 ESDF、FIRI、APF、GNN、formation envelope 等模块，做消融实验。
report_writer.py：把实验结果导出为 Markdown 报告。
输出：

outputs/sci_baseline/summary.csv
outputs/sci_baseline/summary.json
outputs/reports/*.md
四、阶段 P1：连续轨迹优化模块

新增：

next_project/core/planning/trajectory_optimizer.py
next_project/core/planning/trajectory_types.py
tests/test_trajectory_optimizer.py
核心接口：

@dataclass
class TrajectorySample:
    t: float
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    yaw: float = 0.0

class TrajectoryOptimizer:
    def optimize(self, path, corridor=None, max_vel=2.0, max_acc=2.0):
        ...
第一版建议采用 minimum jerk / minimum snap 多项式或 B-spline，不立刻引入重优化依赖。输入为当前 A*/FIRI 输出路径，输出为带时间参数的连续轨迹。

接入点：

ObstacleScenarioSimulation._plan_offline()
WindowReplanner 输出路径后
Controller.compute_control(... target_vel, target_acc ...)
验收指标：

零碰撞不退化。
jerk 代价下降。
leader tracking RMS 不高于当前基线。
轨迹采样点全部通过 SDF clearance 检查。
五、阶段 P2：DroneParams 参数 profile

新增：

next_project/core/drone_params.py
tests/test_drone_params.py
核心设计：

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
    rotor_drag: np.ndarray
提供 profile：

default_1kg
indoor_micro
light_uav
修改：

core/drone.py 接收 DroneParams
core/rotor.py 从 profile 读取推力/力矩参数
core/allocator.py 与 profile 保持一致
SimulationConfig 增加 drone_profile: str = "default_1kg"
验收：

默认 profile 行为尽量保持不变。
hover_omega < omega_max
max_total_thrust > mass * g * margin
惯量矩阵正定。
rotor 与 allocator 参数一致。
六、阶段 P3：轨迹跟踪控制器扩展

修改：

next_project/core/controller.py
统一控制器接口：

compute_control(
    state,
    target_pos,
    target_vel=None,
    target_acc=None,
    target_yaw=0.0,
    target_yaw_rate=0.0,
)
新增可选控制器：

GeometricSE3Controller
建议先作为实验 profile，不替换默认 PID/SMC/Backstepping。

配置：

controller_kind: str = "pid_smc"
# "pid_smc" | "backstepping_smc" | "se3_geometric"
验收：

无障碍和简单障碍场景不发散。
高曲率连续轨迹 tracking RMS 降低。
推力饱和次数不显著增加。
七、阶段 P4：多机安全走廊与冲突消解

新增或扩展：

next_project/core/formation_safety.py
tests/test_formation_safety.py
实现内容：

formation_radius
min_inter_drone_distance
downwash_zone
follower 目标点安全修正
窄通道自动触发 compact/line 队形
核心逻辑：

if corridor_width < 2 * formation_radius:
    switch_to_compact_formation()
指标：

min_inter_drone_distance
inter_drone_violation_count
follower_collision_count
downwash_violation_count
八、阶段 P5：Safety Profile 与风险报告

新增：

next_project/core/safety_profiles.py
next_project/core/risk_report.py
tests/test_safety_profiles.py
tests/test_risk_report.py
profile：

indoor_demo
indoor_research
low_altitude_regulatory
输出示例：

{
  "safety_profile": "indoor_research",
  "collision_count": 0,
  "clearance_violation_count": 0,
  "inter_drone_violation_count": 0,
  "risk_level": "low"
}
注意：低空标准参数不能直接套进室内小地图，应通过 profile 区分尺度。

九、阶段 P6：报告与回放增强

扩展：

next_project/simulations/visualization.py
next_project/visualization/
next_project/outputs/reports/
新增图表：

clearance-time 曲线
speed / acceleration / jerk 曲线
replan latency 分布
inter-drone distance 曲线
collision / blocked / risk event timeline
最终输出：

outputs/reports/{scenario}_{timestamp}.md
outputs/reports/{scenario}_{timestamp}.json
outputs/figures/*.png
十、优先级建议

短期优先：

P0 实验基线与指标提取。
P1 连续轨迹优化原型。
P2 DroneParams profile。
P5 safety profile 与风险报告。
中期推进：

P3 SE(3) 控制器对照。
P4 多机安全走廊。
P6 自动报告和回放增强。
暂缓：

不立即把默认控制器换成 MPC/MPCC。
不立即引入强化学习作为主规划器。
不一次性合并所有论文方法。
不把低空标准安全间隔直接套入室内地图。
十一、建议验收命令

python -m pytest "tests\test_obstacle_scenario.py" -q
python -m pytest "tests\test_replanner_semantics.py" "tests\test_replanner_subgoal.py" -q
python -m pytest "tests\test_esdf_correct.py" -q
python -m pytest "tests\test_gnn_planner.py" -q
python -m pytest "tests\test_fault_tolerance_online.py" -q
新增后补充：

python -m pytest "tests\test_trajectory_optimizer.py" -q
python -m pytest "tests\test_drone_params.py" -q
python -m pytest "tests\test_safety_profiles.py" -q
python -m pytest "tests\test_risk_report.py" -q
这份代码计划的重点是：先把“评估系统”立起来，再做连续轨迹和参数 profile。这样每个后续改动都能判断是真的变好，还是只是动画看起来更顺。