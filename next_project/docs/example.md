# next_project 使用说明（example）

## 1. 这份文档解决什么问题

这份文档用于快速回答两个问题：

1. 各代码文件分别负责什么功能。
2. 如何从“改配置”到“运行仿真”再到“查看结果图和统计数据”。

---

## 2. 项目结构与文件用途

### 2.1 顶层文件

- main.py
  - 统一入口。
  - 负责：创建 SimulationConfig、执行仿真、打印指标、生成图表。

- README.md
  - 项目简要说明和常用命令。

- requirements.txt
  - Python 依赖列表。

- 理论公式与出处.md
  - 动力学、控制、风场、滑模等关键公式与参考文献。

- 需求符合性检查.md
  - 与研究计划书的对照检查记录。

### 2.2 core 目录

- core/rotor.py
  - 旋翼模型与电机一阶动态。
  - 包含简化旋翼和 BEMRotor。

- core/allocator.py
  - 控制分配矩阵。
  - 把 [T, Mx, My, Mz] 映射到四个旋翼推力和转速。

- core/wind_field.py
  - 风场模型。
  - 稳态风 + OU 湍流过程。

- core/drone.py
  - 无人机动力学。
  - 提供 Drone（欧拉角）和 QuaternionDrone（四元数）。

- core/controller.py
  - 串级控制器。
  - 提供 Controller（PID）和 HybridAttitudeController（PID+SMC姿态环）。

- core/smc.py
  - 滑模控制器。
  - 包含 SecondOrderSMC 和 SuperTwistingSMC。

- core/topology.py
  - 编队拓扑管理。
  - 支持 v_shape/diamond/line/triangle/custom，并支持平滑切换。

### 2.3 simulations 目录

- simulations/formation_simulation.py
  - 核心仿真流程。
  - 负责：构建无人机、执行主循环、记录误差、输出指标。

- simulations/visualization.py
  - 结果可视化。
  - 自动输出三类图：轨迹图、三维实时误差图、误差统计图。

- simulations/benchmark.py
  - 批量评测。
  - 用不同随机种子重复仿真并输出 JSON 统计结果。

---

## 3. 仿真配置流程（推荐）

### 第一步：在 main.py 中定义 SimulationConfig

你只需要改 main.py 里的配置对象，不需要改核心算法文件。

关键字段说明：

- 时间与模式
  - dt: 仿真步长。
  - max_sim_time: 仿真总时长。
  - use_smc: True 使用混合控制（姿态环SMC），False 使用纯PID。

- 编队规模
  - num_followers: 从机数量。
  - formation_spacing: 编队间距。
  - initial_formation: 初始队形。
  - custom_initial_offsets: 自定义初始偏移（仅 initial_formation=custom 时使用）。

- 航点
  - waypoints: 航点列表，每个元素是 [x, y, z]。
  - wp_radius: 普通航点切换半径。
  - wp_radius_final: 最后一个航点切换半径。

- 控制与机动平滑
  - leader_gain_scale / leader_max_vel / leader_max_acc: 领航机机动强度。
  - follower_gain_scale / follower_max_vel / follower_max_acc: 从机控制强度。
  - leader_acc_alpha: 从机使用领航机加速度前馈时的低通滤波系数。

- 扰动与复现
  - leader_wind_seed / follower_wind_seed_start: 风场随机种子。

- 动态切换
  - formation_schedule: 队形切换计划，格式为 (触发时间, 目标队形, 过渡时间)。

### 第二步：运行主程序

在 next_project 目录执行：

```bash
python main.py
```

程序会输出：

1. 每架从机的 mean/max/final 误差。
2. 自动生成图表到 outputs 目录。

### 第三步：查看图表与结果

默认输出文件：

- outputs/trajectory_3d.png
  - 领航机与从机三维轨迹图。

- outputs/error_realtime_3d.png
  - 三维实时误差图（误差向量在误差空间的演化轨迹）。

- outputs/error_statistics.png
  - 每架从机平均误差与最大误差的统计对比图。

---

## 4. 配置示例

### 示例 A：5 架从机 + 自定义航点

```python
config = SimulationConfig(
    max_sim_time=30.0,
    use_smc=True,
    num_followers=5,
    formation_spacing=1.8,
    waypoints=[
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 12.0], dtype=float),
        np.array([15.0, 0.0, 12.0], dtype=float),
        np.array([15.0, 15.0, 12.0], dtype=float),
        np.array([0.0, 15.0, 12.0], dtype=float),
        np.array([0.0, 0.0, 0.0], dtype=float),
    ],
)
```

### 示例 B：加入队形切换计划

```python
config = SimulationConfig(
    max_sim_time=40.0,
    num_followers=5,
    initial_formation="v_shape",
    formation_schedule=[
        (10.0, "line", 4.0),
        (22.0, "diamond", 5.0),
    ],
)
```

### 示例 C：自定义队形偏移

```python
config = SimulationConfig(
    num_followers=4,
    initial_formation="custom",
    custom_initial_offsets=[
        np.array([-2.0, -1.0, 0.0]),
        np.array([-2.0,  1.0, 0.0]),
        np.array([-4.0, -1.5, 0.0]),
        np.array([-4.0,  1.5, 0.0]),
    ],
)
```

---

## 5. 批量评测流程

用于验证鲁棒性与复现性：

```bash
python simulations/benchmark.py
```

输出文件：

- outputs/benchmark_results.json

该文件包含：

1. 每次实验的运行时间、平均误差、最大误差、最终误差。
2. 多次实验统计（均值、标准差、最差工况最大误差）。

---

## 6. 常见改动建议

- 想让误差更小：
  - 适当降低 leader_max_vel 和 leader_max_acc。
  - 保持 use_smc=True。
  - 适当增大 wp_radius 减少急转。

- 想让仿真更快：
  - 在可接受精度下适当增大 dt。
  - 减少 max_sim_time 或航点数量。

- 想做论文图：
  - 调整 visualization.py 的 dpi、线宽、配色和标题。

---

## 7. 最小工作流程总结

1. 改 main.py 里的 SimulationConfig（从机数、航点、队形）。
2. 运行 python main.py。
3. 在 outputs 查看三类图。
4. 需要统计报告时运行 python simulations/benchmark.py。
