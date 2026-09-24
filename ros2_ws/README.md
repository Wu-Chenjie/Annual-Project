# ROS 2 / Gazebo 使用说明

早期策略已接入同一三维传感环境实测：2400.6 s 时覆盖 53.53%，未达到 95%；当前优化融合版为 1631.0 s 达到 95%。[原生录像、同口径对照及全部尝试披露](docs/validation/early-policy-3d/README.md)。这是旧策略的三维接口适配对照，不是原二维约 300 s 任务的复现。

当前主入口为 `decentralized_search.launch.py`：三维点云、分层区域、MR-DTG、双机任务/路线协商、观测位姿与连续轨迹组成单一融合流程。完整节点契约、故障恢复与录制命令见 [融合探索说明](DECENTRALIZED_EXPLORATION.md)。下文保留的 `swarm.launch.py` 是原编队入口。

最新 [优化版视频与性能对照](docs/validation/fusion-optimized/README.md) 保留实际轨迹、测试、源码及审计证据：同地图达到 95% 覆盖用时缩短 25.3%，总航程增加 37.1%。

## 环境与启动

目标平台：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。采用官方 [ROS/Gazebo 配套版本](https://gazebosim.org/docs/harmonic/ros_installation/)及 [ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)。不使用 Gazebo Classic、`gazebo_ros_pkgs` 或原来的 Humble 空世界脚本。

安装 ROS 2 Jazzy 后：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-transport-vendor ros-jazzy-gz-plugin-vendor \
  ros-jazzy-actuator-msgs ros-jazzy-tf2-geometry-msgs \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml python3-numpy python3-scipy \
  ros-jazzy-ament-cmake-pytest
source /opt/ros/jazzy/setup.bash
# 如果尚未初始化 rosdep：sudo rosdep init
rosdep update
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch annual_swarm decentralized_search.launch.py headless:=false rviz:=true
```

原编队可执行 `ros2 launch annual_swarm swarm.launch.py`。从仓库根目录也可执行 `./start_gazebo.sh`。Gazebo 自动运行，三机从地面起飞到 1.5 m，经三柱场景绕障后在 `(18,16,1.5)` 周围悬停。GUI 中可在实体列表选择 `drone_0` 聚焦观察。

```bash
# 无 GUI（无须显示器/GPU；仍有真实物理、IMU、碰撞）
ros2 launch annual_swarm swarm.launch.py headless:=true
# 已知地图、起点、终点；起终点必须同高度，并留足编队空间
ros2 launch annual_swarm swarm.launch.py \
  map:=/absolute/path/to/map.json start:=2,3,1.5 goal:=18,16,1.5 \
  output_dir:=/tmp/swarm_results
# 检查通信与飞行状态
ros2 topic echo /swarm/status
ros2 topic echo /drone_0/odometry --once
ros2 topic hz /drone_0/imu
# 可选：录制 ROS bag；输出写到当前目录
ros2 bag record /clock /swarm/path /swarm/status \
  /drone_0/odometry /drone_1/odometry /drone_2/odometry
```

地图支持 `aabb` 和垂直 `cylinder`，格式与 `next_project/maps` 一致。启动时拒绝未知几何类型、非有限数值和无效尺寸，避免规划地图与物理世界不一致。规划失败发布 `FAILED`，不会开始任务。狭窄地图可能不适合本次固定队形，必须检查起点、目标和通道的编队净空；不保证所有旧预设都可直接飞行。

## Docker / macOS

在仓库根目录：

```bash
docker build -f docker/Dockerfile.ros2 -t annual-swarm:jazzy .
docker run --rm -it annual-swarm:jazzy
# 原编队三机飞行自动验收，保存日志/CSV/JSON到本地
mkdir -p artifacts
docker run --rm -v "$PWD/artifacts:/results" annual-swarm:jazzy bash -c \
  'source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && ros2 run annual_swarm smoke_test.py --timeout 240 --output-dir /results'
```

运行镜像默认启动融合探索。macOS 上使用 Docker Linux；无界面运行仍有真实物理与点云传感。录制镜像提供 Xvfb、Openbox、原生 Gazebo/RViz 和 ffmpeg：

```bash
docker build -f docker/Dockerfile.demo -t annual-swarm:demo .
# 同一次运行保留视频、源码快照、轨迹、协商日志和独立审计结果
docker run --rm -v "$PWD/artifacts:/workspace/artifacts" annual-swarm:demo
```

默认输出 `artifacts/fusion/demo`，目录已存在时录制器拒绝覆盖。原生 GUI 在虚拟 X11 屏幕中录制，不需要 HTML 或浏览器。

## 节点与接口

| 节点/组件 | 频率 | 输入 → 输出 | 职责 |
|---|---:|---|---|
| `planner`（Python 调度） | 启动、目标更新、可配置周期 | JSON 地图、目标、odometry → `/swarm/path`、诊断 | 直接复用原规划包；后台线程计算；整队净空检查 |
| `formation`（Python） | 50 Hz | path、三机 odometry → 各机 target、status | 起飞同步、限速参考轨迹、根据最差跟踪误差暂停路径推进 |
| `drone_N/controller`（C++） | 100 Hz，由 odometry 驱动 | target / trajectory_target、odometry → motor_speed | 复用控制器与控制分配器；世界速度/机体系角速度转换 |
| `gazebo_bridge` | 各话题频率 | ROS ↔ Gazebo Transport | 桥接时钟、旋翼命令、真值里程计、IMU、接触 |
| `annual::MotorSystem`（Gazebo C++ 插件，每机一个） | 1 kHz | 四旋翼转速 → 刚体力与力矩；物理状态 → odometry | 电机一阶响应、推力/反扭矩、阻力、命令超时 |
| `metrics`（Python） | 里程计频率，1 Hz 落盘 | odometry、target、contacts、status → CSV/JSON | 跟踪误差、最小机间距、真实接触计数、终态 |

`N=0,1,2`：

| 话题 | ROS 类型 | 坐标/单位 |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo 仿真时间；全部业务节点启用 `use_sim_time` |
| `/swarm/path` | `nav_msgs/Path` | world，米；transient-local 保存初始规划 |
| `/swarm/planner_status`、`/swarm/status` | `std_msgs/String` | READY/FAILED；WAITING/TAKEOFF/FLYING/HOLDING/FAILED |
| `/drone_N/target` | `geometry_msgs/PoseStamped` | world，目标位置；当前任务偏航固定为 0 |
| `/drone_N/odometry` | `nav_msgs/Odometry` | pose: world；twist: drone_N/base_link |
| `/drone_N/command/motor_speed` | `actuator_msgs/Actuators` | velocity[4]，rad/s，ROS → Gazebo |
| `/drone_N/imu` | `sensor_msgs/Imu` | Gazebo 原生惯性传感器 |
| `/drone_N/contacts` | `ros_gz_interfaces/Contacts` | Gazebo 原生接触传感器 |

目前直接使用 world 真值里程计，不发布定位 TF 树，也不伪装成 SLAM/VIO 估计。Gazebo GUI 足以查看模型和场景。录制/重放 bag 时应使用仿真时钟。

## 建模与控制

- 单机总质量 1 kg；惯量对角线 `(0.01,0.01,0.02)` kg·m²。机体为单刚体，四个旋翼视觉件及机臂归于机体。
- 四旋翼在机体系 `(x,y)` 位置依次为 `(-.2,+.2),(-.2,-.2),(+.2,-.2),(+.2,+.2)` m；反扭矩方向 `[-,+,-,+]`，与原 C++ 分配矩阵一致。
- `F_i = 1e-5 * omega_i²` N，`M_i = ±2e-7 * omega_i²` N·m；转速限幅 900 rad/s，电机时间常数 25 ms。
- 机体系 Z 向推力和三轴力矩转到世界系后施加；重力、六自由度积分和接触响应由 Gazebo 处理。线性阻力系数 0.1 N·s/m，角阻尼 0.002 N·m·s/rad。
- 碰撞包络是 `0.64 × 0.64 × 0.16` m 长方体，水平外接半径约 0.453 m。旋翼圆盘是视觉件，不模拟独立叶片接触或旋翼气流相互作用。
- 三机偏移固定在世界系 `(0,0,0),(0,-1,0),(0,+1,0)` m。规划净空默认 1.9 m，涵盖 1 m 队形半宽、机体包络与跟踪裕量。栅格规划使用 0.25 m 分辨率；所有规划、平滑和重规划拼接都经过连续净空检查。
- 移动参考速度 0.5 m/s；任一无人机误差 ≥0.35 m 时暂停推进；起飞误差均 <0.15 m 后开始航线。最终 `HOLDING` 表示到达后持续悬停，不自动落地。
- `controller` 启动参数选择真实控制实现，默认 PID 结构采用保守增益（积分、微分为零）。SMC 使用原 C++ SecondOrderSMC；反步分支使用修复后的虚函数位置环和积分反步导数。控制积分时间步长来自 Gazebo 里程计时间戳。
- 目标过期 0.5 s 时控制节点清零输出；插件在命令超过 0.25 s 未更新后让旋翼衰减停转。队列/节点中断应终止实验；这是仿真失效保护，不是实机紧急降落策略。

飞行增益、编队速度和规划净空集中在 `src/annual_swarm/config/flight.yaml`；修改后重新构建以更新安装目录。

模型物理参数见 `src/annual_swarm/models/quadrotor.sdf` 和 `src/annual_swarm/src/motor_system.cpp`；改质量/惯量/旋翼布局时必须同步飞控与分配器，不能只改视觉模型。

## 验证与结果

```bash
cd ros2_ws
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
# 实际启动 Gazebo、ROS 桥与所有节点：默认完整障碍航线
ros2 run annual_swarm smoke_test.py --timeout 240
# 较快的短距离起飞/平移/悬停检查
ros2 run annual_swarm smoke_test.py --goal 3,3,1.5 --timeout 90
# 目标在柱体内部：验证规划拒绝且三机保持地面
ros2 run annual_swarm smoke_test.py --goal 7,7,1.5 --expect-planner-failure --timeout 45
```

自动飞行检查要求三机都有连续里程计、到达指定终点、机间距 >0.7 m、起飞后无接触，并实际收到连续 3 秒仿真时间的 `HOLDING`，不会把“进程已启动”当作成功。`summary.json` 每秒原子更新；每次运行独立时间戳目录，包含 `trajectory.csv`。误差统计包含起飞过程；接触数是接触消息样本数量，不是去重后的碰撞事件数；最小机间距使用最近收到的位置，存在消息时间偏差。

`.github/workflows/ros2.yml` 在 PR 中构建 Linux 镜像、运行场景单测和全航线 Gazebo 冒烟验收，并保留结果工件。原 C++/Python CI 继续验证离线算法。

## 迁移边界

已接入：下表规划/控制/后处理算法、真实刚体/接触、旋翼动力、共享静态地图、三机编队、时钟、IMU/接触话题、轨迹/碰撞评测。portfolio 模式已接入动态圆柱模型同步和备选切换。未接入：未知地图建图、拓扑故障重构、相机/激光建图和 PX4。常规在线重规划使用已知静态地图及实时 Gazebo 位姿；portfolio 模式额外消费动态几何快照，不声称具有未知环境感知能力。

`next_project` 中历史技术文档描述的是迁移前的算法能力，当前运行入口和接口以本文为准。HTML 页面、网页后端及 Plotly 导出已删除，原 Python 静态 PNG 输出保留；归档脚本中的浏览器绘图也改为 Matplotlib/PNG，未改动其动力学算法。


## 算法选择及运行证据

| 参数 | 可选值 / 行为 |
|---|---|
| `planner` | `astar`、`heading_astar`、`hybrid_astar`、`dijkstra`、`rrt_star`、`informed_rrt_star`、`dstar_lite`、`gnn`、`window` |
| `controller` | `pid`、`smc`、`backstepping`（反步位置＋SMC姿态）、`backstepping_pid` |
| 实验控制器 | `geometric_euler`（原 Python Euler 姿态误差控制移植，非完整 SO(3)）、`super_twisting`（超扭曲切换律，按实际减期望误差修正符号并增加边界层） |
| `replan_interval` | 仿真秒；0 关闭周期规划。`window` 的 0 自动设为 2 秒；目标消息始终触发规划 |
| `esdf` | A* 三种变体的距离软代价；不支持的组合明确拒绝 |
| `firi` | 调用原走廊修正器，使用内切球近似；拒绝越界或净空不足的修正 |
| `trajectory` | `none`、`moving_average`、`minimum_jerk`、`min_snap_proxy`、`min_jerk_cost`；后两者是原工程代理方法 |
| `apf` / `formation_apf` | 单机 / 编队人工势场加速度前馈；编队势场同时保留单机势场，总输出各轴限幅 0.5 m/s² |
| `velocity_feedforward` | 将反馈门控后的参考移动速度送入控制器；默认 false |

规划运行节点直接导入安装时复制的 `next_project/core`，不运行旧数值动力学。规划器 基础节点仍保留可单独调用，但主 launch 使用统一的多算法调度节点。GNN 保留活动传播与图搜索，修复起终点插入后的邻接索引，断图必须报错，禁止以未验证直连伪造路径。

D* Lite 在同一目标的多次规划中保留搜索状态。`window` 复用原 WindowReplanner：Hybrid A* 局部、D* Lite 增量、Informed RRT* 全局、GNN 危险分支、双模式、风险间隔和 Voronoi 子目标选择。局部窗口必须拼接出安全的全局后续路径，失败则保留上一条已验证路径并报告 DEGRADED。

轨迹优化后的几何路径由编队节点限速、反馈门控跟踪；当前不按优化器原时间戳执行整条轨迹。MPCFeasibilityEvaluator 仅生成速度/加速度可行性及误差代理报告，不向电机发送 MPC 控制量。`min_snap_proxy` 也不等同于完整最小 snap 求解器。

```bash
ros2 launch annual_swarm swarm.launch.py planner:=window controller:=backstepping replan_interval:=3.0
ros2 launch annual_swarm swarm.launch.py planner:=astar controller:=smc \
  esdf:=true firi:=true trajectory:=minimum_jerk apf:=true formation_apf:=true
# 飞行中更换目标（必须在安全净空内且保持当前高度）
ros2 topic pub --once /swarm/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 18.0, y: 4.0, z: 1.5}, orientation: {w: 1.0}}}'
ros2 topic echo /swarm/planner_diagnostics
ros2 topic echo /drone_0/controller_status
# 全算法实际飞行矩阵（每一项为完整三机航程，约需十余分钟）
ros2 run annual_swarm validate_matrix.py --output-dir /tmp/matrix
ros2 run annual_swarm smoke_test.py --planner dstar_lite --launch-arg replan_interval:=5.0
ros2 run annual_swarm smoke_test.py --redirect-goal 18,4,1.5 --output-dir /tmp/redirect
```

新增接口：`/swarm/goal` 为 world 系 PoseStamped；`/swarm/planner_diagnostics` 为 JSON String，记录算法、调用次数、D* 状态复用、后处理回退及 MPC 评估；`/drone_N/controller_status` 为控制器类型；`/drone_N/trajectory_target` 为单点 MultiDOFJointTrajectory，携带目标位置、速度和 APF 加速度。诊断与控制器类型也保存到每次运行的 `summary.json`。

实验 Euler 姿态控制在 Gazebo 的默认增益为 `geometric_kR=[48,48,18]`、`geometric_kOmega=[10,10,6]`，通过 ROS 参数可调；旧 Python 默认增益在物理闭环中振荡、推进过慢，未直接沿用。反步增益可通过 `backstepping_k0/k1/k2` 参数调整。

## 轨迹质量评估、五条备选与动态切换

`planner:=portfolio` 选择候选池调度节点。它直接调用前面的原规划算法，由共享的 `core/planning/path_quality.py` 统一评分。默认生成 A*、Hybrid A*、RRT* 的多条候选；也可仅指定一个规划器。

默认保留 **1 条执行路径＋最多 5 条备选路径**。先对整队连续净空进行硬检查，再按路径长度、最小净空、累计转角、名义加速度和 jerk 评分。各项使用固定尺度和可配置权重，低分优先；曲率、名义时间和可行性报告同时公开。动力学指标基于 0.5 m/s 名义参考，是规划质量代理，并非实测跟踪误差或动力学可行性证明。

候选采用不同随机种子与中间航路点组合生成；确定性 A* 也可以生成不同路线。按等弧长重采样后的平均距离去重，避免同一路线重复占用五个名额。评分最优只指当前生成候选集合中的最优；不宣称求出了全局 k 条最短路径。安全且不同的候选不足五条时，公开实际数量，不用重复或不安全路线补位。

```bash
# 多规划器、每个规划器多次尝试，启动实际 Gazebo 动态障碍演示
ros2 launch annual_swarm swarm.launch.py planner:=portfolio \
  candidate_planners:=astar,hybrid_astar,rrt_star candidate_variants:=4 \
  dynamic_obstacle:=true dynamic_demo:=true
# 单个 A* 的多路线评估与五条备选
ros2 launch annual_swarm swarm.launch.py planner:=portfolio \
  candidate_planners:=astar candidate_variants:=12 \
  dynamic_obstacle:=true dynamic_demo:=true
# 查看得分分项、路径来源、备选数量、版本与切换事件
ros2 topic echo /swarm/planner_diagnostics
# 此话题还包含当前路径与全部备选路径的坐标
ros2 topic echo /swarm/candidate_paths
# 手动移动真实 Gazebo 圆柱；dynamic_demo:=false 时完全由命令控制
ros2 topic pub --once /swarm/move_obstacle geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 9.5, y: 5.0, z: 1.5}, orientation: {w: 1.0}}}'
```

权重参数 `quality_weights` 为 JSON，例如 `'{"length":1.0,"clearance":0.8,"turning":0.15,"acceleration":0.15,"jerk":0.05}'`。所需五个键必须齐全，值必须有限、非负且至少一个正值。

动态障碍节点通过 `/world/indoor/set_pose` 服务移动真实 Gazebo 圆柱（半径 0.55 m、高 3 m），服务成功后发布带版本的 `/swarm/dynamic_obstacles` 世界坐标几何快照。此接口目前接收圆柱列表；演示控制一个模型，采用仿真场景真值，不是激光检测/SLAM，也未预测障碍未来运动。手动指令可以反复移动该模型；每次成功移动都触发重新验证。

收到新快照时：先暂停编队参考推进、保持电机闭环悬停；剔除受阻路径；检查从当前位置接入每条备选的连接段及完整剩余路径；优先保持仍安全的执行路径，否则切到最低分的安全缓存备选，并后台补充候选。若全部不可用，则继续悬停并重规划，不继续飞旧的受阻路径。地图或目标更新后，旧版本后台计算结果丢弃。初始起飞前没有路径时保持地面。

`dynamic_demo_mode:=block_goal` 将障碍移到目标附近，测试所有路径失效时的悬停。`smoke_test.py --expect-backup-switch` 验证初始确有五条备选、实际发生缓存切换并到达；`--expect-no-safe-backup` 验证无可用路径时三机保持空中悬停。原始质量指标、候选/切换事件与动态障碍快照都保存于 `summary.json`。

候选池完整坐标与每次评分保存到 `candidate_paths.jsonl`，可以追溯初始排名、失效剔除、缓存切换及后续补充，不仅保留最终统计。

## 动态恢复对照实验

```bash
# 三组各两次，逐次启动真实 Gazebo，次轮反转顺序；输出目录必须尚不存在
ros2 run annual_swarm compare_recovery.py --repeats 2 --output-dir /tmp/recovery_comparison
```

三组均以相同多规划器候选池、种子 42、五条备选和 PID 起飞；在领机 x 首次达到 3 m 后，将圆柱移到固定 `(9.5,5,1.5)`。`dynamic_demo_mode:=benchmark` 不根据各组路径选择障碍位置。初始执行路径 SHA256 必须一致，否则脚本报错。真实物理与服务调度带来的触发位姿差异保留在结果中。

- `recovery_policy:=cache`：重验证缓存并切换，后台补充候选。
- `recovery_policy:=replan_single`：丢弃缓存，复用更新后的占据图，只重新计算一次 A*，通过同一质量/安全检查后恢复。
- `recovery_policy:=replan_pool`：丢弃缓存，重建与初始配置相同的多规划器候选池，评分后恢复。

`results.json` 记录每次恢复墙钟耗时（障碍快照回调开始至安全路径发布）、编队 PAUSED 状态仿真时长、首次 FLYING 至首次 HOLDING 的仿真航行时间、该区间领机实际水平航程、起飞后接触数及最小机间距。恢复耗时不包含障碍命令传输，也不是机体完成转向时间。短暂停顿由状态订阅采样记录，受 ROS 调度影响。每次原始 summary、CSV、候选快照与日志均保留；只有成功完成整段航程并满足安全断言才计入。

这组实验隔离“障碍后的恢复策略”，没有比较启动阶段生成候选池的成本；单一地图、固定障碍事件、少量重复不能证明所有地图上的优势。

可选绘图（需 matplotlib）：`python3 ros2_ws/src/annual_swarm/scripts/plot_recovery_comparison.py <结果目录>/results.json`。本次实际对照记录见 [VALIDATION.md](VALIDATION.md)。

## 复杂地图独立搜索

新增 `search.launch.py`，三机各自执行搜索任务；与本页固定编队入口独立。复杂地图、观测假设、分配策略、动态转交与对照命令见 [EXPLORATION.md](EXPLORATION.md)。
