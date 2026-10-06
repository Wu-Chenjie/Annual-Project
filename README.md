# 室内无人机集群：ROS 2 / Gazebo 仿真

本次归档（2026-10-06）包含[当前工程待办](RACER_GVP_ENGINEERING_TODO.md)、[最新开发配对及完整证据](system-comparison/2026-10-06/README.md)、[规划器优化可行性核对](planner-optimization/2026-10-06-feasibility/README.md)和[归档范围/本地检查](docs/progress-snapshot-20261006.md)。最新结果来自小规模开发检查，正式 E01–E10 与其余70项矩阵仍未完成。

正在按[工程TODO执行记录](ros2_ws/TODO_EXECUTION.md)推进团队新增信息账本、运动中授权交接、规划墙钟预算、私有缓存复用与任务生命周期。一套融合入口保持不变；正式五种子配对、专项场景和消融尚未全部验收。[第一版原生开发录像及完整原始证据](ros2_ws/docs/validation/todo-development/README.md)已保存，不能把开发单次结果当成正式性能结论。

历史[新配色录像（1080p，62.47 秒）](ros2_ws/docs/validation/palette-video/palette-exploration.mp4)：该次890.5 s达到95%三维覆盖，最终覆盖率95.07%，总航程402.4 m，零接触。[当次源码与独立审计](ros2_ws/docs/validation/palette-video/README.md)保留。

此前联合路线/观测收益改进的一次同条件 Gazebo 飞行为 **820.0 s 达到 95% 三维覆盖**，总航程 327.1 m，零接触；较此前 1631.0 s 优化融合版缩短 49.7%。[原录像、三版本对照及独立审计](ros2_ws/docs/validation/fusion-integrated/README.md)完整保留。各次运行的异步调度和路线不同，单次实测不能保证每次复现相同耗时。

早期策略接入同一三维传感环境时，2400.6 s 覆盖 53.53%，未达到 95%。[历史对照与全部尝试披露](ros2_ws/docs/validation/early-policy-3d/README.md) 保留当时的版本和指标；这不是原二维约 300 s 任务的复现。

主运行入口已迁移到 **ROS 2 Jazzy + Gazebo Harmonic（Ubuntu 24.04）**。当前主任务为三架四旋翼的去中心化三维搜索：自适应区域、增量 MR-DTG、图 Voronoi、双机容量路线协商、观测位姿与连续轨迹共同组成一条融合流程。复用仓库原有 C++ 飞控和 Python 规划算法，Gazebo 负责刚体动力学、重力与碰撞。网页服务、HTML 页面和 Plotly HTML 导出已移除。

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch annual_swarm decentralized_search.launch.py headless:=false rviz:=true
```

依赖安装、Docker（含 macOS 无界面验证）、地图切换、节点/话题、模型参数和验证命令见 **[ROS 2 使用说明](ros2_ws/README.md)**；实测结果见 **[验收记录](ros2_ws/VALIDATION.md)**。

```mermaid
flowchart LR
  G[Gazebo 三维动力学与 GPU LiDAR] --> M[各机私有体素地图与状态估计]
  M --> H[Hgrid 与 EROI]
  H --> T[增量 MR-DTG]
  T --> V[局部与全局图 Voronoi]
  V --> P[双机容量任务路线协商]
  P --> O[区域顺序引导的观测位姿搜索]
  O --> Q[质量评估与五条备选]
  Q --> C[连续轨迹与 ROS 控制器]
  C --> G
```

| 目录 | 用途 |
|---|---|
| `ros2_ws/src/annual_swarm` | ROS 节点、Gazebo 插件、SDF 模型、世界、launch、集成测试 |
| `next_project/cpp` | ROS 直接复用的 C++ 控制器、分配器、A* 与地图代码；原独立仿真作为离线基线 |
| `next_project/maps` | 共享障碍地图；启动时生成同源 Gazebo 几何 |
| `next_project/core`、`simulations`、`tests` | ROS 直接复用的 Python 规划/APF 算法、离线回归和静态图表 |
| `docker/Dockerfile.ros2` | 可复现 Linux ROS/Gazebo 环境 |
| `old_code`、`references` | 历史算法与项目资料 |

ROS 可选择 A*、航向约束 A*、Hybrid A*、Dijkstra、RRT*、Informed RRT*、D* Lite、GNN 与分层滑动窗口调度；控制可选 PID、SMC、反步＋SMC、反步＋PID，以及两种实验控制分支。ESDF、FIRI、轨迹后处理、APF 前馈可独立配置。支持飞行中更换目标和周期重规划，运行结果记录实际算法及回退信息。

### GitHub 自动 CI

每个分支的 push 和所有 pull request 自动运行两套检查，也可在 [Actions](https://github.com/Wu-Chenjie/Annual-Project/actions) 页面手动运行：

- **CI**：安装锁定 Python 依赖、构建 C++ 全部目标、运行非 slow 的 pytest 回归；超时 30 分钟，保存 `python-test-report` 测试报告。
- **ROS 2 Gazebo**：构建 Jazzy/Harmonic Docker 环境、运行 ROS 包测试与完整 Gazebo 物理冒烟验收；超时 45 分钟，保存 `gazebo-flight` 中的包测试报告及飞行证据。

同一工作流、同一分支的新提交会取消旧运行，测试报告保留 14 天。`Fused 3D Gazebo exploration` 和 `Complex-map search validation` 继续手动触发，适合耗时较长的专项验收。

```bash
ros2 launch annual_swarm swarm.launch.py planner:=window controller:=backstepping replan_interval:=3.0
ros2 launch annual_swarm swarm.launch.py planner:=informed_rrt_star controller:=smc
ros2 topic echo /swarm/planner_diagnostics
```

历史 `swarm.launch.py` 编队入口使用已知静态地图、定高、固定三机队形及 Gazebo 真值里程计。新增 portfolio 模式支持质量评分、五条备选及真实 Gazebo 圆柱障碍快照切换；未知环境融合探索由独立的 `decentralized_search.launch.py` 入口提供；MPC 仅复用原可行性评估器，不是在线 MPC 控制器。详细能力、实验分支含义和验收命令见 [ROS 使用说明](ros2_ws/README.md)。

实际 Gazebo 截图及动态恢复对照图见 [运行证据](ros2_ws/docs/validation/README.md)。

新增复杂环境三机独立搜索：有限视距观测、动态图任务分区、成对负载优化和动态任务转交，见 [搜索使用说明](ros2_ws/EXPLORATION.md) 和 [复杂地图实测图](ros2_ws/docs/validation/search/README.md)。

### 融合探索录像与证据

三维点云、IMU/定位估计、自适应区域、历史树握手拓扑、图分区与双机 CVRP、连续轨迹以及断连/重启恢复的实现及边界见 [融合架构与接口](ros2_ws/DECENTRALIZED_EXPLORATION.md)。主入口 `decentralized_search.launch.py` 将这些环节共同运行，没有 RACER/GVP 模式选择器。

最新版本将行驶时间与信息加权完成时间共同优化，保留优化后的路线；等待奖励有界，并通过压缩的实际观测回执减少跨机重复收益计算。区域粗评估有预算与轮转，执行视点仍做完整几何和预约核验。[本次视频与三版本数据](ros2_ws/docs/validation/fusion-integrated/README.md) 同时记录 90%→95% 为 67.0 s、173 项测试通过，以及单轮规划仍比此前优化融合版慢的限制。[上轮末端优先级版](ros2_ws/docs/validation/fusion-tail-priority/README.md) 的总耗时退化记录完整保留。

![联合改进的三版本对照](ros2_ws/docs/validation/fusion-integrated/three-way-comparison.png)

[观看优化版 Gazebo 原生视频](ros2_ws/docs/validation/fusion-optimized/fused-exploration.mp4) · [优化前后对照、独立审计与原始证据](ros2_ws/docs/validation/fusion-optimized/README.md)

同地图、相同飞行限速的一次完整对照中，达到 95% 覆盖用时由 2183.5 s 降至 1631.0 s（缩短 25.3%），视点间等待中位数由 10.05 s 降至 2.49 s；零接触。总航程同时增加 37.1%，重复探索仍需改进，不能据此声称能耗或路径长度也得到优化。[原三维融合基线](ros2_ws/docs/validation/fusion/README.md) 保留供复算。

![优化后三维融合探索完成画面](ros2_ws/docs/validation/fusion-optimized/demo-preview.png)

[历史平面版录像与旧口径数据](ros2_ws/docs/validation/decentralized/README.md) 单独保留，不能与当前三维传感结果直接比较。旧集中式探索和编队入口继续用于原算法回归。
