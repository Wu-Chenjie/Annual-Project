# 复杂环境多机搜索与动态任务分配

本实现参考 RACER 的成对任务优化和 GVP-MREP 的图距离分区思路，提供 **集中式、共享观测地图、定高三机搜索原型**。不是原论文完整复现，不具备其分布式通信故障保证。当前图使用稳定 ID 的观测自由栅格节点与四邻接边，尚未压缩为原 MR-DTG 的 H/F 区域结构。

## 运行

ROS 2 Jazzy / Gazebo Harmonic，依赖与构建沿用 README。

```bash
ros2 launch annual_swarm search.launch.py
ros2 launch annual_swarm search.launch.py \
  map:=/workspace/next_project/maps/search_maze.json \
  policy:=gvp_pairwise pause_after:=50.0 pause_duration:=30.0
ros2 run annual_swarm search_smoke.py --output-dir /tmp/search-run \
  --map /workspace/next_project/maps/search_maze.json --pause-after 50
```

输出目录每次单独指定；smoke 要求目录尚不存在。默认图为 search_office。原 swarm.launch.py 固定编队任务不变。当前搜索用三机独立 PID 控制、0.6 m/s 参考上限、反馈限速，保留真实旋翼动力学、IMU、里程计与接触检测。

`policy` 可选 nearest_frontier、graph_voronoi、gvp_pairwise。前两者作为基线；第三种为图初始归属后，对两机间任务转移及交换进行有限次改进。目标是最慢预计完成时间加总行驶时间、任务转移惩罚；使用名义行驶速度与“1 秒 + 邻域未知格数×0.01 秒”的观测工作量代理，**不是测得的观测服务时间或原论文 CVRP 最优解**。当前执行目标固定归属，规划未来任务时保持它优先；只给空闲飞机派发新目标。

## 地图与观测

两张地图均为 24×20 m，九个房间、交错门洞、环路和 U 形/货架遮挡；maze 添加交错障碍和 1.8 m 瓶颈（其他门洞 2.6 m）。地图 JSON 包含三个分散起飞点。地形、障碍与 Gazebo 由同一 JSON 生成。

使用 **理想平面射线传感模型**：3.5 m 范围、360°、720 条射线、首个障碍截断；不是实测激光或深度相机。观测更新 0.25 m 的 unknown/free/occupied 地图。未知区域和地图边界都阻止规划通行；根据已观测自由空间与未知区的边界选择可安全抵达的观测位姿。目标处在已知自由空间，靠传感范围向未知区域扩张。

全图真值只用于场景起飞点合法性检查、生成 Gazebo 几何、模拟射线和计算覆盖率分母；图搜索、任务分区及路径质量评估接收 ObservedMap。实验在真值评测达到 95% 平面自由空间覆盖时结束；这是评测终止阈值，不能当作全部环境已探索的声明。位姿和跨机坐标使用 Gazebo world 真值；尚未接入 SLAM、位姿不确定性或真实通信丢包。

## 节点和接口

- search_node.py：射线观测适配、共享地图、图更新、任务分配、每机路径池与搜索评测。集中协调有单点故障，此版本不称为去中心化。
- search_executor_node.py：独立的 50 Hz 三机参考执行、反馈限速、任务版本检查与机间距联锁。高层规划不阻塞该控制进程。
- 三个原 controller_node：独立目标 → C++ 控制器 → 电机转速 → Gazebo 刚体。
- `/drone_i/search_path`：JSON 路径命令，包含 task、epoch、map_version；旧 epoch 拒绝。
- `/search/execution`：执行 epoch、到达状态、起飞完成状态与故障。高层收到当前 epoch 的完成状态后才能结束该任务。
- `/search/topology`：带 sequence/map_version 的节点、边增删；边权为 resolution 米。
- `/search/topology_snapshot`：约每 5 仿真秒的完整图快照；后来订阅或丢失 delta 的查看器须从快照恢复，再按 sequence 接续。
- `/search/diagnostics`：覆盖率、图大小、任务、候选数、工作量和事件。

graph.py 内复用原 A*，并用边代价惩罚生成路线变体；复用现有 PathQualityEvaluator / RankedPathPool 评分和几何去重，每机最多 1+5 条路线。只保留同一任务、同一 epoch 的安全路线；窄通道缺少不同路线时如实记录不足五条。

路径生成会预留其他飞机尚未执行的整条活动路径以及悬停位置，安全半径 1.15 m；执行器另设 1.0 m 距离联锁。该保守预留方法可能增加等待，并不保证所有会车局面无死锁。对所有给定测试需同时检查覆盖率与安全，不能将长期悬停误判为成功。

## 动态任务和障碍

新观测不断生成/更新前沿。达到一个观测位姿记录 viewpoint_observed；区域搜索成效独立用实际观测覆盖率衡量，不用“到点数”替代覆盖率。

`pause_after` 是受控暂停试验：自搜索开始 50 秒后让 UAV 1 停止推进并悬停，释放未完成任务；其他 UAV 接管，30 秒后恢复其可用状态。日志区分 availability_pause、task_transferred、availability_resume。同一架飞机重新规划是 task_replanned，不计为跨机转交。**这不是丢包、掉线或失控飞机的安全接管协议。**

```bash
# 可选实际圆柱移动接口，启用后手动发送世界坐标
ros2 launch annual_swarm search.launch.py dynamic_obstacle:=true
ros2 topic pub --once /swarm/move_obstacle geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 10, y: 10, z: 1.5}, orientation: {w: 1.0}}}'
```

Gazebo 服务确认后仅更新射线模拟器的世界；未进入传感范围的障碍不会直接写入规划地图。新观测阻塞活动路径时，结合其他飞机的预留重新验证同任务备选；有安全缓存则切换，无缓存则停止推进并将任务留待重新规划/分配。此接口与缓存失效已有核心测试；搜索模式下的任意移动障碍轨迹不作已验收保证。

## 对照与结果

```bash
ros2 run annual_swarm compare_search.py --output-dir /tmp/search-comparison \
  --maps /workspace/next_project/maps/search_office.json /workspace/next_project/maps/search_maze.json
# 可仅选部分策略，所有场景顺序运行，失败保留并明确标注
# --policies nearest_frontier graph_voronoi gvp_pairwise
python3 ros2_ws/src/annual_swarm/scripts/plot_search.py /tmp/search-run --output /tmp/search.png
# 三策略同一地图结果的汇总图（需 matplotlib）
python3 ros2_ws/src/annual_swarm/scripts/plot_search_comparison.py /tmp/search-comparison --map search_office --output /tmp/comparison.png
```

每次保存输入 map.json、真实轨迹 CSV、coverage.json、逐任务候选路径、图 delta、事件与 summary。`published_*_payload_bytes` 仅统计相应 JSON 消息有效负载，不包含 DDS、里程计、传感器与全部诊断流量，不用于声称通信带宽优于原论文。

验收要求：覆盖率≥95%、全部三机实际移动、起飞后无接触、最小间距>0.8 m、受控暂停产生真实跨机任务转交，达到覆盖阈值后继续保持至少 3 仿真秒。当前不是多种子统计结论；具体运行记录与失败修复见 VALIDATION.md。

参考：[RACER](https://arxiv.org/abs/2209.08533)、[GVP-MREP](https://arxiv.org/abs/2408.05808)。完整前期设计见 [设计文档](docs/design/2026-09-23-search-task-allocation.md)。

本次真实飞行图、逐场景摘要和三策略对照见 [搜索实测证据](docs/validation/search/README.md)。仓库提供 `.github/workflows/search.yml` 手动验收工作流定义；本次报告中的测试是本地 Docker 实测，不表示 GitHub Actions 已运行。
