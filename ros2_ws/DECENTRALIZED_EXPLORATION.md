# 融合式多无人机三维探索

入口 `decentralized_search.launch.py` 运行一条融合流程，没有 RACER/GVP 模式选择器：

**实际点云 → 私有三维地图/增量前沿 → 自适应 Hgrid/EROI → 增量 MR-DTG → 局部与全局图 Voronoi → 双机容量约束任务/路线协商 → 区域顺序引导的观测位姿搜索 → 候选轨迹质量评估 → 连续轨迹 → ROS 控制器 → Gazebo 电机动力学。**

本仓库独立实现所引用的算法思想，依据 [RACER](https://arxiv.org/html/2209.08533v1) 和 [GVP-MREP](https://arxiv.org/html/2408.05808v1)。这是融合工程实现，不能等同于两篇论文的逐行移植、实机系统或论文指标复现。历史平面版本的录像和数据继续保存在 `docs/validation/decentralized/`，不得作为本版本的三维验收结果。

## 算法如何结合

| 环节 | 当前实现 | 主要文件 |
|---|---|---|
| 感知与估计 | 原生 Gazebo GPU LiDAR，181×31 射线、120°×120°、4.5 m、5 Hz；点云按估计位姿变换，概率占据更新；IMU 与带协方差定位测量融合 | `pointcloud_mapping_node.py`, `state_estimation.py` |
| 分层区域 | 8/4/2 m 自适应 Hgrid，稳定三维区域 ID，已提交区域固定；前沿局部更新、连通簇缓存；区域及持久视点状态 | `hierarchy.py` |
| MR-DTG | 实际飞行历史节点、有限半径 Dijkstra 树、树交叠握手折线；每个可达 EROI 挂接一个历史节点；跨机图增量合并、缺包快照修复、障碍边否决与区域服务反馈 | `mrdtg.py` |
| 图分区 | 沿图上真实走廊距离计算局部 EROI 与全局历史节点 Voronoi，决定初始服务归属 | `mrdtg.py:graph_voronoi` |
| 任务与路线 | 双机交换窗口上 Held–Karp 子集路线及容量约束划分；窗口之外的任务计入固定负载；已执行区域固定；版本化 prepare/accept/commit/applied 协商 | `pairwise.py`, `fusion.py` |
| 分层搜索 | 全局区域访问序列采用插入与 2-opt；本地区域做位置/偏航两步搜索；远程区域通过 MR-DTG 走廊推进已观测前缀；无本机归属工作时按收益/时间/服务人数分担 | `fusion.py`, `regions.py` |
| 观测/运动 | 信息收益按每条射线的首个未知体素计算，考虑遮挡；两步观测去重，联合平移、偏航、路线质量与下一服务区域出口代价 | `regions.py:ObservationPlanner` |
| 轨迹 | 共用质量评估器评估多规划器/单规划器多候选，保留至多五条有效备选；固定路径节点的最小 jerk 五次样条、段时间优化、速度/加速度/jerk 极值验证、偏航连续与控制前馈 | `path_quality.py`, `continuous_trajectory.py` |

双机精确求解只针对最多 10 个区域的交互窗口（求解器硬上限 12），不是全机队全局 CVRP 最优。窗口外负载计入容量，但其完整路径并不同时进行穷举优化。前沿检测增量更新；连接簇标签仍整体重算以处理分裂/合并。连续轨迹使用本仓库五次样条实现，不冒充原版 MINCO 或 RACER 的 B 样条优化器。

## ROS 节点与数据接口

每架机各有以下独立节点：

1. `localization_measurement_node.py`：仿真定位测量适配器，向真值里程计加入位置/速度噪声及协方差。可替换为发布标准 `nav_msgs/Odometry` 的外部定位系统；它不是 VIO/LIO/SLAM。
2. `state_estimator_node.py`：IMU 预测、测量校正、异常测量门限、正定协方差。机体系速度和世界系速度显式转换。
3. `pointcloud_mapping_node.py`：订阅实际 `PointCloud2` 和本机估计位姿，仅维护本机占据地图。
4. `decentralized_agent_node.py`：本机地图、融合规划、区域协商、候选池及预约协议。没有集中任务分配节点。
5. `view_executor_node.py`：独立核验预约、跟踪连续参考、执行偏航和到点观测；误差过大暂停轨迹时间。
6. `controller_node`：当前融合演示使用 PID，输出真实电机转速。原有其他控制器和规划器入口保留。

| 话题 | 类型/契约 | 接收方与用途 |
|---|---|---|
| `/drone_i/lidar/points` | `sensor_msgs/PointCloud2`，5611 个组织点 | 仅本机映射器；缺失返回按标定射线处理 |
| `/drone_i/imu` | `sensor_msgs/Imu` | 本机估计器 |
| `/drone_i/localization_measurement` | `nav_msgs/Odometry`，含协方差 | 本机估计器；外部定位接入口 |
| `/drone_i/estimated_odometry` | `nav_msgs/Odometry`，世界系位姿、机体系 twist | 本机地图、代理、执行器和控制器 |
| `/drone_i/observation` | `String` JSON，`annual.observation/2` | 本机体素增量；source、sensor_session、sequence、time、indices、values；不转发给同伴 |
| `/drone_i/map_bootstrap` | `String` JSON，session | 本机代理重启后请求映射器全量恢复；不是跨机地图共享 |
| `/drone_i/graph_delta` | `String` JSON，`annual.mrdtg/1` | 同伴交换历史节点、边、区域及边否决；source/session/base/sequence/full/records/changes |
| `/drone_i/peer_state` | `String` JSON | 时间、实例、epoch、位姿、可用性、区域序列/出价、图接收游标、双机事务、预约意图/ACK；不含原始地图 |
| `/drone_i/topology` | `String` JSON 完整图 | RViz/诊断，不作为跨机规划数据输入 |
| `/drone_i/view_command` | `String` JSON，单调 epoch | 本机执行器；取消或 `annual.trajectory/1` 五次多项式、段时长、偏航、预约 token |
| `/drone_i/view_execution` | `String` JSON | 本机执行状态、epoch、参考位置/速度/加速度、到点与停稳信息 |
| `/experiment/control` | `String` JSON | 仅实验暂停、规划通信分区、指定旧实例重启和实验结束，不分配任务 |

实际观测新增不足 30 个体素时，区域暂缓重访 600 s；这条服务结果通过图增量传给同伴并在本机重启后恢复。未知量较多的陈旧区域报告不能覆盖更充分观测后的完成/细分状态。

双机交换只纳入当前可竞价区域。接受提案后，参与方在断连期间保留锁并等待发起方的明确结果；已应用结果持续确认，防止迟到提交或确认丢失造成卡死。

图序列缺失时请求完整源快照；重复或乱序记录不会重复应用。新进程携带新的实例 ID，先取消旧执行、停稳、恢复本机地图，再以更大的 epoch 加入。旧实例消息被隔离。日志采用追加写入，统计按 `(无人机, 实例)` 累计，保留重启前的证据。

## 安全与恢复

- 常规路径需要固定成员的预约确认。确认方检查自己的路径/悬停位置、既有授权以及区域父子范围冲突。
- 断连不释放已确认预约。已提交路径持有原确认凭据；新路径只能在预先约定、留出间隔的本机 Voronoi 应急区域内执行，并继续避开保留预约。
- 当前故障试验切断的是规划状态/拓扑通信；独立的邻机状态安全输入仍存在。不能将此表述为所有链路同时失效下的验证。
- 撤销后保留预约，直到执行器确认对应 epoch、实际速度 <0.08 m/s 连续 0.5 s。进程重启也通过停稳确认隔离旧实例。
- 三维未知体素按障碍处理。常规水平余量 0.6 m、竖直余量 0.2 m；飞行高度限制在 0.7–3.1 m，避免从 3.5 m 高墙上方绕过迷宫。
- 同伴路径在本机路由中以 1.25 m 保留；预约协议采用保守的水平路径互斥。它没有实现允许交叉路线错时共享的完整时空多机轨迹优化。
- 短距离退让限距 0.65 m，固定偏航，限速 0.15 m/s、限加速度 0.2 m/s²；对机体包络与占据/未知体素做分离轴检查，再回到正常规划余量。没有清空未知地图来制造可通行区域。
- 新障碍到来先撤销失效路径，再核验原候选、预约备选或重新规划。平滑曲线碰撞时退回沿已验证折线的 C2 停点五次轨迹；日志明确记录使用的方法。

正常连续参考上限为 0.6 m/s、0.8 m/s²、2 m/s³、偏航 0.65 rad/s。这些是参考曲线约束，不是对有噪声刚体实际运动的数学保证；实际跟踪误差和接触必须另行检验。

## 运行与录制

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch annual_swarm decentralized_search.launch.py \
  headless:=false rviz:=true output_dir:=/tmp/fused_demo
```

默认 `search_fusion_3d.json`：24×20×4 m、九房间、错位门、回路、死胡同、1.8 m 瓶颈、低矮货箱、悬空货架。三机先实际旋转扫描，再开始未知环境探索。Gazebo 服务端使用 EGL 传感器渲染，原生 GUI 作为同一 transport partition 的客户端连接。

```bash
# 无 GUI 验收
python3 src/annual_swarm/scripts/decentralized_smoke.py \
  --output-dir /tmp/fused_acceptance --timeout 7200

# 原生 Gazebo + RViz 同屏录制，需要 Dockerfile.demo 的 Xvfb/Openbox/ffmpeg
python3 src/annual_swarm/scripts/record_decentralized_demo.py \
  --output-dir /tmp/fused_video --timeout 7200 --speed 12

# 独立核对完整录像同次运行的数据
python3 src/annual_swarm/scripts/audit_fusion_run.py /tmp/fused_video
```

暂停、规划通信中断、进程重启分别由 `pause_after`、`network_after`、`restart_after` 控制；设为 0 可关闭实验干预。`dynamic_obstacle:=true` 在满足距离/视角条件时驱动真实圆柱横穿正在执行的路径。上述参数是故障干预开关，不是 RACER/GVP 算法切换。

录像来源是实时 X11 捕获，后处理只做倍速及标题。RViz 的飞行轨迹来自 Gazebo 真值，地图显示 1.5 m 切片，拓扑与参考保留实际三维坐标。画面中的仿真时间与标题中的录屏倍速共同说明时间尺度。

## 任务轨迹质量及证据

任务层记录未知空间工作量、区域访问顺序、协商前后代价、容量/负载、转交及实例恢复；路径层记录长度、净空、转弯、离散平滑度及联合观测运动代价；连续参考层记录真实多项式导数极值、时长和回退方法；执行层记录 Gazebo 真值与参考位置的 RMS/P95/最大误差、实际里程、偏航、高度变化、机间距离及接触。

观测收益预测只计射线遇到的首个未知体素，属于局部信息代理量；实际新观测体素数另行记录。候选日志的历史字段 `gain_m2` 在三维运行中实际保存体素体积代理量，应按地图维度解释，不能当作平方米指标。

`summary.json` 中覆盖率是实际传感器已观测自由体素占真值自由体素的比例。95% 触发的是实验结束，规划代理不读取真值或这个比例。当前并没有基于完全异步网络的分布式全局终止证明。

`peer_payload_bytes` 只累计公共状态与图同步的 JSON 应用负载，不包含 DDS/IP 开销；`graph_payload_bytes` 是其中图同步部分。它们不能直接与历史理想平面传感器的时间或通信数字作性能优劣比较。严谨论文对照需要同一传感器、地图、随机种子和计算平台的重复实验。

`audit_fusion_run.py` 对完成记录检验覆盖率、零接触、间距、三维高度变化、真实偏航、图稀疏度、双机容量、连续轨迹约束、预约证据、五条备选上限，以及暂停、规划通信恢复、动态障碍响应和进程重启。测试通过和运行通过分别报告，不互相替代。
