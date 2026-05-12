# Python / C++ / Web 能力对齐矩阵

> 对齐基线：`config.py`、`core/`、`simulations/`、`cpp/include/`、`cpp/src/`、`web/server.py`。  
> 目的：把“文档说有”和“代码真有”放到同一张表里，作为短期 P0-3 的验收材料。

## 1. 总体结论

当前仓库是三条运行线：

- **Python 主线**：功能最完整，负责算法实验、场景仿真、风险报告、可视化和 benchmark。
- **C++ 部署/回放线**：覆盖核心动力学、控制、规划、在线重规划、动态回放和部分安全报告字段，是可运行子集。
- **Web 展示线**：调用 C++ `sim_dynamic_replay` 做动态回放，前端 follower 为示意代理，不等价于完整 Python 多机闭环。

重要边界：

- `task_waypoints` 是原始任务航点，`replanned_waypoints` 是在线局部路径，二者不能混写。
- 普通 C++ `sim_warehouse` 输出的 `outputs/warehouse/<timestamp>/sim_result.json` 可接 Python 风险报告链路；Web dynamic replay 仍是 leader-centric 回放面。
- 照片重建只生成静态 `maps/*.json` 障碍物地图，不生成任务航点，也不代表完整数字孪生。

## 2. 模块覆盖矩阵

| 功能点 | Python | C++ | Web | 说明 |
| --- | --- | --- | --- | --- |
| 基础动力学 / Rotor / Drone | 完整 | 完整 | 间接使用 C++ | Python `core/drone.py`，C++ `drone.hpp/.cpp` |
| PID / Backstepping / SMC | 完整 | 完整 | 间接使用 C++ | C++ 覆盖 `controller` 和 `smc` |
| 编队拓扑 / 队形切换 | 完整 | 完整 | 动态回放展示 | `topology` 两侧均有实现 |
| A* / Hybrid A* / Dijkstra | 完整 | 部分到完整 | 通过 C++ 回放 | C++ 主要服务障碍场景和动态回放 |
| RRT* / Informed RRT* | 完整 | 缺失/不作为当前子集 | 不适用 | 当前 C++ 未声明完整 RRT* 对齐 |
| D* Lite / WindowReplanner | 完整 | 完整子集 | 通过 C++ 回放 | C++ 保留任务航点层和局部重规划层 |
| ESDF/SDF-aware 规划 | 完整 | 部分 | 通过 C++ 回放 | C++ 有 occupancy/grid 与 ESDF 相关子集 |
| FIRI 走廊优化 | 工程化完整 | 工程化子集 | 通过 C++ 回放 | 两侧均为工程 FIRI-style，不是完整论文 MVIE 实现 |
| GNN Danger / 双模式调度 | 完整 | 可运行子集 | 通过 C++ 回放 | C++ 有 `GNNPlanner`、`DualModeScheduler`、visible graph |
| APF 避障 / 编队 APF | 完整 | 可运行子集 | 通过 C++ 回放 | C++ 明确 runtime fields 与 python-only fields |
| RangeSensor6 / 在线感知 | 完整 | 完整子集 | 通过 C++ 回放 | C++ 使用解析几何射线距离 |
| 故障注入/检测/重构 | 完整 | 可运行子集 | 回放字段展示 | C++ 有 fault log 和 topology reconfig |
| formation_safety | 完整 | 可运行子集 | 普通报告链路可展示 | C++ 输出 `min_inter_drone_distance`、`downwash_hits` |
| risk_report | 完整 | 输出兼容字段 | 不等价 | 普通 C++ obstacle scenario 可由 Python `core/risk_report.py` 处理 |
| benchmark_result schema | 完整 | 完整 | 不适用 | Python/C++ benchmark 均输出统一 JSON |
| sim_result schema | 完整 | 完整 | 最小合法包装 | Web 包装 C++ replay 结果，顶层字段符合 schema |
| Plotly/PNG 可视化 | 完整 | PNG/SVG 子集 | Web 交互展示 | 展示能力不是算法能力 |
| 照片重建 COLMAP/OpenMVS | Web 支线工具 | 不适用 | 实验性支线 | 只转换成静态地图 |

## 2.1 中期新增能力矩阵

| 功能点 | Python | C++ | Web | 说明 |
| --- | --- | --- | --- | --- |
| trajectory optimizer 指标 | 完整 | 不适用 | 不适用 | `TrajectoryResult` 输出 path length、speed、acceleration、mean/max jerk、jerk squared integral、snap squared integral |
| minimum-jerk / snap proxy 轨迹原型 | 完整原型 | 缺失/暂不同步 | 不适用 | Python 支持 `minimum_jerk` 与 `min_snap_proxy`，不是完整 GCOPTER/MINCO |
| 实验场景注册表 | 完整 | 不适用 | 不适用 | `experiments/scenario_registry.py` 覆盖 offline/online 代表场景 |
| 消融实验运行器 | 完整 | 不适用 | 不适用 | `experiments/run_ablation.py` 支持 baseline、no_esdf、no_gnn、no_apf、with_trajectory_optimizer 等变体 |
| 指标提取与报告 | 完整 | 不适用 | 不适用 | `metrics_extractor.py` + `report_writer.py` 输出 CSV/Markdown |
| DroneParams 标定骨架 | 完整原型 | 待同步 | 不适用 | `experiments/calibration/drone_params_calibration.py` 从预处理 CSV 输出候选 profile JSON |
| DroneParams profile 文档 | 完整 | 子集说明 | 不适用 | `docs/drone-params-profile.md` 明确 profile 边界，不宣称实测标定 |

## 3. 场景覆盖矩阵

| 预设/场景 | Python | C++ | Web | 回归用途 |
| --- | --- | --- | --- | --- |
| `basic` | `python main.py --preset basic` | `sim_main` | 不作为 Web 主场景 | 基础动力学/控制跨线 sanity check |
| `warehouse` | `python main.py --preset warehouse` | `sim_warehouse` / `get_config("warehouse")` | 可作为 base_config | 障碍场景、formation_safety、风险报告 |
| `school_corridor` | 离线 A* | `get_config("school_corridor")` | 可作为 base_config | 窄通道任务航点保持 |
| `school_corridor_online` | 在线重规划 | `get_config("school_corridor_online")` | 推荐 Web 回放场景 | task/replanned 层级、clearance gate |
| `meeting_room` | 离线 A* | `get_config("meeting_room")` | 可作为 base_config | 精细室内障碍 |
| `meeting_room_online` | 在线重规划 | `get_config("meeting_room_online")` | 推荐 Web 回放场景 | 近障碍在线响应 |
| `laboratory` | 离线 A* | `get_config("laboratory")` | 可作为 base_config | 多层高度障碍 |
| `laboratory_online` | Hybrid A* 在线 | `get_config("laboratory_online")` | 推荐 Web 回放场景 | Hybrid A* + 传感器 |

## 4. 回归验收方式

短期 P0-3 的回归分两层：

1. 默认 CI 运行：
   - `python -m pytest tests/test_cpp_sync_static.py tests/test_result_schema.py tests/test_cross_line_regression.py`
   - 验证 C++ 关键语义仍在、schema 工具可用、本文档和测试契约存在。
2. 已构建 C++ 可执行文件时运行：
   - Windows PowerShell：`$env:RUN_CROSS_LINE_REGRESSION='1'; python -m pytest tests/test_cross_line_regression.py`
   - Linux/macOS：`RUN_CROSS_LINE_REGRESSION=1 python -m pytest tests/test_cross_line_regression.py`
   - 测试会执行 `basic` Python/C++ `sim_result.json` schema 校验和 benchmark schema 对比。

## 5. 中期入口判定

可以进入中期开发的最低条件：

- 本文档无“未知”项；未覆盖项明确标为“缺失/不适用/子集”。
- `test_cross_line_regression.py` 默认契约测试通过。
- 在至少一台已构建 C++ 的机器上，`RUN_CROSS_LINE_REGRESSION=1` 的 runtime 回归通过或形成有解释的偏差报告。
- Web dynamic replay 不再被描述成完整多机风险报告，只作为 leader-centric 动态回放入口。
