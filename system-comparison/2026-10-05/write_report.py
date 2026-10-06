"""Write the Chinese comparison report directly from verified result artifacts."""
import json
from pathlib import Path
p=Path(__file__).resolve().parent
rows=json.loads((p/'comparison.json').read_text())
checks=json.loads((p/'validation.json').read_text())
names=['Annual（本轮新源码）','RACER','GVP-MREP']
lines=['# 当前 Annual 与 RACER / GVP 的同条件适配对照','',
'日期：2026-10-05。仅做一组 seed=900、每套系统一次，共3次新运行；未启动之前暂缓的70项正式矩阵。', '',
'本次三套系统均达到95%外部自由覆盖，无接触或几何碰撞记录。扣除统一20秒初始化后，Annual用时96.654s，RACER 18.907s，GVP 36.298s；当前Annual在这张小图上的持续探索效率仍明显落后。单次结果不支持统计显著性或普遍优劣结论。', '',
'三套仓库自己的建图、任务选择、协作、路径规划及轨迹采样实际运行，接入相同Gazebo物理无人机、传感器、公共参考整形器和PID。RACER/GVP的ROS1传输及既有适配保留。本表没有复用2026-09-28的旧成绩，也没有使用分配器替身。', '',
'| 系统 | 覆盖率 | T95含初始化（s） | 探索耗时（s） | T90→T95（s） | 探索阶段总航程（m） | 低速占比 |',
'|---|---:|---:|---:|---:|---:|---:|']
for name,row in zip(names,rows):
 r=row['result'];lines.append(f"| {name} | {100*r['coverage']:.3f}% | {r['t95']:.3f} | {row['exploration_t95_s']:.3f} | {row['tail_90_to_95_s']:.3f} | {row['mission_total_distance_m']:.3f} | {100*row['low_speed_fraction']:.2f}% |")
lines+=['','探索耗时=T95−20s；航程和低速同样只统计t≥20s。低速定义为真实Gazebo线速度范数<0.1m/s，按采样区间时长加权。位姿差分的独立低速口径也保存，三组分别约49.54%、4.18%、31.97%。有效时间区间覆盖率分别超过99.96%，没有>0.6秒位姿缺口。覆盖率按约1.02仿真秒采样，T90/T95存在相同采样粒度，不能解释毫秒差异。','',
'![覆盖、耗时、航程与低速对照](comparison.png)','',
'![真实飞行轨迹](trajectories.png)','',
'轨迹为XY投影，右侧矮柜可以从上方飞越；碰撞检查使用完整三维机体，不能用二维图上的交叉判定碰撞。','',
'## 条件与适配边界','',
'- 地图12×12×3m，中央带门隔墙、左侧隔墙和右侧矮柜；2机，起点[-4,-3,1.2]与[-4,3,1.2]。地图及三次生成的世界SHA256一致。',
'- 相同Gazebo GPU LiDAR：水平/垂直各120°、181×31束、4.5m量程、5Hz、距离噪声σ=0.005m；观测图实际积分约2.5Hz。输入估计里程计记录均50Hz，激光记录均约5Hz。',
'- 相同噪声定位与IMU状态估计，相同0.64×0.64×0.16m物理机体、PID电机控制；本测试不对比完整SLAM。',
'- 公共执行参考约束：三维速度≤0.6m/s、加速度≤0.8m/s²、偏航速度≤0.65rad/s。整形器只处理运动参考，不访问环境地图或选择任务。',
'- 公共起飞/扫描持续20仿真秒，20s之前不接受探索运动。各系统内部准备过程仍是各自原生实现；Annual可在初始化末期准备提案，ROS1两套探索由20s触发。不能声称三套内部规划同时起步。',
'- 外部自由覆盖分母12,063，0.3m体素，按真值中几何完全自由体素计；真值仅供外部评估，不发送给规划器。目标95%、上限300仿真秒。',
'- 几何安全停止为物理机体OBB与障碍相交（保留5mm数值擦边容差），或参考跟踪误差>0.8m持续1仿真秒；接触与机间距离另外保存。',
'- Annual使用刚补齐P95/RTF交接窗口和真实帧终点完成的新冻结源码（272项）；135项安装资源一致。公共映射器仅补充积分帧来源/时间/位姿/版本字段，射线、滤波和地图更新保持原框架逻辑，三组使用同一映射器。',
'- RACER与GVP原生规划器二进制与2026-09-28相同；RACER HGrid/ACVRP/LKH/B-spline与GVP MR-DTG/Voronoi/轨迹优化保留。GVP的仿真时钟、有限FOV点云入口与机体自回波处理适配另有完整差异。','',
'ROS2公共仿真与Annual规划在一个6核/8GiB环境内；ROS1规划器在另一个6核/8GiB环境。算力分配、ROS运行时及传输不同，因此这里比较的是适配后的物理闭环，不做CPU/墙钟效率排名。公共整形会改变原生跟踪行为，实际速度也不完全相同。','',
'## 安全与速度','',
'| 系统 | 原始规划参考峰值（m/s） | 公共执行参考峰值（m/s） | 实测速度峰值（m/s） | 跟踪误差P95（m） | 最小采样机距（m） | 接触/几何碰撞 |',
'|---|---:|---:|---:|---:|---:|---|']
for name,row in zip(names,rows):
 r=row['result'];lines.append(f"| {name} | {r['raw_planner_max_speed_mps']:.3f} | {r['max_command_speed_mps']:.3f} | {r['max_measured_speed_mps']:.3f} | {r['tracking_error_p95_m']:.3f} | {r['min_separation_m']:.3f} | {r['contacts']} / {len(r['geometric_collision_events'])} |")
lines+=['','公共执行参考限速/加速度离线检查均通过；实际速度峰值存在超调，不能把本表解释为严格相同物理速度的算法排名。RACER原始参考超过0.6m/s，由公共整形器限速，原始输出没有隐藏。命令接收年龄P95分别0.031、0.038、0.033s。','',
'## 原始传感器离线复算','',
'从记录的原生激光点云和估计位姿重新执行公共建图，再计算最终覆盖；没有读取规划器地图或把已汇总覆盖当作复算输入。', '',
'| 系统 | 离线复算覆盖率 | 与在线结果差异（自由体素） | 每机积分帧数 |',
'|---|---:|---:|---:|']
for name,row in zip(names,rows):
 replay=row['coverage_replay'];difference=round(replay['difference']*12063);counts=replay['integrated_frames']
 lines.append(f"| {name} | {100*replay['recomputed']['coverage']:.3f}% | {difference:+d} | {counts[0]} / {counts[1]} |")
lines+=['','离线结果均超过95%。复算与在线存在1/7/6个边界体素差异（最大0.058个百分点），不声明完全逐体素一致；回调接收顺序与已记录位姿流的时序会影响边界射线/机体扫掠。积分帧数与在线逐机计数一致，压缩原始流完整解码。原始成绩保持在线固定口径，复算作为独立检查。','',
'## Annual 当前耗时构成','',
'新Annual完成53个规划请求，墙钟P50=0.692s、P95=1.484s、最大3.754s；5次运动交接全部实际消费，19次终点完成有真实积分帧凭据。原生参考/授权审计捕获29条曲线命令并通过；其范围是公共整形前的原生执行参考，不能替代公共整形后真实飞行安全记录。', '',
'按原生执行状态与真实位姿低速区间交叉统计（机秒，两机时间之和），低速主要出现在任务等待35.311、预约等待17.201、正常执行/转向加减速29.082、终点观测6.599、规划等待5.000、通行补图2.575。这个分解使用位姿差分与原生状态，因此与主表真实速度口径有约0.12个百分点差异，不把状态重叠强行修正成同一个数。', '',
'规划请求区间并集36.448机秒，其中17.626机秒与实际移动重叠；累计规划耗时不能直接当作任务延误。22次探索服务中4次在线团队新增估计为零，必要通行两次单列；这是在线回执口径，不能当作离线全局重复探索率。', '',
'这次结果支持优先排查任务/预约等待、视点行程和持续观测效率；单独补齐P95提前量及终点帧判据尚不能保证全程提速。RACER最终双机自由格集合交集占团队自由集合约40.0%，仍完成最快，所以不能仅按终态集合重叠认定无效重复探索。','',
'## 证据与复现','',
'- [固定协议](protocol.json)、[新框架冻结哈希](frozen-sha256.json)、[本轮框架差异](harness-adaptation.patch)、[GVP既有适配差异](gvp-adaptation.patch)。',
'- [Annual源码冻结清单](annual-source-manifest.json)、[安装核对](annual-install-verification.json)、[ROS1源码清单](ros1-source-manifest.json)、[源码核对](ros1-source-verification.json)、[环境与二进制](environment.json)。',
'- [机器可读结果](comparison.json)、[自动核对](validation.json)、[Annual原生诊断](trial-annual-900/native-diagnostics.json)、[原生授权核对](trial-annual-900/native-protocol-audit.json)。',
'- 每个`trial-*-900`目录保存原始/执行指令、真实轨迹、覆盖时间序列、完整压缩激光/位姿/时钟输入、仿真/规划器日志、传输计数与精确归属进程清理记录。',
'- [对照SVG](comparison.svg)、[轨迹SVG](trajectories.svg)已生成并检查图面。离线分析与报告生成脚本另列哈希，属于飞行结束后的只读工具。','',
'运行入口为`run.py`，使用本轮两个独立容器和挂载。新运行应使用新目录名，例如`python3 run.py racer repeat-racer-900 300`；seed实际固定为900，目录名不会改变种子。`replay_coverage.py`用于离线复算，`analyze.py`输出指标与图，`write_report.py`由结果生成本报告。', '',
'全部运行结束后，归属进程清理为空。原2026-09-28成绩和冻结框架保留；新结果不与原主图三机五种子数据拼接。完整正式恢复/专项/消融与多种子统计仍暂缓。','']
(p/'REPORT.md').write_text('\n'.join(lines))
print('report written',p/'REPORT.md')
