# 现有日志与同次原生录像补证据

仅复算和剪辑现有候选10a数据；不把旧运行作为新交接提前量/观测完成代码的飞行验收。

| 运行 | 实测低速时间占比（<0.1m/s） | 交接提案→实际消费 | 移动期间规划重叠（机秒） |
|---|---:|---:|---:|
| main-no_fault-900-baseline | 60.2% | 0→0 | 缺请求边界，未推算 |
| main-no_fault-900-combined | 51.5% | 23→21 | 302.198 |
| main-no_fault-901-baseline | 61.5% | 0→0 | 缺请求边界，未推算 |
| main-no_fault-901-combined | 52.1% | 16→14 | 439.657 |
| main-no_fault-902-baseline | 64.7% | 0→0 | 缺请求边界，未推算 |
| main-no_fault-902-combined | 50.7% | 23→20 | 340.762 |
| main-no_fault-903-baseline | 60.9% | 0→0 | 缺请求边界，未推算 |
| main-no_fault-903-combined | 52.4% | 18→16 | 320.100 |
| main-no_fault-904-baseline | 60.7% | 0→0 | 缺请求边界，未推算 |
| main-no_fault-904-combined | 53.4% | 39→31 | 348.114 |

低速基于采样真实位姿差分；状态原因和低速区间按仿真时间对齐。`execution-timeline.csv`保存逐机时间轴，`diagnostics.json`保存驻留分解、交接取消原因、请求/飞行重叠及输入哈希。转向/加减速期间的低速不能全部算作等待规划。

| 原速连续片段 | 仿真时间范围（秒） | 视频时间范围（秒） |
|---|---:|---:|
| [moving-handoff](moving-handoff.mp4) | 32.800–36.800 | 46.035–61.968 |
| [dynamic-backup](dynamic-backup.mp4) | 196.601–211.601 | 297.998–332.337 |
| [necessary-transit](necessary-transit.mp4) | 174.405–186.001 | 263.228–291.308 |
| [low-yield-exit](low-yield-exit.mp4) | 279.602–283.602 | 434.914–452.057 |
| [evidence-reactivation](evidence-reactivation.mp4) | 313.561–317.561 | 493.664–510.708 |
| [tail-coordination](tail-coordination.mp4) | 530.401–546.000 | 853.565–888.537 |
| [tail-90-to-95](tail-90-to-95.mp4) | 427.001–568.501 | 683.021–923.600 |

同次源视频、精确事件、服务窗口及哈希见 `evidence-index.json`。剪辑为1倍速、内部无跳切；仿真事件通过单调墙钟接收记录定位，截取前后各保留5秒，画面HUD用于复核。

在线重激活原因与离线实测收益分开记录；动态备选本次执行只新增1个自由体素，不把后继服务收益归给备选。

表中仿真范围使用绝对仿真时钟；画面HUD的 `t` 是任务耗时，需减去任务开始时刻 16.001 秒。
