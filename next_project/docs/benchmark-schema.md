# Benchmark / Simulation 结果 Schema

> 跨 Python / C++ / Web 三条运行线的统一结果格式。本文件与
> `schemas/benchmark_result.schema.json`、`schemas/sim_result.schema.json`
> 保持同步。当前版本：**1.0.0**。

## 设计目标

1. **字段不漂移**：三条运行线写出的 JSON 必须能被同一份 schema 校验通过；
2. **跨线可对比**：用 `scripts/compare_results.py` 即可逐字段比对差异；
3. **向后兼容**：所有非必填字段均设为可选，新增字段不破坏旧消费者。

## 单位与坐标系

- 距离：**米 (m)**
- 时间：**秒 (s)**
- 坐标：**ENU**（X-East, Y-North, Z-Up）
- 角度：弧度 (rad)

## sim_result.json

单次场景仿真的标准化输出。Python 侧由 `core.result_schema.build_sim_result_payload`
构造，落盘路径为 `outputs/<preset>/<timestamp>/sim_result.json`。

### 顶层必填字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `schema_version` | string (SemVer) | 当前固定为 `1.0.0`；schema 不兼容变更时升大版本号 |
| `preset` | string | `config.py` / 等价 C++ 配置中的预设名 |
| `runtime_engine` | enum (`python` / `cpp` / `web`) | 产生该结果的运行线 |
| `metrics` | object | 见下表 |
| `completed_waypoint_count` | integer ≥ 0 | 实际通过的航点数 |
| `summary` | object | 标量摘要，便于 CLI/Web 直接展示 |

### `metrics`

每架从机的跟踪误差统计，**长度等于 `num_followers`**。

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `mean[i]` | m | 第 i 架从机的平均误差 |
| `max[i]`  | m | 第 i 架从机的最大误差 |
| `final[i]` | m | 第 i 架从机的最终误差 |

### `summary`

| 字段 | 单位 | 必填 | 说明 |
| --- | --- | --- | --- |
| `mean_error_overall` | m | ✓ | `metrics.mean` 的均值 |
| `max_error_overall` | m | ✓ | `metrics.max` 的最大值 |
| `final_error_overall` | m | ✓ | `metrics.final` 的均值 |
| `collision_count` | - | 可选 | 碰撞事件计数 |
| `replan_count` | - | 可选 | 重规划事件计数 |
| `fault_count` | - | 可选 | 故障事件计数 |

### 可选字段

| 字段 | 说明 |
| --- | --- |
| `engine_version` | git 短 hash 或语义版本 |
| `generated_at` | ISO-8601 UTC 时间戳 |
| `runtime_s` | 单次仿真耗时（秒） |
| `config_snapshot` | `SimulationConfig` 字典快照 |
| `trajectories` | `time` / `leader` / `followers` 时序数组（默认不输出，文件较大） |
| `planned_path` / `executed_path` | 规划与执行轨迹 [N×3] |
| `replan_events` / `collision_log` / `fault_log` | 事件日志 |
| `sensor_logs` | 距离传感器日志 |
| `safety_metrics` | `min_inter_drone_distance` / `downwash_hits` |
| `risk_report` | `core.risk_report.build_risk_report` 输出 |

## benchmark_result.json

多次重复实验的聚合统计。Python 侧由 `simulations.benchmark.run_benchmark`
生成，落盘路径默认为 `outputs/benchmark_default/<timestamp>/benchmark_results.json`。

### 顶层必填字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `schema_version` | string | `1.0.0` |
| `preset` | string | 预设名 |
| `runtime_engine` | enum | `python` / `cpp` / `web` |
| `summary` | object | 见下表 |
| `records` | array | 每次重复实验的原始指标 |

### `summary`

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `runs` | - | 重复次数 |
| `runtime_mean_s` / `runtime_std_s` | s | 单次仿真耗时统计 |
| `mean_error_mean[i]` / `mean_error_std[i]` | m | 第 i 架从机平均误差的跨次均/方差 |
| `max_error_mean[i]` / `max_error_std[i]` | m | 第 i 架从机最大误差的跨次均/方差 |
| `worst_case_max_error` | m | 所有重复实验中的最大误差 |
| `all_runs_pass_0_3m` | bool | 是否全部满足 < 0.3 m 阈值 |

### `records[i]`

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `run_index` | ✓ | 0-based 索引 |
| `runtime_s` | ✓ | 该次仿真耗时 |
| `mean_error` / `max_error` / `final_error` | ✓ | 长度等于 `num_followers` |
| `completed_waypoint_count` | ✓ | 通过的航点数 |
| `seed` | 可选 | 该次实验主随机种子 |

## 校验

Python 端：

```python
from core.result_schema import validate
import json
payload = json.loads(open("outputs/basic/<ts>/sim_result.json", encoding="utf-8").read())
validate(payload, "sim_result", strict=True)  # 不通过会抛 ValueError
```

命令行（需 `pip install jsonschema`）：

```bash
python -m core.result_schema sim_result outputs/basic/<ts>/sim_result.json
```

未安装 `jsonschema` 时，会回退到内置的轻量校验（覆盖 `type` / `required` / `enum`）。

## 跨线对比

```bash
python scripts/compare_results.py \
    --py  outputs/basic/<ts>/sim_result.json \
    --cpp outputs/basic_cpp/<ts>/sim_result.json \
    --rel-tol 0.05 --abs-tol 0.02 \
    --report outputs/_compare/basic.md
```

判定规则：

- 标量误差字段：`|a-b| ≤ abs_tol` 或相对误差 `≤ rel_tol`；
- 计数字段（`collision_count` 等）：必须严格相等；
- 任一不通过：脚本以非零状态码退出，便于 CI 集成。

## 版本演进规则

- 1.0.x：仅做向后兼容的字段新增/文档修订，不改类型；
- 1.x.0：新增可选字段、放宽约束；
- 2.0.0：必填字段重命名/类型变更等不兼容修改时升级。
