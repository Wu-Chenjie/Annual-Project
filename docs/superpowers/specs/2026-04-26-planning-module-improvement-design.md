# 路径规划模块改进设计文档

> 基于 `new_plan.md`，覆盖 RRT* 渐近最优规划器与滑动窗口在线重规划调度器的全面改进。

## 实施策略：方案 B（按文件依赖分组）

| 阶段 | 里程碑 | 涉及文件 |
|------|--------|---------|
| Phase 1 | M1+M2 | `rrt_star.py` |
| Phase 2 | M3 | `replanner.py` |
| Phase 3 | M4+M5 | 新增 `informed_rrt_star.py`、`dstar_lite.py`，改 `replanner.py`、`__init__.py` |
| Phase 4 | M6 | 新增 `mpc_tracker.py`，改 `replanner.py`、`obstacle_scenario.py`、`__init__.py`、`requirements.txt` |

---

## Phase 1：RRT* 加速与平滑 (M1+M2)

### 改动范围
单文件 `rrt_star.py`，保持外部接口兼容。

### 子任务

1. **`_IncrementalKDTree` 内部类**
   - 内部持有 `cKDTree` + 缓冲列表
   - 插入到缓冲，查询双路搜索
   - 缓冲超 `max(50, sqrt(N))` 时重建
   - 方法：`insert(point, idx)`、`nearest(point) -> idx`、`query_radius(point, radius) -> list[idx]`

2. **收缩半径策略**
   - 保留 `rewire_radius` 作为上界 `η`
   - 新增 `gamma` 参数，默认自动估算
   - 每轮迭代计算 `r_n = min(gamma * (log(n)/n)^(1/3), rewire_radius)`

3. **主动终点连接**
   - 主循环结束后 KD-Tree 查询 goal 周围候选节点
   - 按 cost + distance 排序，取首个 collision-free 连接

4. **路径平滑**
   - Shortcut：随机两点直线裁剪
   - B 样条：`scipy.interpolate.splprep/splev` 按弧长重采样
   - 通过 `smooth_method` 参数选择

5. **配置变更**
   - 新增参数：`gamma`, `use_kdtree`, `smooth_method`, `smooth_points`

### 验收指标
- benchmark 规划耗时下降 ≥ 50%，成功率 ≥ 旧版

---

## Phase 2：Hausdorff/DTW 路径差异度量 (M3)

### 改动范围
单文件 `replanner.py`。

### 子任务

1. **替换 `_path_deviation`**
   - 新增 `deviation_metric` 参数：`"max_pairwise"` | `"hausdorff"` | `"dtw"`
   - `hausdorff`：`scipy.spatial.distance.directed_hausdorff` 双向
   - `dtw`：纯 numpy DP 实现，弧长重采样 100 点
   - 旧行为 `max_pairwise` 保留

### 验收指标
- 构造三类场景，hausdorff/dtw 能检出全段偏离

---

## Phase 3：Informed RRT* + D* Lite 分层架构 (M4+M5)

### 改动范围
- 新增 `informed_rrt_star.py`：继承 `RRTStar`，替换采样策略为椭球 rejection sampling
- 新增 `dstar_lite.py`：增量式 LPA* 变体，`step()` 接口
- 修改 `replanner.py`：分层调度（local → incremental → global）
- 修改 `__init__.py`：导出新类

### 分层调度
```
on each step:
    local (sliding window) → incremental (D* Lite) → global (Informed RRT*)
```
- 新增构造参数：`global_planner`、`incremental_planner`、`local_fail_threshold`

### 验收指标
- 在线重规划平均耗时 < interval/2
- D* Lite 大范围变更场景耗时较 RRT* 下降 ≥ 70%

---

## Phase 4：MPC 轨迹优化层 (M6)

### 改动范围
- 新增 `mpc_tracker.py`：三阶积分模型 + QP 求解
- 修改 `replanner.py`：`step()` 输出接 MPC
- 修改 `obstacle_scenario.py`：控制器读取 MPC 轨迹 `(p, v, a)` 前馈
- 修改 `requirements.txt`：增加 `cvxpy`, `osqp`

### 模型
- 状态：`[p, v, a] ∈ R^9`，控制：`jerk ∈ R^3`
- 约束：`|v| <= 10`, `|a| <= 10`, `|j| <= 20`
- 求解器：cvxpy + OSQP，超时回退 B 样条

### 验收指标
- 轨迹满足动力学约束，跟踪 RMS 下降 ≥ 10%

---

## 风险与回退

| 风险 | 缓解 |
|------|------|
| KD-Tree 频繁重建抖动 | 退化为固定 √N 策略 |
| Informed RRT* 椭球退化 | 多次迭代后重新计算 c_best |
| MPC 求解超时 | 回退 B 样条 + 三次样条时间参数化 |
| D* Lite 内存积累 | 定期清理 horizon 外状态 |
