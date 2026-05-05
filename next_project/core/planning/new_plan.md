# 路径规划模块改进方案 (new_plan.md)

> 本文档基于 `rrt_star.py` 与 `replanner.py` 文件尾部注释中给出的修改意见整理，给出针对 RRT* 渐近最优规划器与滑动窗口在线重规划调度器的改进方向、技术方案与落地步骤。

---

## 一、目标与范围

- **覆盖文件**
  - `next_project/core/planning/rrt_star.py`：渐近最优 RRT* 离线/全局规划器。
  - `next_project/core/planning/replanner.py`：基于滑动窗口的在线重规划调度器。
- **核心目标**
  1. 提升 RRT* 在大规模/高维空间下的搜索效率与理论最优性。
  2. 增强终点连接的鲁棒性，并对最终路径做平滑处理以适配无人机动力学。
  3. 强化在线重规划的路径差异度量精度。
  4. 引入增量式规划与轨迹优化框架，提升在线响应性能与轨迹连续性。

---

## 二、RRT* 规划器改进 (`rrt_star.py`)

### 2.1 当前实现的不足

| # | 问题 | 现状定位 | 影响 |
|---|------|---------|------|
| R1 | `_nearest` 暴力遍历全部节点 | `rrt_star.py:128-137` | 复杂度 O(N)，N 大时显著拖慢 |
| R2 | rewire 同样暴力扫描所有节点 | `rrt_star.py:94-103` | 每次插入 O(N)，整体 O(N²) |
| R3 | rewire 半径固定 (`self.rewire_radius`) | `rrt_star.py:26-29` | 无法保证 RRT* 的渐近最优性 |
| R4 | 终点连接仅在主循环中被动检测 | `rrt_star.py:106-107` | 终点附近狭窄时易采样不到，连接失败 |
| R5 | 输出路径未做平滑 | `rrt_star.py:161-168` | 锯齿明显，不利于飞行控制跟踪 |

### 2.2 改进方案

#### (1) 引入 KD-Tree 加速最近邻与邻域查询 (R1, R2)

- 使用 `scipy.spatial.cKDTree` 或自实现增量式 KD-Tree。
- 由于 RRT* 不断插入节点，KD-Tree 需支持「批量重建 + 缓冲区」策略：
  - 维护一棵已建好的 KD-Tree + 一个待插入缓冲列表。
  - 缓冲区超过阈值（如 `√N`）时合并重建。
  - 查询时同时在主树与缓冲区中搜索。
- API 设计：

  ```python
  class _IncrementalKDTree:
      def __init__(self, dim: int, rebuild_threshold_ratio: float = 0.3): ...
      def insert(self, point: np.ndarray, idx: int) -> None: ...
      def nearest(self, point: np.ndarray) -> int: ...
      def query_radius(self, point: np.ndarray, radius: float) -> list[int]: ...
  ```

- 替换点：`_nearest()` 调用以及 rewire 阶段的 for 循环。

#### (2) 收缩半径策略 (R3)

按 RRT* 经典理论，rewire 半径应随节点数收缩：

\[ r(n) = \min\left(\gamma \cdot \left(\frac{\log n}{n}\right)^{1/d},\ \eta \right) \]

其中 `d=3`，`γ` 与空间体积、自由空间测度有关，`η` 为最大步长上界。

实现要点：

- 在 `__init__` 中保留 `rewire_radius` 作为 `η`（上界）。
- 新增 `gamma` 参数（默认估算自地图体积）。
- 主循环每次迭代根据当前 `len(nodes)` 计算 `r_n`，传入邻域查询。

#### (3) 主动终点连接 (R4)

在主循环结束后增加「显式连接尝试」：

```python
# 主循环结束后
goal_candidates = kdtree.query_radius(goal, radius=self.rewire_radius * 2.0)
goal_candidates.sort(key=lambda i: cost[i] + np.linalg.norm(goal - nodes[i]))
for cand in goal_candidates:
    if self._collision_free(nodes[cand], goal, grid):
        goal_node = len(nodes)
        nodes.append(goal)
        parent[goal_node] = cand
        cost[goal_node] = cost[cand] + np.linalg.norm(goal - nodes[cand])
        break
```

并保留 `goal_sample_rate` 偏置采样作为补充。

#### (4) 路径平滑 (R5)

提供两级平滑：

- **快速捷径裁剪 (Shortcut)**：随机选取路径上的两点，若直线 collision-free 则替换中间段。可在 `_extract_path` 之后立即执行。
- **B 样条拟合**：使用 `scipy.interpolate.splprep / splev`，按弧长重采样，输出可由控制器直接跟踪的连续航点序列。
- 与基类 `Planner.smooth(path)` 接口对齐，保持调用方一致。

### 2.3 接口与配置变更

```python
class RRTStar(Planner):
    def __init__(
        self,
        max_iter: int = 4000,
        rewire_radius: float = 1.5,        # 作为 η 上界
        goal_sample_rate: float = 0.1,
        gamma: float | None = None,        # 收缩半径系数，None 时自动估算
        use_kdtree: bool = True,
        smooth_method: str = "bspline",    # "none" | "shortcut" | "bspline"
    ): ...
```

---

## 三、在线重规划改进 (`replanner.py`)

### 3.1 当前实现的不足

| # | 问题 | 现状定位 | 影响 |
|---|------|---------|------|
| P1 | `_path_deviation` 仅比较前 K 个航点 | `replanner.py:120-131` | 路径长度差异大时，后半段重大偏离会被遗漏 |
| P2 | 每次重规划都从头调用全局规划器 | `replanner.py:76` | 计算开销大，难以满足 0.4 s 周期 |
| P3 | 输出仅为离散航点，未做轨迹级优化 | `replanner.py:43-96` | 离散航点不满足无人机动力学约束，跟踪误差大 |

### 3.2 改进方案

#### (1) 全面的路径差异度量 (P1)

替换 `_path_deviation`，提供以下两种全局度量并支持配置选择：

- **Hausdorff 距离**：

  \[ d_H(A,B) = \max\bigl(\sup_{a\in A}\inf_{b\in B}\|a-b\|,\ \sup_{b\in B}\inf_{a\in A}\|a-b\|\bigr) \]

  对路径整体覆盖范围敏感，能捕获后半段偏离。可用 `scipy.spatial.distance.directed_hausdorff` 双向计算后取最大值。

- **动态时间规整 (DTW)**：

  对路径作弧长重采样后计算 DTW 距离，对路径形状差异更敏感、对采样密度不敏感。

- 接口：

  ```python
  WindowReplanner(..., deviation_metric: str = "hausdorff")
  # 可选 "max_pairwise" (旧行为) | "hausdorff" | "dtw"
  ```

#### (2) 增量式重规划：D* Lite / LPA* + 滑动窗口 (P2)

提出三档分层策略：

1. **全局参考层 — Informed RRT***：
   一次性生成高质量参考路径，作为局部规划的「先验解」。Informed RRT* 在已知初始解后，将采样限制在以起终点为焦点的椭球内，显著提升收敛速度。
2. **局部跟踪层 — 滑动窗口**：
   当前 `WindowReplanner` 的能力，在 horizon 内做局部修正。
3. **退化层 — D* Lite / LPA***：
   仅在「环境完全未知」或「全局参考路径被新观测大幅破坏」时触发。增量式特性允许仅更新受影响的节点代价，避免从零重建搜索树。

调度逻辑：

```
on each step:
    if 全局参考路径仍可行 (滑动窗口范围内无新增致命障碍):
        do 滑动窗口局部修正
    else if 局部修正失败 K 次 or 检测到大范围地图变更:
        触发 D* Lite 增量更新全局参考
    else:
        Informed RRT* 重新初始化全局参考
```

实现拆分建议：

- 新增 `core/planning/dstar_lite.py`，实现 `IncrementalPlanner` 协议。
- 新增 `core/planning/informed_rrt_star.py`，复用 `RRTStar` 的 KD-Tree 与平滑模块，仅替换采样策略为椭球内采样。
- `WindowReplanner` 增加成员 `global_planner` 与 `incremental_planner`，并在 `step()` 中按上述调度切换。

#### (3) MPC 轨迹优化层 (P3)

- 在 `WindowReplanner.step()` 输出航点之后，新增 MPC 优化阶段，将离散航点作为参考轨迹，求解动力学可行的连续轨迹（位置-速度-加速度三阶光滑）。
- 模型：四旋翼简化双积分/三阶积分模型 + 输入约束（最大速度、加速度、jerk）。
- 求解器：建议使用 `osqp` / `cvxpy` / `acados`（追求实时性能可选 `acados`）。
- 输出：时间参数化的轨迹点 `(t, p, v, a)`，可直接喂给底层 PID+SMC 混合控制器。
- 与现有 `simulations/formation_simulation.py` 的接入点：替换其中读取 `path` 的位置插值逻辑，改为读取 MPC 输出的 `(t, p, v, a)`。

---

## 四、落地里程碑

| 阶段 | 任务 | 涉及文件 | 状态 | 验收指标 |
|------|------|---------|------|---------|
| M1 | KD-Tree + 收缩半径 + 主动终点连接 | `rrt_star.py` | ✅ 已实现 | 在 `simulations/benchmark.py` 中规划耗时下降 ≥ 50%，成功率 ≥ 旧版 |
| M2 | 路径平滑（shortcut + B 样条） | `rrt_star.py`、`base.py` | ✅ 已实现 | 输出路径曲率上界下降，控制器跟踪 RMS 误差下降 |
| M3 | Hausdorff/DTW 路径差异度量 | `replanner.py` | ✅ 已实现 | 单元测试覆盖三类构造场景，全段偏离可被检出 |
| M4 | Informed RRT* + 滑动窗口分层 | `informed_rrt_star.py`、`replanner.py` | ✅ 已实现 | 在线重规划平均耗时 < `interval/2` |
| M5 | D* Lite 增量式退化层 | `dstar_lite.py` | ✅ 已实现 | 大范围地图变更场景下，重规划耗时较 RRT* 下降 ≥ 70% |
| M6 | MPC 轨迹优化层 | `mpc_tracker.py`、`obstacle_scenario.py` | ✅ 已实现 | 轨迹满足速度/加速度约束，跟踪误差进一步下降 |
| M7 | 真正的 Hybrid A*（连续航向 + Dubins/RS 解析扩展） | `astar.py` | ⏳ 待启动 | 路径满足最小转弯半径 R = v²/a_max，固定翼场景可用 |
| M8 | SDF-aware planning 全面替代栅格化预筛 | `obstacles.py`、`obstacle_scenario.py` | ✅ 已实现（`SDFAwareGrid` 包装器） | 半径 < 栅格分辨率的薄障碍物零漏检 |
| M9 | RRT* rewire 子树代价递归传播 | `rrt_star.py`、`informed_rrt_star.py` | ✅ 已实现（`_propagate_cost_subtree`） | 路径代价随节点数单调下降 |
| M10 | DStarLite 堆/集合一致性 + 内存清理 | `dstar_lite.py` | ✅ 已实现（版本号机制 + `_gc_heap`） | 长时间运行堆条目数受控 |

---

## 五、测试与验证

- **单元测试**（建议引入 `pytest`）
  - `test_rrt_star_kdtree.py`：对比暴力 vs KD-Tree 的最近邻/邻域查询结果一致。
  - `test_rrt_star_optimality.py`：在简单二维障碍场景下，节点数增加时路径代价单调下降。
  - `test_path_deviation.py`：构造前段一致后段偏离的两条路径，验证 Hausdorff/DTW 能检出而 max-pairwise 漏检。
- **回归测试**
  - 在 `simulations/benchmark.py` 增加新指标列：规划耗时均值/方差、路径长度、最大曲率、跟踪 RMS。
  - 保留旧版 RRT* 实现（以 `legacy=True` 开关）以便横向对比。
- **C++ 同步**
  - 任一改进合入 Python 主线后，需在 `next_project/CPP重构/` 内同步实现，保证两侧算法一致（参见根 `CLAUDE.md`「AI 使用指引」）。

---

## 六、风险与回退

- **KD-Tree 增量重建抖动**：若插入热点过密导致频繁重建，可退化为「每 √N 次插入重建一次」的固定策略。
- **Informed RRT* 椭球退化**：当初始解极差时，椭球接近全空间，无加速。需要在多次迭代后重新计算 `c_best` 并收缩椭球。
- **MPC 实时性不足**：若求解超时，回退至直接使用平滑后的航点 + 三次样条时间参数化，保证控制器有可用参考。
- **D* Lite 内存占用**：长时间运行可能积累过期节点，需要定期清理 horizon 之外的状态。

---

## 七、对应注释来源

- `rrt_star.py:169-179`：KD-Tree 加速、rewire 优化、收缩半径、主动终点连接、路径平滑。
- `replanner.py:128`、`replanner.py:136-144`：Hausdorff/DTW 路径差异度量、D* Lite/LPA* 增量式重规划、Informed RRT* + 滑动窗口分层、MPC 轨迹优化。
