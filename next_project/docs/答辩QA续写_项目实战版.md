# 答辩QA续写：项目实战版

> 本文是《完整答辩攻防手册》和项目内《答辩QA预演.md》之后的补充版，不覆盖原有文档。重点回答老师可能继续追问的工程实战问题：安全门禁、Python/C++ 对齐、在线重规划边界、动态回放、现场演示和不足说明。

---

## 1. 开场定位

### Q1：这份补充版和原来的 QA 有什么区别？

**答：**

原来的 QA 主要覆盖项目整体、算法原理和常见答辩问题。这份补充版更偏实战追问：如果老师看了代码、看了动态回放、或者问“你这个安全机制到底在哪里生效”，就用这里的回答。

一句话说：原 QA 讲“系统有什么”，这份补充版讲“系统如何兜底、如何对齐、如何证明”。

### Q2：现在项目安全链路的核心变化是什么？

**答：**

现在不只是“规划器生成路径”，而是增加了接收和执行两级安全门禁：

1. Python 在线重规划接收新路径前，会做连续 SDF clearance 检查。
2. 如果 FIRI 精修后仍不达标，就尝试 fallback 重新规划。
3. 如果 fallback 也失败，就拒绝这条新路径，不把它写进 `local_path`。
4. C++ 侧也在候选路径替换 `active_path` 前做连续 SDF clearance 验收；动态回放还会在执行推进前做连续胶囊体检查，发现下一小段不安全就阻止前进，并记录 `clearance_blocked`。

这样文档里说的“安全机制”不再只是规划阶段的理想约束，而是落到了在线接收和执行推进两个关口。

---

## 2. 在线重规划安全门禁

### Q3：为什么只检查航点不够？

**答：**

因为障碍物可能位于两个航点之间。只检查离散航点会出现“端点安全、连线穿障”的情况。

所以项目里用的是连续线段采样检查：对路径每一段按较小间距采样，查询障碍物 SDF，取最小 clearance。如果任意采样点距离障碍物小于要求，就认为整段不安全。

### Q4：Python 侧新增的安全门禁在哪里？

**答：**

在 `simulations/obstacle_scenario.py::_accept_online_path`。

它的输入是在线重规划得到的 `new_path`，输出要么是可以接收的安全路径，要么是 `None`。

核心逻辑是：

```text
new_path
  -> 拼接回当前任务航点
  -> FIRI refine
  -> 连续 SDF clearance 检查
  -> 不合格则 fallback 重新规划
  -> fallback 仍不合格则拒绝接收
```

### Q5：为什么要在接收新路径时再检查一次？规划器不是已经避障了吗？

**答：**

规划器的避障通常发生在栅格或局部窗口里，可能受到分辨率、障碍物薄厚、路径平滑、FIRI 失败和动态环境变化影响。

接收门禁是最后一道“路径级验收”。它不相信路径来源，只看最终路径本身是否满足连续 clearance。这样即使 A*、D* Lite、GNN Danger 或 FIRI 任一环节产生了边界情况，也不会直接进入执行路径。

### Q6：FIRI 精修失败怎么办？

**答：**

FIRI 是增强项，不是唯一安全保证。Python 侧现在的顺序是：

1. 先尝试 FIRI 精修。
2. 精修后达标就接收。
3. 精修失败或精修后仍不达标，就检查原始路径。
4. 原始路径也不达标，就调用 fallback 重新规划。
5. fallback 仍失败，拒绝新路径。

答辩时可以强调：FIRI 负责“尽量优化路径”，接收门禁负责“不能让不安全路径进入主执行线”。

### Q7：`plan_clearance_extra` 在门禁里有没有生效？

**答：**

有。接收门禁现在使用：

```text
min_clearance = max(
    collision_margin + 0.03,
    safety_margin + plan_clearance_extra
)
```

这保证了配置里“规划时额外推远距离”不仅用于规划器和 FIRI，也用于在线新路径的最终验收。

### Q8：如果新路径不安全，系统会不会原地卡死？

**答：**

它不会盲目沿不安全路径走。具体策略是：

- 普通重规划失败时，保留已有执行路径，不用坏路径覆盖旧路径。
- 如果是任务航点切换导致的强制重规划，且没有安全路径，就把局部路径收缩为当前位置，避免继续向危险方向推进。
- 事件日志里会记录 `mode: clearance_blocked` 或 `mode: clearance_fallback`，方便回放和答辩解释。

这是一种保守策略：宁可暂停或继续用旧安全路径，也不执行已知不安全的新路径。

---

## 3. Python 与 C++ 对齐

### Q9：Python 主线和 C++ 侧现在安全机制是否一致？

**答：**

目标一致，但实现位置不同：

| 层次 | Python 主线 | C++ 动态回放 |
|---|---|---|
| 路径生成 | A*/D* Lite/Hybrid A*/GNN/FIRI | A*/D* Lite/Hybrid A* |
| 额外安全距离 | `safety_margin + plan_clearance_extra` | `planning_margin()` |
| 路径接收 | `_accept_online_path` 连续 SDF clearance 门禁 | `path_is_clearance_safe` 连续 SDF clearance 门禁 |
| 执行兜底 | 不安全新路径不覆盖 `local_path` | 不安全候选不覆盖 `active_path`；动态回放再由 `safe_advance_along_path` 阻止危险推进 |
| 日志 | `replan_events` | `clearance_blocked` 写入 replay frame |

所以不是逐行完全一样，而是在“新路径不能未经连续安全检查就执行”这个安全语义上对齐。

### Q10：C++ 现在和 Python 的接收门禁怎么对齐？

**答：**

Python 在 `_accept_online_path()` 里对候选路径做连续 SDF clearance 验收。C++ 现在也有同层级门禁：

- 普通在线仿真：`ObstacleScenarioSimulation::path_is_clearance_safe(candidate_path, compute_clearance())`
- 动态回放：`DynamicScenarioRunner::path_is_clearance_safe(selected_path, accept_clearance)`

只有整条候选路径通过连续采样检查后，才允许替换当前执行路径。也就是说，Python 的 `local_path` 和 C++ 的 `active_path` 都不会被不安全新路径覆盖。

### Q11：C++ 里的“胶囊体检查”是什么意思？

**答：**

无人机从当前位置走到下一小步目标，不是一个点瞬移过去，而是扫过一段空间。胶囊体检查就是把这段运动轨迹看成“线段 + 安全半径”的体积，只要这个体积和障碍物有冲突，就认为下一步不安全。

答辩时可以说：这比只检查下一目标点更严格，因为它覆盖了运动过程中间区域。

### Q12：C++ 侧发现 clearance 不足时怎么处理？

**答：**

C++ 动态回放中，`safe_advance_along_path` 会先计算期望前进点。如果当前位置到期望点的胶囊体安全，就正常前进；如果不安全，就二分搜索可以前进的最大安全比例。

如果几乎不能前进，就保持当前位置，清空当前活跃路径，并触发可达重规划网格调整。当前帧会写入：

```json
"clearance_blocked": true
```

Web 回放会把它显示为“连续clearance不足，已阻止前进”。

### Q13：如果老师问“Python 侧为什么不是也叫胶囊体检查”怎么答？

**答：**

Python 侧和 C++ 路径接收层现在都做路径级连续 SDF clearance 检查，本质上是在验证路径线段附近是否满足安全距离。C++ 动态回放额外在执行推进时使用“胶囊体”描述一小段实际运动扫过的空间，这是比路径接收更靠后的执行兜底。

两者都不是只看离散航点，而是检查连续运动段。

---

## 4. 在线路径语义

### Q14：为什么在线重规划不能覆盖原始任务航点？

**答：**

原始 `task_waypoints` 表示任务目标，比如必须从起点到中间巡检点再到终点。在线 `replanned_waypoints` 只是当前局部绕障路径。

如果用局部路径覆盖任务航点，就会把“临时绕障点”误认为“任务目标”，导致任务语义被破坏。所以项目里保留两层：

- `task_waypoints`：任务层，不被在线重规划覆盖。
- `replanned_waypoints`：执行层，可以随环境变化更新。

### Q15：在线路径为什么要拼接回当前任务航点？

**答：**

窗口重规划通常只规划前方一小段。如果不拼接回当前任务航点，leader 可能走到窗口末端后停止，或者下一步不知道应该继续去哪里。

所以 `_extend_online_path_to_task` 会尝试把窗口路径尾部接回当前任务航点。接回去之后还要通过连续 clearance 检查，不能为了“到达目标”而强行穿障。

### Q16：强制重规划失败时，为什么有时局部路径设为当前位置？

**答：**

这是保守兜底。强制重规划通常发生在任务航点切换或局部路径语义变化时，如果此时没有安全新路径，继续沿旧路径可能指向错误任务，直接走向目标又可能穿障。

把局部路径设为当前位置，等价于先停住或低速保持，再等待下一次安全重规划。这比执行未验证路径更安全。

---

## 5. 现场演示追问

### Q17：如果演示中出现 `clearance_blocked`，是不是说明算法失败？

**答：**

不是。`clearance_blocked` 表示安全门禁生效了：系统发现下一运动段连续安全距离不足，所以阻止继续前进。

它说明系统没有盲目执行不安全路径。真正需要分析的是为什么局部可行路径没有及时生成，比如障碍物太近、通道宽度不足、规划分辨率偏粗、或者安全裕度设置过大。

### Q18：怎么向老师展示这不是碰撞，而是安全阻止？

**答：**

看两个信号：

1. `collision=false`，说明没有发生真实碰撞。
2. `clearance_blocked=true`，说明系统主动阻止了危险推进。

在 Web 动态回放里，时间轴会显示 clearance 阻塞事件，当前帧说明也会提示“连续clearance不足，已阻止前进”。

### Q19：如果老师要求你现场定位代码，怎么说？

**答：**

可以按这个顺序定位：

1. Python 接收门禁：`next_project/simulations/obstacle_scenario.py::_accept_online_path`
2. Python 线段采样：`_path_segment_clearance` 和 `_segment_is_safe`
3. Python fallback：`_plan_segment_fallback`
4. C++ 普通在线接收门禁：`next_project/cpp/src/obstacle_scenario.cpp::path_is_clearance_safe`
5. C++ 动态回放接收门禁：`next_project/cpp/include/dynamic_scenario.hpp::path_is_clearance_safe`
6. C++ 胶囊体执行：`next_project/cpp/include/dynamic_scenario.hpp::capsule_clear`
7. C++ 安全推进：`safe_advance_along_path`
8. C++ 回放字段：`next_project/cpp/src/dynamic_main.cpp` 输出 `clearance_blocked`
9. Web 显示：`next_project/web/dynamic_replay.html`

---

## 6. 可靠性与测试

### Q20：如何证明这次 Python/C++ 门禁不是只写了代码、没有验证？

**答：**

新增了回归测试：

```text
test_online_accept_path_rejects_unsafe_clearance_without_fallback
```

测试构造一条穿过球形障碍物的在线新路径，并模拟 FIRI 不修复、fallback 失败。期望结果是 `_accept_online_path` 返回 `None`，并记录 `mode: clearance_blocked` 事件。

这个测试锁住了 Python 最关键的失败分支：不安全路径不能被接收。C++ 侧新增静态同步检查，确认普通在线仿真和动态回放都存在 `path_is_clearance_safe` 接收门禁，并且只有通过门禁后才替换 `active_path`。

### Q21：这次跑了哪些测试？

**答：**

已跑：

```text
python -m pytest next_project/tests/test_obstacle_scenario.py -k "online_accept_path_rejects_unsafe_clearance_without_fallback or online_window_path_extends_to_task_goal or online_mode_preserves_task_waypoint_semantics" -q
```

结果：3 个通过。

也跑了 C++ 静态同步测试：

```text
python -m pytest next_project/tests/test_cpp_sync_static.py -q
```

结果：6 个通过。

### Q22：为什么只跑这些测试，不跑全量？

**答：**

这次改动集中在在线新路径接收门禁、任务航点语义和 Python/C++ 同步检查，所以优先跑了最相关的回归测试。

如果正式提交前还要进一步确认，可以补跑：

```text
python -m pytest next_project/tests/test_obstacle_scenario.py -q
python -m pytest next_project/tests -q
```

---

## 7. 工程边界与诚实回答

### Q23：这个安全机制能不能保证绝对不会碰撞？

**答：**

不能承诺绝对不会。它能保证的是：在当前 SDF 模型、采样间距、控制跟踪误差和障碍物模型成立的条件下，系统不会接收连续 clearance 明显不合格的新路径，并会在 C++ 执行推进时阻止不安全运动段。

真实系统还需要考虑传感器延迟、定位误差、动力学跟踪超调和执行器饱和，所以仍然需要硬件级安全策略和实机验证。

### Q24：这套方法最大的局限是什么？

**答：**

主要有四个：

1. 连续检查本质上还是采样，采样间距越小越安全，但计算量更大。
2. SDF 依赖障碍物模型，模型错误会影响判断。
3. 控制器跟踪路径时有误差，路径安全不等于真实轨迹一定完全贴合。
4. FIRI 是工程化版本，不是完整论文级 MVIE 反复优化。

但这些局限都被显式承认，并通过 fallback、执行门禁、回归测试和可视化日志降低风险。

### Q25：如果老师说“你这是补丁式安全，不是理论证明”，怎么答？

**答：**

可以承认：这是工程仿真系统，不是形式化安全证明系统。我的目标不是证明所有连续时间状态绝对安全，而是在多无人机仿真框架中建立多层防线：

- 栅格膨胀降低规划穿障概率。
- SDF-aware grid 处理薄障碍物。
- FIRI 精修提高路径 clearance。
- 在线接收门禁拒绝不合格新路径。
- C++ 执行胶囊体检查阻止危险推进。
- 回放日志暴露安全事件，便于复盘。

这是一套可运行、可测试、可解释的工程安全链路。

### Q26：如果老师问“你为什么现在才补 Python/C++ 门禁”，怎么答？

**答：**

原来 Python 侧已经有 SDF-aware grid、FIRI、fallback 和线段安全检查，但在线新路径接收处还缺少一个统一验收口。C++ 侧原来更偏执行推进前胶囊体兜底，也需要补上和 Python 同层级的“候选路径接收前整路径验收”。

这次补的是工程闭环：把已有能力串成最后的门禁，避免文档和实现不一致。

---

## 8. 现场短句库

### 安全机制一句话

项目不是只靠规划器避障，而是在路径生成、FIRI 精修、在线接收和执行推进四个位置都做安全约束。

### Python/C++ 对齐一句话

Python 和 C++ 现在都在新路径接收前做连续 SDF clearance 验收；C++ 动态回放额外保留执行推进前的胶囊体检查，形成“接收前验收 + 执行前刹车”的双门禁。

### FIRI 兜底一句话

FIRI 是优化器，不是安全唯一来源；如果 FIRI 没修好，门禁会继续检查原路径和 fallback，仍不安全就拒绝接收。

### 在线语义一句话

在线重规划只更新局部执行路径，不覆盖任务航点，这保证了绕障不会改变原任务目标。

### 局限说明一句话

当前系统是仿真验证和工程安全链路，不是实机级形式化证明；真实部署还需要传感器、定位、动力学误差和硬件急停的进一步验证。

---

## 9. 被追问时的推荐回答顺序

1. 先说现象：系统发现连续 clearance 不足。
2. 再说机制：Python/C++ 接收门禁拒绝不安全新路径，C++ 动态回放胶囊体检查阻止不安全下一步。
3. 再说证据：Python 事件日志里有 `mode: clearance_blocked`，C++ replay 里有 `clearance_blocked` 字段，测试覆盖不安全路径拒绝分支。
4. 最后说边界：这是工程安全兜底，不是绝对安全证明。

这个顺序最稳：先承认问题，再解释机制，再给证据，最后说明边界。
