# 候选10续推进（2026-10-05）

按用户“无需完成那么多次仿真实验”的要求，本轮范围缩减为：统一复审已有9次完整运行、补跑一次904融合策略、更新主图五种子配对报告。恢复组、三张专项地图和六份消融不再于本轮运行，原80项协议保留为历史验收要求，不修改阈值或标记为已完成。

原6核、8GiB的 `annual-fusion` 虚拟机和 `annual-fusion-run` 容器已恢复。冻结候选10a的260项源码、133项安装资源与基线9b的203项源码、111项安装资源全部匹配；没有重新编译或变更飞行策略，容器启动时没有遗留ROS/Gazebo进程。

只读10e审核器已统一复审900–903四组及904基线，9次完整样本的原始飞行/覆盖检查与修订后的授权检查均通过。原901/902融合的冻结审核报错保持可查。904原中止样本缺少完整审核输入，继续按基础设施失败披露，不作成功样本。

新增外部续跑工具 `ros2_ws/experiments/tools/resume_todo_study.py`：只允许有已核对清理记录的基础设施中止样本创建独立重试；成功、算法失败和超时不重试。原矩阵、运行结果与原审核不覆盖，新的尝试登记在 `resume-attempts.json`，新汇总单独写入 `study-report-10e-resumed.json`，包含原尝试与重试的完整历史。临时报告视图复用冻结10e报告器和原门槛。续跑工具9项回归、既有审核/报告33项回归，共42项通过，涵盖冻结源码变更、未清理进程、失败重试限制和重试后安全失败不能被成功耗时掩盖。

904独立重试已完成，目录为 `artifacts/todo-execution/formal-10/infrastructure-retries/main-no_fault-904-combined/attempt-001/`。T95为546.500仿真秒，最终覆盖95.017836%，起飞后零接触，采样最小机间距1.613787m，31次实际运动中交接，全部已测规划请求墙钟P95为4.115335s。原冻结账本、独立飞行/覆盖及授权审核均通过，只读10e授权复审亦通过；进程清理记录 `remaining=[]`。

主图无故障阶段现有5组完整配对、10次成功运行，原中止尝试作为第11条历史记录保留。冻结10e报告器复核该组全部11项门槛通过；其余70次未运行，完整E01–E10仍不勾选。

| 种子 | 基线T95（仿真秒） | 融合T95（仿真秒） | 差值（融合−基线） |
|---|---:|---:|---:|
| 900 | 455.500 | 400.000 | −55.500 |
| 901 | 636.500 | 497.500 | −139.000 |
| 902 | 702.011 | 418.996 | −283.015 |
| 903 | 703.999 | 420.505 | −283.494 |
| 904 | 560.500 | 546.500 | −14.000 |

| 指标（五次运行中位数） | 基线 | 融合 | 改善 |
|---|---:|---:|---:|
| T95（仿真秒） | 636.500 | 420.505 | 降低33.9% |
| T90→T95（仿真秒） | 79.502 | 48.500 | 降低39.0% |
| 等待规划（机秒） | 391.799 | 145.067 | 降低63.0% |
| 团队总航程（米） | 299.668 | 261.436 | 降低12.8% |
| 低团队收益探索服务次数 | 77 | 34 | 降低 |
| 零团队收益探索服务次数 | 41 | 8 | 降低 |
| 低团队收益服务时长占比 | 41.47% | 28.65% | 降低 |

五组均改善，但904仅快14秒（2.5%），各种子的改善幅度不均衡。以上只证明这一冻结版本在当前主图无故障五种子阶段通过既定门槛，不替代恢复组、专项地图和消融，也不把必要通行补图计入无效探索。此阶段复用2026-09-28的9次完整冻结运行，904融合独立重试于2026-10-05完成；没有混用其他候选版本。

结果汇总后已停止本次恢复的容器及 `annual-fusion` 虚拟机，释放运行资源；冻结源码、安装产物、失败记录和本次结果均保留。

证据位于本地 `artifacts/todo-execution/`：

- `candidate-10a-resume-install-verification.json`、`baseline-9b-resume-install-verification.json`：冻结安装资源检查。
- `candidate-10-resume-tests.json`：42项检查结果、运行环境及续跑工具/测试的源码哈希。
- `formal-10/auditor-10e-recheck.json`、`formal-10/reaudit-10e-20261005.log`：全部原样本的统一复审，含中止样本缺失输入记录。
- `formal-10/resume-preflight-20261005.log`：续跑前主图阶段报告。
- `formal-10/resume-attempts.json`、`resume-progress.json`、`resume-stage-20261005.log`：独立重试、当前阶段及输出。
- `formal-10/study-report-10e-resumed.json`、`all-attempts-10e-resumed.csv`：独立续跑报告，70项待运行仍显式保留。

在既有容器内加载ROS与冻结候选10a安装环境后，以下命令仅处理主图无故障阶段；已有成功、失败和超时不会重复飞行。需要只刷新报告时加 `--report-only`。

```bash
python3 /workspace/ros2_ws/experiments/tools/resume_todo_study.py \
  --study-root /workspace/artifacts/todo-execution/formal-10 \
  --baseline-root /tmp/annual-todo-baseline-9b \
  --combined-root /tmp/annual-todo-candidate-10a \
  --harness-root /workspace/artifacts/todo-execution/study-tools-10b \
  --auditor-root /workspace/artifacts/todo-execution/auditor-10e
```
