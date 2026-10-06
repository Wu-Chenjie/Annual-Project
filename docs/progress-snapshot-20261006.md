# 2026-10-06 当前成果归档

本提交保存当前融合探索实现、交接计时与真实观测完成、服务时间成本、走廊预计等待、执行出价生命周期过滤、诊断/录像/恢复工具及相关回归测试。

## 成果入口

- [当前工程 TODO 与优化清单](../RACER_GVP_ENGINEERING_TODO.md)
- [10月6日开发配对、源码冻结与原始证据](../system-comparison/2026-10-06/README.md)
- [10月5日三系统对照](../system-comparison/2026-10-05/REPORT.md)、[效率复测](../system-comparison/2026-10-05/EFFICIENCY_REPORT.md)、[视点预览微基准](../system-comparison/2026-10-05/PREVIEW_OPTIMIZATION.md)
- [9月28日三系统对照](../system-comparison/2026-09-28/REPORT.md)
- [规划器优化可行性探针](../planner-optimization/2026-10-06-feasibility/README.md)
- [工程缺口补齐](../ros2_ws/docs/validation/gap-closure-20261005/README.md)

`system-comparison` 和 `planner-optimization` 从原工作区复制入库，保留源码归档、成功/失败样本、原始点云压缩流、真实轨迹、命令/状态/事件、汇总、图表和审核记录。重建用的 build/devel/install/log 目录与 Python 缓存排除；各运行本身的实验日志保留。实验文件禁用 Git 换行转换，以保留原有 SHA256 凭据。

历史冻结脚本、清单和日志中的原始绝对路径、容器路径、旧工作区结构及部分符号链接原样保存；它们描述当时环境，迁移复跑时需要重新映射路径。仓库原 `artifacts/` 的历史大型录像/运行目录继续按既有忽略规则留在本地，已入库审核索引与原有视频不改写成新的飞行结果。

## 本次提交前的本地检查

- `next_project` 非 slow Python 回归：155 passed，2 skipped，75 deselected。2项跳过为需显式启用的 Python/C++ 运行时对齐检查。
- 新改动定向回归：59 passed，4 skipped；ROS 相关检查因本机没有 rclpy 跳过。
- 本次没有新增 Gazebo 飞行、没有重跑正式70项矩阵。既有 ROS/Gazebo 冻结结果按各报告的原始范围保存。

这些检查不等于完整 ROS/Gazebo 重新验收，不继承历史版本的性能结论。
