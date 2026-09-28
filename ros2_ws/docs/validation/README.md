# 实测证据

早期策略已接入同一三维传感环境实测：2400.6 s 时覆盖 53.53%，未达到 95%；当前优化融合版为 1631.0 s 达到 95%。[原生录像、同口径对照及全部尝试披露](early-policy-3d/README.md)。这是旧策略的三维接口适配对照，不是原二维约 300 s 任务的复现。

最新三维融合探索的 [优化版原生视频、前后对照与审计](fusion-optimized/README.md) 包含复杂三维场景、规划通信中断、动态障碍和进程重启：95% 覆盖时间缩短 25.3%，总航程增加 37.1%。[原三维融合基线](fusion/README.md) 保留供复算。以下为 2026-09-22 首轮迁移的历史记录，口径和任务均不同。

以下是本次迁移在 Gazebo 中实际运行的记录。完整方法、口径、限制和测试结果见 [验收记录](../../VALIDATION.md)。

## Gazebo GUI 截图

![Gazebo 场景](gazebo-overview.png)

![三机编队](gazebo-formation-closeup.png)

## 动态恢复对照

下图根据真实里程计和状态记录绘制，不是 GUI 截图。三组各两次，六次均到达，起飞后接触均为 0。

![动态恢复对照](recovery-comparison.png)

[逐次结果](recovery-results.json) 的 summary 字段指向同目录的原始运行摘要。完整轨迹 CSV、候选快照和进程日志保存在本地 artifacts/controlled_comparison；重新运行 compare_recovery.py 可生成完整工件。
