# 复现入口

本次正式输出位于 trial-annual-900、trial-racer-900、trial-gvp-900；smoke-* 为不计分预检。环境已停止，镜像和专用容器保留。

从工作区根目录执行：

```sh
colima start --profile annual-fusion --activate=false
colima start --profile racer-gvp --activate=false
docker --context colima-annual-fusion start comparison-ros2-0928
docker --context colima-racer-gvp start comparison-ros1-0928
python3 system-comparison/2026-09-28/run.py annual repeat-annual-900 300
python3 system-comparison/2026-09-28/run.py racer repeat-racer-900 300
python3 system-comparison/2026-09-28/run.py gvp repeat-gvp-900 300
```

逐组等待结束，不并行执行。输出目录必须是新目录。种子固定为 900；修改输出名不会改种子。更换种子须同时调整场景、定位随机源和协议，建立新实验批次。

依赖本机当前的工作区绝对挂载路径、两个已构建镜像及 GVP 副本的编译产物。环境二进制指纹见 environment.json，冻结配置指纹见 frozen-sha256.json。

prepare.py 是早期预检脚手架；复现正式实验直接使用已冻结 launch/XML/JSON 与源码副本，不运行该脚手架。report.py 默认汇总本次 trial-*-900，重复批次应使用对应的新输出目录。

传感器流为 gzip JSONL：clock、odom（估计值）、cloud（原始点云字段，data 为 base64；compressed=true 时先 zlib 解压）。commands.jsonl 记录整形前轨迹采样；executed-commands.jsonl 记录公共约束后的执行参考；trajectory.csv 是仿真真值飞行轨迹，仅用于评价。
