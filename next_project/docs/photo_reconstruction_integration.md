# 照片建模接入方案

本文说明如何把室内照片重建出的三维模型接入 `next_project`，目标是让真实房间、走廊、实验室或仓储空间能够变成项目已有的 `maps/*.json` 障碍物地图，再进入 Python/C++ 避障、动态回放和风险报告链路。

## 1. 推荐总路线

当前项目已经有现成入口：

- `web/server.py` 的 `/api/maps/import-model` 可以上传 `.ply`、`.obj`、`.stl`。
- 后端会把模型顶点/三角面按 `voxel_size` 体素化。
- 输出会写入 `next_project/maps/<map_name>.json`。
- 地图格式仍是项目已有的 `bounds + obstacles`，可以直接被 `core/map_loader.py`、C++ `map_loader` 和 Web 动态回放使用。

所以照片建模不要直接接进规划器，应该先接成一条外部预处理流水线：

```text
室内照片
  -> SfM / MVS 重建
  -> mesh 或 dense point cloud (.ply/.obj/.stl)
  -> /api/maps/import-model
  -> maps/*.json
  -> obstacle_scenario / dynamic_replay / risk_report
```

这样做的好处是边界清楚：照片重建负责提供几何，项目仿真仍只消费已验证的障碍物地图格式。

## 2. COLMAP + OpenMVS 主链路

适合作为默认工程路线。COLMAP 负责 Structure-from-Motion 和相机位姿，OpenMVS 负责稠密点云、网格、贴图。它的稳定性、论文资料和命令行自动化都比较适合写进项目论文和答辩材料。

### 2.1 目录约定

建议新增工作目录，不把原始照片放进 `maps/`：

```text
next_project/artifacts/reconstruction/<scene_name>/
  images/
    001.jpg
    002.jpg
  colmap/
  openmvs/
  outputs/
```

### 2.2 COLMAP 一键重建

当前机器已安装路径：

```text
D:\tools\photogrammetry\colmap-4.0.4\bin\colmap.exe
D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\
```

```powershell
$scene = "meeting_room_photo"
$root = (Resolve-Path "next_project/artifacts/reconstruction/$scene").Path
& "D:\tools\photogrammetry\colmap-4.0.4\bin\colmap.exe" `
  automatic_reconstructor `
  --workspace_path "$root\colmap" `
  --image_path "$root\images"
```

COLMAP 官方 CLI 支持 `automatic_reconstructor`，会从照片自动执行特征提取、匹配、SfM 和稠密重建。若重建失败，先检查照片重叠率和纹理，不要先调规划器参数。

### 2.3 接 OpenMVS 生成网格

COLMAP 结果通常需要先导出/整理成 MVS 可读数据，再交给 OpenMVS：

```powershell
& "D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\InterfaceCOLMAP.exe" `
  -i "$root\colmap\dense\0" `
  -o "$root\openmvs\scene.mvs" `
  --image-folder "$root\colmap\dense\0\images"

& "D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\DensifyPointCloud.exe" `
  "$root\openmvs\scene.mvs" -w "$root\openmvs"
& "D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\ReconstructMesh.exe" `
  "$root\openmvs\scene_dense.mvs" -w "$root\openmvs"
& "D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\RefineMesh.exe" `
  "$root\openmvs\scene_dense_mesh.mvs" -w "$root\openmvs"
& "D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release\TextureMesh.exe" `
  "$root\openmvs\scene_dense_mesh_refine.mvs" -w "$root\openmvs"
```

项目实际只需要几何障碍物，优先取 `.ply` 网格即可；贴图主要用于展示和答辩，不是避障必需输入。

### 2.4 导入项目地图

启动 Web 后端后，把网格导入为项目地图：

```powershell
curl.exe -X POST "http://127.0.0.1:8000/api/maps/import-model" `
  -F "file=@next_project/artifacts/reconstruction/meeting_room_photo/openmvs/scene_dense_mesh.ply" `
  -F "map_name=meeting_room_photo" `
  -F "voxel_size=0.35" `
  -F "scale=1.0" `
  -F "padding=1.0" `
  -F "max_obstacles=8000"
```

导入后检查：

```powershell
python -m pytest next_project/tests/test_obstacle_scenario.py -q
```

如果地图障碍物数量过多，先增大 `voxel_size`，例如从 `0.25` 调到 `0.35` 或 `0.5`。如果地图比例不对，调整 `scale`，不要手改规划器安全半径来掩盖尺度错误。

## 3. Meshroom / AliceVision 备选链路

Meshroom 更适合作为人工建图或展示用备选工具，但当前这台机器没有完成稳定安装，所以本文档不把它列为当前可直接执行的本地主链。

### 3.1 备选用途

1. 把照片拖入 Meshroom。
2. 使用默认 Photogrammetry pipeline。
3. 运行到 Texturing 或 MeshFiltering 后，找到输出 `.obj` 或 `.ply`。
4. 用 `/api/maps/import-model` 导入项目。

## 4. MUSt3R / DUSt3R 研究链路

MUSt3R / DUSt3R 系列适合作为论文味更强的扩展，不建议现在替代主链路。原因是：

- 它们能从未标定、多视角照片直接预测 3D 结构，适合弱纹理、相机参数不完整的场景。
- 输出通常更偏 dense point map / point cloud，需要额外清理、尺度对齐和网格化，才能稳定进入 `maps/*.json`。
- 环境依赖偏重，尤其是 PyTorch、CUDA、模型权重和显存。

推荐定位：

```text
COLMAP/OpenMVS 失败或效果差
  -> 用 DUSt3R/MUSt3R 生成补充点云
  -> 用 Open3D / Blender / MeshLab 清理、配准、网格化
  -> 导出 .ply
  -> /api/maps/import-model
```

论文表述可以写成：传统 SfM/MVS 作为稳定建图基线，DUSt3R/MUSt3R 作为弱纹理、少标定或新型学习式重建的对比与增强路线。

## 5. 室内采集规范

照片质量决定重建质量。室内场景建议：

- 每张照片与前后照片保持 60% 到 80% 重叠。
- 沿墙、家具、通道绕行拍摄，不要只站在房间中央原地转圈。
- 同一高度拍一圈，再补拍高处管线、低处桌脚和门槛。
- 白墙、玻璃、镜子、反光金属会严重破坏匹配，必要时贴临时纹理标记。
- 光照尽量稳定，避免运动模糊。
- 拍摄前在房间放 1 到 2 个已知长度标尺，用来校正 `scale`。
- 动态物体、人、屏幕反光尽量避开。

## 6. 对当前项目的最小改造建议

第一阶段不改规划器，只补工具和文档：

- 新增 `tools/photo_reconstruction_import.py`：封装 `curl /api/maps/import-model` 或直接调用模型解析函数。
- 在 Web 地图管理界面增加“照片重建模型导入”的参数说明，暴露 `voxel_size`、`scale`、`max_obstacles`。
- 增加一个 `maps/photo_demo.json` 小样例，用低面数 `.ply` 导入生成，作为回归素材。
- 增加测试：验证导入地图包含 `bounds`、`obstacles`，并能被 `core/map_loader.py` 加载。

第二阶段再做真实三维地图质量增强：

- 用 Open3D 对点云/网格做裁剪、离群点过滤和体素下采样。
- 对墙面、地面、天花板做平面拟合，减少凹凸噪声。
- 保持 `task_waypoints` 与 `replanned_waypoints` 分离：照片建模只提供静态障碍物地图，不应该伪造任务航点或重规划轨迹。
- 动态 replay 仍作为独立验证面，不要把普通模型导入结果直接说成动态障碍跟随安全报告。

## 7. 推荐结论

项目落地优先级：

1. `COLMAP + OpenMVS`：当前机器已装好，作为默认主线，适合自动化、论文和工程复现。
2. `Meshroom / AliceVision`：只保留为备选 GUI 工具，不作为当前机器已完成的安装链路。
3. `MUSt3R / DUSt3R`：研究增强线，适合弱纹理、少标定和论文对比，不作为当前稳定生产入口。

最关键的项目接入点不是重写避障算法，而是把照片重建结果稳定转换成项目已有地图格式：

```text
mesh/point cloud -> voxelized AABB map -> existing planner/replay/report pipeline
```

## 8. 资料入口

- COLMAP GitHub: https://github.com/colmap/colmap
- COLMAP CLI: https://colmap.github.io/cli.html
- OpenMVS GitHub: https://github.com/cdcseacave/openMVS
- OpenMVS usage: https://github-wiki-see.page/m/cdcseacave/openMVS/wiki/Usage
- Meshroom GitHub: https://github.com/alicevision/meshroom
- Meshroom command line reference: https://meshroom-manual.readthedocs.io/en/latest/feature-documentation/cmd/photogrammetry.html
- AliceVision GitHub: https://github.com/alicevision/AliceVision
- DUSt3R GitHub: https://github.com/naver/dust3r
- MUSt3R GitHub: https://github.com/naver/must3r
