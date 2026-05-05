# 工业产线+管道车间：复杂三维避障场景设计

## 概述

为低空经济室内无人机集群技术验证平台设计一个高复杂度三维避障验证场景。场景模拟化工厂/制造车间环境，包含反应釜（球体）、管道网络（圆柱）、操作平台（AABB多层）和龙门吊（横梁），强制无人机集群在不同高度层级间穿行。

## 验证目标

- **三维避障**：无人机集群在多个高度层级间穿行，躲避立体障碍物
- **高低差机动**：航点包含 ±3.5m 高度变化，路径呈"M型"波浪剖面
- **在线重规划**：传感器 + WindowReplanner 三级分层 + MPC 轨迹优化完整闭环
- **编队压缩**：窄空间自动切换直线队形

## 障碍物地图（~42 个原语）

### 区域1：入口预处理区

| 障碍物 | 类型 | 参数 |
|--------|------|------|
| 储罐A | Cylinder | cxy=(3,3) r=0.6 z=(0,3) |
| 储罐B | Cylinder | cxy=(3,7) r=0.5 z=(0,2.5) |
| 反应釜1(小) | Sphere | c=(5,5,3) r=1.0 |
| 低位平台 | AABB | min=(5.5,2,0) max=(8.5,8,0.3) |
| 竖管L1 | Cylinder | cxy=(7,1) r=0.15 z=(0,4) |
| 竖管L2 | Cylinder | cxy=(8.5,8.5) r=0.15 z=(0,4) |

### 区域2：主反应区

| 障碍物 | 类型 | 参数 |
|--------|------|------|
| 反应釜2(大) | Sphere | c=(13,7,3.2) r=1.5 |
| 反应釜3(大) | Sphere | c=(15,15,3.5) r=1.8 |
| 水平管A1(低) | Cylinder | cxy=(12,12) r=0.15 z=(1.5,1.5) |
| 水平管A2(中) | Cylinder | cxy=(13,11) r=0.15 z=(3.5,3.5) |
| 水平管A3(高) | Cylinder | cxy=(14,10) r=0.15 z=(5.8,5.8) |
| 水平管B1(低) | Cylinder | cxy=(10,14) r=0.15 z=(1.8,1.8) |
| 水平管B2(中) | Cylinder | cxy=(11,15) r=0.15 z=(3.2,3.2) |
| 水平管B3(高) | Cylinder | cxy=(12,16) r=0.15 z=(6.0,6.0) |
| 猫道A | AABB | min=(16,10,2.5) max=(20,12,3.0) |
| 猫道B | AABB | min=(16,14,2.5) max=(20,16,3.0) |
| 立柱 x6 | Cylinder | cxy=(12-20格) r=0.25 z=(0,7) |

### 区域3：分离/储存区

| 障碍物 | 类型 | 参数 |
|--------|------|------|
| 储罐C | Cylinder | cxy=(22,6) r=0.6 z=(0,4) |
| 储罐D | Cylinder | cxy=(24,6) r=0.7 z=(0,4.5) |
| 储罐E | Cylinder | cxy=(22,12) r=0.5 z=(0,3.5) |
| 储罐F | Cylinder | cxy=(24,12) r=0.65 z=(0,5) |
| 反应釜4(中) | Sphere | c=(24,14,4.0) r=1.2 |
| 上层平台A | AABB | min=(20,4,5.0) max=(25,8,5.5) |
| 上层平台B | AABB | min=(20,12,5.0) max=(25,16,5.5) |
| 龙门吊A | AABB | min=(20,8.5,6.5) max=(28,10,7.5) |
| 龙门吊B | AABB | min=(20,14,6.5) max=(28,15.5,7.5) |
| 管道廊C1-C4 | Cylinder | cxy=(格) r=0.15 z=(6,7) |

### 区域4：出口通道

| 障碍物 | 类型 | 参数 |
|--------|------|------|
| 低空横管 | Cylinder | cxy=(29,4) r=0.12 z=(2.8,3.0) |
| 边界墙 (4面) | AABB | 包围整个场景 |

## 边界

bounds: [[-2, -2, 0], [36, 20, 8]]

## 航点路径

| WP | 位置 | 高度(m) | 说明 |
|----|------|---------|------|
| WP0 | (2, 4, 3.0) | 3.0 | 入口起始 |
| WP1 | (9, 5, 1.5) | 1.5 | 钻低管道下方 (**-1.5m**) |
| WP2 | (14, 12, 5.0) | 5.0 | 越猫道+反应釜间隙 (**+3.5m**) |
| WP3 | (20, 14, 2.2) | 2.2 | 管道间隙穿行 (**-2.8m**) |
| WP4 | (26, 10, 5.8) | 5.8 | 上层平台标高 (**+3.6m**) |
| WP5 | (33, 4, 2.5) | 2.5 | 出口通道，钻过低空横管下方 (**-3.3m**) |

高度剖面: 3.0 → 1.5 ↑3.5→ 5.0 ↓2.8→ 2.2 ↑3.6→ 5.8 ↓3.3→ 2.5

## 编队策略

```python
formation_schedule = [
    (16.0, "line",    4.0),
    (30.0, "diamond", 4.0),
]
```

## 仿真配置

```python
config = SimulationConfig(
    max_sim_time=45.0,
    use_smc=True,
    num_followers=3,
    formation_spacing=1.5,
    initial_formation="diamond",
    leader_max_vel=3.0,
    leader_max_acc=4.0,
    follower_max_vel=8.0,
    follower_max_acc=8.0,
    enable_obstacles=True,
    map_file="maps/sample_warehouse.json",
    planner_kind="rrt_star",
    planner_mode="online",
    planner_resolution=0.4,
    planner_replan_interval=0.6,
    planner_horizon=5.0,
    planner_max_iter=2000,
    planner_rewire_radius=1.5,
    safety_margin=0.3,
    sensor_enabled=True,
    sensor_max_range=8.0,
    sensor_noise_std=0.02,
    mpc_enabled=True,
    mpc_dt=0.05,
    mpc_horizon=15,
    formation_schedule=[...],
    waypoints=[...],
)
```

## 成功指标

| 指标 | 阈值 | 说明 |
|------|------|------|
| 从机平均误差 (per follower) | < 0.15 m | 严格精度要求 |
| 从机最大误差 | < 0.30 m | 峰值约束 |
| 碰撞次数 | 0 | 硬约束 |
| 航点完成率 | 6/6 (100%) | 全部到达 |
| 重规划成功次数 | ≥ 2 | 验证在线规划 |

## 实现产出

1. `maps/sample_warehouse.json` — 障碍物地图原语文件
2. `main.py` 新增 `main_warehouse()` 入口
3. 命令行调用：`python main.py --preset warehouse`

## 变更日志

- 2026-04-26：初始设计文档，选定工业产线+管道车间方案B
