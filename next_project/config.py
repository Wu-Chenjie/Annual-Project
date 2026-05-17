"""统一仿真参数配置文件。

所有可调参数集中在一个文件，支持预设场景快速切换。
使用方法：在 main.py 或测试脚本中 `from config import get_config` 即可。
"""

from __future__ import annotations

import numpy as np

from simulations.formation_simulation import SimulationConfig

AVAILABLE_PRESETS = [
    "basic",
    "obstacle",
    "obstacle_unknown",
    "warehouse",
    "warehouse_a",
    "warehouse_online",
    "warehouse_danger",
    "warehouse_unknown",
    "warehouse_a_unknown",
    "warehouse_online_unknown",
    "warehouse_danger_unknown",
    "fault_tolerance",
    "fault_tolerance_online",
    "fault_tolerance_unknown",
    "fault_tolerance_online_unknown",
    "school_corridor",
    "school_corridor_online",
    "school_corridor_unknown",
    "school_corridor_online_unknown",
    "company_cubicles",
    "company_cubicles_online",
    "company_cubicles_unknown",
    "company_cubicles_online_unknown",
    "meeting_room",
    "meeting_room_online",
    "meeting_room_unknown",
    "meeting_room_online_unknown",
    "rrt_dual_channel_online",
    "rrt_dual_channel_online_unknown",
    "formation_maze_stress_online",
    "formation_maze_stress_online_unknown",
    "unknown_map_online",
    "laboratory",
    "laboratory_online",
    "laboratory_unknown",
    "laboratory_online_unknown",
    "custom",
]


def get_config(preset: str = "basic") -> SimulationConfig:
    """返回指定预设场景的仿真配置。

    可用预设:
        basic                基础编队验证（30s，方形航线）
        obstacle             简单障碍物避障（三柱，厘米级精度）
        obstacle_unknown     简单障碍物完全未知版（初始地图为空 + 传感器探索）
        warehouse            工业仓库复杂场景（在线 A* + 传感器 + 队形切换）
        warehouse_a          仓库场景 A* 版（在线 A* + ESDF + Danger 模式）
        warehouse_online     仓库场景在线版（A* + 传感器 + D* Lite）
        warehouse_danger     仓库在线版 + GNN 双模式 + 改进 APF（保守档）
        warehouse_unknown    仓库完全未知版（初始地图为空 + 传感器逐步发现）
        warehouse_a_unknown  仓库 A* 完全未知版
        warehouse_online_unknown 仓库在线完全未知版
        warehouse_danger_unknown 仓库 Danger 完全未知版
        fault_tolerance      容错测试场景（故障注入+拓扑重构）
        fault_tolerance_online 容错在线版
        fault_tolerance_unknown 容错完全未知版
        fault_tolerance_online_unknown 容错在线完全未知版
        school_corridor      学校走廊场景（离线 A*）
        school_corridor_online 学校走廊在线版（A* + 传感器 + D* Lite）
        school_corridor_unknown 学校走廊完全未知版
        school_corridor_online_unknown 学校走廊在线完全未知版
        company_cubicles     公司格子间场景（离线 Hybrid A*）
        company_cubicles_online 公司格子间在线版（Hybrid A* + 传感器 + D* Lite）
        company_cubicles_unknown 公司格子间完全未知版
        company_cubicles_online_unknown 公司格子间在线完全未知版
        meeting_room         会议室场景（离线 A*，cm 级精度）
        meeting_room_online  会议室在线版（A* + 传感器 + 实时重规划）
        meeting_room_unknown 会议室完全未知版
        meeting_room_online_unknown 会议室在线完全未知版
        rrt_dual_channel_online RRT 双通道绕行压力测试（前瞻窗口 + RRT escape）
        rrt_dual_channel_online_unknown RRT 双通道完全未知版
        formation_maze_stress_online 编队迷宫压力测试（狭长通道 + 急转弯 + 队形切换）
        formation_maze_stress_online_unknown 编队迷宫完全未知版
        unknown_map_online  完全未知地图在线探索（初始规划地图为空，传感器发现障碍）
        laboratory           实验室场景（离线 A*）
        laboratory_online    实验室在线版（Hybrid A* + 传感器 + D* Lite）
        laboratory_unknown   实验室完全未知版
        laboratory_online_unknown 实验室在线完全未知版
        custom               自定义（修改此函数返回值即可）
    """
    if preset == "basic":
        return _config_basic()
    elif preset == "obstacle":
        return _config_obstacle()
    elif preset == "obstacle_unknown":
        return _config_obstacle_unknown()
    elif preset == "warehouse":
        return _config_warehouse()
    elif preset == "warehouse_a":
        return _config_warehouse_a()
    elif preset == "warehouse_online":
        return _config_warehouse_online()
    elif preset == "warehouse_danger":
        return _config_warehouse_danger()
    elif preset == "warehouse_unknown":
        return _config_warehouse_unknown()
    elif preset == "warehouse_a_unknown":
        return _config_warehouse_a_unknown()
    elif preset == "warehouse_online_unknown":
        return _config_warehouse_online_unknown()
    elif preset == "warehouse_danger_unknown":
        return _config_warehouse_danger_unknown()
    elif preset == "fault_tolerance":
        return _config_fault_tolerance()
    elif preset == "fault_tolerance_online":
        return _config_fault_tolerance_online()
    elif preset == "fault_tolerance_unknown":
        return _config_fault_tolerance_unknown()
    elif preset == "fault_tolerance_online_unknown":
        return _config_fault_tolerance_online_unknown()
    elif preset == "school_corridor":
        return _config_school_corridor()
    elif preset == "company_cubicles":
        return _config_company_cubicles()
    elif preset == "meeting_room":
        return _config_meeting_room()
    elif preset == "school_corridor_online":
        return _config_school_corridor_online()
    elif preset == "school_corridor_unknown":
        return _config_school_corridor_unknown()
    elif preset == "school_corridor_online_unknown":
        return _config_school_corridor_online_unknown()
    elif preset == "company_cubicles_online":
        return _config_company_cubicles_online()
    elif preset == "company_cubicles_unknown":
        return _config_company_cubicles_unknown()
    elif preset == "company_cubicles_online_unknown":
        return _config_company_cubicles_online_unknown()
    elif preset == "meeting_room_online":
        return _config_meeting_room_online()
    elif preset == "meeting_room_unknown":
        return _config_meeting_room_unknown()
    elif preset == "meeting_room_online_unknown":
        return _config_meeting_room_online_unknown()
    elif preset == "rrt_dual_channel_online":
        return _config_rrt_dual_channel_online()
    elif preset == "rrt_dual_channel_online_unknown":
        return _config_rrt_dual_channel_online_unknown()
    elif preset == "formation_maze_stress_online":
        return _config_formation_maze_stress_online()
    elif preset == "formation_maze_stress_online_unknown":
        return _config_formation_maze_stress_online_unknown()
    elif preset == "unknown_map_online":
        return _config_unknown_map_online()
    elif preset == "laboratory_online":
        return _config_laboratory_online()
    elif preset == "laboratory":
        return _config_laboratory()
    elif preset == "laboratory_unknown":
        return _config_laboratory_unknown()
    elif preset == "laboratory_online_unknown":
        return _config_laboratory_online_unknown()
    elif preset == "custom":
        return _config_custom()
    else:
        raise ValueError(f"未知预设: {preset}，可用: {', '.join(AVAILABLE_PRESETS)}")


# ============================================================
# 预设场景
# ============================================================

def _config_basic() -> SimulationConfig:
    """基础编队验证：3 从机 diamond 编队，方形航线，30s。"""
    return SimulationConfig(
        max_sim_time=30.0,
        use_smc=True,
        use_backstepping=False,
        num_followers=3,
        formation_spacing=2.0,
        initial_formation="v_shape",
        waypoints=[
            np.array([0.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 0.0, 15.0], dtype=float),
            np.array([20.0, 0.0, 15.0], dtype=float),
            np.array([20.0, 20.0, 15.0], dtype=float),
            np.array([0.0, 20.0, 15.0], dtype=float),
            np.array([0.0, 0.0, 0.0], dtype=float),
        ],
    )


def _config_obstacle() -> SimulationConfig:
    """简单障碍物避障：2 从机，三柱地图，低速 cm 级精度。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=False,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.2,
        wp_radius_final=0.1,
        leader_max_vel=0.9,
        leader_max_acc=1.0,
        leader_gain_scale=0.8,
        follower_gain_scale=1.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )


def _config_warehouse() -> SimulationConfig:
    """工业仓库：3 从机，在线 A* + 传感器 + D* Lite，Backstepping+SMC，队形切换。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=65.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        sensor_enabled=True,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        formation_schedule=[
            (15.0, "line", 6.0),
            (28.0, "diamond", 6.0),
        ],
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([11.4, 10.2, 2.4], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([29.2, 22.4, 3.7], dtype=float),
            np.array([35.8, 10.0, 8.3], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_a() -> SimulationConfig:
    """仓库 A* 版：在线 + GNN Danger 模式，ESDF 软代价。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=65.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        sensor_enabled=True,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        danger_mode_enabled=True,
        formation_schedule=[
            (15.0, "line", 6.0),
            (28.0, "diamond", 6.0),
        ],
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([11.4, 10.2, 2.4], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([29.2, 22.4, 3.7], dtype=float),
            np.array([35.8, 10.0, 8.3], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_online() -> SimulationConfig:
    """仓库在线版：A* + 传感器 + D* Lite + WindowReplanner。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=30.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=6.0,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_danger() -> SimulationConfig:
    """仓库在线版 + GNN 双模式 + 改进 APF（保守档）。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=30.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        sensor_enabled=True,
        danger_mode_enabled=True,
        apf_paper1_profile="conservative",
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_fault_tolerance() -> SimulationConfig:
    """容错测试场景：注入单机故障，验证拓扑重构。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=30.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="offline",
        safety_margin=0.3,
        fault_injection_enabled=True,
        fault_detection_enabled=True,
        fault_reconfig_enabled=True,
        apf_paper1_profile="off",
        danger_mode_enabled=False,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_fault_tolerance_online() -> SimulationConfig:
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=35.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        sensor_enabled=True,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        danger_mode_enabled=True,
        apf_paper1_profile="conservative",
        replan_adaptive_interval=True,
        fault_injection_enabled=True,
        fault_detection_enabled=True,
        fault_reconfig_enabled=True,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_school_corridor() -> SimulationConfig:
    """学校走廊：3 从机，在线 + GNN Danger，窄通道+L型转角，测试编队收缩与转弯。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=500.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.4,
        initial_formation="line",
        wp_radius=0.5,
        wp_radius_final=0.3,
        leader_max_vel=1.5,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=6.0,
        follower_max_acc=6.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_z_bounds=(1.5, 3.0),
        sensor_enabled=True,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        danger_mode_enabled=True,
        formation_schedule=[
            (20.0, "diamond", 2.0),
            (38.0, "line", 3.0),
        ],
        waypoints=[
            np.array([1.0, 2.0, 2.0], dtype=float),
            np.array([10.0, 2.0, 2.0], dtype=float),
            np.array([23.0, 2.0, 2.0], dtype=float),
            np.array([27.5, 1.0, 2.0], dtype=float),
            np.array([34.0, 5.5, 2.0], dtype=float),
            np.array([41.0, 2.0, 2.0], dtype=float),
            np.array([47.0, 2.0, 2.5], dtype=float),
        ],
    )


def _config_company_cubicles() -> SimulationConfig:
    """公司格子间：3 从机，Hybrid A* 离线，3x3隔间矩阵+会议室，测试低矮障碍越顶飞行。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=50.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.8,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        sensor_enabled=False,
        formation_schedule=[
            (18.0, "line", 4.0),
            (32.0, "diamond", 4.0),
        ],
        waypoints=[
            np.array([3.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 21.0, 2.5], dtype=float),
        ],
    )


def _config_meeting_room() -> SimulationConfig:
    """会议室：2 从机，在线 A* + 传感器 + D* Lite，椭圆桌+座椅环绕，适合精确环绕飞行测试。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.4,
        wp_radius_final=0.2,
        leader_max_vel=0.8,
        leader_max_acc=1.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_z_bounds=(1.4, 2.8),
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=4.0,
        waypoints=[
            np.array([1.0, 5.0, 2.0], dtype=float),
            np.array([7.0, 1.5, 2.0], dtype=float),
            np.array([13.5, 5.0, 2.0], dtype=float),
            np.array([7.0, 11.0, 2.0], dtype=float),
            np.array([1.0, 8.0, 2.0], dtype=float),
        ],
    )


def _config_laboratory() -> SimulationConfig:
    """实验室：3 从机，离线 A*，实验台+通风橱+试剂架，多层高度障碍物。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=55.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="v_shape",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.5,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "laboratory.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_z_bounds=(2.0, 3.2),
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=6.0,
        formation_schedule=[
            (20.0, "diamond", 3.0),
            (38.0, "line", 5.0),
        ],
        waypoints=[
            np.array([1.0, 4.0, 2.5], dtype=float),
            np.array([3.0, 10.0, 2.5], dtype=float),
            np.array([8.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([16.0, 10.0, 2.5], dtype=float),
            np.array([21.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 16.0, 2.5], dtype=float),
            np.array([3.0, 16.0, 2.5], dtype=float),
        ],
    )


def _config_school_corridor_online() -> SimulationConfig:
    """学校走廊在线版：GNN 可见图 + Danger 模式 + 自适应间隔，窄通道实时避障。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.4,
        initial_formation="line",
        wp_radius=0.5,
        wp_radius_final=0.3,
        leader_max_vel=1.5,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=6.0,
        follower_max_acc=6.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_z_bounds=(1.5, 3.0),
        sensor_enabled=True,
        planner_replan_interval=0.3,
        planner_horizon=5.0,
        # 论文2: GNN 双模式（窄通道 A* 栅格搜索易被膨胀层阻断，切换可见图规划）
        danger_mode_enabled=True,
        # 论文3: 风险驱动自适应重规划间隔
        replan_adaptive_interval=True,
        formation_schedule=[
            (16.0, "diamond", 2.0),
            (30.0, "line", 3.0),
        ],
        waypoints=[
            np.array([1.0, 2.0, 2.0], dtype=float),
            np.array([15.0, 2.0, 2.0], dtype=float),
            np.array([27.5, 1.0, 2.0], dtype=float),
            np.array([41.0, 2.0, 2.0], dtype=float),
            np.array([47.0, 2.0, 2.5], dtype=float),
        ],
    )


def _config_company_cubicles_online() -> SimulationConfig:
    """公司格子间在线版：Hybrid A* + 传感器 + D* Lite + WindowReplanner，实时检测低矮隔板越顶。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=400.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.8,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=5.0,
        formation_schedule=[
            (14.0, "line", 4.0),
            (26.0, "diamond", 4.0),
        ],
        waypoints=[
            np.array([3.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 21.0, 2.5], dtype=float),
        ],
    )


def _config_meeting_room_online() -> SimulationConfig:
    """会议室在线版：A* + 传感器 + 实时重规划，椭圆桌+座椅精确环绕，cm 级动态响应。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=35.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.3,
        wp_radius_final=0.15,
        leader_max_vel=0.8,
        leader_max_acc=1.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_z_bounds=(1.4, 2.8),
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=3.5,
        waypoints=[
            np.array([1.0, 5.0, 2.0], dtype=float),
            np.array([7.0, 1.5, 2.0], dtype=float),
            np.array([13.5, 5.0, 2.0], dtype=float),
            np.array([7.0, 11.0, 2.0], dtype=float),
        ],
    )


def _config_rrt_dual_channel_online() -> SimulationConfig:
    """RRT双通道绕行对照图：局部前瞻假死路 + RRT旁路escape + 编队提前变换。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=28.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.55,
        initial_formation="diamond",
        wp_radius=0.45,
        wp_radius_final=0.25,
        leader_max_vel=1.0,
        leader_max_acc=1.4,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "rrt_dual_channel_escape.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_z_bounds=(1.4, 2.4),
        sensor_enabled=True,
        planner_replan_interval=1.0,
        planner_horizon=4.0,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=True,
        formation_lookahead_enabled=True,
        formation_lookahead_rrt_enabled=True,
        formation_lookahead_distance=4.2,
        formation_lookahead_turn_threshold_rad=1.0,
        formation_lookahead_min_interval=0.8,
        formation_lookahead_rrt_max_iter=900,
        formation_lookahead_rrt_rewire_radius=1.2,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([5.5, 0.0, 1.8], dtype=float),
            np.array([12.0, 4.5, 1.8], dtype=float),
            np.array([22.5, 4.5, 1.8], dtype=float),
        ],
    )


def _config_formation_maze_stress_online() -> SimulationConfig:
    """编队迷宫压力图：窄门、假分支、RRT旁路、U形转弯与宽区恢复。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=38.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.55,
        initial_formation="diamond",
        wp_radius=0.50,
        wp_radius_final=0.28,
        leader_max_vel=1.0,
        leader_max_acc=1.4,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "formation_maze_stress.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_z_bounds=(1.4, 2.4),
        sensor_enabled=True,
        planner_replan_interval=1.0,
        planner_horizon=4.5,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=True,
        formation_lookahead_enabled=True,
        formation_lookahead_rrt_enabled=True,
        formation_lookahead_distance=4.5,
        formation_lookahead_turn_threshold_rad=1.0,
        formation_lookahead_min_interval=0.8,
        formation_lookahead_rrt_max_iter=1000,
        formation_lookahead_rrt_rewire_radius=1.25,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([4.5, 3.5, 1.8], dtype=float),
            np.array([10.0, 3.5, 1.8], dtype=float),
            np.array([10.0, -2.5, 1.8], dtype=float),
            np.array([17.0, -2.5, 1.8], dtype=float),
            np.array([20.5, 5.0, 1.8], dtype=float),
        ],
    )


def _config_unknown_map_online() -> SimulationConfig:
    """完全未知地图在线探索：规划器初始地图为空，仅通过传感器逐步发现障碍。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=26.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.45,
        initial_formation="line",
        wp_radius=0.45,
        wp_radius_final=0.25,
        leader_max_vel=1.0,
        leader_max_acc=1.3,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "unknown_map_arena.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.10,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=4.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.35,
        planner_horizon=3.2,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=False,
        formation_lookahead_enabled=False,
        formation_lookahead_rrt_enabled=False,
        waypoints=[
            np.array([1.0, 0.0, 1.8], dtype=float),
            np.array([5.5, 0.0, 1.8], dtype=float),
            np.array([9.5, 2.8, 1.8], dtype=float),
            np.array([16.0, 2.8, 1.8], dtype=float),
        ],
    )


def _config_obstacle_unknown() -> SimulationConfig:
    """简单障碍物完全未知版：初始地图为空，仅靠传感器逐步发现三柱，低速探索。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=60.0,
        use_smc=True,
        use_backstepping=False,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.2,
        wp_radius_final=0.1,
        leader_max_vel=0.7,
        leader_max_acc=0.8,
        leader_gain_scale=0.8,
        follower_gain_scale=1.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=3.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )


def _config_warehouse_unknown() -> SimulationConfig:
    """仓库完全未知版：初始地图为空，传感器逐步发现货架/通道，在线 A* + D* Lite。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=80.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (20.0, "line", 6.0),
            (36.0, "diamond", 6.0),
        ],
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([11.4, 10.2, 2.4], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([29.2, 22.4, 3.7], dtype=float),
            np.array([35.8, 10.0, 8.3], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_a_unknown() -> SimulationConfig:
    """仓库 A* 版完全未知：初始地图为空 + 在线探索。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=80.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (20.0, "line", 6.0),
            (36.0, "diamond", 6.0),
        ],
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([11.4, 10.2, 2.4], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([29.2, 22.4, 3.7], dtype=float),
            np.array([35.8, 10.0, 8.3], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_online_unknown() -> SimulationConfig:
    """仓库在线版完全未知：简化为 3 航点 + 传感器逐步发现。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_danger_unknown() -> SimulationConfig:
    """仓库 Danger 完全未知版：初始地图为空，传感器逐步发现，GNN 双模式关闭（依赖已知障碍场）。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.8,
        wp_radius_final=0.4,
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_fault_tolerance_unknown() -> SimulationConfig:
    """容错测试完全未知版：初始地图为空 + 在线传感器 + 故障注入 + 拓扑重构。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        fault_injection_enabled=True,
        fault_detection_enabled=True,
        fault_reconfig_enabled=True,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_fault_tolerance_online_unknown() -> SimulationConfig:
    """容错在线完全未知版：初始地图为空 + 传感器 + 故障注入 + 双模式（Danger 关闭）。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=45.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        leader_max_vel=1.6,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.4,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=5.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        replan_adaptive_interval=True,
        fault_injection_enabled=True,
        fault_detection_enabled=True,
        fault_reconfig_enabled=True,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_school_corridor_unknown() -> SimulationConfig:
    """学校走廊完全未知版：初始地图为空，传感器逐步发现窄通道+L型转角，测试编队通过能力。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=600.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.4,
        initial_formation="line",
        wp_radius=0.5,
        wp_radius_final=0.3,
        leader_max_vel=1.2,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=6.0,
        follower_max_acc=6.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.5, 3.0),
        sensor_enabled=True,
        sensor_max_range=4.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.35,
        planner_horizon=4.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (30.0, "diamond", 2.0),
            (60.0, "line", 3.0),
        ],
        waypoints=[
            np.array([1.0, 2.0, 2.0], dtype=float),
            np.array([10.0, 2.0, 2.0], dtype=float),
            np.array([23.0, 2.0, 2.0], dtype=float),
            np.array([27.5, 1.0, 2.0], dtype=float),
            np.array([34.0, 5.5, 2.0], dtype=float),
            np.array([41.0, 2.0, 2.0], dtype=float),
            np.array([47.0, 2.0, 2.5], dtype=float),
        ],
    )


def _config_school_corridor_online_unknown() -> SimulationConfig:
    """学校走廊在线完全未知版：初始地图为空 + 自适应间隔 + 窄通道传感器发现。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=55.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.4,
        initial_formation="line",
        wp_radius=0.5,
        wp_radius_final=0.3,
        leader_max_vel=1.2,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=6.0,
        follower_max_acc=6.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.5, 3.0),
        sensor_enabled=True,
        sensor_max_range=4.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.3,
        planner_horizon=4.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        replan_adaptive_interval=True,
        formation_schedule=[
            (20.0, "diamond", 2.0),
            (38.0, "line", 3.0),
        ],
        waypoints=[
            np.array([1.0, 2.0, 2.0], dtype=float),
            np.array([15.0, 2.0, 2.0], dtype=float),
            np.array([27.5, 1.0, 2.0], dtype=float),
            np.array([41.0, 2.0, 2.0], dtype=float),
            np.array([47.0, 2.0, 2.5], dtype=float),
        ],
    )


def _config_company_cubicles_unknown() -> SimulationConfig:
    """公司格子间完全未知版：初始地图为空，传感器逐步发现隔板矩阵，越顶飞行。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=65.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.4,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=4.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (22.0, "line", 4.0),
            (40.0, "diamond", 4.0),
        ],
        waypoints=[
            np.array([3.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 21.0, 2.5], dtype=float),
        ],
    )


def _config_company_cubicles_online_unknown() -> SimulationConfig:
    """公司格子间在线完全未知版：初始地图为空 + Hybrid A* + 传感器逐步发现。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=500.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.4,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        sensor_enabled=True,
        sensor_max_range=4.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (18.0, "line", 4.0),
            (34.0, "diamond", 4.0),
        ],
        waypoints=[
            np.array([3.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([22.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 21.0, 2.5], dtype=float),
        ],
    )


def _config_meeting_room_unknown() -> SimulationConfig:
    """会议室完全未知版：初始地图为空，传感器逐步发现椭圆桌+座椅，cm 级精度。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=55.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.4,
        wp_radius_final=0.2,
        leader_max_vel=0.6,
        leader_max_acc=0.8,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.4, 2.8),
        sensor_enabled=True,
        sensor_max_range=3.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        waypoints=[
            np.array([1.0, 5.0, 2.0], dtype=float),
            np.array([7.0, 1.5, 2.0], dtype=float),
            np.array([13.5, 5.0, 2.0], dtype=float),
            np.array([7.0, 11.0, 2.0], dtype=float),
            np.array([0.8, 7.0, 2.0], dtype=float),
        ],
    )


def _config_meeting_room_online_unknown() -> SimulationConfig:
    """会议室在线完全未知版：初始地图为空 + 在线重规划 + cm 级传感器发现。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=50.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        wp_radius=0.3,
        wp_radius_final=0.15,
        leader_max_vel=0.6,
        leader_max_acc=0.8,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.4, 2.8),
        sensor_enabled=True,
        sensor_max_range=3.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        waypoints=[
            np.array([1.0, 5.0, 2.0], dtype=float),
            np.array([7.0, 1.5, 2.0], dtype=float),
            np.array([13.5, 5.0, 2.0], dtype=float),
            np.array([7.0, 11.0, 2.0], dtype=float),
        ],
    )


def _config_rrt_dual_channel_online_unknown() -> SimulationConfig:
    """RRT 双通道绕行完全未知版：初始地图为空 + 前瞻窗口 + RRT escape，传感器逐步发现通道。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=38.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.55,
        initial_formation="diamond",
        wp_radius=0.45,
        wp_radius_final=0.25,
        leader_max_vel=0.8,
        leader_max_acc=1.1,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "rrt_dual_channel_escape.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.4, 2.4),
        sensor_enabled=True,
        sensor_max_range=4.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=3.5,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=False,
        formation_lookahead_enabled=False,
        formation_lookahead_rrt_enabled=False,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([5.5, 0.0, 1.8], dtype=float),
            np.array([12.25, 4.0, 1.8], dtype=float),
            np.array([22.5, 4.5, 1.8], dtype=float),
        ],
    )


def _config_formation_maze_stress_online_unknown() -> SimulationConfig:
    """编队迷宫压力完全未知版：初始地图为空，传感器逐步发现窄门/假分支/U形弯。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=50.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.55,
        initial_formation="diamond",
        wp_radius=0.50,
        wp_radius_final=0.28,
        leader_max_vel=0.8,
        leader_max_acc=1.1,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=5.0,
        follower_max_acc=5.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "formation_maze_stress.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(1.4, 2.4),
        sensor_enabled=True,
        sensor_max_range=4.0,
        sensor_noise_std=0.0,
        planner_replan_interval=0.4,
        planner_horizon=4.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=False,
        formation_lookahead_enabled=False,
        formation_lookahead_rrt_enabled=False,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([4.5, 3.5, 1.8], dtype=float),
            np.array([10.0, 3.5, 1.8], dtype=float),
            np.array([10.0, -2.5, 1.8], dtype=float),
            np.array([17.0, -2.5, 1.8], dtype=float),
            np.array([20.5, 5.0, 1.8], dtype=float),
        ],
    )


def _config_laboratory_unknown() -> SimulationConfig:
    """实验室完全未知版：初始地图为空，传感器逐步发现实验台/通风橱/试剂架。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=70.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="v_shape",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.2,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "laboratory.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(2.0, 3.2),
        sensor_enabled=True,
        sensor_max_range=4.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=5.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (24.0, "diamond", 3.0),
            (46.0, "line", 5.0),
        ],
        waypoints=[
            np.array([1.0, 4.0, 2.5], dtype=float),
            np.array([3.0, 10.0, 2.5], dtype=float),
            np.array([8.0, 4.0, 2.5], dtype=float),
            np.array([13.0, 4.0, 2.5], dtype=float),
            np.array([16.0, 10.0, 2.5], dtype=float),
            np.array([21.0, 12.0, 2.5], dtype=float),
            np.array([13.0, 16.0, 2.5], dtype=float),
            np.array([3.0, 16.0, 2.5], dtype=float),
        ],
    )


def _config_laboratory_online_unknown() -> SimulationConfig:
    """实验室在线完全未知版：初始地图为空 + Hybrid A* + 传感器逐步发现多层障碍物。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=60.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="v_shape",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.2,
        leader_max_acc=1.6,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "laboratory.json"),
        planner_kind="hybrid_astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_initial_map_unknown=True,
        planner_sdf_aware=False,
        planner_esdf_aware=False,
        planner_use_formation_envelope=False,
        planner_z_bounds=(2.0, 3.2),
        sensor_enabled=True,
        sensor_max_range=4.5,
        sensor_noise_std=0.0,
        planner_replan_interval=0.5,
        planner_horizon=5.0,
        apf_paper1_profile="conservative",
        danger_mode_enabled=True,
        trajectory_optimizer_enabled=True,
        formation_schedule=[
            (20.0, "diamond", 3.0),
            (38.0, "line", 5.0),
        ],
        waypoints=[
            np.array([1.0, 4.0, 2.5], dtype=float),
            np.array([3.0, 10.0, 2.5], dtype=float),
            np.array([8.0, 4.0, 2.5], dtype=float),
            np.array([16.0, 10.0, 2.5], dtype=float),
            np.array([21.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 16.0, 2.5], dtype=float),
        ],
    )


def _config_laboratory_online() -> SimulationConfig:
    """实验室在线版：Hybrid A* + 传感器 + D* Lite + WindowReplanner，多层高度障碍物实时响应。"""
    from pathlib import Path
    pkg = Path(__file__).resolve().parent
    return SimulationConfig(
        max_sim_time=45.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="v_shape",
        wp_radius=0.6,
        wp_radius_final=0.3,
        leader_max_vel=1.5,
        leader_max_acc=2.0,
        leader_gain_scale=0.80,
        follower_gain_scale=1.0,
        follower_max_vel=7.0,
        follower_max_acc=7.0,
        leader_acc_alpha=0.30,
        enable_obstacles=True,
        map_file=str(pkg / "maps" / "laboratory.json"),
        planner_kind="hybrid_astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_z_bounds=(2.0, 3.2),
        sensor_enabled=True,
        planner_replan_interval=2.0,
        planner_horizon=5.0,
        formation_schedule=[
            (16.0, "diamond", 3.0),
            (30.0, "line", 5.0),
        ],
        waypoints=[
            np.array([1.0, 4.0, 2.5], dtype=float),
            np.array([3.0, 10.0, 2.5], dtype=float),
            np.array([8.0, 4.0, 2.5], dtype=float),
            np.array([16.0, 10.0, 2.5], dtype=float),
            np.array([21.0, 12.0, 2.5], dtype=float),
            np.array([3.0, 16.0, 2.5], dtype=float),
        ],
    )


def _config_custom() -> SimulationConfig:
    """自定义配置模板：修改此函数中的参数即可快速实验。"""
    return SimulationConfig(
        # ---- 仿真时间 ----
        max_sim_time=65.0,
        # ---- 控制器 ----
        use_smc=True,               # SMC 姿态环
        use_backstepping=True,      # 反步法位置环
        # ---- 编队 ----
        num_followers=3,
        formation_spacing=0.5,
        initial_formation="diamond",
        # ---- 领航机机动 ----
        leader_max_vel=2.0,
        leader_max_acc=2.5,
        leader_gain_scale=0.80,
        # ---- 从机机动 ----
        follower_gain_scale=1.0,
        follower_max_vel=8.0,
        follower_max_acc=8.0,
        leader_acc_alpha=0.30,
        # ---- 障碍物 ----
        enable_obstacles=True,
        safety_margin=0.3,
        detect_margin_scale=0.5,
        # ---- 路径规划 ----
        planner_kind="hybrid_astar",  # "astar" | "hybrid_astar"
        planner_mode="offline",        # "offline" | "online"
        planner_resolution=0.4,
        planner_sdf_aware=True,
        planner_esdf_aware=True,
        # ---- 在线模式（仅 planner_mode="online" 时生效） ----
        sensor_enabled=False,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        # ---- 队形切换 ----
        formation_schedule=[
            (15.0, "line", 6.0),
            (28.0, "diamond", 6.0),
        ],
        # ---- 航点 ----
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([11.4, 10.2, 2.4], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([29.2, 22.4, 3.7], dtype=float),
            np.array([35.8, 10.0, 8.3], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )
