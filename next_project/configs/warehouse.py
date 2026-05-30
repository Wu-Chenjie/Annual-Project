from __future__ import annotations

import numpy as np
from pathlib import Path

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent

def _config_warehouse() -> SimulationConfig:
    """工业仓库：3 从机，在线 A* + 传感器 + D* Lite，Backstepping+SMC，队形切换。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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


def _config_school_corridor() -> SimulationConfig:
    """学校走廊：3 从机，在线 + GNN Danger，窄通道+L型转角，测试编队收缩与转弯。"""
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
        map_file=str(PKG / "maps" / "school_corridor.json"),
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


def _config_school_corridor_online() -> SimulationConfig:
    """学校走廊在线版：GNN 可见图 + Danger 模式 + 自适应间隔，窄通道实时避障。"""
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
        map_file=str(PKG / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_z_bounds=(1.5, 3.0),
        sensor_enabled=True,
        planner_replan_interval=0.3,
        planner_horizon=5.0,
        danger_mode_enabled=True,
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


def _config_company_cubicles() -> SimulationConfig:
    """公司格子间：3 从机，Hybrid A* 离线，3x3隔间矩阵+会议室，测试低矮障碍越顶飞行。"""
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
        map_file=str(PKG / "maps" / "company_cubicles.json"),
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


def _config_company_cubicles_online() -> SimulationConfig:
    """公司格子间在线版：Hybrid A* + 传感器 + D* Lite + WindowReplanner，实时检测低矮隔板越顶。"""
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
        map_file=str(PKG / "maps" / "company_cubicles.json"),
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


def _config_meeting_room() -> SimulationConfig:
    """会议室：2 从机，在线 A* + 传感器 + D* Lite，椭圆桌+座椅环绕，适合精确环绕飞行测试。"""
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
        map_file=str(PKG / "maps" / "meeting_room.json"),
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


def _config_meeting_room_online() -> SimulationConfig:
    """会议室在线版：A* + 传感器 + 实时重规划，椭圆桌+座椅精确环绕，cm 级动态响应。"""
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
        map_file=str(PKG / "maps" / "meeting_room.json"),
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


def _config_laboratory() -> SimulationConfig:
    """实验室：3 从机，离线 A*，实验台+通风橱+试剂架，多层高度障碍物。"""
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
        map_file=str(PKG / "maps" / "laboratory.json"),
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


def _config_laboratory_online() -> SimulationConfig:
    """实验室在线版：Hybrid A* + 传感器 + D* Lite + WindowReplanner，多层高度障碍物实时响应。"""
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
        map_file=str(PKG / "maps" / "laboratory.json"),
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

