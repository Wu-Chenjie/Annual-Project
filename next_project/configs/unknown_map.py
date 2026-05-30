from __future__ import annotations

import numpy as np
from pathlib import Path

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent


_UNKNOWN_MAP_DEFAULTS = dict(
    planner_initial_map_unknown=True,
    planner_sdf_aware=False,
    planner_esdf_aware=False,
    planner_use_formation_envelope=False,
    sensor_enabled=True,
    sensor_noise_std=0.0,
    planner_mode="online",
    apf_paper1_profile="conservative",
    danger_mode_enabled=True,
    trajectory_optimizer_enabled=True,
)

def _config_obstacle_unknown() -> SimulationConfig:
    """简单障碍物完全未知版：初始地图为空，仅靠传感器逐步发现三柱，低速探索。"""
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
        map_file=str(PKG / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_resolution=0.3,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=3.5,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )


def _config_warehouse_unknown() -> SimulationConfig:
    """仓库完全未知版：初始地图为空，传感器逐步发现货架/通道，在线 A* + D* Lite。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_warehouse_danger_unknown() -> SimulationConfig:
    """仓库 Danger 完全未知版：初始地图为空，传感器逐步发现，GNN 双模式关闭（依赖已知障碍场）。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
        waypoints=[
            np.array([4.2, 13.2, 5.7], dtype=float),
            np.array([19.8, 15.2, 7.0], dtype=float),
            np.array([42.0, 4.8, 1.0], dtype=float),
        ],
    )


def _config_school_corridor_unknown() -> SimulationConfig:
    """学校走廊完全未知版：初始地图为空，传感器逐步发现窄通道+L型转角，测试编队通过能力。"""
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
        map_file=str(PKG / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_z_bounds=(1.5, 3.0),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.0,
        planner_replan_interval=0.35,
        planner_horizon=4.5,
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
        map_file=str(PKG / "maps" / "school_corridor.json"),
        planner_kind="astar",
        planner_resolution=0.3,
        safety_margin=0.25,
        planner_z_bounds=(1.5, 3.0),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.0,
        planner_replan_interval=0.3,
        planner_horizon=4.5,
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
        map_file=str(PKG / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_resolution=0.3,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.5,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
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
        map_file=str(PKG / "maps" / "company_cubicles.json"),
        planner_kind="hybrid_astar",
        planner_resolution=0.3,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.5,
        planner_replan_interval=0.5,
        planner_horizon=4.5,
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
        map_file=str(PKG / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_z_bounds=(1.4, 2.8),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=3.0,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
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
        map_file=str(PKG / "maps" / "meeting_room.json"),
        planner_kind="astar",
        planner_resolution=0.2,
        safety_margin=0.2,
        planner_z_bounds=(1.4, 2.8),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=3.0,
        planner_replan_interval=0.5,
        planner_horizon=3.0,
        waypoints=[
            np.array([1.0, 5.0, 2.0], dtype=float),
            np.array([7.0, 1.5, 2.0], dtype=float),
            np.array([13.5, 5.0, 2.0], dtype=float),
            np.array([7.0, 11.0, 2.0], dtype=float),
        ],
    )


def _config_unknown_map_online() -> SimulationConfig:
    """完全未知地图在线探索：规划器初始地图为空，仅通过传感器逐步发现障碍。"""
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
        map_file=str(PKG / "maps" / "unknown_map_arena.json"),
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


def _config_laboratory_unknown() -> SimulationConfig:
    """实验室完全未知版：初始地图为空，传感器逐步发现实验台/通风橱/试剂架。"""
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
        map_file=str(PKG / "maps" / "laboratory.json"),
        planner_kind="astar",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_z_bounds=(2.0, 3.2),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.5,
        planner_replan_interval=0.5,
        planner_horizon=5.0,
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
        map_file=str(PKG / "maps" / "laboratory.json"),
        planner_kind="hybrid_astar",
        planner_resolution=0.3,
        safety_margin=0.3,
        planner_z_bounds=(2.0, 3.2),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.5,
        planner_replan_interval=0.5,
        planner_horizon=5.0,
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

