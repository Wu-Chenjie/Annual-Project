from __future__ import annotations

import numpy as np
from pathlib import Path

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent

from .unknown_map import _UNKNOWN_MAP_DEFAULTS

def _config_rrt_dual_channel_online() -> SimulationConfig:
    """RRT双通道绕行对照图：局部前瞻假死路 + RRT旁路escape + 编队提前变换。"""
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
        map_file=str(PKG / "maps" / "rrt_dual_channel_escape.json"),
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


def _config_rrt_dual_channel_online_unknown() -> SimulationConfig:
    """RRT 双通道绕行完全未知版：初始地图为空 + 前瞻窗口 + RRT escape，传感器逐步发现通道。"""
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
        map_file=str(PKG / "maps" / "rrt_dual_channel_escape.json"),
        planner_kind="astar",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_z_bounds=(1.4, 2.4),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.0,
        planner_replan_interval=0.4,
        planner_horizon=3.5,
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


def _config_formation_maze_stress_online() -> SimulationConfig:
    """编队迷宫压力图：窄门、假分支、RRT旁路、U形转弯与宽区恢复。"""
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
        map_file=str(PKG / "maps" / "formation_maze_stress.json"),
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
            np.array([5.0, 4.0, 1.8], dtype=float),
            np.array([10.0, 2.0, 1.8], dtype=float),
            np.array([10.0, -2.5, 1.8], dtype=float),
            np.array([16.5, 3.5, 1.8], dtype=float),
            np.array([20.5, 5.0, 1.8], dtype=float),
        ],
    )


def _config_formation_maze_stress_online_unknown() -> SimulationConfig:
    """编队迷宫压力完全未知版：初始地图为空，传感器逐步发现窄门/假分支/U形弯。"""
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
        map_file=str(PKG / "maps" / "formation_maze_stress.json"),
        planner_kind="astar",
        planner_resolution=0.25,
        safety_margin=0.22,
        plan_clearance_extra=0.18,
        planner_z_bounds=(1.4, 2.4),
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=4.0,
        planner_replan_interval=0.4,
        planner_horizon=4.0,
        formation_safety_enabled=True,
        formation_min_inter_drone_distance=0.35,
        formation_downwash_radius=0.45,
        formation_downwash_height=0.80,
        formation_adaptation_enabled=False,
        formation_lookahead_enabled=False,
        formation_lookahead_rrt_enabled=False,
        waypoints=[
            np.array([0.0, 0.0, 1.8], dtype=float),
            np.array([5.0, 4.0, 1.8], dtype=float),
            np.array([10.0, 2.0, 1.8], dtype=float),
            np.array([10.0, -2.5, 1.8], dtype=float),
            np.array([16.5, 3.5, 1.8], dtype=float),
            np.array([20.5, 5.0, 1.8], dtype=float),
        ],
    )

