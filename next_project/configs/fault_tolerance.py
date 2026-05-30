from __future__ import annotations

import numpy as np
from pathlib import Path

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent

from .unknown_map import _UNKNOWN_MAP_DEFAULTS

def _config_fault_tolerance() -> SimulationConfig:
    """容错测试场景：注入单机故障，验证拓扑重构。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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
    """容错在线版：故障注入 + 在线重规划 + 拓扑重构。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
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


def _config_fault_tolerance_unknown() -> SimulationConfig:
    """容错测试完全未知版：初始地图为空 + 在线传感器 + 故障注入 + 拓扑重构。"""
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
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
        map_file=str(PKG / "maps" / "sample_warehouse.json"),
        planner_kind="astar",
        planner_resolution=0.4,
        safety_margin=0.3,
        **_UNKNOWN_MAP_DEFAULTS,
        sensor_max_range=5.0,
        planner_replan_interval=0.4,
        planner_horizon=6.0,
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

