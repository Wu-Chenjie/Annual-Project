from __future__ import annotations

import numpy as np
from pathlib import Path

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent

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
        map_file=str(PKG / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )


def _config_custom() -> SimulationConfig:
    """自定义配置模板：修改此函数中的参数即可快速实验。"""
    return SimulationConfig(
        max_sim_time=65.0,
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
        safety_margin=0.3,
        detect_margin_scale=0.5,
        planner_kind="hybrid_astar",
        planner_mode="offline",
        planner_resolution=0.4,
        planner_sdf_aware=True,
        planner_esdf_aware=True,
        sensor_enabled=False,
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

