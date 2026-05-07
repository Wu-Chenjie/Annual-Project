"""核心模块聚合。

用途：统一导出动力学、控制与环境建模类，便于上层仿真入口调用。
原理：采用分层封装，把旋翼、控制分配、风场、无人机动力学与控制器拆成独立对象，再由仿真器组合。
"""

from .rotor import Rotor, BEMRotor
from .allocator import ControlAllocator
from .wind_field import WindField
from .drone import Drone, QuaternionDrone
from .controller import Controller, HybridAttitudeController, BacksteppingController, GeometricSE3Controller
from .smc import SecondOrderSMC, SuperTwistingSMC
from .topology import FormationTopology
from .obstacles import AABB, Sphere, Cylinder, ObstacleField, OccupancyGrid
from .sensors import RangeSensor6
from .map_loader import load_from_json, load_from_npz
from .artificial_potential_field import ImprovedArtificialPotentialField
from .formation_safety import (
    FormationSafetyConfig,
    DownwashZone,
    downwash_zone,
    is_in_downwash_zone,
    min_inter_drone_distance,
    follower_safety_correction,
    deconflict_follower_target,
)
from .planning import AStar, TurnConstrainedAStar, HybridAStar, Dijkstra, RRTStar, Planner, PlannerError, WindowReplanner

__all__ = [
    "Rotor",
    "BEMRotor",
    "ControlAllocator",
    "WindField",
    "Drone",
    "QuaternionDrone",
    "Controller",
    "HybridAttitudeController",
    "BacksteppingController",
    "GeometricSE3Controller",
    "SecondOrderSMC",
    "SuperTwistingSMC",
    "FormationTopology",
    "AABB",
    "Sphere",
    "Cylinder",
    "ObstacleField",
    "OccupancyGrid",
    "RangeSensor6",
    "load_from_json",
    "load_from_npz",
    "AStar",
    "TurnConstrainedAStar",
    "HybridAStar",
    "Dijkstra",
    "RRTStar",
    "Planner",
    "PlannerError",
    "WindowReplanner",
    "ImprovedArtificialPotentialField",
    "FormationSafetyConfig",
    "DownwashZone",
    "downwash_zone",
    "is_in_downwash_zone",
    "min_inter_drone_distance",
    "follower_safety_correction",
    "deconflict_follower_target",
]
