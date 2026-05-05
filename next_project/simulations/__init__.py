"""仿真层模块聚合。

用途：集中管理所有仿真场景入口。
原理：把单个场景封装为可复用对象，便于后续扩展多机、多场景和批量评测。
"""

from .formation_simulation import FormationSimulation, SimulationConfig
from .visualization import SimulationVisualizer
from .obstacle_scenario import ObstacleScenarioSimulation, make_planner

__all__ = ["FormationSimulation", "SimulationConfig", "SimulationVisualizer", "ObstacleScenarioSimulation", "make_planner"]
