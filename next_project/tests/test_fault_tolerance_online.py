from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import get_config
from simulations.obstacle_scenario import ObstacleScenarioSimulation


def test_fault_tolerance_online_runs_and_logs_fault_events():
    cfg = get_config("fault_tolerance_online")
    cfg.max_sim_time = 12.0
    sim = ObstacleScenarioSimulation(cfg)
    result = sim.run()
    fault_log = result.get("fault_log", [])
    assert any(item.get("type") == "inject" for item in fault_log)
