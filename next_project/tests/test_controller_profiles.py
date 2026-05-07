from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.controller import Controller, HybridAttitudeController, BacksteppingController, GeometricSE3Controller
from simulations.formation_simulation import FormationSimulation, SimulationConfig


def test_controller_signatures_accept_target_yaw_rate():
    state = np.zeros(12, dtype=float)
    target = np.array([0.0, 0.0, 1.0], dtype=float)
    for ctrl in [
        Controller(),
        HybridAttitudeController(),
        BacksteppingController(),
        GeometricSE3Controller(),
    ]:
        u = ctrl.compute_control(
            state,
            target,
            target_vel=np.zeros(3, dtype=float),
            target_acc=np.zeros(3, dtype=float),
            target_yaw=0.1,
            target_yaw_rate=0.2,
        )
        assert u.shape == (4,)
        assert np.all(np.isfinite(u))


def test_geometric_controller_roll_error_commands_correct_direction():
    ctrl = GeometricSE3Controller()
    state = np.zeros(12, dtype=float)
    state[6] = 0.1
    u = ctrl.compute_control(
        state,
        target_pos=np.array([0.0, 0.0, 1.0], dtype=float),
        target_vel=np.zeros(3, dtype=float),
        target_acc=np.zeros(3, dtype=float),
        target_yaw=0.0,
    )
    assert u[1] < 0.0


def test_formation_simulation_uses_geometric_controller_when_selected():
    sim = FormationSimulation(
        config=SimulationConfig(
            num_followers=1,
            max_sim_time=0.024,
            controller_kind="se3_geometric",
        )
    )
    assert isinstance(sim.leader_ctrl, GeometricSE3Controller)
    assert isinstance(sim.follower_ctrls[0], GeometricSE3Controller)


def test_formation_simulation_preserves_legacy_backstepping_mapping():
    sim = FormationSimulation(
        config=SimulationConfig(
            num_followers=1,
            max_sim_time=0.024,
            use_backstepping=True,
            controller_kind="pid_smc",
        )
    )
    assert isinstance(sim.leader_ctrl, BacksteppingController)
