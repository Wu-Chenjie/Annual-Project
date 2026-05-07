from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from core.drone import Drone
from core.drone_params import DRONE_PARAM_PROFILES, DroneParams, get_drone_params
from core.rotor import BEMRotor
from simulations.formation_simulation import FormationSimulation, SimulationConfig


def test_drone_params_registry_contains_expected_profiles():
    assert {"default_1kg", "indoor_micro", "light_uav_regulatory"} <= set(DRONE_PARAM_PROFILES)


def test_default_1kg_profile_preserves_historical_baseline():
    params = get_drone_params("default_1kg")
    assert params.mass == 1.0
    assert np.allclose(params.inertia_matrix, np.diag([0.01, 0.01, 0.02]))
    assert params.arm_length == 0.2
    assert params.kf == 1.0e-5
    assert params.km == 2.0e-7


def test_drone_params_post_init_rejects_unhoverable_profile():
    try:
        DroneParams(name="bad", mass=1.0, kf=1.0e-7, omega_max=100.0)
    except ValueError as exc:
        assert "hover" in str(exc)
    else:
        raise AssertionError("expected invalid DroneParams to raise ValueError")


def test_drone_uses_bem_rotor_when_profile_requests_it():
    drone = Drone(drone_profile="light_uav_regulatory")
    assert all(isinstance(rotor, BEMRotor) for rotor in drone.rotors)


def test_formation_simulation_uses_selected_drone_profile():
    config = SimulationConfig(num_followers=1, max_sim_time=0.024, drone_profile="indoor_micro")
    sim = FormationSimulation(config=config)
    assert sim.leader.params.name == "indoor_micro"
    assert sim.followers[0].params.name == "indoor_micro"
    assert sim.leader_ctrl.m == sim.drone_params.mass
