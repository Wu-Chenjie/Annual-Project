from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np


@dataclass(frozen=True)
class DroneParams:
    name: str
    mass: float
    inertia: tuple[float, float, float] = (0.01, 0.01, 0.02)
    arm_length: float = 0.2
    k_drag: float = 0.05
    j_rotor: float = 3.0e-5
    kf: float = 1.0e-5
    km: float = 2.0e-7
    tau_motor: float = 0.02
    omega_max: float = 1000.0
    rotor_model: str = "simple"
    bem_radius: float = 0.12
    bem_chord: float = 0.02
    bem_num_blades: int = 2
    bem_theta_tip: float = 0.05
    bem_theta_root: float = 0.40
    notes: str = ""
    hover_omega: float = field(init=False)

    def __post_init__(self) -> None:
        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        if any(v <= 0.0 for v in self.inertia):
            raise ValueError("inertia diagonal entries must be positive")
        if self.arm_length <= 0.0:
            raise ValueError("arm_length must be positive")
        if self.kf <= 0.0 or self.km <= 0.0:
            raise ValueError("kf and km must be positive")
        if self.tau_motor < 0.0:
            raise ValueError("tau_motor must be non-negative")
        if self.omega_max <= 0.0:
            raise ValueError("omega_max must be positive")
        if self.rotor_model not in {"simple", "bem"}:
            raise ValueError("rotor_model must be 'simple' or 'bem'")

        hover_omega = float(np.sqrt(self.mass * 9.81 / (4.0 * self.kf)))
        if hover_omega >= self.omega_max:
            raise ValueError("profile cannot hover within omega_max limit")
        object.__setattr__(self, "hover_omega", hover_omega)

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag(self.inertia).astype(float)

    def to_dict(self) -> dict:
        data = asdict(self)
        data["inertia"] = list(self.inertia)
        return data


DRONE_PARAM_PROFILES: dict[str, DroneParams] = {
    "default_1kg": DroneParams(
        name="default_1kg",
        mass=1.0,
        inertia=(0.01, 0.01, 0.02),
        arm_length=0.2,
        k_drag=0.05,
        j_rotor=3.0e-5,
        kf=1.0e-5,
        km=2.0e-7,
        tau_motor=0.02,
        omega_max=1000.0,
        rotor_model="simple",
        notes="Regression baseline matching the historical hard-coded drone model.",
    ),
    "indoor_micro": DroneParams(
        name="indoor_micro",
        mass=0.027,
        inertia=(1.4e-5, 1.4e-5, 2.2e-5),
        arm_length=0.046,
        k_drag=0.01,
        j_rotor=1.5e-7,
        kf=1.7e-8,
        km=2.4e-10,
        tau_motor=0.015,
        omega_max=2200.0,
        rotor_model="simple",
        bem_radius=0.022,
        bem_chord=0.007,
        notes="Indoor micro-UAV profile for Crazyflie-scale comparison studies.",
    ),
    "light_uav_regulatory": DroneParams(
        name="light_uav_regulatory",
        mass=1.8,
        inertia=(0.021, 0.021, 0.038),
        arm_length=0.28,
        k_drag=0.08,
        j_rotor=5.0e-5,
        kf=1.5e-5,
        km=3.0e-7,
        tau_motor=0.03,
        omega_max=1100.0,
        rotor_model="bem",
        bem_radius=0.14,
        bem_chord=0.026,
        notes="Light UAV profile aligned to low-altitude regulatory demo assumptions.",
    ),
}


def get_drone_params(profile: str | DroneParams = "default_1kg") -> DroneParams:
    if isinstance(profile, DroneParams):
        return profile
    try:
        return DRONE_PARAM_PROFILES[profile]
    except KeyError as exc:
        raise ValueError(f"Unknown drone_profile: {profile}") from exc
