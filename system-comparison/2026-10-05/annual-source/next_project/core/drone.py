"""Quadrotor dynamics models used by the simulation stack."""

from __future__ import annotations

import numpy as np

from .allocator import ControlAllocator
from .drone_params import DroneParams, get_drone_params
from .rotor import BEMRotor, Rotor


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the ZYX Euler rotation matrix."""
    c_r, s_r = np.cos(roll), np.sin(roll)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    c_y, s_y = np.cos(yaw), np.sin(yaw)

    return np.array([
        [c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r],
        [s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r],
        [-s_p, c_p * s_r, c_p * c_r],
    ], dtype=float)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles to a quaternion."""
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ], dtype=float)


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert a quaternion to Euler angles."""
    q0, q1, q2, q3 = q
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)


def quat_multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product."""
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return np.array([
        p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
        p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
        p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
        p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
    ], dtype=float)


class Drone:
    """Euler-angle quadrotor dynamics."""

    def __init__(
        self,
        m: float = 1.0,
        inertia=None,
        arm_length: float = 0.2,
        dt: float = 0.01,
        *,
        params: DroneParams | None = None,
        drone_profile: str = "default_1kg",
    ):
        self.params = self._resolve_params(m, inertia, arm_length, params, drone_profile)
        self.m = float(self.params.mass)
        self.I = self.params.inertia_matrix if inertia is None else np.array(inertia, dtype=float)
        self.I_inv = np.linalg.inv(self.I)
        self.g = 9.81
        self.L = float(self.params.arm_length if arm_length == 0.2 else arm_length)
        self.dt = float(dt)
        self.k_drag = float(self.params.k_drag)
        self.wind = np.zeros(3, dtype=float)
        self.J_rotor = float(self.params.j_rotor)
        self.state = np.zeros(12, dtype=float)
        self.rotors = [
            self._build_rotor(direction=1),
            self._build_rotor(direction=-1),
            self._build_rotor(direction=1),
            self._build_rotor(direction=-1),
        ]
        self.allocator = ControlAllocator(arm_length=self.L, kf=self.rotors[0].kf, km=self.rotors[0].km)

        self.fault_mask = np.ones(4, dtype=float)
        self.fault_active = False

    @staticmethod
    def _resolve_params(
        m: float,
        inertia,
        arm_length: float,
        params: DroneParams | None,
        drone_profile: str,
    ) -> DroneParams:
        if params is not None:
            return params
        if inertia is None and arm_length == 0.2 and m == 1.0:
            return get_drone_params(drone_profile)
        return DroneParams(
            name="custom",
            mass=float(m),
            inertia=(0.01, 0.01, 0.02) if inertia is None else tuple(np.array(inertia, dtype=float).tolist()),
            arm_length=float(arm_length),
            notes="Constructed from legacy Drone constructor arguments.",
        )

    def _build_rotor(self, direction: int) -> Rotor:
        rotor_kwargs = dict(
            kf=self.params.kf,
            km=self.params.km,
            direction=direction,
            tau_motor=self.params.tau_motor,
            omega_max=self.params.omega_max,
        )
        if self.params.rotor_model == "bem":
            return BEMRotor(
                **rotor_kwargs,
                num_blades=self.params.bem_num_blades,
                radius=self.params.bem_radius,
                chord=self.params.bem_chord,
                theta_tip=self.params.bem_theta_tip,
                theta_root=self.params.bem_theta_root,
            )
        return Rotor(**rotor_kwargs)

    def set_initial_state(self, position, velocity, attitude=None, angular_velocity=None, dt=None):
        if attitude is None:
            attitude = [0.0, 0.0, 0.0]
        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        if dt is not None:
            self.dt = float(dt)
        self.state[0:3] = np.array(position, dtype=float)
        self.state[3:6] = np.array(velocity, dtype=float)
        self.state[6:9] = np.array(attitude, dtype=float)
        self.state[9:12] = np.array(angular_velocity, dtype=float)

    def inject_fault(self, rotor_index: int, severity: float) -> None:
        if 0 <= rotor_index < 4:
            self.fault_mask[rotor_index] = float(np.clip(severity, 0.0, 1.0))
            self.fault_active = True

    def clear_faults(self) -> None:
        self.fault_mask = np.ones(4, dtype=float)
        self.fault_active = False

    def set_wind(self, wind) -> None:
        self.wind = np.array(wind, dtype=float)

    def _resolve_wind(self, wind, position=None) -> np.ndarray:
        if wind is None:
            return self.wind
        if callable(wind):
            pos = self.state[0:3] if position is None else position
            return np.array(wind(pos, self.dt), dtype=float)
        return np.array(wind, dtype=float)

    def _rotor_step(self, desired_u) -> np.ndarray:
        thrusts_cmd = self.allocator.allocate_thrusts(desired_u)
        thrusts_cmd = thrusts_cmd * self.fault_mask
        max_thrust = self.rotors[0].kf * (self.rotors[0].omega_max ** 2)
        thrusts_cmd = np.clip(thrusts_cmd, 0.0, max_thrust)
        omegas_cmd = self.allocator.thrusts_to_omegas(thrusts_cmd)
        for rotor, omega_cmd in zip(self.rotors, omegas_cmd):
            rotor.update(omega_cmd, self.dt)
        omegas = np.array([r.omega for r in self.rotors], dtype=float)
        return self.allocator.omegas_to_u(omegas)

    def dynamics(self, t, state, u, wind=None) -> np.ndarray:
        _, _, _, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = state
        R = rotation_matrix(roll, pitch, yaw)
        wind_vec = self._resolve_wind(wind, position=state[0:3])
        v_vector = np.array([vx, vy, vz], dtype=float)
        v_rel = v_vector - wind_vec
        drag = -self.k_drag * np.linalg.norm(v_rel) * v_rel
        acc = (R @ np.array([0.0, 0.0, u[0]], dtype=float) + drag - np.array([0.0, 0.0, self.m * self.g], dtype=float)) / self.m

        omega = np.array([wx, wy, wz], dtype=float)
        tau = np.array([u[1], u[2], u[3]], dtype=float)
        omega_net = sum(r.direction * r.omega for r in self.rotors)
        tau_gyro = -self.J_rotor * omega_net * np.cross(omega, np.array([0.0, 0.0, 1.0]))
        omega_dot = self.I_inv @ (tau + tau_gyro - np.cross(omega, self.I @ omega))

        c_pitch = np.cos(pitch)
        if abs(c_pitch) < 1e-2:
            c_pitch = 1e-2 if c_pitch >= 0 else -1e-2
        t_pitch = np.clip(np.sin(pitch) / c_pitch, -50.0, 50.0)
        sec_pitch = np.clip(1.0 / c_pitch, -50.0, 50.0)
        c_roll = np.cos(roll)
        s_roll = np.sin(roll)
        roll_dot = wx + s_roll * t_pitch * wy + c_roll * t_pitch * wz
        pitch_dot = c_roll * wy - s_roll * wz
        yaw_dot = (s_roll * sec_pitch) * wy + (c_roll * sec_pitch) * wz

        return np.array([
            vx, vy, vz,
            acc[0], acc[1], acc[2],
            roll_dot, pitch_dot, yaw_dot,
            omega_dot[0], omega_dot[1], omega_dot[2],
        ], dtype=float)

    def update_state(self, control, wind=None) -> None:
        control = np.array(control, dtype=float)
        if control.shape[0] == 4:
            control_eff = self._rotor_step(control)
        elif control.shape[0] == 3:
            total_thrust = self.m * (self.g + control[2])
            control_eff = np.array([total_thrust, 0.0, 0.0, 0.0], dtype=float)
        else:
            raise ValueError("control must be a 3D or 4D vector")

        dt = self.dt
        state = self.state
        k1 = self.dynamics(0.0, state, control_eff, wind)
        k2 = self.dynamics(0.0, state + 0.5 * dt * k1, control_eff, wind)
        k3 = self.dynamics(0.0, state + 0.5 * dt * k2, control_eff, wind)
        k4 = self.dynamics(0.0, state + dt * k3, control_eff, wind)
        self.state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def get_state(self):
        return self.state[0:3].copy(), self.state[3:6].copy(), self.state[6:9].copy(), self.state[9:12].copy()


class QuaternionDrone(Drone):
    """Quaternion-state quadrotor dynamics."""

    def __init__(
        self,
        m: float = 1.0,
        inertia=None,
        arm_length: float = 0.2,
        dt: float = 0.01,
        *,
        params: DroneParams | None = None,
        drone_profile: str = "default_1kg",
    ):
        super().__init__(
            m=m,
            inertia=inertia,
            arm_length=arm_length,
            dt=dt,
            params=params,
            drone_profile=drone_profile,
        )
        self.state = np.zeros(13, dtype=float)
        self.state[6] = 1.0

    def set_initial_state(self, position, velocity, attitude=None, angular_velocity=None, dt=None):
        if attitude is None:
            attitude = [0.0, 0.0, 0.0]
        if angular_velocity is None:
            angular_velocity = [0.0, 0.0, 0.0]
        if dt is not None:
            self.dt = float(dt)
        self.state[0:3] = np.array(position, dtype=float)
        self.state[3:6] = np.array(velocity, dtype=float)
        self.state[6:10] = euler_to_quat(*np.array(attitude, dtype=float))
        self.state[10:13] = np.array(angular_velocity, dtype=float)

    def get_state(self):
        pos = self.state[0:3].copy()
        vel = self.state[3:6].copy()
        euler = quat_to_euler(self.state[6:10])
        ang_vel = self.state[10:13].copy()
        return pos, vel, euler, ang_vel

    def dynamics(self, t, state, u, wind=None) -> np.ndarray:
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        R = self._quat_to_rotation_matrix(quat)
        wind_vec = self._resolve_wind(wind, position=pos)
        v_rel = vel - wind_vec
        drag = -self.k_drag * np.linalg.norm(v_rel) * v_rel
        acc = (R @ np.array([0.0, 0.0, u[0]], dtype=float) + drag - np.array([0.0, 0.0, self.m * self.g], dtype=float)) / self.m
        tau = np.array([u[1], u[2], u[3]], dtype=float)
        omega_net = sum(r.direction * r.omega for r in self.rotors)
        tau_gyro = -self.J_rotor * omega_net * np.cross(omega, np.array([0.0, 0.0, 1.0]))
        omega_dot = self.I_inv @ (tau + tau_gyro - np.cross(omega, self.I @ omega))
        q_dot = 0.5 * quat_multiply(quat, np.array([0.0, omega[0], omega[1], omega[2]], dtype=float))
        state_dot = np.zeros(13, dtype=float)
        state_dot[0:3] = vel
        state_dot[3:6] = acc
        state_dot[6:10] = q_dot
        state_dot[10:13] = omega_dot
        return state_dot

    @staticmethod
    def _quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        q0, q1, q2, q3 = q
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q3 = q2 * q3
        return np.array([
            [q0q0 + q1q1 - q2q2 - q3q3, 2.0 * (q1q2 - q0q3), 2.0 * (q1q3 + q0q2)],
            [2.0 * (q1q2 + q0q3), q0q0 - q1q1 + q2q2 - q3q3, 2.0 * (q2q3 - q0q1)],
            [2.0 * (q1q3 - q0q2), 2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3],
        ], dtype=float)

    def update_state(self, control, wind=None) -> None:
        control = np.array(control, dtype=float)
        if control.shape[0] == 4:
            control_eff = self._rotor_step(control)
        elif control.shape[0] == 3:
            total_thrust = self.m * (self.g + control[2])
            control_eff = np.array([total_thrust, 0.0, 0.0, 0.0], dtype=float)
        else:
            raise ValueError("control must be a 3D or 4D vector")

        dt = self.dt
        state = self.state

        def _normalized_state(s: np.ndarray) -> np.ndarray:
            s_norm = s.copy()
            q = s_norm[6:10]
            q_norm = np.linalg.norm(q)
            if q_norm > 1e-10:
                s_norm[6:10] = q / q_norm
            else:
                s_norm[6:10] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            return s_norm

        k1 = self.dynamics(0.0, state, control_eff, wind)
        k2 = self.dynamics(0.0, _normalized_state(state + 0.5 * dt * k1), control_eff, wind)
        k3 = self.dynamics(0.0, _normalized_state(state + 0.5 * dt * k2), control_eff, wind)
        k4 = self.dynamics(0.0, _normalized_state(state + dt * k3), control_eff, wind)
        self.state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        q_norm = np.linalg.norm(self.state[6:10])
        if q_norm > 1e-10:
            self.state[6:10] /= q_norm
        else:
            self.state[6:10] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
