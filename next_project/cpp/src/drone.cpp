#include "drone.hpp"

#include <cmath>

namespace sim {

namespace {

Vec3 inertia_mul(const std::array<double, 3>& diag, const Vec3& v) {
    return Vec3{diag[0] * v.x, diag[1] * v.y, diag[2] * v.z};
}

std::array<double, 4> quat_multiply(const std::array<double, 4>& p, const std::array<double, 4>& q) {
    return std::array<double, 4>{
        p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
        p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
        p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
        p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0],
    };
}

std::array<double, 4> euler_to_quat(const Vec3& euler) {
    const double cr = std::cos(euler.x / 2.0);
    const double sr = std::sin(euler.x / 2.0);
    const double cp = std::cos(euler.y / 2.0);
    const double sp = std::sin(euler.y / 2.0);
    const double cy = std::cos(euler.z / 2.0);
    const double sy = std::sin(euler.z / 2.0);

    return std::array<double, 4>{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
}

Vec3 quat_to_euler(const std::array<double, 4>& q) {
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];

    const double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    const double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0);
    const double pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    const double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return Vec3{roll, pitch, yaw};
}

Mat3 quat_to_rotation_matrix(const std::array<double, 4>& q) {
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];

    const double q0q0 = q0 * q0;
    const double q1q1 = q1 * q1;
    const double q2q2 = q2 * q2;
    const double q3q3 = q3 * q3;
    const double q0q1 = q0 * q1;
    const double q0q2 = q0 * q2;
    const double q0q3 = q0 * q3;
    const double q1q2 = q1 * q2;
    const double q1q3 = q1 * q3;
    const double q2q3 = q2 * q3;

    Mat3 R{};
    R.m = {{
        {{q0q0 + q1q1 - q2q2 - q3q3, 2.0 * (q1q2 - q0q3), 2.0 * (q1q3 + q0q2)}},
        {{2.0 * (q1q2 + q0q3), q0q0 - q1q1 + q2q2 - q3q3, 2.0 * (q2q3 - q0q1)}},
        {{2.0 * (q1q3 - q0q2), 2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3}},
    }};
    return R;
}

std::array<double, 13> state_add_scaled13(
    const std::array<double, 13>& base,
    const std::array<double, 13>& delta,
    double scale
) {
    std::array<double, 13> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = base[i] + scale * delta[i];
    }
    return out;
}

}  // namespace

Drone::Drone(double m, const std::array<double, 3>& inertia_diag, double arm_length, double dt)
    : m_(m),
      inertia_diag_(inertia_diag),
      inertia_inv_diag_{1.0 / inertia_diag[0], 1.0 / inertia_diag[1], 1.0 / inertia_diag[2]},
      arm_length_(arm_length),
      dt_(dt),
      rotors_{Rotor(1e-5, 2e-7, 1), Rotor(1e-5, 2e-7, -1), Rotor(1e-5, 2e-7, 1), Rotor(1e-5, 2e-7, -1)},
      allocator_(arm_length_, rotors_[0].kf(), rotors_[0].km()) {
    state_.fill(0.0);
}

void Drone::set_initial_state(const Vec3& position, const Vec3& velocity,
                              const Vec3& attitude, const Vec3& angular_velocity,
                              double dt_override) {
    if (dt_override > 0.0) {
        dt_ = dt_override;
    }

    state_[0] = position.x;
    state_[1] = position.y;
    state_[2] = position.z;
    state_[3] = velocity.x;
    state_[4] = velocity.y;
    state_[5] = velocity.z;
    state_[6] = attitude.x;
    state_[7] = attitude.y;
    state_[8] = attitude.z;
    state_[9] = angular_velocity.x;
    state_[10] = angular_velocity.y;
    state_[11] = angular_velocity.z;
}

void Drone::set_wind(const Vec3& wind) {
    wind_ = wind;
}

void Drone::inject_fault(int rotor_index, double severity) {
    if (rotor_index < 0 || rotor_index >= static_cast<int>(fault_mask_.size())) return;
    fault_mask_[static_cast<std::size_t>(rotor_index)] = clamp(severity, 0.0, 1.0);
    fault_active_ = true;
}

void Drone::clear_fault() {
    fault_mask_.fill(1.0);
    fault_active_ = false;
}

std::array<double, 4> Drone::rotor_step(const std::array<double, 4>& desired_u) {
    std::array<double, 4> thrusts_cmd = allocator_.allocate_thrusts(desired_u);
    for (std::size_t i = 0; i < thrusts_cmd.size(); ++i) {
        thrusts_cmd[i] *= fault_mask_[i];
    }
    const std::array<double, 4> omegas_cmd = allocator_.thrusts_to_omegas(thrusts_cmd);

    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        rotors_[i].update(omegas_cmd[i], dt_);
    }

    std::array<double, 4> omegas{};
    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        omegas[i] = rotors_[i].omega();
    }

    return allocator_.omegas_to_u(omegas);
}

std::array<double, 12> Drone::dynamics(const std::array<double, 12>& state,
                                       const std::array<double, 4>& u,
                                       const Vec3& wind) const {
    const double vx = state[3];
    const double vy = state[4];
    const double vz = state[5];
    const double roll = state[6];
    const double pitch = state[7];
    const double yaw = state[8];
    const double wx = state[9];
    const double wy = state[10];
    const double wz = state[11];

    const Mat3 R = rotation_matrix(roll, pitch, yaw);

    const Vec3 v{vx, vy, vz};
    const Vec3 v_rel = v - wind;
    const Vec3 drag = (-k_drag_ * norm(v_rel)) * v_rel;

    const Vec3 thrust_in_inertial = R * Vec3{0.0, 0.0, u[0]};
    const Vec3 gravity{0.0, 0.0, m_ * g_};
    const Vec3 acc = (thrust_in_inertial + drag - gravity) / m_;

    const Vec3 omega{wx, wy, wz};
    const Vec3 tau{u[1], u[2], u[3]};

    double omega_net = 0.0;
    for (const Rotor& rotor : rotors_) {
        omega_net += static_cast<double>(rotor.direction()) * rotor.omega();
    }

    const Vec3 tau_gyro = (-j_rotor_ * omega_net) * cross(omega, Vec3{0.0, 0.0, 1.0});
    const Vec3 inertia_omega = inertia_mul(inertia_diag_, omega);
    const Vec3 omega_dot_body = tau + tau_gyro - cross(omega, inertia_omega);
    const Vec3 omega_dot{
        inertia_inv_diag_[0] * omega_dot_body.x,
        inertia_inv_diag_[1] * omega_dot_body.y,
        inertia_inv_diag_[2] * omega_dot_body.z,
    };

    double c_pitch = std::cos(pitch);
    if (std::abs(c_pitch) < 1e-6) {
        c_pitch = (c_pitch >= 0.0) ? 1e-6 : -1e-6;
    }
    const double t_pitch = std::sin(pitch) / c_pitch;
    const double c_roll = std::cos(roll);
    const double s_roll = std::sin(roll);

    const double roll_dot = wx + s_roll * t_pitch * wy + c_roll * t_pitch * wz;
    const double pitch_dot = c_roll * wy - s_roll * wz;
    const double yaw_dot = (s_roll / c_pitch) * wy + (c_roll / c_pitch) * wz;

    std::array<double, 12> out{};
    out[0] = vx;
    out[1] = vy;
    out[2] = vz;
    out[3] = acc.x;
    out[4] = acc.y;
    out[5] = acc.z;
    out[6] = roll_dot;
    out[7] = pitch_dot;
    out[8] = yaw_dot;
    out[9] = omega_dot.x;
    out[10] = omega_dot.y;
    out[11] = omega_dot.z;
    return out;
}

void Drone::update_state(const std::array<double, 4>& control, const Vec3& wind) {
    const std::array<double, 4> control_eff = rotor_step(control);

    const std::array<double, 12> k1 = dynamics(state_, control_eff, wind);
    const std::array<double, 12> k2 = dynamics(state_add_scaled(state_, k1, 0.5 * dt_), control_eff, wind);
    const std::array<double, 12> k3 = dynamics(state_add_scaled(state_, k2, 0.5 * dt_), control_eff, wind);
    const std::array<double, 12> k4 = dynamics(state_add_scaled(state_, k3, dt_), control_eff, wind);

    for (std::size_t i = 0; i < state_.size(); ++i) {
        state_[i] += (dt_ / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

KinematicState Drone::get_state() const {
    return KinematicState{
        Vec3{state_[0], state_[1], state_[2]},
        Vec3{state_[3], state_[4], state_[5]},
        Vec3{state_[6], state_[7], state_[8]},
        Vec3{state_[9], state_[10], state_[11]},
    };
}

QuaternionDrone::QuaternionDrone(double m, const std::array<double, 3>& inertia_diag, double arm_length, double dt)
    : m_(m),
      inertia_diag_(inertia_diag),
      inertia_inv_diag_{1.0 / inertia_diag[0], 1.0 / inertia_diag[1], 1.0 / inertia_diag[2]},
      arm_length_(arm_length),
      dt_(dt),
      rotors_{Rotor(1e-5, 2e-7, 1), Rotor(1e-5, 2e-7, -1), Rotor(1e-5, 2e-7, 1), Rotor(1e-5, 2e-7, -1)},
      allocator_(arm_length_, rotors_[0].kf(), rotors_[0].km()) {
    state_.fill(0.0);
    state_[6] = 1.0;
}

void QuaternionDrone::set_initial_state(const Vec3& position, const Vec3& velocity,
                                        const Vec3& attitude, const Vec3& angular_velocity,
                                        double dt_override) {
    if (dt_override > 0.0) {
        dt_ = dt_override;
    }

    state_[0] = position.x;
    state_[1] = position.y;
    state_[2] = position.z;
    state_[3] = velocity.x;
    state_[4] = velocity.y;
    state_[5] = velocity.z;

    const std::array<double, 4> q = euler_to_quat(attitude);
    state_[6] = q[0];
    state_[7] = q[1];
    state_[8] = q[2];
    state_[9] = q[3];

    state_[10] = angular_velocity.x;
    state_[11] = angular_velocity.y;
    state_[12] = angular_velocity.z;
}

void QuaternionDrone::set_wind(const Vec3& wind) {
    wind_ = wind;
}

void QuaternionDrone::inject_fault(int rotor_index, double severity) {
    if (rotor_index < 0 || rotor_index >= static_cast<int>(fault_mask_.size())) return;
    fault_mask_[static_cast<std::size_t>(rotor_index)] = clamp(severity, 0.0, 1.0);
    fault_active_ = true;
}

void QuaternionDrone::clear_fault() {
    fault_mask_.fill(1.0);
    fault_active_ = false;
}

std::array<double, 4> QuaternionDrone::rotor_step(const std::array<double, 4>& desired_u) {
    std::array<double, 4> thrusts_cmd = allocator_.allocate_thrusts(desired_u);
    for (std::size_t i = 0; i < thrusts_cmd.size(); ++i) {
        thrusts_cmd[i] *= fault_mask_[i];
    }
    const std::array<double, 4> omegas_cmd = allocator_.thrusts_to_omegas(thrusts_cmd);

    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        rotors_[i].update(omegas_cmd[i], dt_);
    }

    std::array<double, 4> omegas{};
    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        omegas[i] = rotors_[i].omega();
    }

    return allocator_.omegas_to_u(omegas);
}

std::array<double, 13> QuaternionDrone::dynamics(const std::array<double, 13>& state,
                                                  const std::array<double, 4>& u,
                                                  const Vec3& wind) const {
    const Vec3 vel{state[3], state[4], state[5]};
    const std::array<double, 4> quat{state[6], state[7], state[8], state[9]};
    const Vec3 omega{state[10], state[11], state[12]};

    const Mat3 R = quat_to_rotation_matrix(quat);
    const Vec3 v_rel = vel - wind;
    const Vec3 drag = (-k_drag_ * norm(v_rel)) * v_rel;

    const Vec3 thrust_in_inertial = R * Vec3{0.0, 0.0, u[0]};
    const Vec3 gravity{0.0, 0.0, m_ * g_};
    const Vec3 acc = (thrust_in_inertial + drag - gravity) / m_;

    const Vec3 tau{u[1], u[2], u[3]};
    double omega_net = 0.0;
    for (const Rotor& rotor : rotors_) {
        omega_net += static_cast<double>(rotor.direction()) * rotor.omega();
    }

    const Vec3 tau_gyro = (-j_rotor_ * omega_net) * cross(omega, Vec3{0.0, 0.0, 1.0});
    const Vec3 inertia_omega = inertia_mul(inertia_diag_, omega);
    const Vec3 omega_dot_body = tau + tau_gyro - cross(omega, inertia_omega);
    const Vec3 omega_dot{
        inertia_inv_diag_[0] * omega_dot_body.x,
        inertia_inv_diag_[1] * omega_dot_body.y,
        inertia_inv_diag_[2] * omega_dot_body.z,
    };

    const std::array<double, 4> omega_quat{0.0, omega.x, omega.y, omega.z};
    const std::array<double, 4> quat_dot_raw = quat_multiply(quat, omega_quat);
    const std::array<double, 4> quat_dot{
        0.5 * quat_dot_raw[0],
        0.5 * quat_dot_raw[1],
        0.5 * quat_dot_raw[2],
        0.5 * quat_dot_raw[3],
    };

    std::array<double, 13> out{};
    out[0] = vel.x;
    out[1] = vel.y;
    out[2] = vel.z;
    out[3] = acc.x;
    out[4] = acc.y;
    out[5] = acc.z;
    out[6] = quat_dot[0];
    out[7] = quat_dot[1];
    out[8] = quat_dot[2];
    out[9] = quat_dot[3];
    out[10] = omega_dot.x;
    out[11] = omega_dot.y;
    out[12] = omega_dot.z;
    return out;
}

void QuaternionDrone::update_state(const std::array<double, 4>& control, const Vec3& wind) {
    const std::array<double, 4> control_eff = rotor_step(control);

    const std::array<double, 13> k1 = dynamics(state_, control_eff, wind);
    const std::array<double, 13> k2 = dynamics(state_add_scaled13(state_, k1, 0.5 * dt_), control_eff, wind);
    const std::array<double, 13> k3 = dynamics(state_add_scaled13(state_, k2, 0.5 * dt_), control_eff, wind);
    const std::array<double, 13> k4 = dynamics(state_add_scaled13(state_, k3, dt_), control_eff, wind);

    for (std::size_t i = 0; i < state_.size(); ++i) {
        state_[i] += (dt_ / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    const double q_norm = std::sqrt(
        state_[6] * state_[6] + state_[7] * state_[7] + state_[8] * state_[8] + state_[9] * state_[9]
    );
    if (q_norm > 1e-10) {
        state_[6] /= q_norm;
        state_[7] /= q_norm;
        state_[8] /= q_norm;
        state_[9] /= q_norm;
    } else {
        state_[6] = 1.0;
        state_[7] = 0.0;
        state_[8] = 0.0;
        state_[9] = 0.0;
    }
}

KinematicState QuaternionDrone::get_state() const {
    const std::array<double, 4> q{state_[6], state_[7], state_[8], state_[9]};
    const Vec3 euler = quat_to_euler(q);
    return KinematicState{
        Vec3{state_[0], state_[1], state_[2]},
        Vec3{state_[3], state_[4], state_[5]},
        euler,
        Vec3{state_[10], state_[11], state_[12]},
    };
}

}  // namespace sim
