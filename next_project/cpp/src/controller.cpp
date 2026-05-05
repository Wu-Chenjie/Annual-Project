#include "controller.hpp"

#include <cmath>

namespace sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

Vec3 mul(const std::array<double, 3>& gain, const Vec3& v) {
    return Vec3{gain[0] * v.x, gain[1] * v.y, gain[2] * v.z};
}

void scale_array(std::array<double, 3>* arr, double s) {
    (*arr)[0] *= s;
    (*arr)[1] *= s;
    (*arr)[2] *= s;
}

Vec3 clip_vec_symmetric(const Vec3& v, double limit) {
    return Vec3{
        clamp(v.x, -limit, limit),
        clamp(v.y, -limit, limit),
        clamp(v.z, -limit, limit),
    };
}

}  // namespace

Controller::Controller(double m, double g, const std::array<double, 3>& inertia_diag, double dt)
    : kp_pos{1.9263, 1.9320, 2.3606},
      ki_pos{0.5982, 0.5982, 0.6183},
      kp_vel{3.6808, 3.2772, 3.2623},
      ki_vel{0.8264, 0.8244, 0.8212},
      kd_vel{1.5675, 1.5856, 1.6716},
      kp_att{8.8343, 9.2681, 4.1071},
      kp_rate{2.5789, 2.5789, 1.7014},
      ki_rate{0.7444, 0.7444, 0.7185},
      kd_rate{0.8544, 0.8544, 0.7710},
      max_acc(10.0),
    max_tilt(30.0 * kPi / 180.0),
      max_vel(10.0),
      integral_pos_limit(2.0),
      integral_vel_limit(2.0),
      integral_rate_limit(1.0),
      inertia_diag_(inertia_diag),
      m_(m),
      g_(g),
      dt_(dt) {}

void Controller::apply_profile(double gain_scale, double max_vel_in, double max_acc_in) {
    scale_array(&kp_pos, gain_scale);
    scale_array(&ki_pos, gain_scale);
    scale_array(&kp_vel, gain_scale);
    scale_array(&ki_vel, gain_scale);
    scale_array(&kd_vel, gain_scale);
    max_vel = max_vel_in;
    max_acc = max_acc_in;
}

Controller::PositionLoopOutput Controller::compute_position_loop(
    const std::array<double, 12>& state,
    const Vec3& target_pos,
    const Vec3& target_vel,
    const Vec3& target_acc,
    double target_yaw
) {
    const Vec3 pos{state[0], state[1], state[2]};
    const Vec3 vel{state[3], state[4], state[5]};

    const Vec3 pos_err = target_pos - pos;
    integral_pos_ = clip_vec_symmetric(integral_pos_ + pos_err * dt_, integral_pos_limit);

    Vec3 des_vel = target_vel + mul(kp_pos, pos_err) + mul(ki_pos, integral_pos_);
    des_vel = clip_vec_symmetric(des_vel, max_vel);

    const Vec3 vel_err = des_vel - vel;
    integral_vel_ = clip_vec_symmetric(integral_vel_ + vel_err * dt_, integral_vel_limit);

    const Vec3 vel_deriv = (dt_ > 0.0) ? ((vel_err - prev_vel_err_) / dt_) : Vec3{0.0, 0.0, 0.0};
    prev_vel_err_ = vel_err;

    Vec3 des_acc = target_acc + mul(kp_vel, vel_err) + mul(ki_vel, integral_vel_) + mul(kd_vel, vel_deriv);
    const double acc_norm = norm(des_acc);
    if (acc_norm > max_acc) {
        des_acc = des_acc * (max_acc / acc_norm);
    }

    const Vec3 thrust_vec = des_acc + Vec3{0.0, 0.0, g_};
    const double thrust_mag = norm(thrust_vec);

    double thrust = m_ * thrust_mag;
    thrust = clamp(thrust, 0.1 * m_ * g_, 2.0 * m_ * g_);

    const Vec3 thrust_norm = (thrust_mag < 1e-6) ? Vec3{0.0, 0.0, 1.0} : (thrust_vec / thrust_mag);

    const double sy = std::sin(target_yaw);
    const double cy = std::cos(target_yaw);

    const double roll_arg = clamp(thrust_norm.x * sy - thrust_norm.y * cy, -1.0, 1.0);
    double des_roll = std::asin(roll_arg);

    const double cos_roll = std::cos(des_roll);
    double des_pitch = 0.0;
    if (std::abs(cos_roll) > 1e-6) {
        const double pitch_arg = clamp((thrust_norm.x * cy + thrust_norm.y * sy) / cos_roll, -1.0, 1.0);
        des_pitch = std::asin(pitch_arg);
    }

    des_roll = clamp(des_roll, -max_tilt, max_tilt);
    des_pitch = clamp(des_pitch, -max_tilt, max_tilt);

    last_des_att_ = Vec3{des_roll, des_pitch, target_yaw};

    return PositionLoopOutput{thrust, last_des_att_};
}

Vec3 Controller::compute_attitude_loop(const std::array<double, 12>& state, const Vec3& des_att) {
    const Vec3 att{state[6], state[7], state[8]};
    const Vec3 rate{state[9], state[10], state[11]};

    Vec3 att_err = des_att - att;
    att_err.z = std::fmod(att_err.z + kPi, 2.0 * kPi);
    if (att_err.z < 0.0) {
        att_err.z += 2.0 * kPi;
    }
    att_err.z -= kPi;

    const Vec3 des_rate = mul(kp_att, att_err);
    const Vec3 rate_err = des_rate - rate;

    integral_rate_ = clip_vec_symmetric(integral_rate_ + rate_err * dt_, integral_rate_limit);
    const Vec3 rate_deriv = (dt_ > 0.0) ? ((rate_err - prev_rate_err_) / dt_) : Vec3{0.0, 0.0, 0.0};
    prev_rate_err_ = rate_err;

    const Vec3 rate_cmd = mul(kp_rate, rate_err) + mul(ki_rate, integral_rate_) + mul(kd_rate, rate_deriv);
    return Vec3{
        inertia_diag_[0] * rate_cmd.x,
        inertia_diag_[1] * rate_cmd.y,
        inertia_diag_[2] * rate_cmd.z,
    };
}

std::array<double, 4> Controller::compute_control(
    const std::array<double, 12>& state,
    const Vec3& target_pos,
    const Vec3& target_vel,
    const Vec3& target_acc,
    double target_yaw
) {
    const PositionLoopOutput out = compute_position_loop(state, target_pos, target_vel, target_acc, target_yaw);
    const Vec3 torques = compute_attitude_loop(state, out.des_att);

    return std::array<double, 4>{out.thrust, torques.x, torques.y, torques.z};
}

void Controller::reset() {
    integral_pos_ = Vec3{0.0, 0.0, 0.0};
    integral_vel_ = Vec3{0.0, 0.0, 0.0};
    integral_rate_ = Vec3{0.0, 0.0, 0.0};
    prev_vel_err_ = Vec3{0.0, 0.0, 0.0};
    prev_rate_err_ = Vec3{0.0, 0.0, 0.0};
    last_des_att_ = Vec3{0.0, 0.0, 0.0};
}

HybridAttitudeController::HybridAttitudeController(double m, double g,
                                                   const std::array<double, 3>& inertia_diag,
                                                   double dt)
    : Controller(m, g, inertia_diag, dt),
      smc_(dt) {}

std::array<double, 4> HybridAttitudeController::compute_control(
    const std::array<double, 12>& state,
    const Vec3& target_pos,
    const Vec3& target_vel,
    const Vec3& target_acc,
    double target_yaw
) {
    const PositionLoopOutput out = compute_position_loop(state, target_pos, target_vel, target_acc, target_yaw);

    if (!use_smc_) {
        const Vec3 torques = compute_attitude_loop(state, out.des_att);
        return std::array<double, 4>{out.thrust, torques.x, torques.y, torques.z};
    }

    const Vec3 att{state[6], state[7], state[8]};
    const Vec3 rate{state[9], state[10], state[11]};

    Vec3 att_err = out.des_att - att;
    att_err.z = std::fmod(att_err.z + kPi, 2.0 * kPi);
    if (att_err.z < 0.0) {
        att_err.z += 2.0 * kPi;
    }
    att_err.z -= kPi;

    const Vec3 des_rate = mul(kp_att, att_err);
    const Vec3 ang_acc = smc_.update(att_err, rate, des_rate);
    const Vec3 torques{
        inertia_diag_[0] * ang_acc.x,
        inertia_diag_[1] * ang_acc.y,
        inertia_diag_[2] * ang_acc.z,
    };

    return std::array<double, 4>{out.thrust, torques.x, torques.y, torques.z};
}

void HybridAttitudeController::reset() {
    Controller::reset();
    smc_.reset();
}

// ====================================================================
// BacksteppingController — 反步法位置环 + SMC 姿态环
// ====================================================================

BacksteppingController::BacksteppingController(double m, double g,
                                               const std::array<double, 3>& inertia_diag,
                                               double dt)
    : HybridAttitudeController(m, g, inertia_diag, dt),
      K0{0.5, 0.5, 0.6},
      K1{1.8, 1.8, 2.2},
      K2{3.0, 3.0, 3.0} {
    use_smc_ = true;
}

void BacksteppingController::apply_profile(double gain_scale, double max_vel_in, double max_acc_in) {
    scale_array(&K0, gain_scale);
    scale_array(&K1, gain_scale);
    scale_array(&K2, gain_scale);
    max_vel = max_vel_in;
    max_acc = max_acc_in;
}

void BacksteppingController::reset() {
    HybridAttitudeController::reset();
    integral_z0_ = Vec3{0.0, 0.0, 0.0};
}

Controller::PositionLoopOutput BacksteppingController::compute_position_loop(
    const std::array<double, 12>& state,
    const Vec3& target_pos,
    const Vec3& target_vel,
    const Vec3& target_acc,
    double target_yaw
) {
    const Vec3 pos{state[0], state[1], state[2]};
    const Vec3 vel{state[3], state[4], state[5]};

    // ---- Step 0: 积分误差 ----
    const Vec3 z1_raw = pos - target_pos;
    integral_z0_ = clip_vec_symmetric(integral_z0_ + z1_raw * dt_, z0_limit);
    const Vec3 z0 = integral_z0_;

    // ---- Step 1: 位置误差 → 虚拟速度控制 ----
    const Vec3 z1 = clip_vec_symmetric(z1_raw, z1_limit);
    Vec3 alpha1 = -mul(K1, z1) - mul(K0, z0) + target_vel;
    alpha1 = clip_vec_symmetric(alpha1, alpha1_limit);

    // ---- Step 2: 速度误差 → 期望推力向量 ----
    const Vec3 z2 = clip_vec_symmetric(vel - alpha1, z2_limit);
    const Vec3 alpha1_dot = -mul(K1, vel - target_vel) + target_acc;

    const Vec3 f_des = {
        m_ * (-K2[0] * z2.x - z1.x - K0[0] * z0.x + alpha1_dot.x),
        m_ * (-K2[1] * z2.y - z1.y - K0[1] * z0.y + alpha1_dot.y),
        m_ * (-K2[2] * z2.z - z1.z - K0[2] * z0.z + g_ + alpha1_dot.z),
    };

    const double thrust_mag = norm(f_des);
    double thrust = clamp(thrust_mag, 0.1 * m_ * g_, 2.5 * m_ * g_);

    const Vec3 b3_des = (thrust_mag > 1e-6) ? (f_des / thrust_mag) : Vec3{0.0, 0.0, 1.0};

    // ---- Step 3: 从 b3_des + yaw 提取 roll/pitch ----
    const double sy = std::sin(target_yaw);
    const double cy = std::cos(target_yaw);
    const double roll_arg = clamp(b3_des.x * sy - b3_des.y * cy, -1.0, 1.0);
    double des_roll = std::asin(roll_arg);

    const double cos_roll = std::cos(des_roll);
    double des_pitch = 0.0;
    if (std::abs(cos_roll) > 1e-6) {
        const double pitch_arg = clamp((b3_des.x * cy + b3_des.y * sy) / cos_roll, -1.0, 1.0);
        des_pitch = std::asin(pitch_arg);
    }

    des_roll = clamp(des_roll, -max_tilt, max_tilt);
    des_pitch = clamp(des_pitch, -max_tilt, max_tilt);

    last_des_att_ = Vec3{des_roll, des_pitch, target_yaw};
    return PositionLoopOutput{thrust, last_des_att_};
}

}  // namespace sim
