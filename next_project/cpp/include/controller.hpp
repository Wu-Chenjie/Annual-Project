#pragma once

#include <array>

#include "math_utils.hpp"
#include "smc.hpp"

namespace sim {

class Controller {
public:
    Controller(double m = 1.0, double g = 9.81,
               const std::array<double, 3>& inertia_diag = {0.01, 0.01, 0.02},
               double dt = 0.01);
    virtual ~Controller() = default;

    virtual std::array<double, 4> compute_control(
        const std::array<double, 12>& state,
        const Vec3& target_pos,
        const Vec3& target_vel = Vec3{0.0, 0.0, 0.0},
        const Vec3& target_acc = Vec3{0.0, 0.0, 0.0},
        double target_yaw = 0.0
    );

    virtual void reset();
    virtual void apply_profile(double gain_scale, double max_vel, double max_acc);

    std::array<double, 3> kp_pos;
    std::array<double, 3> ki_pos;
    std::array<double, 3> kp_vel;
    std::array<double, 3> ki_vel;
    std::array<double, 3> kd_vel;
    std::array<double, 3> kp_att;
    std::array<double, 3> kp_rate;
    std::array<double, 3> ki_rate;
    std::array<double, 3> kd_rate;

    double max_acc;
    double max_tilt;
    double max_vel;
    double integral_pos_limit;
    double integral_vel_limit;
    double integral_rate_limit;

protected:
    struct PositionLoopOutput {
        double thrust = 0.0;
        Vec3 des_att{};
    };

    PositionLoopOutput compute_position_loop(
        const std::array<double, 12>& state,
        const Vec3& target_pos,
        const Vec3& target_vel,
        const Vec3& target_acc,
        double target_yaw
    );

    Vec3 compute_attitude_loop(const std::array<double, 12>& state, const Vec3& des_att);

    std::array<double, 3> inertia_diag_;
    double m_;
    double g_;
    double dt_;

    Vec3 integral_pos_{};
    Vec3 integral_vel_{};
    Vec3 integral_rate_{};
    Vec3 prev_vel_err_{};
    Vec3 prev_rate_err_{};
    Vec3 last_des_att_{};
};

class HybridAttitudeController : public Controller {
public:
    HybridAttitudeController(double m = 1.0, double g = 9.81,
                             const std::array<double, 3>& inertia_diag = {0.01, 0.01, 0.02},
                             double dt = 0.01);

    std::array<double, 4> compute_control(
        const std::array<double, 12>& state,
        const Vec3& target_pos,
        const Vec3& target_vel = Vec3{0.0, 0.0, 0.0},
        const Vec3& target_acc = Vec3{0.0, 0.0, 0.0},
        double target_yaw = 0.0
    ) override;

    void reset() override;

    void set_use_smc(bool value) { use_smc_ = value; }
    [[nodiscard]] bool use_smc() const { return use_smc_; }

protected:
    bool use_smc_ = true;

private:
    SecondOrderSMC smc_;
};

class BacksteppingController : public HybridAttitudeController {
public:
    BacksteppingController(double m = 1.0, double g = 9.81,
                           const std::array<double, 3>& inertia_diag = {0.01, 0.01, 0.02},
                           double dt = 0.01);

    void reset() override;
    void apply_profile(double gain_scale, double max_vel, double max_acc) override;

    // 反步法增益（可随 apply_profile 缩放）
    std::array<double, 3> K0;  // 积分增益
    std::array<double, 3> K1;  // 位置误差增益
    std::array<double, 3> K2;  // 速度误差增益

    // 误差饱和限幅
    double z0_limit = 3.0;
    double z1_limit = 5.0;
    double z2_limit = 10.0;
    double alpha1_limit = 12.0;

private:
    PositionLoopOutput compute_position_loop(
        const std::array<double, 12>& state,
        const Vec3& target_pos,
        const Vec3& target_vel,
        const Vec3& target_acc,
        double target_yaw
    );

    Vec3 integral_z0_{};
};

}  // namespace sim
