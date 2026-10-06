#include "smc.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

namespace {

Vec3 mul(const std::array<double, 3>& gain, const Vec3& v) {
    return Vec3{gain[0] * v.x, gain[1] * v.y, gain[2] * v.z};
}

}  // namespace

SecondOrderSMC::SecondOrderSMC(double dt, const std::array<double, 3>& lam,
                               const std::array<double, 3>& epsilon,
                               const std::array<double, 3>& k,
                               const std::array<double, 3>& ki,
                               double delta, double ki_limit)
    : lam_(lam),
      epsilon_(epsilon),
      k_(k),
      ki_(ki),
      dt_(dt),
      delta_(delta),
      ki_limit_(ki_limit) {}

Vec3 SecondOrderSMC::sat(const Vec3& s) const {
    auto sat_scalar = [this](double v) {
        if (std::abs(v) <= delta_) {
            return v / delta_;
        }
        return (v > 0.0 ? 1.0 : -1.0);
    };

    return Vec3{sat_scalar(s.x), sat_scalar(s.y), sat_scalar(s.z)};
}

Vec3 SecondOrderSMC::update(const Vec3& angle_error, const Vec3& angle_rate, const Vec3& des_rate) {
    const Vec3 rate_error = des_rate - angle_rate;
    const Vec3 s = rate_error + mul(lam_, angle_error);
    const Vec3 sat_s = sat(s);

    integral_s_ += s * dt_;
    integral_s_ = Vec3{
        clamp(integral_s_.x, -ki_limit_, ki_limit_),
        clamp(integral_s_.y, -ki_limit_, ki_limit_),
        clamp(integral_s_.z, -ki_limit_, ki_limit_),
    };

    return mul(lam_, rate_error) + mul(epsilon_, sat_s) + mul(k_, s) + mul(ki_, integral_s_);
}

void SecondOrderSMC::reset() {
    integral_s_ = Vec3{0.0, 0.0, 0.0};
}

}  // namespace sim
