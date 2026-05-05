#pragma once

#include <array>

#include "math_utils.hpp"

namespace sim {

class SecondOrderSMC {
public:
    explicit SecondOrderSMC(double dt, const std::array<double, 3>& lam = {7.8579, 8.2384, 3.9090},
                            const std::array<double, 3>& epsilon = {5.1161, 5.0003, 3.0693},
                            const std::array<double, 3>& k = {7.9432, 8.2315, 3.9736},
                            const std::array<double, 3>& ki = {0.8638, 1.1520, 0.9825},
                            double delta = 0.05, double ki_limit = 2.0);

    Vec3 update(const Vec3& angle_error, const Vec3& angle_rate, const Vec3& des_rate = Vec3{0.0, 0.0, 0.0});
    void reset();

private:
    std::array<double, 3> lam_;
    std::array<double, 3> epsilon_;
    std::array<double, 3> k_;
    std::array<double, 3> ki_;
    double dt_;
    double delta_;
    double ki_limit_;
    Vec3 integral_s_{};

    [[nodiscard]] Vec3 sat(const Vec3& s) const;
};

}  // namespace sim
