#pragma once

#include <array>

namespace sim {

class ControlAllocator {
public:
    ControlAllocator(double arm_length = 0.2, double kf = 1e-5, double km = 2e-7);

    [[nodiscard]] std::array<double, 4> allocate_thrusts(const std::array<double, 4>& u) const;
    [[nodiscard]] std::array<double, 4> thrusts_to_omegas(const std::array<double, 4>& thrusts) const;
    [[nodiscard]] std::array<double, 4> omegas_to_u(const std::array<double, 4>& omegas) const;

private:
    double arm_length_;
    double kf_;
    double km_;
    std::array<std::array<double, 4>, 4> B_{};
    std::array<std::array<double, 4>, 4> B_inv_{};

    static std::array<std::array<double, 4>, 4> invert_4x4(const std::array<std::array<double, 4>, 4>& m);
};

}  // namespace sim
