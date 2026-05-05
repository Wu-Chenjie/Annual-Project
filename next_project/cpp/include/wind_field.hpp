#pragma once

#include <random>

#include "math_utils.hpp"

namespace sim {

class WindField {
public:
    WindField(const Vec3& steady = Vec3{0.0, 0.0, 0.0}, double turbulence_std = 0.1,
              double tau = 0.5, unsigned int seed = 0U);

    Vec3 sample(double dt);

private:
    Vec3 steady_;
    double turbulence_std_;
    double tau_;
    Vec3 state_{};
    std::mt19937 rng_;
    std::normal_distribution<double> dist_{0.0, 1.0};
};

}  // namespace sim
