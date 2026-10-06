#include "wind_field.hpp"

#include <cmath>

namespace sim {

WindField::WindField(const Vec3& steady, double turbulence_std, double tau, unsigned int seed)
    : steady_(steady), turbulence_std_(turbulence_std), tau_(tau), rng_(seed) {}

Vec3 WindField::sample(double dt) {
    const double coeff = turbulence_std_ * std::sqrt(2.0 / tau_) * std::sqrt(dt);

    const Vec3 noise{
        coeff * dist_(rng_),
        coeff * dist_(rng_),
        coeff * dist_(rng_),
    };

    const Vec3 drift = (-1.0 / tau_) * state_ * dt;
    state_ += drift + noise;
    return steady_ + state_;
}

}  // namespace sim
