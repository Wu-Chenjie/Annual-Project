#pragma once

#include <array>
#include <random>

#include "math_utils.hpp"
#include "obstacles.hpp"

namespace sim {

class RangeSensor6 {
public:
    RangeSensor6(double max_range = 8.0, double noise_std = 0.02, unsigned int seed = 0)
        : max_range_(max_range), noise_std_(noise_std), rng_(seed) {}

    std::array<double, 6> sense(const Vec3& pose, const ObstacleField& field) const;

private:
    static const std::array<Vec3, 6>& dirs() {
        static const std::array<Vec3, 6> d{Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,1,0},Vec3{0,-1,0},Vec3{0,0,1},Vec3{0,0,-1}};
        return d;
    }
    double max_range_, noise_std_;
    mutable std::mt19937 rng_;
};

inline std::array<double, 6> RangeSensor6::sense(const Vec3& pose, const ObstacleField& field) const {
    std::array<double, 6> dists;
    std::normal_distribution<double> noise(0.0, noise_std_);
    const auto& d = dirs();
    for (int di = 0; di < 6; ++di) {
        double t = 0.0, step = max_range_ / 200.0;
        while (t < max_range_) {
            Vec3 p = pose + d[di] * t;
            if (field.is_collision(p)) { t = std::max(t - step, 0.0); break; }
            t += step;
        }
        if (t >= max_range_) t = max_range_;
        dists[di] = std::max(0.0, std::min(t + noise(rng_), max_range_));
    }
    return dists;
}

}  // namespace sim
