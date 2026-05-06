#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <variant>

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
    static double ray_aabb(const Vec3& origin, const Vec3& direction,
                           const Vec3& bmin, const Vec3& bmax);
    static double ray_sphere(const Vec3& origin, const Vec3& direction,
                             const Vec3& center, double radius);
    static double ray_cylinder(const Vec3& origin, const Vec3& direction,
                               const Vec3& center_xy, double radius,
                               double z_min, double z_max);
    static double ray_obstacle_intersect(const Vec3& origin, const Vec3& direction,
                                         const ObstacleVariant& obstacle);
    double max_range_, noise_std_;
    mutable std::mt19937 rng_;
};

inline std::array<double, 6> RangeSensor6::sense(const Vec3& pose, const ObstacleField& field) const {
    std::array<double, 6> dists;
    std::normal_distribution<double> noise(0.0, noise_std_);
    const auto& d = dirs();
    for (int di = 0; di < 6; ++di) {
        double t_min = max_range_;
        for (const auto& obstacle : field.obstacles()) {
            double t = ray_obstacle_intersect(pose, d[di], obstacle);
            if (t > 0.0 && t < t_min) {
                t_min = t;
            }
        }
        dists[di] = std::max(0.0, std::min(t_min + noise(rng_), max_range_));
    }
    return dists;
}

inline double RangeSensor6::ray_aabb(const Vec3& origin, const Vec3& direction,
                                     const Vec3& bmin, const Vec3& bmax) {
    double t_enter = -std::numeric_limits<double>::infinity();
    double t_exit = std::numeric_limits<double>::infinity();
    const std::array<double, 3> o{origin.x, origin.y, origin.z};
    const std::array<double, 3> d{direction.x, direction.y, direction.z};
    const std::array<double, 3> mn{bmin.x, bmin.y, bmin.z};
    const std::array<double, 3> mx{bmax.x, bmax.y, bmax.z};
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) < 1e-12) {
            if (o[axis] < mn[axis] || o[axis] > mx[axis]) {
                return std::numeric_limits<double>::infinity();
            }
        } else {
            const double inv = 1.0 / d[axis];
            double t1 = (mn[axis] - o[axis]) * inv;
            double t2 = (mx[axis] - o[axis]) * inv;
            if (t1 > t2) std::swap(t1, t2);
            t_enter = std::max(t_enter, t1);
            t_exit = std::min(t_exit, t2);
        }
    }
    if (t_enter <= t_exit && t_exit >= 0.0) {
        return t_enter >= 0.0 ? t_enter : t_exit;
    }
    return std::numeric_limits<double>::infinity();
}

inline double RangeSensor6::ray_sphere(const Vec3& origin, const Vec3& direction,
                                       const Vec3& center, double radius) {
    const Vec3 oc = origin - center;
    const double a = dot(direction, direction);
    const double b = 2.0 * dot(oc, direction);
    const double c = dot(oc, oc) - radius * radius;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    const double sqrt_disc = std::sqrt(disc);
    const double t1 = (-b - sqrt_disc) / (2.0 * a);
    const double t2 = (-b + sqrt_disc) / (2.0 * a);
    if (t1 >= 0.0) return t1;
    if (t2 >= 0.0) return t2;
    return std::numeric_limits<double>::infinity();
}

inline double RangeSensor6::ray_cylinder(const Vec3& origin, const Vec3& direction,
                                         const Vec3& center_xy, double radius,
                                         double z_min, double z_max) {
    const double a = direction.x * direction.x + direction.y * direction.y;
    const double b = 2.0 * (direction.x * (origin.x - center_xy.x)
                          + direction.y * (origin.y - center_xy.y));
    const double c = (origin.x - center_xy.x) * (origin.x - center_xy.x)
                   + (origin.y - center_xy.y) * (origin.y - center_xy.y)
                   - radius * radius;
    std::array<double, 4> candidates{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    };
    int count = 0;
    if (a >= 1e-12) {
        const double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            const double sqrt_disc = std::sqrt(disc);
            const std::array<double, 2> roots{
                (-b - sqrt_disc) / (2.0 * a),
                (-b + sqrt_disc) / (2.0 * a),
            };
            for (double t : roots) {
                if (t >= 0.0) {
                    const double z = origin.z + t * direction.z;
                    if (z >= z_min && z <= z_max) {
                        candidates[count++] = t;
                    }
                }
            }
        }
    }
    for (double z_plane : {z_min, z_max}) {
        if (std::abs(direction.z) > 1e-12) {
            const double t = (z_plane - origin.z) / direction.z;
            if (t >= 0.0) {
                const double px = origin.x + t * direction.x;
                const double py = origin.y + t * direction.y;
                if ((px - center_xy.x) * (px - center_xy.x)
                    + (py - center_xy.y) * (py - center_xy.y) <= radius * radius) {
                    candidates[count++] = t;
                }
            }
        }
    }
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < count; ++i) {
        best = std::min(best, candidates[i]);
    }
    return best;
}

inline double RangeSensor6::ray_obstacle_intersect(const Vec3& origin, const Vec3& direction,
                                                   const ObstacleVariant& obstacle) {
    return std::visit([&](const auto& obs) -> double {
        using T = std::decay_t<decltype(obs)>;
        if constexpr (std::is_same_v<T, AABB>) {
            return ray_aabb(origin, direction, obs.min_corner, obs.max_corner);
        } else if constexpr (std::is_same_v<T, Sphere>) {
            return ray_sphere(origin, direction, obs.center, obs.radius);
        } else {
            return ray_cylinder(origin, direction, obs.center_xy, obs.radius, obs.z_min, obs.z_max);
        }
    }, obstacle);
}

}  // namespace sim
