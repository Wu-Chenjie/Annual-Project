#include "formation_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {

DownwashZone downwash_zone(double radius, double height) {
    return DownwashZone{std::max(0.0, radius), std::max(0.0, height)};
}

bool is_in_downwash_zone(const Vec3& upper, const Vec3& lower, const DownwashZone& zone) {
    const double dz = upper.z - lower.z;
    if (dz <= 0.0 || dz > zone.height) return false;
    const double lateral = std::hypot(upper.x - lower.x, upper.y - lower.y);
    return lateral <= zone.radius;
}

double min_inter_drone_distance(const std::vector<Vec3>& positions) {
    if (positions.size() < 2) return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < positions.size(); ++i) {
        for (std::size_t j = i + 1; j < positions.size(); ++j) {
            best = std::min(best, norm(positions[i] - positions[j]));
        }
    }
    return best;
}

bool nominal_target_ready_for_recovery(
    const Vec3& nominal_target,
    const std::vector<Vec3>& reserved_positions,
    const Vec3* current_pos,
    double signed_distance,
    bool segment_safe,
    double min_clearance,
    double min_inter_distance,
    const DownwashZone* downwash,
    double recovery_margin) {
    if (signed_distance < (min_clearance + recovery_margin) - 1e-8) return false;
    if (current_pos && !segment_safe) return false;
    for (const auto& reserved : reserved_positions) {
        if (norm(nominal_target - reserved) < (min_inter_distance + recovery_margin * 0.5) - 1e-8) {
            return false;
        }
        if (downwash) {
            if (is_in_downwash_zone(nominal_target, reserved, *downwash)
                || is_in_downwash_zone(reserved, nominal_target, *downwash)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace sim
