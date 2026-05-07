#pragma once

#include <vector>

#include "math_utils.hpp"

namespace sim {

struct FormationSafetyConfig {
    bool enabled = false;
    double min_inter_drone_distance = 0.35;
    double downwash_radius = 0.45;
    double downwash_height = 0.80;
    int recovery_hold_steps = 4;
    double recovery_clearance_margin = 0.10;
    int follower_shrink_steps = 10;
    double conflict_vertical_step = 0.35;
    double conflict_lateral_step = 0.30;
};

struct DownwashZone {
    double radius = 0.45;
    double height = 0.80;
};

[[nodiscard]] DownwashZone downwash_zone(double radius, double height);
[[nodiscard]] bool is_in_downwash_zone(const Vec3& upper, const Vec3& lower, const DownwashZone& zone);
[[nodiscard]] double min_inter_drone_distance(const std::vector<Vec3>& positions);
[[nodiscard]] bool nominal_target_ready_for_recovery(
    const Vec3& nominal_target,
    const std::vector<Vec3>& reserved_positions,
    const Vec3* current_pos,
    double signed_distance,
    bool segment_safe,
    double min_clearance,
    double min_inter_distance,
    const DownwashZone* downwash,
    double recovery_margin = 0.10);

}  // namespace sim
